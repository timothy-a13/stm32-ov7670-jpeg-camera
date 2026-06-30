import argparse
import struct
import sys
from io import BytesIO
from pathlib import Path

import numpy as np
from PIL import Image
import serial


MAGIC = bytes([0xA5, 0x5A, 0x12, 0x34])
FORMAT_RGB565 = 1
FORMAT_YUV422 = 2
FORMAT_JPEG = 3
HEADER_STRUCT = struct.Struct("<4sHHHHII")
# <       little-endian
# 4s      magic
# H H     width, height
# H H     format, bytes_per_pixel
# I I     payload_len, checksum


def read_exact(ser: serial.Serial, n: int) -> bytes:
    data = bytearray()

    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            raise TimeoutError(f"Timeout while reading {n} bytes, got {len(data)} bytes")
        data.extend(chunk)

    return bytes(data)


def wait_for_magic(ser: serial.Serial) -> None:
    window = bytearray()

    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout while waiting for OV76 frame magic")

        window.extend(b)

        if len(window) > len(MAGIC):
            del window[0]

        if bytes(window) == MAGIC:
            return


def checksum8(data: bytes) -> int:
    return sum(data) & 0xFFFFFFFF


def rgb565_to_rgb888(payload: bytes, width: int, height: int,
                     swap_bytes: bool = False,
                     swap_rb: bool = False) -> np.ndarray:
    expected = width * height * 2

    if len(payload) != expected:
        raise ValueError(f"RGB565 payload size mismatch: expected {expected}, got {len(payload)}")

    raw = np.frombuffer(payload, dtype=np.uint8).reshape((height, width, 2))

    if swap_bytes:
        rgb565 = (raw[:, :, 1].astype(np.uint16) << 8) | raw[:, :, 0].astype(np.uint16)
    else:
        rgb565 = (raw[:, :, 0].astype(np.uint16) << 8) | raw[:, :, 1].astype(np.uint16)

    r = ((rgb565 >> 11) & 0x1F) * 255 // 31
    g = ((rgb565 >> 5) & 0x3F) * 255 // 63
    b = np.zeros_like(g)
    b[:, 1:] = ((rgb565 & 0x1F) * 255 // 31)[:, :-1]

    rgb = np.dstack((r, g, b)).astype(np.uint8)

    if swap_rb:
        rgb = rgb[:, :, ::-1]

    return rgb


def yuv_to_rgb_pixel(y, u, v):
    c = y.astype(np.int32)
    d = u.astype(np.int32) - 128
    e = v.astype(np.int32) - 128

    r = c + ((359 * e) >> 8)
    g = c - ((88 * d + 183 * e) >> 8)
    b = c + ((454 * d) >> 8)

    r = np.clip(r, 0, 255)
    g = np.clip(g, 0, 255)
    b = np.clip(b, 0, 255)

    return r.astype(np.uint8), g.astype(np.uint8), b.astype(np.uint8)


def yuv422_to_rgb888(payload: bytes, width: int, height: int, order: str) -> np.ndarray:
    expected = width * height * 2

    if len(payload) != expected:
        raise ValueError(f"YUV422 payload size mismatch: expected {expected}, got {len(payload)}")

    raw = np.frombuffer(payload, dtype=np.uint8).reshape((height, width // 2, 4))

    b0 = raw[:, :, 0]
    b1 = raw[:, :, 1]
    b2 = raw[:, :, 2]
    b3 = raw[:, :, 3]

    if order == "YUYV":
        y0, u, y1, v = b0, b1, b2, b3
    elif order == "YVYU":
        y0, v, y1, u = b0, b1, b2, b3
    elif order == "UYVY":
        u, y0, v, y1 = b0, b1, b2, b3
    elif order == "VYUY":
        v, y0, u, y1 = b0, b1, b2, b3
    else:
        raise ValueError(f"Unknown YUV422 order: {order}")

    r0, g0, b0_rgb = yuv_to_rgb_pixel(y0, u, v)
    r1, g1, b1_rgb = yuv_to_rgb_pixel(y1, u, v)

    rgb = np.zeros((height, width, 3), dtype=np.uint8)

    rgb[:, 0::2, 0] = r0
    rgb[:, 0::2, 1] = g0
    rgb[:, 0::2, 2] = b0_rgb

    rgb[:, 1::2, 0] = r1
    rgb[:, 1::2, 1] = g1
    rgb[:, 1::2, 2] = b1_rgb

    return rgb


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output", default="ov7670_frame.png")
    parser.add_argument("--swap-bytes", action="store_true",
                        help="Use this if the image colors look wrong due to RGB565 byte order")
    parser.add_argument("--swap-rb", action="store_true",
                        help="Use this if red and blue are swapped")
    args = parser.parse_args()

    output_path = Path(args.output)

    print(f"Opening serial port {args.port} @ {args.baud}...")
    print("Make sure PuTTY / Serial Monitor is closed.")

    with serial.Serial(args.port, args.baud, timeout=5) as ser:
        print("Waiting for binary frame magic A5 5A 12 34...")
        wait_for_magic(ser)

        rest_header = read_exact(ser, HEADER_STRUCT.size - len(MAGIC))
        header_bytes = MAGIC + rest_header

        magic, width, height, fmt, bpp, payload_len, checksum = HEADER_STRUCT.unpack(header_bytes)

        print("Frame header:")
        print(f"  width           = {width}")
        print(f"  height          = {height}")
        print(f"  format          = {fmt}")
        print(f"  bytes_per_pixel = {bpp}")
        print(f"  payload_len     = {payload_len}")
        print(f"  checksum        = 0x{checksum:08X}")

        if magic != MAGIC:
            raise ValueError("Invalid magic")

        if fmt not in (FORMAT_RGB565, FORMAT_YUV422, FORMAT_JPEG):
            raise ValueError(f"Unsupported format: {fmt}, expected 1 = RGB565, 2 = YUV422, 3 = JPEG")

        if fmt in (FORMAT_RGB565, FORMAT_YUV422):
            if bpp != 2:
                raise ValueError(f"Unsupported bytes_per_pixel: {bpp}, expected 2")

            expected_len = width * height * bpp
            if payload_len != expected_len:
                raise ValueError(f"Payload length mismatch: header={payload_len}, expected={expected_len}")
        elif fmt == FORMAT_JPEG:
            if payload_len == 0:
                raise ValueError("JPEG payload length is zero")

        print("Reading payload...")
        payload = read_exact(ser, payload_len)

    calc_checksum = checksum8(payload)
    print(f"Calculated checksum = 0x{calc_checksum:08X}")

    if calc_checksum != checksum:
        print("WARNING: checksum mismatch. Image may be corrupted.")
    else:
        print("Checksum OK.")

    if fmt == FORMAT_RGB565:
        print("Decoding as RGB565...")

        variants = [
            (False, False, "rgb565_normal"),
            (True,  False, "rgb565_swap_bytes"),
            (False, True,  "rgb565_swap_rb"),
            (True,  True,  "rgb565_swap_bytes_swap_rb"),
        ]

        for swap_bytes, swap_rb, name in variants:
            rgb = rgb565_to_rgb888(
                payload,
                width,
                height,
                swap_bytes=swap_bytes,
                swap_rb=swap_rb,
            )

            img = Image.fromarray(rgb, mode="RGB")
            variant_path = output_path.with_name(f"{output_path.stem}_{name}.png")
            img.save(variant_path)
            print(f"Saved {name}: {variant_path}")

    elif fmt == FORMAT_YUV422:
        print("Decoding as YUV422...")

        orders = ["YUYV", "YVYU", "UYVY", "VYUY"]

        for order in orders:
            rgb = yuv422_to_rgb888(payload, width, height, order)

            img = Image.fromarray(rgb, mode="RGB")
            variant_path = output_path.with_name(f"{output_path.stem}_{order}.png")
            img.save(variant_path)
            print(f"Saved {order}: {variant_path}")

    elif fmt == FORMAT_JPEG:
        print("Saving and decoding JPEG...")

        jpeg_path = output_path.with_suffix(".jpg")
        jpeg_path.write_bytes(payload)
        print(f"Saved JPEG: {jpeg_path}")

        with Image.open(BytesIO(payload)) as img:
            decoded = img.convert("RGB")

            if decoded.size != (width, height):
                print(f"WARNING: decoded JPEG size is {decoded.size}, header says {(width, height)}")

            if output_path.suffix.lower() in (".jpg", ".jpeg"):
                decoded_path = output_path.with_name(f"{output_path.stem}_decoded.png")
            else:
                decoded_path = output_path

            decoded.save(decoded_path)
            print(f"Saved decoded image: {decoded_path}")

    else:
        raise ValueError(f"Unsupported format: {fmt}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        raise SystemExit(1)
    

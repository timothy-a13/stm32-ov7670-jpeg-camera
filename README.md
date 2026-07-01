# NUCLEO-F446RE OV7670 JPEG Camera

[English](./README.md) | [中文](./README.zh.md)

This project captures images from an OV7670 camera module with a NUCLEO-F446RE board, compresses the RGB565 frame into JPEG on the STM32F446RE, and sends the compressed image to a PC over UART. The PC-side Python script receives the binary frame, verifies the checksum, saves the JPEG payload, and decodes it into an image file.

![OV7670 result](assets/result.png)

## Key Features

- STM32CubeIDE project for the NUCLEO-F446RE / STM32F446RETx.
- OV7670 parallel camera capture through DCMI + DMA.
- OV7670 SCCB control implemented with GPIO bit-banging.
- Camera XCLK generated from PA8 / MCO1.
- Input frame format is RGB565 at 160 x 120.
- Low-memory baseline JPEG encoder designed for MCU SRAM limits.
- JPEG processing is block based: RGB565 is converted to YCbCr one 8 x 8 block at a time, then immediately passed to DCT and quantization.
- Huffman tables are generated dynamically from the current image statistics; the encoder does not use the default JPEG Huffman tables.
- UART frame protocol includes magic bytes, image metadata, payload length, and checksum.
- PC script supports JPEG payloads and also provides RGB565 / YUV422 decoding flows.

## Project Layout

```text
Core/
  Inc/
    main.h
    jpeg_encoder.h
  Src/
    main.c
    jpeg_encoder.c
script/
  receive_ov7670.py
assets/
  result.png
test_cam.ioc
STM32F446RETX_FLASH.ld
```

Important files:

- `Core/Src/main.c`: board initialization, OV7670 register setup, DCMI snapshot capture, and UART transmission.
- `Core/Src/jpeg_encoder.c`: low-memory RGB565-to-JPEG encoder.
- `script/receive_ov7670.py`: PC receiver and decoder.
- `test_cam.ioc`: STM32CubeMX / STM32CubeIDE configuration.

## Hardware

### Required Parts

- NUCLEO-F446RE development board.
- OV7670 camera module with 8-bit parallel output.
- Jumper wires.
- USB cable for ST-LINK power, flashing, debugging, and UART virtual COM port.

Use 3.3 V logic. Do not connect OV7670 signal pins to 5 V.

### Pin Mapping

| OV7670 Signal | NUCLEO-F446RE / STM32F446RE Pin | Purpose |
|---|---|---|
| D0 | PC6 | DCMI data bit 0 |
| D1 | PC7 | DCMI data bit 1 |
| D2 | PC8 | DCMI data bit 2 |
| D3 | PC9 | DCMI data bit 3 |
| D4 | PC11 | DCMI data bit 4 |
| D5 | PB6 | DCMI data bit 5 |
| D6 | PB8 | DCMI data bit 6 |
| D7 | PB9 | DCMI data bit 7 |
| PCLK | PA6 | DCMI pixel clock |
| HREF / HSYNC | PA4 | DCMI horizontal sync |
| VSYNC | PB7 | DCMI vertical sync |
| XCLK | PA8 | MCO1 camera clock output |
| SIOC | PB13 | SCCB clock |
| SIOD | PB14 | SCCB data |
| 3V3 | 3V3 | Camera power |
| GND | GND | Ground |

UART uses USART2 through the ST-LINK virtual COM port:

| USART2 Signal | Pin |
|---|---|
| TX | PA2 |
| RX | PA3 |

## Firmware Build and Flash

### Software Requirements

- STM32CubeIDE, tested with STM32CubeIDE 1.18.1.
- STM32Cube FW_F4 package, tested with V1.28.3.
- The project `.ioc` was generated with STM32CubeMX database version 6.14.1.

### Build in STM32CubeIDE

1. Open STM32CubeIDE.
2. Choose `File > Import...`.
3. Select `General > Existing Projects into Workspace`.
4. Select this repository folder as the root directory.
5. Import the `test_cam2` project.
6. Right-click the project and choose `Refresh`.
7. Build with `Project > Build Project`.

### Flash

1. Connect the NUCLEO-F446RE board through USB.
2. Build the project.
3. Use `Run > Debug` or `Run > Run` in STM32CubeIDE.
4. Let STM32CubeIDE flash the board through ST-LINK.

After reset, the firmware:

1. Starts XCLK on PA8.
2. Initializes SCCB on PB13/PB14.
3. Reads the OV7670 PID/VER registers.
4. Writes the RGB565 QQVGA camera register table.
5. Captures several warm-up frames.
6. Captures the final frame.
7. Compresses it as JPEG.
8. Sends the JPEG frame over USART2 at 921600 baud.

## PC Receiver

### Python Requirements

Use Python 3 and install:

```bash
python -m pip install pyserial pillow numpy
```

### Run

Close any serial monitor before running the script. Replace `COM5` with the ST-LINK virtual COM port on your PC.

```bash
python script/receive_ov7670.py --port COM5 --baud 921600 --output assets/result.png
```

On Linux/macOS, the port may look like `/dev/ttyACM0`:

```bash
python script/receive_ov7670.py --port /dev/ttyACM0 --baud 921600 --output assets/result.png
```

For a JPEG frame, the script writes:

- `assets/result.jpg`: raw JPEG payload received from the board.
- `assets/result.png`: decoded RGB image.

The script also verifies the payload checksum from the MCU header.

## UART Frame Protocol

The firmware sends a binary header followed by the payload:

```c
typedef struct __attribute__((packed))
{
    uint8_t  magic[4];        // A5 5A 12 34
    uint16_t width;           // 160
    uint16_t height;          // 120
    uint16_t format;          // 3 = JPEG
    uint16_t bytes_per_pixel; // 0 for JPEG
    uint32_t payload_len;     // JPEG byte count
    uint32_t checksum;        // byte sum of JPEG payload
} CameraFrameHeader_t;
```

Current format IDs:

| Format | Meaning |
|---:|---|
| 1 | Raw RGB565 |
| 2 | Raw YUV422 |
| 3 | JPEG |

The current firmware sends format `3`.

## JPEG Encoder Design

The encoder is intentionally written for limited STM32F446RE SRAM:

- No full-frame YCbCr buffer is allocated.
- No full JPEG payload buffer is allocated.
- No dynamic memory allocation is used.
- The RGB565 frame buffer remains the source image.
- One 8 x 8 block is converted, transformed, quantized, and consumed before the next block is processed.

The JPEG generation flow is:

```text
RGB565 frame buffer
  -> load one 8 x 8 block
  -> convert only that block to one Y/Cb/Cr component
  -> DCT
  -> quantization
  -> collect Huffman symbol statistics or emit entropy bits
  -> next component / next block
```

Because the Huffman tables are generated from the actual image, the encoder uses multiple passes over the captured RGB565 frame:

1. First pass: collect DC/AC symbol frequencies block by block.
2. Build dynamic JPEG Huffman tables.
3. Second pass: dry-run the JPEG output to compute payload length and checksum.
4. Third pass: emit the same JPEG stream to UART.

This costs CPU time, but it avoids storing the compressed JPEG in SRAM.

#ifndef JPEG_ENCODER_H
#define JPEG_ENCODER_H

#include <stdint.h>

typedef uint8_t (*JpegEncoderWriteFn)(const uint8_t *data, uint32_t len, void *user);

typedef struct
{
    uint32_t bytes_written;
    uint32_t checksum;
} JpegEncodeResult_t;

uint8_t JpegEncoder_PrepareRgb565(const uint8_t *frame,
                                  uint16_t width,
                                  uint16_t height);

uint8_t JpegEncoder_EmitRgb565(const uint8_t *frame,
                               uint16_t width,
                               uint16_t height,
                               JpegEncoderWriteFn write,
                               void *user,
                               JpegEncodeResult_t *result);

const char *JpegEncoder_GetLastError(void);

#endif /* JPEG_ENCODER_H */

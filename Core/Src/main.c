/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jpeg_encoder.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed))
{
    uint8_t  magic[4];         // 'O', 'V', '7', '6'
    uint16_t width;            // 160
    uint16_t height;           // 120
    uint16_t format;           // 1 = RGB565, 3 = JPEG
    uint16_t bytes_per_pixel;  // 2 for raw RGB565, 0 for JPEG
    uint32_t payload_len;      // raw bytes or JPEG payload bytes
    uint32_t checksum;         // simple byte-sum
} CameraFrameHeader_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCCB_SIOC_GPIO_Port GPIOB
#define SCCB_SIOC_Pin       GPIO_PIN_13

#define SCCB_SIOD_GPIO_Port GPIOB
#define SCCB_SIOD_Pin       GPIO_PIN_14

#define OV7670_ADDR_WR      0x42
#define OV7670_ADDR_RD      0x43

#define OV7670_REG_PID      0x0A
#define OV7670_REG_VER      0x0B

#define CAM_W               160
#define CAM_H               120
#define CAM_BYTES_PER_PIXEL 2

#define FRAME_BYTES         (CAM_W * CAM_H * CAM_BYTES_PER_PIXEL)
#define FRAME_WORDS         (FRAME_BYTES / 4)

/* OV7670 common registers */
#define REG_GAIN            0x00
#define REG_BLUE            0x01
#define REG_RED             0x02
#define REG_VREF            0x03
#define REG_COM1            0x04
#define REG_BAVE            0x05
#define REG_GbAVE           0x06
#define REG_AECHH           0x07
#define REG_RAVE            0x08
#define REG_COM2            0x09
#define REG_PID             0x0A
#define REG_VER             0x0B
#define REG_COM3            0x0C
#define REG_COM4            0x0D
#define REG_COM5            0x0E
#define REG_COM6            0x0F
#define REG_AECH            0x10
#define REG_CLKRC           0x11
#define REG_COM7            0x12
#define REG_COM8            0x13
#define REG_COM9            0x14
#define REG_COM10           0x15
#define REG_HSTART          0x17
#define REG_HSTOP           0x18
#define REG_VSTART          0x19
#define REG_VSTOP           0x1A
#define REG_PSHFT           0x1B
#define REG_MIDH            0x1C
#define REG_MIDL            0x1D
#define REG_MVFP            0x1E
#define REG_AEW             0x24
#define REG_AEB             0x25
#define REG_VPT             0x26
#define REG_HREF            0x32
#define REG_TSLB            0x3A
#define REG_COM11           0x3B
#define REG_COM12           0x3C
#define REG_COM13           0x3D
#define REG_COM14           0x3E
#define REG_EDGE            0x3F
#define REG_COM15           0x40
#define REG_COM16           0x41
#define REG_COM17           0x42
#define REG_RGB444          0x8C
#define REG_SCALING_XSC     0x70
#define REG_SCALING_YSC     0x71
#define REG_SCALING_DCWCTR  0x72
#define REG_SCALING_PCLK_DIV 0x73
#define REG_SCALING_PCLK_DELAY 0xA2
#define REG_MTX1 0x4F
#define REG_MTX2 0x50
#define REG_MTX3 0x51
#define REG_MTX4 0x52
#define REG_MTX5 0x53
#define REG_MTX6 0x54
#define REG_MTXS 0x58

#define OV7670_REG_END      0xFF
#define OV7670_REG_DELAY    0xFE

#define FRAME_FORMAT_RGB565 1
#define FRAME_FORMAT_YUV422 2
#define FRAME_FORMAT_JPEG   3

#define FRAME_MAGIC0 0xA5
#define FRAME_MAGIC1 0x5A
#define FRAME_MAGIC2 0x12
#define FRAME_MAGIC3 0x34
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
__attribute__((aligned(4))) static uint32_t frame_buffer[FRAME_WORDS];

static volatile uint8_t frame_done = 0;
static volatile uint8_t frame_error = 0;
static volatile uint32_t dcmi_error_code = 0;
static volatile uint32_t dma_error_code = 0;
static volatile uint32_t dcmi_sr = 0;
static volatile uint32_t dcmi_risr = 0;
static volatile uint32_t dcmi_misr = 0;
static volatile uint32_t dma_ndtr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DCMI_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_Delay_Init(void);
static void delay_us(uint32_t us);

static void OV7670_XCLK_Init(void);
static void SCCB_GPIO_Init(void);

static void SCCB_Start(void);
static void SCCB_Stop(void);
static uint8_t SCCB_WriteByte(uint8_t data);
static uint8_t SCCB_ReadByte_NACK(void);
static uint8_t OV7670_ReadReg(uint8_t reg, uint8_t *data);

static uint8_t OV7670_WriteReg(uint8_t reg, uint8_t data);
static uint8_t OV7670_WriteArray(const uint8_t (*regs)[2]);
static uint8_t OV7670_Init_RGB565_QQVGA_Normal(void);
static HAL_StatusTypeDef Camera_StartSnapshot(void);

static void Camera_SendJpegFrame_UART(void);
static uint8_t Camera_UART_Write(const uint8_t *data, uint32_t len, void *user);

static uint8_t Camera_CaptureOneFrame(uint32_t timeout_ms);
static void Camera_PrintDcmiError(void);

static uint8_t Camera_IsDmaTransferDone(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint8_t ov7670_rgb565_qqvga_colorbar[][2] = {
	/* Reset sensor */
	{REG_COM7, 0x80},
	{OV7670_REG_DELAY, 10},

	/* Lower internal clock */
	{REG_CLKRC, 0x01},

	/* RGB565 */
	{REG_COM7,   0x04},
	{REG_RGB444, 0x00},
	{REG_COM15,  0xD0},
	{REG_TSLB,   0x04},
	{REG_COM13,  0x88},   // 0x88 C0

	/* Auto exposure / gain / white balance */
	{REG_COM8,   0x8F},   // 8F
	{REG_COM9,   0x38},
	{REG_COM16,  0x38},

	{REG_BLUE, 0x80},
	{REG_RED,  0x80},

	/* Neutral color gain start */
//	{REG_BLUE,   0x80},
//	{REG_RED,    0x80},
	{REG_GAIN,   0x00},

	/* Window setting */
	{REG_HSTART, 0x16},
	{REG_HSTOP,  0x04},
	{REG_HREF,   0x24},
	{REG_VSTART, 0x02},
	{REG_VSTOP,  0x7A},
	{REG_VREF,   0x0A},

	/* QQVGA scaler */
	{REG_COM3,   0x04},
	{REG_COM14,  0x1A},

	/*
	 * Normal image:
	 * 0x3A / 0x35 = normal
	 * 0xBA / 0xB5 = color bar
	 */
	{REG_SCALING_XSC,        0x3A},
	{REG_SCALING_YSC,        0x35},
	{REG_SCALING_DCWCTR,     0x22},
	{REG_SCALING_PCLK_DIV,   0xF2},
	{REG_SCALING_PCLK_DELAY, 0x02},

	{REG_COM10, 0x00},

	{OV7670_REG_END, 0x00}
};

static const uint8_t ov7670_rgb565_qqvga_normal[][2] = {
		/* Reset sensor */
		{REG_COM7, 0x80},
		{OV7670_REG_DELAY, 10},

		/* Lower internal clock */
		{REG_CLKRC, 0x01},

		/* RGB565 */
		{REG_COM7,   0x04},
		{REG_RGB444, 0x00},
		{REG_COM15,  0xD0},
		{REG_TSLB,   0x04},
		{REG_COM13,  0x88},   // 0x88 C0

		/* Auto exposure / gain / white balance */
		{REG_COM8,   0x8F},   // 8F
		{REG_COM9,   0x38},
		{REG_COM16,  0x38},

		{REG_BLUE, 0x80},
		{REG_RED,  0x80},

		/* Neutral color gain start */
	//	{REG_BLUE,   0x80},
	//	{REG_RED,    0x80},
		{REG_GAIN,   0x00},

		/* Window setting */
		{REG_HSTART, 0x16},
		{REG_HSTOP,  0x04},
		{REG_HREF,   0x24},
		{REG_VSTART, 0x02},
		{REG_VSTOP,  0x7A},
		{REG_VREF,   0x0A},

		/* QQVGA scaler */
		{REG_COM3,   0x04},
		{REG_COM14,  0x1A},

		/*
		 * Normal image:
		 * 0x3A / 0x35 = normal
		 * 0xBA / 0xB5 = color bar
		 */
		{REG_SCALING_XSC,        0x3A},
		{REG_SCALING_YSC,        0x35},
		{REG_SCALING_DCWCTR,     0x22},
		{REG_SCALING_PCLK_DIV,   0xF2},
		{REG_SCALING_PCLK_DELAY, 0x02},

		{REG_COM10, 0x00},

		{OV7670_REG_END, 0x00}
};

static const uint8_t ov7670_rgb565_qqvga_normal_tuned[][2] = {
		/* Reset sensor */
		{REG_COM7, 0x80},
		{OV7670_REG_DELAY, 10},

		/* Lower internal clock */
		{REG_CLKRC, 0x01},

		/* RGB565 */
		{REG_COM7,   0x04},
		{REG_RGB444, 0x00},
		{REG_COM15,  0xD0},
		{REG_TSLB,   0x04},
		{REG_COM9,   0x18},
		{REG_COM13,  0xC0},   // 0x88 0xC0

		/* MTXs */
//		{0x4F, 0xB3}, //MTX1
//		{0x50, 0xB3}, //MTX2
//		{0x51, 0x00}, //MTX3
//		{0x52, 0x3D}, //MTX4
//		{0x53, 0xA7}, //MTX5
//		{0x54, 0xE4}, //MTX6
//		{0x58, 0x9E}, //MTXS
		{0x4F, 0x90},
		{0x50, 0x90},
		{0x51, 0x00},
		{0x52, 0x30},
		{0x53, 0x80},
		{0x54, 0xC0},
		{0x58, 0x9E},

		/* Window setting */
		{REG_HSTART, 0x16},
		{REG_HSTOP,  0x04},
		{REG_HREF,   0x24},
		{REG_VSTART, 0x02},
		{REG_VSTOP,  0x7A},
		{REG_VREF,   0x0A},

		{REG_COM6, 0x41},
		{0x33, 0x0B},
		{0x3C, 0x78},
		{0x69, 0x00},
		{0x74, 0x00},
		{0xB0, 0x84},
		{0xB1, 0x0C},
		{0xB2, 0x0E},
		{0xB3, 0x80},

		/* QQVGA scaler */
		{REG_COM3,   0x04},
		{REG_COM14,  0x1A},

		/* Gamma curve values */
		{0x7A, 0x20},
		{0x7B, 0x10},
		{0x7C, 0x1E},
		{0x7D, 0x35},
		{0x7E, 0x5A},
		{0x7F, 0x69},
		{0x80, 0x76},
		{0x81, 0x80},
		{0x82, 0x88},
		{0x83, 0x8F},
		{0x84, 0x96},
		{0x85, 0xA3},
		{0x86, 0xAF},
		{0x87, 0xC4},
		{0x88, 0xD7},
		{0x89, 0xE8},

		/* AGC and AEC */
		{0x13, 0xE0}, //COM8, disable AGC / AEC
		{0x00, 0x00}, //set gain reg to 0 for AGC
		{0x10, 0x00}, //set ARCJ reg to 0
		{0x0D, 0x40}, //magic reserved bit for COM4
//		{0x14, 0x18}, //COM9, 4x gain + magic bit
		{0xA5, 0x05}, //BD50MAX
		{0xAB, 0x07}, //DB60MAX
		{0x24, 0x95}, //AGC upper limit
		{0x25, 0x33}, //AGC lower limit
		{0x26, 0xE3}, //AGC/AEC fast mode op region
		{0x9F, 0x78}, //HAECC1
		{0xA0, 0x68}, //HAECC2
		{0xA1, 0x03}, //magic
		{0xA6, 0xD8}, //HAECC3
		{0xA7, 0xD8}, //HAECC4
		{0xA8, 0xF0}, //HAECC5
		{0xA9, 0x90}, //HAECC6
		{0xAA, 0x94}, //HAECC7
		{0x13, 0xE5}, //COM8, enable AGC / AEC

		{REG_BLUE, 0xC0},
		{REG_RED,  0xA8},

		/*
		 * Normal image:
		 * 0x3A / 0x35 = normal
		 * 0xBA / 0xB5 = color bar
		 */
		{REG_SCALING_XSC,        0x3A},
		{REG_SCALING_YSC,        0x35},
		{REG_SCALING_DCWCTR,     0x22},
		{REG_SCALING_PCLK_DIV,   0xF2},
		{REG_SCALING_PCLK_DELAY, 0x02},

		{REG_COM10, 0x00},

		{OV7670_REG_END, 0x00}
};

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

static void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);

    while ((DWT->CYCCNT - start) < ticks) {
        ;
    }
}

static void OV7670_XCLK_Init(void)
{
    __HAL_RCC_HSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET) {
        ;
    }

    // PA8 = MCO1, output HSI 16 MHz
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

static void SCCB_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = SCCB_SIOC_Pin | SCCB_SIOD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SCCB_SIOC_GPIO_Port, SCCB_SIOC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCCB_SIOD_GPIO_Port, SCCB_SIOD_Pin, GPIO_PIN_SET);
}

static inline void SIOC_HIGH(void)
{
    HAL_GPIO_WritePin(SCCB_SIOC_GPIO_Port, SCCB_SIOC_Pin, GPIO_PIN_SET);
}

static inline void SIOC_LOW(void)
{
    HAL_GPIO_WritePin(SCCB_SIOC_GPIO_Port, SCCB_SIOC_Pin, GPIO_PIN_RESET);
}

static inline void SIOD_HIGH(void)
{
    // open-drain high = release line
    HAL_GPIO_WritePin(SCCB_SIOD_GPIO_Port, SCCB_SIOD_Pin, GPIO_PIN_SET);
}

static inline void SIOD_LOW(void)
{
    HAL_GPIO_WritePin(SCCB_SIOD_GPIO_Port, SCCB_SIOD_Pin, GPIO_PIN_RESET);
}

static inline GPIO_PinState SIOD_READ(void)
{
    return HAL_GPIO_ReadPin(SCCB_SIOD_GPIO_Port, SCCB_SIOD_Pin);
}

static void SCCB_Start(void)
{
    SIOD_HIGH();
    SIOC_HIGH();
    delay_us(5);

    SIOD_LOW();
    delay_us(5);

    SIOC_LOW();
    delay_us(5);
}

static void SCCB_Stop(void)
{
    SIOD_LOW();
    delay_us(5);

    SIOC_HIGH();
    delay_us(5);

    SIOD_HIGH();
    delay_us(5);
}

static uint8_t SCCB_WriteByte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) {
            SIOD_HIGH();
        } else {
            SIOD_LOW();
        }

        delay_us(5);
        SIOC_HIGH();
        delay_us(5);
        SIOC_LOW();
        delay_us(5);

        data <<= 1;
    }

    // Release SIOD for ACK
    SIOD_HIGH();
    delay_us(5);

    SIOC_HIGH();
    delay_us(5);

    // ACK = SIOD pulled low by slave
    uint8_t ack = (SIOD_READ() == GPIO_PIN_RESET);

    SIOC_LOW();
    delay_us(5);

    return ack;
}

static uint8_t SCCB_ReadByte_NACK(void)
{
    uint8_t data = 0;

    // Release SIOD so camera can drive it
    SIOD_HIGH();

    for (int i = 0; i < 8; i++) {
        data <<= 1;

        SIOC_HIGH();
        delay_us(5);

        if (SIOD_READ() == GPIO_PIN_SET) {
            data |= 0x01;
        }

        SIOC_LOW();
        delay_us(5);
    }

    // Send NACK: keep SIOD high during 9th clock
    SIOD_HIGH();
    delay_us(5);

    SIOC_HIGH();
    delay_us(5);

    SIOC_LOW();
    delay_us(5);

    return data;
}

static uint8_t OV7670_ReadReg(uint8_t reg, uint8_t *data)
{
    // Write register address
    SCCB_Start();

    if (!SCCB_WriteByte(OV7670_ADDR_WR)) {
        SCCB_Stop();
        return 0;
    }

    if (!SCCB_WriteByte(reg)) {
        SCCB_Stop();
        return 0;
    }

    SCCB_Stop();
    delay_us(10);

    // Read register data
    SCCB_Start();

    if (!SCCB_WriteByte(OV7670_ADDR_RD)) {
        SCCB_Stop();
        return 0;
    }

    *data = SCCB_ReadByte_NACK();

    SCCB_Stop();

    return 1;
}

static uint8_t OV7670_WriteReg(uint8_t reg, uint8_t data)
{
    SCCB_Start();

    if (!SCCB_WriteByte(OV7670_ADDR_WR)) {
        SCCB_Stop();
        return 0;
    }

    if (!SCCB_WriteByte(reg)) {
        SCCB_Stop();
        return 0;
    }

    if (!SCCB_WriteByte(data)) {
        SCCB_Stop();
        return 0;
    }

    SCCB_Stop();
    delay_us(100);

    return 1;
}

static uint8_t OV7670_WriteArray(const uint8_t (*regs)[2])
{
    uint16_t i = 0;

    while (1) {
        uint8_t reg = regs[i][0];
        uint8_t val = regs[i][1];

        if (reg == OV7670_REG_END) {
            break;
        }

        if (reg == OV7670_REG_DELAY) {
            HAL_Delay(val);
        } else {
            if (!OV7670_WriteReg(reg, val)) {
                printf("Write reg failed: reg=0x%02X val=0x%02X\r\n", reg, val);
                return 0;
            }
        }

        i++;
    }

    return 1;
}

static uint8_t OV7670_Init_RGB565_QQVGA_Normal(void)
{
    uint8_t pid = 0;
    uint8_t ver = 0;

    printf("Checking OV7670 ID...\r\n");

    if (!OV7670_ReadReg(OV7670_REG_PID, &pid)) {
        printf("PID read failed\r\n");
        return 0;
    }

    if (!OV7670_ReadReg(OV7670_REG_VER, &ver)) {
        printf("VER read failed\r\n");
        return 0;
    }

    printf("PID=0x%02X VER=0x%02X\r\n", pid, ver);

    if (pid != 0x76 || ver != 0x73) {
        printf("OV7670 ID mismatch\r\n");
        return 0;
    }

    printf("Writing OV7670 RGB565 QQVGA init table...\r\n");

    if (!OV7670_WriteArray(ov7670_rgb565_qqvga_normal_tuned)) {
        printf("OV7670 init failed\r\n");
        return 0;
    }

    HAL_Delay(100);

    printf("OV7670 init OK\r\n");
    return 1;
}

static HAL_StatusTypeDef Camera_StartSnapshot(void)
{
    HAL_StatusTypeDef ret;

    frame_done = 0;
    frame_error = 0;
    dcmi_error_code = 0;
    dma_error_code = 0;
    dcmi_sr = 0;
    dcmi_risr = 0;
    dcmi_misr = 0;
    dma_ndtr = 0;

    /*
     * 重要：
     * 每次重新 capture 前，先停止 DCMI/DMA。
     * 尤其是連續 warm-up frame 時，這步很重要。
     */
    HAL_DCMI_Stop(&hdcmi);
    HAL_Delay(2);

    /*
     * 清掉 DCMI interrupt flags。
     */
    __HAL_DCMI_CLEAR_FLAG(&hdcmi,
                          DCMI_FLAG_FRAMERI |
                          DCMI_FLAG_OVRRI   |
                          DCMI_FLAG_ERRRI   |
                          DCMI_FLAG_VSYNCRI |
                          DCMI_FLAG_LINERI);

    hdcmi.ErrorCode = HAL_DCMI_ERROR_NONE;

    for (uint32_t i = 0; i < FRAME_WORDS; i++) {
        frame_buffer[i] = 0x00000000;
    }

    printf("Start DCMI snapshot...\r\n");
    printf("Frame size = %lu bytes, DMA words = %lu\r\n",
           (uint32_t)FRAME_BYTES,
           (uint32_t)FRAME_WORDS);

    ret = HAL_DCMI_Start_DMA(&hdcmi,
                             DCMI_MODE_SNAPSHOT,
                             (uint32_t)frame_buffer,
                             FRAME_WORDS);

    if (ret != HAL_OK) {
        printf("HAL_DCMI_Start_DMA failed, ret=%d\r\n", ret);
        frame_error = 1;
    }

    return ret;
}

static uint8_t Camera_CaptureOneFrame(uint32_t timeout_ms)
{
    if (Camera_StartSnapshot() != HAL_OK) {
        return 0;
    }

    uint32_t start_tick = HAL_GetTick();

    while (!frame_done && !frame_error) {
        /*
         * 有些情況 DCMI FrameEvent callback 沒進來，
         * 但 DMA 其實已經把 buffer 搬完。
         * 對固定大小 frame 來說，NDTR == 0 可以視為完成。
         */
        if (Camera_IsDmaTransferDone()) {
            frame_done = 1;
            break;
        }

        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            /*
             * timeout 前再檢查一次 NDTR。
             * 你的 log 就是這種情況：NDTR=0，但 frame_done 沒被設起來。
             */
            if (Camera_IsDmaTransferDone()) {
                frame_done = 1;
                break;
            }

            printf("DCMI snapshot timeout\r\n");
            frame_error = 1;
            break;
        }
    }

    HAL_DCMI_Stop(&hdcmi);

    if (frame_error) {
        Camera_PrintDcmiError();
        return 0;
    }

    if (!frame_done) {
        printf("Frame not captured\r\n");
        return 0;
    }

    printf("Frame captured OK\r\n");
    return 1;
}

static uint8_t Camera_UART_Write(const uint8_t *data, uint32_t len, void *user)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)user;

    return (HAL_UART_Transmit(huart,
                              (uint8_t *)data,
                              len,
                              HAL_MAX_DELAY) == HAL_OK);
}

static void Camera_SendJpegFrame_UART(void)
{
    const uint8_t *rgb565_payload = (const uint8_t *)frame_buffer;
    JpegEncodeResult_t measured = {0};
    JpegEncodeResult_t sent = {0};

    printf("Preparing dynamic JPEG Huffman tables...\r\n");

    if (!JpegEncoder_PrepareRgb565(rgb565_payload, CAM_W, CAM_H)) {
        printf("JPEG prepare failed: %s\r\n", JpegEncoder_GetLastError());
        return;
    }

    if (!JpegEncoder_EmitRgb565(rgb565_payload,
                                CAM_W,
                                CAM_H,
                                NULL,
                                NULL,
                                &measured)) {
        printf("JPEG measure failed: %s\r\n", JpegEncoder_GetLastError());
        return;
    }

    printf("JPEG payload = %lu bytes, checksum = 0x%08lX\r\n",
           measured.bytes_written,
           measured.checksum);

    CameraFrameHeader_t header = {
        .magic = {FRAME_MAGIC0, FRAME_MAGIC1, FRAME_MAGIC2, FRAME_MAGIC3},
        .width = CAM_W,
        .height = CAM_H,
        .format = FRAME_FORMAT_JPEG,
        .bytes_per_pixel = 0,
        .payload_len = measured.bytes_written,
        .checksum = measured.checksum
    };

    printf("\r\nBEGIN_BINARY_FRAME\r\n");
    HAL_Delay(50);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)&header,
                      sizeof(header),
                      HAL_MAX_DELAY);

    if (!JpegEncoder_EmitRgb565(rgb565_payload,
                                CAM_W,
                                CAM_H,
                                Camera_UART_Write,
                                &huart2,
                                &sent)) {
        printf("JPEG send failed: %s\r\n", JpegEncoder_GetLastError());
        return;
    }

    if ((sent.bytes_written != measured.bytes_written) ||
        (sent.checksum != measured.checksum)) {
        printf("JPEG send mismatch: sent=%lu checksum=0x%08lX\r\n",
               sent.bytes_written,
               sent.checksum);
    }

    HAL_Delay(20);
    printf("\r\nEND_BINARY_FRAME\r\n");
}

static uint8_t Camera_IsDmaTransferDone(void)
{
    if (hdcmi.DMA_Handle == NULL) {
        return 0;
    }

    if (hdcmi.DMA_Handle->Instance == NULL) {
        return 0;
    }

    return (hdcmi.DMA_Handle->Instance->NDTR == 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DCMI_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  printf("\r\n===== OV7670 DCMI Snapshot Test =====\r\n");

  OV7670_XCLK_Init();
  printf("XCLK started on PA8\r\n");

  SCCB_GPIO_Init();
  printf("SCCB ready: PB13=SIOC, PB14=SIOD\r\n");

  HAL_Delay(50);

  if (!OV7670_Init_RGB565_QQVGA_Normal()) {
      printf("Camera init failed. Stop here.\r\n");
      while (1) {
          HAL_Delay(1000);
      }
  }

  printf("Waiting for AWB/AEC to settle...\r\n");
  HAL_Delay(1500);

  for (int i = 0; i < 5; i++) {
      printf("Capture warm-up frame %d...\r\n", i + 1);

      if (!Camera_CaptureOneFrame(5000)) {
          printf("Capture failed at frame %d\r\n", i + 1);

          while (1) {
              HAL_Delay(1000);
          }
      }

      HAL_Delay(100);
  }

  printf("Sending final frame as JPEG...\r\n");
  Camera_SendJpegFrame_UART();
  printf("JPEG frame sent over UART\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    frame_done = 1;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
    dcmi_error_code = HAL_DCMI_GetError(hdcmi);

    if (hdcmi->DMA_Handle != NULL) {
        dma_error_code = HAL_DMA_GetError(hdcmi->DMA_Handle);
        dma_ndtr = hdcmi->DMA_Handle->Instance->NDTR;
    }

    dcmi_sr = hdcmi->Instance->SR;
    dcmi_risr = hdcmi->Instance->RISR;
    dcmi_misr = hdcmi->Instance->MISR;

    frame_error = 1;
}

static void Camera_PrintDcmiError(void)
{
    printf("DCMI error: 0x%08lX\r\n", dcmi_error_code);

    if (dcmi_error_code & HAL_DCMI_ERROR_OVR) {
        printf("  - HAL_DCMI_ERROR_OVR: DCMI FIFO overrun\r\n");
    }

    if (dcmi_error_code & HAL_DCMI_ERROR_SYNC) {
        printf("  - HAL_DCMI_ERROR_SYNC: synchronization error\r\n");
    }

    if (dcmi_error_code & HAL_DCMI_ERROR_TIMEOUT) {
        printf("  - HAL_DCMI_ERROR_TIMEOUT\r\n");
    }

    if (dcmi_error_code & HAL_DCMI_ERROR_DMA) {
        printf("  - HAL_DCMI_ERROR_DMA\r\n");
    }

    printf("DMA error : 0x%08lX\r\n", dma_error_code);
    printf("DMA NDTR  : %lu\r\n", dma_ndtr);
    printf("DCMI SR   : 0x%08lX\r\n", dcmi_sr);
    printf("DCMI RISR : 0x%08lX\r\n", dcmi_risr);
    printf("DCMI MISR : 0x%08lX\r\n", dcmi_misr);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

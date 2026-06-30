#include "jpeg_encoder.h"

#include <stddef.h>
#include <string.h>

#define JPEG_COMPONENTS        3U
#define JPEG_BLOCK_SIZE        64U
#define JPEG_HUFF_SYMBOLS      256U
#define JPEG_PSEUDO_SYMBOL     256U
#define JPEG_LEAF_SYMBOLS      257U
#define JPEG_TREE_NODES        513U
#define JPEG_MAX_CODE_LEN      16U
#define JPEG_DCT_SCALE_BITS    14U
#define JPEG_DCT_TOTAL_BITS    (JPEG_DCT_SCALE_BITS * 2U)

typedef struct
{
    uint8_t bits[JPEG_MAX_CODE_LEN + 1U];
    uint8_t values[JPEG_HUFF_SYMBOLS];
    uint16_t codes[JPEG_HUFF_SYMBOLS];
    uint8_t sizes[JPEG_HUFF_SYMBOLS];
    uint16_t value_count;
} JpegHuffmanTable_t;

typedef struct
{
    uint16_t symbol;
    uint32_t freq;
} JpegSymbolFreq_t;

typedef struct
{
    uint32_t freq;
    int16_t parent;
} JpegTreeNode_t;

typedef struct
{
    uint32_t dc_freq[2][12];
    uint32_t ac_freq[2][JPEG_HUFF_SYMBOLS];
    JpegHuffmanTable_t dc_table[2];
    JpegHuffmanTable_t ac_table[2];

    JpegTreeNode_t nodes[JPEG_TREE_NODES];
    JpegSymbolFreq_t symbols[JPEG_LEAF_SYMBOLS];
    uint16_t symbol_count;

    int16_t sample_block[JPEG_BLOCK_SIZE];
    int16_t quant_block[JPEG_BLOCK_SIZE];
    int32_t dct_temp[JPEG_BLOCK_SIZE];

    uint16_t prepared_width;
    uint16_t prepared_height;
    uint8_t prepared;
} JpegEncoderWorkspace_t;

typedef struct
{
    JpegEncoderWriteFn write;
    void *user;
    uint32_t bytes_written;
    uint32_t checksum;
    uint8_t error;
    uint8_t bit_buffer;
    uint8_t bit_count;
} JpegEmit_t;

static JpegEncoderWorkspace_t jpeg_ws;
static const char *jpeg_last_error = "OK";

static const uint8_t jpeg_zigzag[JPEG_BLOCK_SIZE] = {
    0,  1,  8, 16,  9,  2,  3, 10,
   17, 24, 32, 25, 18, 11,  4,  5,
   12, 19, 26, 33, 40, 48, 41, 34,
   27, 20, 13,  6,  7, 14, 21, 28,
   35, 42, 49, 56, 57, 50, 43, 36,
   29, 22, 15, 23, 30, 37, 44, 51,
   58, 59, 52, 45, 38, 31, 39, 46,
   53, 60, 61, 54, 47, 55, 62, 63
};

static const uint8_t jpeg_quant_luma[JPEG_BLOCK_SIZE] = {
    16, 11, 10, 16, 24,  40,  51,  61,
    12, 12, 14, 19, 26,  58,  60,  55,
    14, 13, 16, 24, 40,  57,  69,  56,
    14, 17, 22, 29, 51,  87,  80,  62,
    18, 22, 37, 56, 68, 109, 103,  77,
    24, 35, 55, 64, 81, 104, 113,  92,
    49, 64, 78, 87,103, 121, 120, 101,
    72, 92, 95, 98,112, 100, 103,  99
};

static const uint8_t jpeg_quant_chroma[JPEG_BLOCK_SIZE] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99
};

static const int16_t jpeg_dct_matrix[8][8] = {
    { 5793,  5793,  5793,  5793,  5793,  5793,  5793,  5793},
    { 8035,  6811,  4551,  1598, -1598, -4551, -6811, -8035},
    { 7568,  3135, -3135, -7568, -7568, -3135,  3135,  7568},
    { 6811, -1598, -8035, -4551,  4551,  8035,  1598, -6811},
    { 5793, -5793, -5793,  5793,  5793, -5793, -5793,  5793},
    { 4551, -8035,  1598,  6811, -6811, -1598,  8035, -4551},
    { 3135, -7568,  7568, -3135, -3135,  7568, -7568,  3135},
    { 1598, -4551,  6811, -8035,  8035, -6811,  4551, -1598}
};

static void JPEG_SetError(const char *error)
{
    jpeg_last_error = error;
}

const char *JpegEncoder_GetLastError(void)
{
    return jpeg_last_error;
}

static uint8_t JPEG_Category(int16_t value)
{
    uint16_t magnitude;
    uint8_t category = 0;

    if (value < 0) {
        magnitude = (uint16_t)(-value);
    } else {
        magnitude = (uint16_t)value;
    }

    while (magnitude != 0U) {
        category++;
        magnitude >>= 1;
    }

    return category;
}

static uint16_t JPEG_AmplitudeBits(int16_t value, uint8_t size)
{
    if (size == 0U) {
        return 0;
    }

    if (value >= 0) {
        return (uint16_t)value;
    }

    return (uint16_t)(value + ((int16_t)1 << size) - 1);
}

static int16_t JPEG_RoundDiv64(int64_t value, int64_t divisor)
{
    int64_t result;

    if (value >= 0) {
        result = (value + (divisor / 2)) / divisor;
    } else {
        result = -((-value + (divisor / 2)) / divisor);
    }

    if (result > 32767) {
        result = 32767;
    } else if (result < -32768) {
        result = -32768;
    }

    return (int16_t)result;
}

static uint16_t JPEG_ReadRgb565(const uint8_t *frame, uint16_t width, uint16_t x, uint16_t y)
{
    uint32_t offset = (((uint32_t)y * width) + x) * 2U;
    return (uint16_t)(((uint16_t)frame[offset] << 8) | frame[offset + 1U]);
}

static uint8_t JPEG_Clamp8(int32_t value)
{
    if (value < 0) {
        return 0U;
    }

    if (value > 255) {
        return 255U;
    }

    return (uint8_t)value;
}

static void JPEG_LoadBlockRgb565(const uint8_t *frame,
                                 uint16_t width,
                                 uint16_t height,
                                 uint16_t block_x,
                                 uint16_t block_y,
                                 uint8_t component,
                                 int16_t *block)
{
    for (uint8_t y = 0; y < 8U; y++) {
        uint16_t py = (uint16_t)(block_y + y);
        if (py >= height) {
            py = (uint16_t)(height - 1U);
        }

        for (uint8_t x = 0; x < 8U; x++) {
            uint16_t px = (uint16_t)(block_x + x);
            if (px >= width) {
                px = (uint16_t)(width - 1U);
            }

            uint16_t pixel = JPEG_ReadRgb565(frame, width, px, py);
            uint8_t r = (uint8_t)(((pixel >> 11) & 0x1FU) << 3);
            uint8_t g = (uint8_t)(((pixel >> 5) & 0x3FU) << 2);
            uint8_t b = (uint8_t)((pixel & 0x1FU) << 3);
            int32_t y_value;
            int32_t cb_value;
            int32_t cr_value;
            uint8_t value;

            r |= (r >> 5);
            g |= (g >> 6);
            b |= (b >> 5);

            y_value  = ((77L * r) + (150L * g) + (29L * b)) >> 8;
            cb_value = 128L + (((-43L * r) - (85L * g) + (128L * b)) >> 8);
            cr_value = 128L + (((128L * r) - (107L * g) - (21L * b)) >> 8);

            if (component == 0U) {
                value = JPEG_Clamp8(y_value);
            } else if (component == 1U) {
                value = JPEG_Clamp8(cb_value);
            } else {
                value = JPEG_Clamp8(cr_value);
            }

            block[(y * 8U) + x] = (int16_t)value - 128;
        }
    }
}

static void JPEG_ForwardDctQuant(const int16_t *samples,
                                 const uint8_t *quant_table,
                                 int16_t *quant_block)
{
    for (uint8_t y = 0; y < 8U; y++) {
        for (uint8_t u = 0; u < 8U; u++) {
            int32_t sum = 0;

            for (uint8_t x = 0; x < 8U; x++) {
                sum += (int32_t)jpeg_dct_matrix[u][x] * samples[(y * 8U) + x];
            }

            jpeg_ws.dct_temp[(y * 8U) + u] = sum;
        }
    }

    for (uint8_t v = 0; v < 8U; v++) {
        for (uint8_t u = 0; u < 8U; u++) {
            int64_t sum = 0;
            uint8_t natural_index = (uint8_t)((v * 8U) + u);
            int16_t qvalue;

            for (uint8_t y = 0; y < 8U; y++) {
                sum += (int64_t)jpeg_ws.dct_temp[(y * 8U) + u] * jpeg_dct_matrix[v][y];
            }

            qvalue = JPEG_RoundDiv64(sum,
                                     ((int64_t)quant_table[natural_index]) << JPEG_DCT_TOTAL_BITS);

            for (uint8_t z = 0; z < JPEG_BLOCK_SIZE; z++) {
                if (jpeg_zigzag[z] == natural_index) {
                    quant_block[z] = qvalue;
                    break;
                }
            }
        }
    }
}

static void JPEG_MakeQuantBlock(const uint8_t *frame,
                                uint16_t width,
                                uint16_t height,
                                uint16_t block_x,
                                uint16_t block_y,
                                uint8_t component,
                                int16_t *quant_block)
{
    const uint8_t *quant_table = (component == 0U) ? jpeg_quant_luma : jpeg_quant_chroma;

    JPEG_LoadBlockRgb565(frame,
                         width,
                         height,
                         block_x,
                         block_y,
                         component,
                         jpeg_ws.sample_block);

    JPEG_ForwardDctQuant(jpeg_ws.sample_block, quant_table, quant_block);
}

static void JPEG_CollectBlockSymbols(const int16_t *block,
                                     int16_t *previous_dc,
                                     uint8_t table_index)
{
    int16_t dc_diff = (int16_t)(block[0] - *previous_dc);
    uint8_t dc_category = JPEG_Category(dc_diff);
    uint8_t zero_run = 0;

    *previous_dc = block[0];
    if (dc_category < 12U) {
        jpeg_ws.dc_freq[table_index][dc_category]++;
    }

    for (uint8_t i = 1; i < JPEG_BLOCK_SIZE; i++) {
        int16_t coeff = block[i];

        if (coeff == 0) {
            zero_run++;
            continue;
        }

        while (zero_run > 15U) {
            jpeg_ws.ac_freq[table_index][0xF0U]++;
            zero_run = (uint8_t)(zero_run - 16U);
        }

        uint8_t ac_category = JPEG_Category(coeff);
        uint8_t symbol = (uint8_t)((zero_run << 4) | ac_category);

        jpeg_ws.ac_freq[table_index][symbol]++;
        zero_run = 0;
    }

    if (zero_run > 0U) {
        jpeg_ws.ac_freq[table_index][0x00U]++;
    }
}

static uint8_t JPEG_CollectFrequencies(const uint8_t *frame, uint16_t width, uint16_t height)
{
    int16_t previous_dc[JPEG_COMPONENTS] = {0, 0, 0};

    memset(jpeg_ws.dc_freq, 0, sizeof(jpeg_ws.dc_freq));
    memset(jpeg_ws.ac_freq, 0, sizeof(jpeg_ws.ac_freq));

    for (uint16_t y = 0; y < height; y = (uint16_t)(y + 8U)) {
        for (uint16_t x = 0; x < width; x = (uint16_t)(x + 8U)) {
            for (uint8_t component = 0; component < JPEG_COMPONENTS; component++) {
                uint8_t table_index = (component == 0U) ? 0U : 1U;

                JPEG_MakeQuantBlock(frame,
                                    width,
                                    height,
                                    x,
                                    y,
                                    component,
                                    jpeg_ws.quant_block);

                JPEG_CollectBlockSymbols(jpeg_ws.quant_block,
                                         &previous_dc[component],
                                         table_index);
            }
        }
    }

    return 1U;
}

static void JPEG_AddSymbol(uint16_t symbol, uint32_t freq)
{
    jpeg_ws.symbols[jpeg_ws.symbol_count].symbol = symbol;
    jpeg_ws.symbols[jpeg_ws.symbol_count].freq = freq;
    jpeg_ws.symbol_count++;
}

static void JPEG_SortSymbolsByFrequency(void)
{
    for (uint16_t i = 1; i < jpeg_ws.symbol_count; i++) {
        JpegSymbolFreq_t key = jpeg_ws.symbols[i];
        int16_t j = (int16_t)i - 1;

        while (j >= 0) {
            uint8_t move = 0;

            if (jpeg_ws.symbols[j].freq < key.freq) {
                move = 1U;
            } else if ((jpeg_ws.symbols[j].freq == key.freq) &&
                       (jpeg_ws.symbols[j].symbol > key.symbol)) {
                move = 1U;
            }

            if (!move) {
                break;
            }

            jpeg_ws.symbols[j + 1] = jpeg_ws.symbols[j];
            j--;
        }

        jpeg_ws.symbols[j + 1] = key;
    }
}

static int16_t JPEG_FindSmallestNode(uint16_t node_count, int16_t exclude)
{
    int16_t best = -1;

    for (uint16_t i = 0; i < node_count; i++) {
        if ((jpeg_ws.nodes[i].parent >= 0) || ((int16_t)i == exclude)) {
            continue;
        }

        if (best < 0) {
            best = (int16_t)i;
            continue;
        }

        if (jpeg_ws.nodes[i].freq < jpeg_ws.nodes[best].freq) {
            best = (int16_t)i;
        } else if ((jpeg_ws.nodes[i].freq == jpeg_ws.nodes[best].freq) && (i < (uint16_t)best)) {
            best = (int16_t)i;
        }
    }

    return best;
}

static uint8_t JPEG_BuildLengthCounts(uint16_t *bit_count)
{
    uint16_t node_count = jpeg_ws.symbol_count;
    uint16_t max_len = 0;

    memset(bit_count, 0, JPEG_LEAF_SYMBOLS * sizeof(bit_count[0]));

    for (uint16_t i = 0; i < jpeg_ws.symbol_count; i++) {
        jpeg_ws.nodes[i].freq = jpeg_ws.symbols[i].freq;
        jpeg_ws.nodes[i].parent = -1;
    }

    while (1) {
        int16_t first = JPEG_FindSmallestNode(node_count, -1);
        int16_t second = JPEG_FindSmallestNode(node_count, first);

        if (second < 0) {
            break;
        }

        jpeg_ws.nodes[node_count].freq = jpeg_ws.nodes[first].freq + jpeg_ws.nodes[second].freq;
        jpeg_ws.nodes[node_count].parent = -1;
        jpeg_ws.nodes[first].parent = (int16_t)node_count;
        jpeg_ws.nodes[second].parent = (int16_t)node_count;
        node_count++;

        if (node_count >= JPEG_TREE_NODES) {
            JPEG_SetError("JPEG Huffman tree overflow");
            return 0U;
        }
    }

    for (uint16_t i = 0; i < jpeg_ws.symbol_count; i++) {
        uint16_t len = 0;
        int16_t node = (int16_t)i;

        while (jpeg_ws.nodes[node].parent >= 0) {
            len++;
            node = jpeg_ws.nodes[node].parent;
        }

        if (len == 0U) {
            len = 1U;
        }

        if (len >= JPEG_LEAF_SYMBOLS) {
            JPEG_SetError("JPEG Huffman code length overflow");
            return 0U;
        }

        bit_count[len]++;

        if (len > max_len) {
            max_len = len;
        }
    }

    for (int16_t len = max_len; len > (int16_t)JPEG_MAX_CODE_LEN; len--) {
        while (bit_count[len] > 0U) {
            int16_t shorter = (int16_t)(len - 2);

            while ((shorter > 0) && (bit_count[shorter] == 0U)) {
                shorter--;
            }

            if (shorter <= 0) {
                JPEG_SetError("JPEG Huffman length limiting failed");
                return 0U;
            }

            if (bit_count[len] < 2U) {
                JPEG_SetError("JPEG Huffman length count underflow");
                return 0U;
            }

            bit_count[len] = (uint16_t)(bit_count[len] - 2U);
            bit_count[len - 1]++;
            bit_count[shorter + 1] = (uint16_t)(bit_count[shorter + 1] + 2U);
            bit_count[shorter]--;
        }
    }

    return 1U;
}

static uint8_t JPEG_BuildHuffmanTable(const uint32_t *freq,
                                      uint16_t freq_count,
                                      JpegHuffmanTable_t *table)
{
    uint16_t bit_count[JPEG_LEAF_SYMBOLS];
    uint16_t assign_index = 0;

    memset(table, 0, sizeof(*table));
    jpeg_ws.symbol_count = 0;

    for (uint16_t symbol = 0; symbol < freq_count; symbol++) {
        if (freq[symbol] != 0U) {
            JPEG_AddSymbol(symbol, freq[symbol]);
        }
    }

    if (jpeg_ws.symbol_count == 0U) {
        JPEG_AddSymbol(0U, 1U);
    }

    JPEG_AddSymbol(JPEG_PSEUDO_SYMBOL, 1U);

    if (!JPEG_BuildLengthCounts(bit_count)) {
        return 0U;
    }

    JPEG_SortSymbolsByFrequency();

    for (uint8_t len = 1; len <= JPEG_MAX_CODE_LEN; len++) {
        for (uint16_t n = 0; n < bit_count[len]; n++) {
            uint16_t symbol = jpeg_ws.symbols[assign_index].symbol;
            assign_index++;

            if (symbol == JPEG_PSEUDO_SYMBOL) {
                continue;
            }

            if (table->value_count >= JPEG_HUFF_SYMBOLS) {
                JPEG_SetError("JPEG Huffman value overflow");
                return 0U;
            }

            table->bits[len]++;
            table->values[table->value_count] = (uint8_t)symbol;
            table->value_count++;
        }
    }

    uint16_t code = 0;
    uint16_t value_index = 0;

    for (uint8_t len = 1; len <= JPEG_MAX_CODE_LEN; len++) {
        for (uint8_t n = 0; n < table->bits[len]; n++) {
            uint8_t symbol = table->values[value_index];

            table->codes[symbol] = code;
            table->sizes[symbol] = len;
            code++;
            value_index++;
        }

        code <<= 1;
    }

    return 1U;
}

static uint8_t JPEG_BuildHuffmanTables(void)
{
    if (!JPEG_BuildHuffmanTable(jpeg_ws.dc_freq[0], 12U, &jpeg_ws.dc_table[0])) {
        return 0U;
    }

    if (!JPEG_BuildHuffmanTable(jpeg_ws.ac_freq[0], JPEG_HUFF_SYMBOLS, &jpeg_ws.ac_table[0])) {
        return 0U;
    }

    if (!JPEG_BuildHuffmanTable(jpeg_ws.dc_freq[1], 12U, &jpeg_ws.dc_table[1])) {
        return 0U;
    }

    if (!JPEG_BuildHuffmanTable(jpeg_ws.ac_freq[1], JPEG_HUFF_SYMBOLS, &jpeg_ws.ac_table[1])) {
        return 0U;
    }

    return 1U;
}

static uint8_t JPEG_EmitRawByte(JpegEmit_t *emit, uint8_t value)
{
    if (emit->write != NULL) {
        if (!emit->write(&value, 1U, emit->user)) {
            emit->error = 1U;
            return 0U;
        }
    }

    emit->bytes_written++;
    emit->checksum += value;
    return 1U;
}

static uint8_t JPEG_EmitBytes(JpegEmit_t *emit, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (!JPEG_EmitRawByte(emit, data[i])) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t JPEG_EmitWord(JpegEmit_t *emit, uint16_t value)
{
    return JPEG_EmitRawByte(emit, (uint8_t)(value >> 8)) &&
           JPEG_EmitRawByte(emit, (uint8_t)(value & 0xFFU));
}

static uint8_t JPEG_EmitMarker(JpegEmit_t *emit, uint8_t marker)
{
    return JPEG_EmitRawByte(emit, 0xFFU) && JPEG_EmitRawByte(emit, marker);
}

static uint8_t JPEG_EmitEntropyByte(JpegEmit_t *emit, uint8_t value)
{
    if (!JPEG_EmitRawByte(emit, value)) {
        return 0U;
    }

    if (value == 0xFFU) {
        return JPEG_EmitRawByte(emit, 0x00U);
    }

    return 1U;
}

static uint8_t JPEG_WriteBits(JpegEmit_t *emit, uint16_t bits, uint8_t size)
{
    for (int8_t i = (int8_t)size - 1; i >= 0; i--) {
        emit->bit_buffer = (uint8_t)((emit->bit_buffer << 1) | ((bits >> i) & 0x01U));
        emit->bit_count++;

        if (emit->bit_count == 8U) {
            if (!JPEG_EmitEntropyByte(emit, emit->bit_buffer)) {
                return 0U;
            }

            emit->bit_buffer = 0;
            emit->bit_count = 0;
        }
    }

    return 1U;
}

static uint8_t JPEG_FlushBits(JpegEmit_t *emit)
{
    if (emit->bit_count == 0U) {
        return 1U;
    }

    uint8_t padding = (uint8_t)(8U - emit->bit_count);
    uint8_t value = (uint8_t)((emit->bit_buffer << padding) | ((1U << padding) - 1U));

    emit->bit_buffer = 0;
    emit->bit_count = 0;

    return JPEG_EmitEntropyByte(emit, value);
}

static uint8_t JPEG_EmitQuantTable(JpegEmit_t *emit, uint8_t table_id, const uint8_t *table)
{
    if (!JPEG_EmitMarker(emit, 0xDBU) ||
        !JPEG_EmitWord(emit, 67U) ||
        !JPEG_EmitRawByte(emit, table_id)) {
        return 0U;
    }

    for (uint8_t i = 0; i < JPEG_BLOCK_SIZE; i++) {
        if (!JPEG_EmitRawByte(emit, table[jpeg_zigzag[i]])) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t JPEG_EmitHuffmanTable(JpegEmit_t *emit,
                                     uint8_t table_class,
                                     uint8_t table_id,
                                     const JpegHuffmanTable_t *table)
{
    uint16_t length = (uint16_t)(2U + 1U + 16U + table->value_count);

    if (!JPEG_EmitMarker(emit, 0xC4U) ||
        !JPEG_EmitWord(emit, length) ||
        !JPEG_EmitRawByte(emit, (uint8_t)((table_class << 4) | table_id))) {
        return 0U;
    }

    for (uint8_t i = 1; i <= JPEG_MAX_CODE_LEN; i++) {
        if (!JPEG_EmitRawByte(emit, table->bits[i])) {
            return 0U;
        }
    }

    return JPEG_EmitBytes(emit, table->values, table->value_count);
}

static uint8_t JPEG_EmitHeaders(JpegEmit_t *emit, uint16_t width, uint16_t height)
{
    static const uint8_t jfif_payload[] = {
        0x4A, 0x46, 0x49, 0x46, 0x00,
        0x01, 0x01,
        0x00,
        0x00, 0x01,
        0x00, 0x01,
        0x00, 0x00
    };

    if (!JPEG_EmitMarker(emit, 0xD8U)) {
        return 0U;
    }

    if (!JPEG_EmitMarker(emit, 0xE0U) ||
        !JPEG_EmitWord(emit, 16U) ||
        !JPEG_EmitBytes(emit, jfif_payload, sizeof(jfif_payload))) {
        return 0U;
    }

    if (!JPEG_EmitQuantTable(emit, 0U, jpeg_quant_luma) ||
        !JPEG_EmitQuantTable(emit, 1U, jpeg_quant_chroma)) {
        return 0U;
    }

    if (!JPEG_EmitMarker(emit, 0xC0U) ||
        !JPEG_EmitWord(emit, 17U) ||
        !JPEG_EmitRawByte(emit, 8U) ||
        !JPEG_EmitWord(emit, height) ||
        !JPEG_EmitWord(emit, width) ||
        !JPEG_EmitRawByte(emit, 3U)) {
        return 0U;
    }

    if (!JPEG_EmitRawByte(emit, 1U) ||
        !JPEG_EmitRawByte(emit, 0x11U) ||
        !JPEG_EmitRawByte(emit, 0U) ||
        !JPEG_EmitRawByte(emit, 2U) ||
        !JPEG_EmitRawByte(emit, 0x11U) ||
        !JPEG_EmitRawByte(emit, 1U) ||
        !JPEG_EmitRawByte(emit, 3U) ||
        !JPEG_EmitRawByte(emit, 0x11U) ||
        !JPEG_EmitRawByte(emit, 1U)) {
        return 0U;
    }

    if (!JPEG_EmitHuffmanTable(emit, 0U, 0U, &jpeg_ws.dc_table[0]) ||
        !JPEG_EmitHuffmanTable(emit, 1U, 0U, &jpeg_ws.ac_table[0]) ||
        !JPEG_EmitHuffmanTable(emit, 0U, 1U, &jpeg_ws.dc_table[1]) ||
        !JPEG_EmitHuffmanTable(emit, 1U, 1U, &jpeg_ws.ac_table[1])) {
        return 0U;
    }

    if (!JPEG_EmitMarker(emit, 0xDAU) ||
        !JPEG_EmitWord(emit, 12U) ||
        !JPEG_EmitRawByte(emit, 3U)) {
        return 0U;
    }

    if (!JPEG_EmitRawByte(emit, 1U) ||
        !JPEG_EmitRawByte(emit, 0x00U) ||
        !JPEG_EmitRawByte(emit, 2U) ||
        !JPEG_EmitRawByte(emit, 0x11U) ||
        !JPEG_EmitRawByte(emit, 3U) ||
        !JPEG_EmitRawByte(emit, 0x11U) ||
        !JPEG_EmitRawByte(emit, 0U) ||
        !JPEG_EmitRawByte(emit, 63U) ||
        !JPEG_EmitRawByte(emit, 0U)) {
        return 0U;
    }

    return 1U;
}

static uint8_t JPEG_EmitBlock(const int16_t *block,
                              int16_t *previous_dc,
                              const JpegHuffmanTable_t *dc_table,
                              const JpegHuffmanTable_t *ac_table,
                              JpegEmit_t *emit)
{
    int16_t dc_diff = (int16_t)(block[0] - *previous_dc);
    uint8_t dc_category = JPEG_Category(dc_diff);
    uint8_t zero_run = 0;

    *previous_dc = block[0];

    if (!JPEG_WriteBits(emit, dc_table->codes[dc_category], dc_table->sizes[dc_category]) ||
        !JPEG_WriteBits(emit, JPEG_AmplitudeBits(dc_diff, dc_category), dc_category)) {
        return 0U;
    }

    for (uint8_t i = 1; i < JPEG_BLOCK_SIZE; i++) {
        int16_t coeff = block[i];

        if (coeff == 0) {
            zero_run++;
            continue;
        }

        while (zero_run > 15U) {
            if (!JPEG_WriteBits(emit, ac_table->codes[0xF0U], ac_table->sizes[0xF0U])) {
                return 0U;
            }

            zero_run = (uint8_t)(zero_run - 16U);
        }

        uint8_t ac_category = JPEG_Category(coeff);
        uint8_t symbol = (uint8_t)((zero_run << 4) | ac_category);

        if (!JPEG_WriteBits(emit, ac_table->codes[symbol], ac_table->sizes[symbol]) ||
            !JPEG_WriteBits(emit, JPEG_AmplitudeBits(coeff, ac_category), ac_category)) {
            return 0U;
        }

        zero_run = 0;
    }

    if (zero_run > 0U) {
        if (!JPEG_WriteBits(emit, ac_table->codes[0x00U], ac_table->sizes[0x00U])) {
            return 0U;
        }
    }

    return 1U;
}

uint8_t JpegEncoder_PrepareRgb565(const uint8_t *frame, uint16_t width, uint16_t height)
{
    JPEG_SetError("OK");

    if ((frame == NULL) || (width == 0U) || (height == 0U)) {
        JPEG_SetError("JPEG invalid input");
        return 0U;
    }

    if (((width % 8U) != 0U) || ((height % 8U) != 0U)) {
        JPEG_SetError("JPEG width/height must be multiples of 8");
        return 0U;
    }

    jpeg_ws.prepared = 0U;

    if (!JPEG_CollectFrequencies(frame, width, height)) {
        return 0U;
    }

    if (!JPEG_BuildHuffmanTables()) {
        return 0U;
    }

    jpeg_ws.prepared_width = width;
    jpeg_ws.prepared_height = height;
    jpeg_ws.prepared = 1U;
    return 1U;
}

uint8_t JpegEncoder_EmitRgb565(const uint8_t *frame,
                               uint16_t width,
                               uint16_t height,
                               JpegEncoderWriteFn write,
                               void *user,
                               JpegEncodeResult_t *result)
{
    JpegEmit_t emit = {0};
    int16_t previous_dc[JPEG_COMPONENTS] = {0, 0, 0};

    JPEG_SetError("OK");

    if ((frame == NULL) || (result == NULL)) {
        JPEG_SetError("JPEG invalid emit input");
        return 0U;
    }

    if ((!jpeg_ws.prepared) ||
        (jpeg_ws.prepared_width != width) ||
        (jpeg_ws.prepared_height != height)) {
        JPEG_SetError("JPEG Huffman tables are not prepared");
        return 0U;
    }

    emit.write = write;
    emit.user = user;

    if (!JPEG_EmitHeaders(&emit, width, height)) {
        JPEG_SetError("JPEG header emit failed");
        return 0U;
    }

    for (uint16_t y = 0; y < height; y = (uint16_t)(y + 8U)) {
        for (uint16_t x = 0; x < width; x = (uint16_t)(x + 8U)) {
            for (uint8_t component = 0; component < JPEG_COMPONENTS; component++) {
                uint8_t table_index = (component == 0U) ? 0U : 1U;

                JPEG_MakeQuantBlock(frame,
                                    width,
                                    height,
                                    x,
                                    y,
                                    component,
                                    jpeg_ws.quant_block);

                if (!JPEG_EmitBlock(jpeg_ws.quant_block,
                                    &previous_dc[component],
                                    &jpeg_ws.dc_table[table_index],
                                    &jpeg_ws.ac_table[table_index],
                                    &emit)) {
                    JPEG_SetError("JPEG entropy emit failed");
                    return 0U;
                }
            }
        }
    }

    if (!JPEG_FlushBits(&emit) ||
        !JPEG_EmitMarker(&emit, 0xD9U)) {
        JPEG_SetError("JPEG final marker emit failed");
        return 0U;
    }

    result->bytes_written = emit.bytes_written;
    result->checksum = emit.checksum;
    return emit.error ? 0U : 1U;
}

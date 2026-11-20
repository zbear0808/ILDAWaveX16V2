/*
 * Host tests for firmware/ESP32S3/main/idn_decoder.c, from plan 01's
 * acceptance cases: a configuration the board cannot play exactly is
 * rejected whole, and an accepted one puts each value on the right line.
 *
 * Dictionaries are IDN-Stream rev002 tags (section 3.4), mostly the base
 * XYR plus the tags a case adds. Expected outputs are literal RP sample
 * values for X, Y, R, G, B, I, U1, U2.
 */

#include <stdio.h>
#include <string.h>

#include "idn_decoder.h"

static int s_failures;

#define CHECK(cond, ...) do {                                   \
        if (!(cond)) {                                          \
            s_failures++;                                       \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);         \
            printf(__VA_ARGS__);                                \
            printf("\n");                                       \
        }                                                       \
    } while (0)

#define N(a)        (sizeof(a) / sizeof((a)[0]))
#define BYTES(...)  (const uint8_t[]){ __VA_ARGS__ }, sizeof((const uint8_t[]){ __VA_ARGS__ })
#define R(x)        IDN_DECODER_REASON_##x

#define XYR   0x42,0x00, 0x42,0x10, 0x52,0x7E   /* 8-bit X, 8-bit Y, red 638 nm (3.4.7, 3.4.8) */
#define PAD   0x00,0x00                         /* Void: fills the dictionary to whole 32-bit words */
#define OFF   (-32768)                          /* DAC code 0: dark, also every line nothing drives */
#define FULL  32512                             /* colour octet 0xFF */

static const uint8_t k_break[] = { XYR, 0x10,0x00 };   /* 3.4.3 Break: a second scan head, unsupported */

static idn_decoder_caps_t s_caps;

/* One sample through a dictionary; the sample's length is the expected stride. */
static const struct {
    const char    *name;
    const uint8_t *dict;
    size_t         dict_len;
    const uint8_t *sample;
    size_t         stride;
    int16_t        out[IDN_DECODER_CHANNELS];
} k_decode[] = {
    /* 3.4.11 X/Y/R/G/B with 16-bit X and Y: X=0x1234 Y=0xFEDC R=0xFF G=0x80 B=0x00, either tag order */
    { "RGB order", BYTES(0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x52,0x7E, 0x52,0x14, 0x51,0xCC, PAD),
      BYTES(0x12,0x34, 0xFE,0xDC, 0xFF, 0x80, 0x00), { 4660, -292, FULL, 0, OFF, FULL, OFF, OFF } },
    { "BGR order", BYTES(0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x51,0xCC, 0x52,0x14, 0x52,0x7E, PAD),
      BYTES(0x12,0x34, 0xFE,0xDC, 0x00, 0x80, 0xFF), { 4660, -292, FULL, 0, OFF, FULL, OFF, OFF } },
    /* Colour tags are wavelengths, mapped to the nearest line; a tie goes to the shorter one */
    { "532 nm only is green", BYTES(0x42,0x00, 0x42,0x10, 0x52,0x14, PAD), BYTES(0, 0, 0xFF),
      { 0, 0, OFF, FULL, OFF, FULL, OFF, OFF } },
    { "585 nm ties to green", BYTES(0x42,0x00, 0x42,0x10, 0x52,0x49, PAD), BYTES(0, 0, 0xFF),
      { 0, 0, OFF, FULL, OFF, FULL, OFF, OFF } },
    { "600 nm is red", BYTES(0x42,0x00, 0x42,0x10, 0x52,0x58, PAD), BYTES(0, 0, 0xFF),
      { 0, 0, FULL, OFF, OFF, FULL, OFF, OFF } },
    /* 3.4.9 intensity: an explicit value is used as sent; without the tag it is the largest colour */
    { "explicit intensity 0", BYTES(XYR, 0x5C,0x10), BYTES(0, 0, 0xFF, 0x00), { 0, 0, FULL, OFF, OFF, OFF, OFF, OFF } },
    { "explicit intensity", BYTES(XYR, 0x5C,0x10), BYTES(0, 0, 0x00, 0x80), { 0, 0, OFF, OFF, OFF, 0, OFF, OFF } },
    { "derived intensity", BYTES(XYR, 0x52,0x14, 0x51,0xCC, PAD), BYTES(0, 0, 0x10, 0x40, 0x20),
      { 0, 0, -28672, -16384, -24576, -16384, OFF, OFF } },
    { "all zero is dark", BYTES(XYR, PAD), BYTES(0, 0, 0), { 0, 0, OFF, OFF, OFF, OFF, OFF, OFF } },
    /* 3.4.5 precision octets extend the value before them, most significant first */
    { "X 12 34 56 is 0x1234", BYTES(0x42,0x00, 0x40,0x10, 0x40,0x10, 0x42,0x10, 0x52,0x7E, PAD),
      BYTES(0x12, 0x34, 0x56, 0x7F, 0xFF), { 4660, 32512, FULL, OFF, OFF, FULL, OFF, OFF } },
    { "16-bit red", BYTES(XYR, 0x40,0x10), BYTES(0, 0, 0x12, 0x34), { 0, 0, -28108, OFF, OFF, -28108, OFF, OFF } },
    /* Z (3.4.7) and Beam-Brush (3.4.9) each take an octet and are ignored; a Void's suffix word takes none */
    { "Z, Beam-Brush ignored", BYTES(0x42,0x00, 0x42,0x10, 0x42,0x20, 0x5C,0x20, 0x52,0x7E, PAD),
      BYTES(0x01, 0x02, 0xAA, 0xBB, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
    { "Void suffix", BYTES(XYR, 0x00,0x01, 0x12,0x34, PAD), BYTES(0x01, 0x02, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
    /* 3.4.4 the two space modifier settings that ask for no correction */
    { "space: projector specific", BYTES(0x11,0x00, XYR), BYTES(0x01, 0x02, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
    { "space: no transformation", BYTES(XYR, 0x11,0xFF), BYTES(0x01, 0x02, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
};

static void test_decode(void)
{
    for (size_t c = 0; c < N(k_decode); c++) {
        idn_decoder_layout_t lay;
        int16_t out[1][IDN_DECODER_CHANNELS];
        idn_decoder_result_t r = idn_decoder_compile(k_decode[c].dict, k_decode[c].dict_len, &s_caps, &lay);
        CHECK(r.status == IDN_DECODER_OK && lay.stride == k_decode[c].stride, "%s: %s, stride %u",
              k_decode[c].name, idn_decoder_reason_name(r.reason), lay.stride);
        if (r.status != IDN_DECODER_OK)
            continue;
        idn_decoder_decode(&lay, k_decode[c].sample, 1, out);
        for (int ch = 0; ch < IDN_DECODER_CHANNELS; ch++)
            CHECK(out[0][ch] == k_decode[c].out[ch], "%s: output %d is %d, expected %d",
                  k_decode[c].name, ch, out[0][ch], k_decode[c].out[ch]);
    }
}

static void test_hint_scales(void)
{
    /* 3.4.6: hint bits 7-6 (CSCL) divide colour and bits 5-4 (ISCL) intensity by 1, 4, 16, 64. */
    static const int16_t scaled[4] = { FULL, -16448, -28688, -31748 };   /* 0xFF00, 0x3FC0, 0x0FF0, 0x03FC */
    static const uint8_t dict[] = { 0x41,0x00, XYR, 0x5C,0x10, PAD };
    idn_decoder_layout_t lay;
    CHECK(idn_decoder_compile(dict, sizeof(dict), &s_caps, &lay).status == IDN_DECODER_OK, "hint layout");
    for (int k = 0; k < 4; k++) {
        const uint8_t sample[] = { (uint8_t)(k << 6 | (3 - k) << 4), 0, 0, 0xFF, 0xFF };
        int16_t out[1][IDN_DECODER_CHANNELS];
        idn_decoder_decode(&lay, sample, 1, out);
        CHECK(out[0][2] == scaled[k] && out[0][5] == scaled[3 - k], "CSCL %d, ISCL %d: R %d, I %d",
              k, 3 - k, out[0][2], out[0][5]);
    }
}

static void test_rejections(void)
{
    const struct {
        const char          *name;
        const uint8_t       *dict;
        size_t               len;
        idn_decoder_reason_t reason;
    } cases[] = {
        { "empty",                     NULL, 0,                                     R(MISSING_X) },
        { "1.5 words",                 BYTES(XYR),                                  R(DICT_LENGTH) },
        { "unknown category",          BYTES(XYR, 0x20,0x00),                       R(UNKNOWN_CATEGORY) },
        { "unknown colour subcategory", BYTES(XYR, 0x54,0x00),                      R(UNKNOWN_CATEGORY) },
        { "Void suffix past the end",  BYTES(XYR, 0x00,0x03),                       R(TRUNCATED_SUFFIX) },
        { "Break",                     k_break, sizeof(k_break),                    R(BREAK) },
        { "sample shutter 0x4101",     BYTES(0x41,0x01, XYR),                       R(SAMPLE_SHUTTER) },
        { "no Y",                      BYTES(0x42,0x00, 0x52,0x7E),                 R(MISSING_Y) },
        { "no colour",                 BYTES(0x42,0x00, 0x42,0x10, 0x5C,0x10, PAD), R(MISSING_COLOR) },
        { "X twice",                   BYTES(0x42,0x00, XYR),                       R(DUPLICATE_X) },
        { "hint twice",                BYTES(0x41,0x00, 0x41,0x00, XYR, PAD),       R(DUPLICATE_HINT) },
        { "two colours on one line",   BYTES(XYR, 0x52,0x80),                       R(COLOR_CONFLICT) },
        { "zero wavelength",           BYTES(0x42,0x00, 0x42,0x10, 0x50,0x00, PAD), R(ZERO_WAVELENGTH) },
        { "dynamic wavelength",        BYTES(0x42,0x00, 0x42,0x10, 0x5C,0x00, 0x50,0x00), R(DYNAMIC_WAVELENGTH) },
        { "prefix then static colour", BYTES(0x42,0x00, 0x42,0x10, 0x5C,0x00, 0x52,0x7E), R(WAVELENGTH_PREFIX) },
        { "precision extends nothing", BYTES(0x40,0x10, XYR),                       R(PRECISION_PLACEMENT) },
        { "precision on a NOP",        BYTES(0x40,0x00, 0x40,0x10, XYR, PAD),       R(PRECISION_PLACEMENT) },
        { "precision on the hint",     BYTES(0x41,0x00, 0x40,0x10, XYR, PAD),       R(PRECISION_PLACEMENT) },
        { "precision after a Void",    BYTES(0x42,0x00, PAD, 0x40,0x10, 0x42,0x10, 0x52,0x7E, PAD), R(PRECISION_PLACEMENT) },
        { "five octets for one value", BYTES(0x42,0x00, 0x40,0x10, 0x40,0x10, 0x40,0x10, 0x40,0x10, 0x42,0x10, 0x52,0x7E, PAD),
                                                                                    R(PRECISION_LIMIT) },
        { "space: visual colour",      BYTES(0x11,0x20, XYR),                       R(SPACE_UNSUPPORTED) },
        { "space: reserved value",     BYTES(0x11,0x80, XYR),                       R(SPACE_MODIFIER) },
        { "space twice",               BYTES(0x11,0x00, 0x11,0x00, XYR, PAD),       R(DUPLICATE_SPACE) },
    };
    idn_decoder_layout_t lay;
    idn_decoder_result_t r;

    for (size_t c = 0; c < N(cases); c++) {
        r = idn_decoder_compile(cases[c].dict, cases[c].len, &s_caps, &lay);
        CHECK(r.status != IDN_DECODER_OK && r.reason == cases[c].reason && !lay.valid,
              "%s: got %s", cases[c].name, idn_decoder_reason_name(r.reason));
    }

    r = idn_decoder_compile(k_break, sizeof(k_break), &s_caps, &lay);
    CHECK(r.tag_index == 3 && r.tag == 0x1000, "offending tag %u, 0x%04X", r.tag_index, r.tag);

    /* At most 32 octets per sample: X, Y, red and 29 NOPs (0x4000) fit, 30 do not. */
    for (int nops = 29; nops <= 30; nops++) {
        uint8_t dict[80] = { XYR };
        size_t len = 6;
        for (int k = 0; k < nops; k++) {
            dict[len++] = 0x40;
            dict[len++] = 0x00;
        }
        len += len % 4;
        r = idn_decoder_compile(dict, len, &s_caps, &lay);
        CHECK(nops == 29 ? r.status == IDN_DECODER_OK : r.reason == R(STRIDE_LIMIT), "%d NOPs: %s",
              nops, idn_decoder_reason_name(r.reason));
    }
}

static void test_wave_chunks(void)
{
    /* 6.1: flags, a 24-bit duration in us, then whole samples, at least 20. Sample k has X = k. */
    static const uint8_t dict[] = { XYR, PAD };
    uint8_t chunk[4 + 3 * 21] = { 0x00, 0x00, 0x02, 0x9A };   /* 666 us */
    for (int k = 0; k < 21; k++) {
        chunk[4 + 3 * k] = (uint8_t)k;
        chunk[6 + 3 * k] = 0xFF;
    }
    idn_decoder_layout_t lay;
    int16_t out[20][IDN_DECODER_CHANNELS];
    uint32_t n = 0, duration = 0;
    CHECK(idn_decoder_compile(dict, sizeof(dict), &s_caps, &lay).status == IDN_DECODER_OK, "chunk layout");

    idn_decoder_result_t r = idn_decoder_check_wave_chunk(&lay, chunk, 4 + 3 * 20, &n, &duration);
    CHECK(r.status == IDN_DECODER_OK && n == 20 && duration == 666, "20 samples: %s", idn_decoder_reason_name(r.reason));
    idn_decoder_decode(&lay, chunk + 4, 20, out);
    CHECK(out[19][0] == 19 * 256 && out[19][2] == FULL, "sample 19: X %d, R %d", out[19][0], out[19][2]);

    static const struct { size_t len; idn_decoder_reason_t reason; } bad[] = {
        { 4 + 3 * 19,     R(TOO_FEW_SAMPLES) },
        { 4 + 3 * 20 + 1, R(PARTIAL_SAMPLE) },
        { 3,              R(CHUNK_HEADER) },
    };
    for (size_t c = 0; c < N(bad); c++) {
        r = idn_decoder_check_wave_chunk(&lay, chunk, bad[c].len, &n, NULL);
        CHECK(r.status == IDN_DECODER_ERR_INVALID_SAMPLES && r.reason == bad[c].reason, "%u octets: %s",
              (unsigned)bad[c].len, idn_decoder_reason_name(r.reason));
    }
    chunk[2] = chunk[3] = 0;
    r = idn_decoder_check_wave_chunk(&lay, chunk, 4 + 3 * 20, &n, NULL);
    CHECK(r.reason == R(ZERO_DURATION), "zero duration: %s", idn_decoder_reason_name(r.reason));

    /* A rejected replacement leaves no decoder: data in the old format is refused. */
    chunk[2] = 0x02;
    chunk[3] = 0x9A;
    idn_decoder_compile(k_break, sizeof(k_break), &s_caps, &lay);
    r = idn_decoder_check_wave_chunk(&lay, chunk, 4 + 3 * 20, &n, NULL);
    CHECK(r.reason == R(INVALID_LAYOUT), "old layout reused: %s", idn_decoder_reason_name(r.reason));
}

int main(void)
{
    idn_decoder_default_caps(&s_caps);
    test_decode();
    test_hint_scales();
    test_rejections();
    test_wave_chunks();
    printf("idn_decoder: %s\n", s_failures ? "FAILED" : "all checks passed");
    return s_failures != 0;
}

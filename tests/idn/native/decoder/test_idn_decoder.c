/*
 * Host-native tests for firmware/ESP32S3/main/idn_decoder.c.
 *
 * Dictionaries and samples are literal bytes written from ILDA IDN-Stream
 * rev002 (section numbers next to each case); expected outputs are literal
 * RP channel values, not recomputed with the decoder's own arithmetic.
 *
 * RP channel map: ch[0]=X ch[1]=Y ch[2]=R ch[3]=G ch[4]=B ch[5]=I ch[6]=U1 ch[7]=U2.
 */

#include <stdio.h>
#include <string.h>

#include "idn_decoder.h"

static int s_checks;
static int s_failures;

#define CHECK(cond, ...) do {                                        \
        s_checks++;                                                  \
        if (!(cond)) {                                               \
            s_failures++;                                            \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);              \
            printf(__VA_ARGS__);                                     \
            printf("\n");                                            \
        }                                                            \
    } while (0)

#define N(a) (sizeof(a) / sizeof((a)[0]))

/* A literal byte array and its length, for the case tables. */
#define D(...) (const uint8_t[]){ __VA_ARGS__ }, sizeof((const uint8_t[]){ __VA_ARGS__ })

/* Colour and intensity outputs: 0x00 is raw DAC 0 (also every unassigned line), 0xFF is 0xFF00. */
#define OFF  (-32768)
#define FULL 32512

static idn_decoder_caps_t s_caps;

static void expect_channels(const char *name, const int16_t got[IDN_DECODER_CHANNELS],
                            const int16_t exp[IDN_DECODER_CHANNELS])
{
    for (int ch = 0; ch < IDN_DECODER_CHANNELS; ch++)
        CHECK(got[ch] == exp[ch], "%s: ch[%d] = %d, expected %d", name, ch, got[ch], exp[ch]);
}

static bool compile_ok(const char *name, const uint8_t *dict, size_t len, idn_decoder_layout_t *lay)
{
    idn_decoder_result_t r = idn_decoder_compile(dict, len, &s_caps, lay);
    CHECK(r.status == IDN_DECODER_OK && lay->valid, "%s: compile %s/%s",
          name, idn_decoder_status_name(r.status), idn_decoder_reason_name(r.reason));
    return r.status == IDN_DECODER_OK;
}

/* -- Layouts ---------------------------------------------------------------- */

/* 3.4.11 IDTF X/Y/R/G/B: X+precision, Y+precision, 638/532/460 nm, Void pad */
#define DICT_RGB16 0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x52,0x7E, 0x52,0x14, 0x51,0xCC, 0x00,0x00
/* Same colours described blue, green, red */
#define DICT_BGR16 0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x51,0xCC, 0x52,0x14, 0x52,0x7E, 0x00,0x00
/* 3.4.8 green only, 532 nm (0x5214), 8-bit X/Y */
#define DICT_GREEN 0x42,0x00, 0x42,0x10, 0x52,0x14, 0x00,0x00
/* 8-bit X/Y/R/G/B with explicit intensity 0x5C10 (3.4.9) */
#define DICT_RGBI8 0x42,0x00, 0x42,0x10, 0x52,0x7E, 0x52,0x14, 0x51,0xCC, 0x5C,0x10
/* 8-bit X, Y and red: three tags, so a case adds one more (or a Void pad) to fill whole words */
#define XYR        0x42,0x00, 0x42,0x10, 0x52,0x7E

static const uint8_t k_rgb16[] = { DICT_RGB16 };

/* -- Output semantics ------------------------------------------------------- */

/* One sample per case; the sample's length is the layout's stride. */
typedef struct {
    const char    *name;
    const uint8_t *dict;
    size_t         dict_len;
    const uint8_t *sample;
    size_t         stride;
    int16_t        exp[IDN_DECODER_CHANNELS];
} decode_case_t;

static const decode_case_t k_decode[] = {
    /* X=0x1234 Y=0xFEDC R=0xFF G=0x80 B=0x00; intensity is derived from the largest colour */
    { "rgb16", D(DICT_RGB16), D(0x12,0x34, 0xFE,0xDC, 0xFF, 0x80, 0x00), { 4660, -292, FULL, 0, OFF, FULL, OFF, OFF } },
    { "bgr16", D(DICT_BGR16), D(0x12,0x34, 0xFE,0xDC, 0x00, 0x80, 0xFF), { 4660, -292, FULL, 0, OFF, FULL, OFF, OFF } },
    { "green only", D(DICT_GREEN), D(0x00, 0x00, 0xFF), { 0, 0, OFF, FULL, OFF, FULL, OFF, OFF } },

    /* Explicit intensity zero stays raw zero and leaves RGB as sent. */
    { "explicit intensity 0", D(DICT_RGBI8), D(0, 0, 0xFF, 0xFF, 0xFF, 0x00), { 0, 0, FULL, FULL, FULL, OFF, OFF, OFF } },
    { "explicit intensity, colours off", D(DICT_RGBI8), D(0, 0, 0, 0, 0, 0x80), { 0, 0, OFF, OFF, OFF, 0, OFF, OFF } },
    /* Absent intensity: largest colour (R=0x10 G=0x40 B=0x20 -> 0x4000). */
    { "absent intensity", D(DICT_RGB16), D(0,0, 0,0, 0x10, 0x40, 0x20), { 0, 0, -28672, -16384, -24576, -16384, OFF, OFF } },
    /* All zero is dark on every line, U1/U2 included (raw DAC 0, not mid-scale). */
    { "all zero, explicit I", D(DICT_RGBI8), D(0, 0, 0, 0, 0, 0), { 0, 0, OFF, OFF, OFF, OFF, OFF, OFF } },
    { "all zero, derived I", D(DICT_RGB16), D(0,0, 0,0, 0, 0, 0), { 0, 0, OFF, OFF, OFF, OFF, OFF, OFF } },

    /* 3.4.5: X = 12 34 56 -> 0x1234 (third octet truncated), Y = 0x7F -> 0x7F00 */
    { "X 24-bit", D(0x42,0x00, 0x40,0x10, 0x40,0x10, 0x42,0x10, 0x52,0x14, 0x00,0x00),
      D(0x12, 0x34, 0x56, 0x7F, 0xFF), { 4660, 32512, OFF, FULL, OFF, FULL, OFF, OFF } },
    { "coordinate extremes", D(DICT_RGB16), D(0x80,0x00, 0x7F,0xFF, 0, 0, 0), { -32768, 32767, OFF, OFF, OFF, OFF, OFF, OFF } },
    { "coordinate -1/+1", D(DICT_RGB16), D(0xFF,0xFF, 0x00,0x01, 0, 0, 0), { -1, 1, OFF, OFF, OFF, OFF, OFF, OFF } },
    /* 8-bit coordinates are left aligned: 0x80 -> -32768 */
    { "8-bit coordinates", D(DICT_GREEN), D(0x80, 0x01, 0x00), { -32768, 256, OFF, OFF, OFF, OFF, OFF, OFF } },
    /* 16-bit colour precision is kept: R = 0x1234 */
    { "16-bit colour", D(XYR, 0x40,0x10), D(0, 0, 0x12, 0x34), { 0, 0, -28108, OFF, OFF, -28108, OFF, OFF } },
    /* 3.4.6: absent intensity derives from the scaled colour (CSCL 1, R=0xFF -> 0x3FC0). */
    { "derived intensity after CSCL", D(0x41,0x00, XYR), D(0x70, 0, 0, 0xFF), { 0, 0, -16448, OFF, OFF, -16448, OFF, OFF } },

    /* X, Y, Z+precision (3.4.7), unknown 0x4020, Beam-Brush 0x5C20, red */
    { "ignored descriptors", D(0x42,0x00, 0x42,0x10, 0x42,0x20, 0x40,0x10, 0x40,0x20, 0x5C,0x20, 0x52,0x7E, 0x00,0x00),
      D(0x7F, 0x80, 0xAA, 0xBB, 0xCC, 0xDD, 0xFF), { 32512, -32768, FULL, OFF, OFF, FULL, OFF, OFF } },
    /* NOP 0x4000 consumes an octet */
    { "NOP", D(0x40,0x00, XYR), D(0x55, 0x01, 0x02, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
    /* X-prime 0x4201 + precision (3.4.10 U4) is consumed, not mixed into X */
    { "X-prime ignored", D(0x42,0x00, 0x42,0x10, 0x42,0x01, 0x40,0x10, 0x52,0x7E, 0x00,0x00),
      D(0x01, 0x02, 0x7F, 0xFF, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
    /* Void with one suffix word (3.4.2): the suffix is not a sample field */
    { "void suffix", D(XYR, 0x00,0x01, 0x12,0x34, 0x00,0x00), D(0x01, 0x02, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
    /* Space modifier settings this target accepts (3.4.4) */
    { "space projector specific", D(0x11,0x00, XYR), D(0x01, 0x02, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
    { "space no transformation", D(XYR, 0x11,0xFF), D(0x01, 0x02, 0xFF), { 256, 512, FULL, OFF, OFF, FULL, OFF, OFF } },
};

static void test_decode(void)
{
    for (size_t c = 0; c < N(k_decode); c++) {
        const decode_case_t *t = &k_decode[c];
        idn_decoder_layout_t lay;
        int16_t out[1][IDN_DECODER_CHANNELS];
        if (!compile_ok(t->name, t->dict, t->dict_len, &lay)) continue;
        CHECK(lay.stride == t->stride, "%s: stride %u, expected %u", t->name, lay.stride, (unsigned)t->stride);
        idn_decoder_decode(&lay, t->sample, 1, out);
        expect_channels(t->name, out[0], t->exp);
    }

    CHECK(idn_decoder_optical_to_signed(0) == -32768, "optical 0");
    CHECK(idn_decoder_optical_to_signed(0xFFFF) == 32767, "optical max");
}

static void test_scales(void)
{
    /* 3.4.6: CSCL bits 7-6 and ISCL bits 5-4 divide by 1, 4, 16, 64 (R = I = 0xFF). */
    static const struct { uint8_t hint; int16_t r; int16_t i; } cases[] = {
        { 0x00,  32512,  32512 },
        { 0x40, -16448,  32512 },   /* 0xFF00 / 4  = 0x3FC0 */
        { 0x80, -28688,  32512 },   /* 0xFF00 / 16 = 0x0FF0 */
        { 0xC0, -31748,  32512 },   /* 0xFF00 / 64 = 0x03FC */
        { 0x10,  32512, -16448 },
        { 0x20,  32512, -28688 },
        { 0x30,  32512, -31748 },
        { 0xF0, -31748, -31748 },
        { 0x70, -16448, -31748 },
        { 0x0F,  32512,  32512 },   /* reserved low bits ignored */
    };
    /* hint format 0, X, Y, red, intensity */
    static const uint8_t dict[] = { 0x41,0x00, 0x42,0x00, 0x42,0x10, 0x52,0x7E, 0x5C,0x10, 0x00,0x00 };

    idn_decoder_layout_t lay;
    if (!compile_ok("hint layout", dict, sizeof(dict), &lay)) return;

    for (size_t c = 0; c < N(cases); c++) {
        uint8_t s[5] = { cases[c].hint, 0x00, 0x00, 0xFF, 0xFF };
        int16_t out[1][IDN_DECODER_CHANNELS];
        idn_decoder_decode(&lay, s, 1, out);
        CHECK(out[0][2] == cases[c].r, "hint 0x%02X: R %d, expected %d", cases[c].hint, out[0][2], cases[c].r);
        CHECK(out[0][5] == cases[c].i, "hint 0x%02X: I %d, expected %d", cases[c].hint, out[0][5], cases[c].i);
        CHECK(out[0][3] == OFF && out[0][6] == OFF, "hint 0x%02X: unassigned", cases[c].hint);
    }
}

static void test_wavelength_mapping(void)
{
    /* 3.4.8 single colour at a given wavelength; which RP line lights */
    static const struct { uint8_t hi, lo; uint16_t nm; int ch; } cases[] = {
        { 0x52, 0x7E, 638, 2 },
        { 0x52, 0x80, 640, 2 },
        { 0x52, 0x14, 532, 3 },
        { 0x52, 0x41, 577, 3 },
        { 0x52, 0x49, 585, 3 },   /* tie 638/532 -> shorter */
        { 0x51, 0xF0, 496, 4 },   /* tie 532/460 -> shorter */
        { 0x51, 0xCC, 460, 4 },
        { 0x51, 0xC2, 450, 4 },
        { 0x51, 0x95, 405, 4 },
    };

    for (size_t c = 0; c < N(cases); c++) {
        uint8_t d[] = { 0x42,0x00, 0x42,0x10, cases[c].hi, cases[c].lo, 0x00,0x00 };
        uint8_t s[] = { 0x00, 0x00, 0xFF };
        idn_decoder_layout_t lay;
        int16_t out[1][IDN_DECODER_CHANNELS];
        char name[32];
        snprintf(name, sizeof(name), "%u nm", cases[c].nm);
        if (!compile_ok(name, d, sizeof(d), &lay)) continue;
        idn_decoder_decode(&lay, s, 1, out);
        for (int ch = 2; ch <= 4; ch++)
            CHECK(out[0][ch] == (ch == cases[c].ch ? FULL : OFF), "%s: ch[%d] = %d", name, ch, out[0][ch]);
    }

    /* Custom capability: a 445 nm line on U1 takes deep blue. */
    idn_decoder_caps_t caps;
    idn_decoder_default_caps(&caps);
    caps.colors[3].wavelength_nm = 445;
    caps.colors[3].channel = 6;
    caps.color_count = 4;
    static const uint8_t d445[] = { 0x42,0x00, 0x42,0x10, 0x51,0xBD, 0x51,0xCC };
    idn_decoder_layout_t lay;
    idn_decoder_result_t r = idn_decoder_compile(d445, sizeof(d445), &caps, &lay);
    CHECK(r.status == IDN_DECODER_OK, "custom caps compile %s", idn_decoder_reason_name(r.reason));
    if (r.status == IDN_DECODER_OK) {
        static const uint8_t s[] = { 0x00, 0x00, 0xFF, 0x00 };
        static const int16_t e[8] = { 0, 0, OFF, OFF, OFF, FULL, FULL, OFF };
        int16_t out[1][IDN_DECODER_CHANNELS];
        idn_decoder_decode(&lay, s, 1, out);
        expect_channels("445 nm on U1", out[0], e);
    }
}

/* -- Configuration rejection ----------------------------------------------- */

typedef struct {
    const char           *name;
    const uint8_t        *dict;
    size_t                len;
    idn_decoder_status_t  status;
    idn_decoder_reason_t  reason;
} reject_case_t;

#define MALFORMED   IDN_DECODER_ERR_MALFORMED
#define INVALID     IDN_DECODER_ERR_INVALID_CONFIG
#define UNSUPPORTED IDN_DECODER_ERR_UNSUPPORTED
#define RESOURCE    IDN_DECODER_ERR_RESOURCE
#define R(x)        IDN_DECODER_REASON_##x

static void test_rejections(void)
{
    const reject_case_t cases[] = {
        { "cat 2",                 D(XYR, 0x20,0x00),                             INVALID,     R(UNKNOWN_CATEGORY) },
        { "cat 12 (dynamic)",      D(XYR, 0xC0,0x00),                             INVALID,     R(UNKNOWN_CATEGORY) },
        { "cat 0 sub 1",           D(XYR, 0x01,0x00),                             INVALID,     R(UNKNOWN_CATEGORY) },
        { "cat 1 sub 2",           D(XYR, 0x12,0x00),                             INVALID,     R(UNKNOWN_CATEGORY) },
        { "cat 4 sub 3",           D(XYR, 0x43,0x00),                             INVALID,     R(UNKNOWN_CATEGORY) },
        { "cat 5 sub 4",           D(XYR, 0x54,0x00),                             INVALID,     R(UNKNOWN_CATEGORY) },
        { "truncated suffix",      D(XYR, 0x00,0x03),                             MALFORMED,   R(TRUNCATED_SUFFIX) },
        { "odd word count",        D(XYR),                                        MALFORMED,   R(DICT_LENGTH) },
        { "empty",                 NULL, 0,                                       INVALID,     R(MISSING_X) },
        { "break",                 D(XYR, 0x10,0x00, 0x42,0x01, 0x42,0x11, 0x52,0x7E, 0x00,0x00), UNSUPPORTED, R(BREAK) },
        { "sample shutter",        D(0x41,0x01, XYR),                             UNSUPPORTED, R(SAMPLE_SHUTTER) },
        /* 3.4.10 full ISP-DB25 default mapping, then padding */
        { "ISP-DB25 default",      D(0x41,0x01, 0x5C,0x10, 0x52,0x7E, 0x52,0x14, 0x51,0xCC, 0x51,0xBD, 0x52,0x41, 0x51,0xE8,
                                     0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x42,0x01, 0x40,0x10, 0x00,0x00, 0x00,0x00),
                                                                                  UNSUPPORTED, R(SAMPLE_SHUTTER) },
        /* Same without 0x4101: 445 nm lands on the 460 nm line already used */
        { "ISP-DB25, no shutter",  D(0x5C,0x10, 0x52,0x7E, 0x52,0x14, 0x51,0xCC, 0x51,0xBD, 0x52,0x41, 0x51,0xE8,
                                     0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x42,0x01, 0x40,0x10, 0x00,0x00),
                                                                                  UNSUPPORTED, R(COLOR_CONFLICT) },
        { "hint format 2",         D(0x41,0x02, XYR),                             UNSUPPORTED, R(HINT_FORMAT) },
        { "duplicate hint",        D(0x41,0x00, 0x41,0x00, XYR, 0x00,0x00),       INVALID,     R(DUPLICATE_HINT) },
        { "duplicate X",           D(0x42,0x00, XYR),                             INVALID,     R(DUPLICATE_X) },
        { "duplicate Y",           D(0x42,0x00, 0x42,0x10, 0x42,0x10, 0x52,0x7E), INVALID,     R(DUPLICATE_Y) },
        { "duplicate I",           D(XYR, 0x5C,0x10, 0x5C,0x10, 0x00,0x00),       INVALID,     R(DUPLICATE_INTENSITY) },
        { "colour conflict",       D(XYR, 0x52,0x80),                             UNSUPPORTED, R(COLOR_CONFLICT) },
        { "missing X",             D(0x42,0x10, 0x52,0x7E),                       INVALID,     R(MISSING_X) },
        { "missing Y",             D(0x42,0x00, 0x52,0x7E),                       INVALID,     R(MISSING_Y) },
        { "intensity only",        D(0x42,0x00, 0x42,0x10, 0x5C,0x10, 0x00,0x00), INVALID,     R(MISSING_COLOR) },
        { "X-prime only",          D(0x42,0x01, 0x42,0x10, 0x52,0x7E, 0x00,0x00), INVALID,     R(MISSING_X) },
        { "precision first",       D(0x40,0x10, XYR),                             INVALID,     R(PRECISION_PLACEMENT) },
        { "precision on hint",     D(0x41,0x00, 0x40,0x10, XYR, 0x00,0x00),       INVALID,     R(PRECISION_PLACEMENT) },
        { "precision on NOP",      D(0x40,0x00, 0x40,0x10, XYR, 0x00,0x00),       INVALID,     R(PRECISION_PLACEMENT) },
        { "precision after void",  D(0x42,0x00, 0x00,0x00, 0x40,0x10, 0x42,0x10, 0x52,0x7E, 0x00,0x00), INVALID, R(PRECISION_PLACEMENT) },
        { "precision limit",       D(0x42,0x00, 0x40,0x10, 0x40,0x10, 0x40,0x10, 0x40,0x10, 0x42,0x10, 0x52,0x7E, 0x00,0x00),
                                                                                  RESOURCE,    R(PRECISION_LIMIT) },
        { "zero wavelength",       D(0x42,0x00, 0x42,0x10, 0x50,0x00, 0x00,0x00), INVALID,     R(ZERO_WAVELENGTH) },
        { "dynamic wavelength",    D(0x42,0x00, 0x42,0x10, 0x5C,0x00, 0x50,0x00), UNSUPPORTED, R(DYNAMIC_WAVELENGTH) },
        { "dynamic wl + prec",     D(0x42,0x00, 0x42,0x10, 0x5C,0x00, 0x40,0x10, 0x50,0x00, 0x00,0x00), UNSUPPORTED, R(DYNAMIC_WAVELENGTH) },
        { "prefix then Y",         D(0x42,0x00, 0x5C,0x00, 0x42,0x10, 0x52,0x7E), INVALID,     R(WAVELENGTH_PREFIX) },
        { "prefix, static colour", D(0x42,0x00, 0x42,0x10, 0x5C,0x00, 0x52,0x7E), INVALID,     R(WAVELENGTH_PREFIX) },
        { "prefix at end",         D(XYR, 0x5C,0x00),                             INVALID,     R(WAVELENGTH_PREFIX) },
        { "unknown struct tag",    D(XYR, 0x00,0x10),                             UNSUPPORTED, R(UNKNOWN_MODIFIER) },
        { "unknown modifier",      D(XYR, 0x10,0x10),                             UNSUPPORTED, R(UNKNOWN_MODIFIER) },
        { "space GLIN linear",     D(0x11,0x40, XYR),                             UNSUPPORTED, R(SPACE_UNSUPPORTED) },
        { "space CLIN visual",     D(0x11,0x20, XYR),                             UNSUPPORTED, R(SPACE_UNSUPPORTED) },
        { "space GLIN reserved",   D(0x11,0x80, XYR),                             INVALID,     R(SPACE_MODIFIER) },
        { "space twice",           D(0x11,0x00, 0x11,0x00, XYR, 0x00,0x00),       INVALID,     R(DUPLICATE_SPACE) },
    };

    for (size_t c = 0; c < N(cases); c++) {
        idn_decoder_layout_t lay;
        memset(&lay, 0xA5, sizeof(lay));
        idn_decoder_result_t r = idn_decoder_compile(cases[c].dict, cases[c].len, &s_caps, &lay);
        CHECK(r.status == cases[c].status && r.reason == cases[c].reason,
              "%s: got %s/%s, expected %s/%s", cases[c].name,
              idn_decoder_status_name(r.status), idn_decoder_reason_name(r.reason),
              idn_decoder_status_name(cases[c].status), idn_decoder_reason_name(cases[c].reason));
        CHECK(!lay.valid && lay.stride == 0 && lay.field_count == 0, "%s: layout published", cases[c].name);
    }

    /* Stride limit: X, Y, R plus NOPs. 32 octets accepted, 33 rejected. */
    uint8_t d[2 * 40];
    for (int nops = 29; nops <= 30; nops++) {
        size_t w = 0;
        d[w++] = 0x42; d[w++] = 0x00;
        d[w++] = 0x42; d[w++] = 0x10;
        d[w++] = 0x52; d[w++] = 0x7E;
        for (int k = 0; k < nops; k++) { d[w++] = 0x40; d[w++] = 0x00; }
        while (w % 4) { d[w++] = 0x00; d[w++] = 0x00; }
        idn_decoder_layout_t lay;
        idn_decoder_result_t r = idn_decoder_compile(d, w, &s_caps, &lay);
        if (nops == 29)
            CHECK(r.status == IDN_DECODER_OK && lay.stride == 32, "stride 32 accepted (%s)", idn_decoder_reason_name(r.reason));
        else
            CHECK(r.status == RESOURCE && r.reason == R(STRIDE_LIMIT) && !lay.valid,
                  "stride 33 rejected (%s)", idn_decoder_reason_name(r.reason));
    }

    /* Dictionary larger than SCWC can declare */
    static uint8_t big[IDN_DECODER_MAX_DICT_BYTES + 4];
    idn_decoder_layout_t lay;
    idn_decoder_result_t r = idn_decoder_compile(big, sizeof(big), &s_caps, &lay);
    CHECK(r.status == RESOURCE && r.reason == R(DICT_TOO_LARGE), "oversized dictionary");

    /* Offending tag is reported */
    static const uint8_t db[] = { 0x42,0x00, 0x42,0x10, 0x52,0x7E, 0x10,0x00 };
    r = idn_decoder_compile(db, sizeof(db), &s_caps, &lay);
    CHECK(r.tag_index == 3 && r.tag == 0x1000, "break tag index %u tag 0x%04X", r.tag_index, r.tag);
}

/* -- Wave chunks (6.1) ------------------------------------------------------ */

static size_t build_chunk(uint8_t *buf, uint32_t duration, const uint8_t *sample, size_t stride, size_t count)
{
    buf[0] = 0x00;
    buf[1] = (uint8_t)(duration >> 16);
    buf[2] = (uint8_t)(duration >> 8);
    buf[3] = (uint8_t)duration;
    for (size_t i = 0; i < count; i++)
        memcpy(buf + 4 + i * stride, sample, stride);
    return 4 + count * stride;
}

static void test_wave_chunks(void)
{
    static const uint8_t s[] = { 0x12,0x34, 0xFE,0xDC, 0xFF, 0x80, 0x00 };
    static const int16_t exp[8] = { 4660, -292, FULL, 0, OFF, FULL, OFF, OFF };
    uint8_t buf[4 + 7 * 64 + 8];
    uint32_t n = 0, dur = 0;
    idn_decoder_result_t r;
    idn_decoder_layout_t lay;
    if (!compile_ok("chunk layout", k_rgb16, sizeof(k_rgb16), &lay)) return;

    size_t len = build_chunk(buf, 675, s, 7, 20);
    r = idn_decoder_check_wave_chunk(&lay, buf, len, &n, &dur);
    CHECK(r.status == IDN_DECODER_OK && n == 20 && dur == 675, "20 samples accepted (%s)", idn_decoder_reason_name(r.reason));
    if (r.status == IDN_DECODER_OK) {
        int16_t out[20][IDN_DECODER_CHANNELS];
        idn_decoder_decode(&lay, buf + 4, n, out);
        expect_channels("chunk sample 0", out[0], exp);
        expect_channels("chunk sample 19", out[19], exp);
    }

    const struct { const char *name; size_t len; uint32_t dur; idn_decoder_reason_t reason; } bad[] = {
        { "19 samples",          4 + 7 * 19,     675, R(TOO_FEW_SAMPLES) },
        { "malformed last",      4 + 7 * 20 + 1, 675, R(PARTIAL_SAMPLE) },
        { "missing final octet", 4 + 7 * 21 - 1, 675, R(PARTIAL_SAMPLE) },
        { "zero duration",       4 + 7 * 20,     0,   R(ZERO_DURATION) },
        { "short header",        3,              675, R(CHUNK_HEADER) },
    };
    for (size_t c = 0; c < N(bad); c++) {
        build_chunk(buf, bad[c].dur, s, 7, 21);
        n = 0xDEAD;
        r = idn_decoder_check_wave_chunk(&lay, buf, bad[c].len, &n, NULL);
        CHECK(r.status == IDN_DECODER_ERR_INVALID_SAMPLES && r.reason == bad[c].reason && n == 0xDEAD,
              "%s: got %s/%s", bad[c].name, idn_decoder_status_name(r.status), idn_decoder_reason_name(r.reason));
    }
}

static void test_rejected_replacement(void)
{
    /* Good layout, then a replacement with Break: data in the old format must no longer decode. */
    static const uint8_t s[] = { 0x12,0x34, 0xFE,0xDC, 0xFF, 0x80, 0x00 };
    static const uint8_t brk[] = { 0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x52,0x7E, 0x52,0x14, 0x51,0xCC, 0x10,0x00 };
    uint8_t buf[4 + 7 * 20];
    uint32_t n = 0;
    idn_decoder_layout_t lay;

    if (!compile_ok("replacement base", k_rgb16, sizeof(k_rgb16), &lay)) return;
    size_t len = build_chunk(buf, 675, s, 7, 20);
    CHECK(idn_decoder_check_wave_chunk(&lay, buf, len, &n, NULL).status == IDN_DECODER_OK, "base chunk");

    idn_decoder_result_t r = idn_decoder_compile(brk, sizeof(brk), &s_caps, &lay);
    CHECK(r.status == UNSUPPORTED && !lay.valid, "replacement rejected");
    r = idn_decoder_check_wave_chunk(&lay, buf, len, &n, NULL);
    CHECK(r.status == IDN_DECODER_ERR_INVALID_SAMPLES && r.reason == R(INVALID_LAYOUT),
          "old decoder reused after rejected replacement");
}

int main(void)
{
    idn_decoder_default_caps(&s_caps);

    test_decode();
    test_scales();
    test_wavelength_mapping();
    test_rejections();
    test_wave_chunks();
    test_rejected_replacement();

    printf("idn_decoder: %d checks, %d failures\n", s_checks, s_failures);
    return s_failures ? 1 : 0;
}

/*
 * Host tests for firmware/ESP32S3/main/idn_session.c.
 *
 * Each scenario is a list of steps: an input (RT packet, clock tick, group
 * request, local cancel) with literal expectations for the result code,
 * session/channel state, ACK event flags and the exact output actions the
 * step caused. The fake output sink also checks the ownership contract:
 * every acquire uses a newer generation, enqueue only under the generation
 * it acquired, release only what it holds.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "idn_session.h"

#define N(a) (sizeof(a) / sizeof((a)[0]))

/* -- Fake decoder/output sink ------------------------------------------------ */

typedef struct {
    int compiles;
    int discards;
    int acquires;
    int enqueues;
    int releases;
    int violations;

    bool acquire_ok;
    bool release_ok;
    bool quiescent;

    bool held;
    uint32_t held_gen;
    uint32_t last_gen;
    idn_stop_reason_t last_reason;
    bool decoder[IDN_SESSION_CHANNELS];
} fake_t;

static fake_t g_fake;

static idn_cfg_result_t fake_compile(void *ctx, uint8_t ch, uint8_t mode,
                                     const uint8_t *cfg, uint16_t len)
{
    fake_t *f = ctx;
    f->compiles++;
    if (mode != IDN_MODE_GRAPHIC_CONTINUOUS) {
        f->violations++;
    }
    /* Tag 0xFFFF stands in for any configuration the decoder rejects. */
    for (uint16_t i = 0; i + 1 < len; i += 2) {
        if (cfg[i] == 0xFF && cfg[i + 1] == 0xFF) {
            return IDN_CFG_INVALID;
        }
    }
    f->decoder[ch] = true;
    return IDN_CFG_OK;
}

static void fake_discard(void *ctx, uint8_t ch)
{
    fake_t *f = ctx;
    f->discards++;
    f->decoder[ch] = false;
}

static idn_chunk_result_t fake_check(void *ctx, uint8_t ch, const uint8_t *chunk, uint16_t len)
{
    fake_t *f = ctx;
    (void)chunk;
    if (!f->decoder[ch]) {
        f->violations++;
    }
    if (len < 4 + 20 * 8 || (len - 4) % 8 != 0) {
        return IDN_CHUNK_TOO_SHORT;
    }
    return IDN_CHUNK_OK;
}

static bool fake_acquire(void *ctx, uint32_t gen)
{
    fake_t *f = ctx;
    f->acquires++;
    if (f->held || gen <= f->last_gen) {
        f->violations++;
    }
    if (!f->acquire_ok) {
        return false;
    }
    f->held = true;
    f->held_gen = gen;
    f->last_gen = gen;
    return true;
}

static idn_chunk_result_t fake_enqueue(void *ctx, uint8_t ch, uint32_t gen,
                                       const uint8_t *chunk, uint16_t len)
{
    fake_t *f = ctx;
    (void)chunk;
    (void)len;
    f->enqueues++;
    if (!f->held || gen != f->held_gen || !f->decoder[ch]) {
        f->violations++;
    }
    return IDN_CHUNK_OK;
}

static bool fake_release(void *ctx, uint32_t gen, idn_stop_reason_t reason)
{
    fake_t *f = ctx;
    f->releases++;
    f->last_reason = reason;
    if (!f->held || gen != f->held_gen) {
        f->violations++;
    }
    f->held = false;
    return f->release_ok;
}

static bool fake_quiescent(void *ctx)
{
    fake_t *f = ctx;
    return f->quiescent;
}

static const idn_session_ops_t k_ops = {
    .ctx = &g_fake,
    .config_compile = fake_compile,
    .config_discard = fake_discard,
    .wave_check = fake_check,
    .output_acquire = fake_acquire,
    .wave_enqueue = fake_enqueue,
    .output_release = fake_release,
    .output_quiescent = fake_quiescent,
};

/* -- Packet builder --------------------------------------------------------- */

typedef struct {
    bool    present;
    bool    not_channel_msg;
    uint8_t ch;
    uint8_t chunk;
    bool    cfg;
    uint8_t cfl;
    uint8_t sid;
    uint8_t mode;
    uint8_t scwc;
    bool    bad_cfg;        /* decoder will reject */
    bool    alt_cfg;        /* different but valid dictionary */
    uint8_t nsamples;
    uint8_t scm;
    int     size_delta;     /* corrupt the declared total size */
    uint8_t scwc_extra;     /* declare more configuration words than sent */
} msg_spec_t;

static size_t build_msg(const msg_spec_t *m, uint8_t *buf)
{
    if (!m->present) {
        return 0;
    }

    /* XYRGBI dictionary from plan 04: 8 tags, SCWC 4 */
    static const uint16_t dict[8] = {
        0x4200, 0x4010, 0x4210, 0x4010, 0x527E, 0x5214, 0x51CC, 0x5C10,
    };

    size_t n = 8;
    buf[2] = (uint8_t)((m->not_channel_msg ? 0x00 : 0x80) | (m->cfg ? 0x40 : 0) | (m->ch & 0x3F));
    buf[3] = m->chunk;
    buf[4] = 0x00; buf[5] = 0x01; buf[6] = 0x00; buf[7] = 0x00;

    if (m->cfg) {
        buf[n++] = (uint8_t)(m->scwc + m->scwc_extra);
        buf[n++] = m->cfl;
        buf[n++] = m->sid;
        buf[n++] = m->mode;
        for (unsigned w = 0; w < m->scwc * 2u; w++) {
            uint16_t tag = dict[w % 8];
            if (m->bad_cfg && w == 0) tag = 0xFFFF;
            if (m->alt_cfg && w == 7) tag = 0x0000;
            buf[n++] = (uint8_t)(tag >> 8);
            buf[n++] = (uint8_t)tag;
        }
    }

    if (m->chunk == IDN_CHUNK_WAVE) {
        uint32_t dur = m->nsamples * 33u;
        buf[n++] = (uint8_t)((m->scm & 3) << 4);
        buf[n++] = (uint8_t)(dur >> 16);
        buf[n++] = (uint8_t)(dur >> 8);
        buf[n++] = (uint8_t)dur;
        memset(buf + n, 0, (size_t)m->nsamples * 8u);
        n += (size_t)m->nsamples * 8u;
    }

    size_t declared = (size_t)((int)n + m->size_delta);
    buf[0] = (uint8_t)(declared >> 8);
    buf[1] = (uint8_t)declared;
    return n;
}

#define ROUTING(sdm)    (uint8_t)(0x01 | ((sdm) << 4))
#define CLOSE           0x02

#define M_EMPTY                 { .present = false }
#define M_VOID(c)               { .present = true, .ch = (c), .chunk = IDN_CHUNK_VOID }
#define M_ROUTE(c, sdm)         { .present = true, .ch = (c), .chunk = IDN_CHUNK_VOID, .cfg = true, \
                                  .cfl = ROUTING(sdm), .sid = 1, .mode = 1, .scwc = 4 }
#define M_ROUTE_SVC(c, s, md)   { .present = true, .ch = (c), .chunk = IDN_CHUNK_VOID, .cfg = true, \
                                  .cfl = ROUTING(0), .sid = (s), .mode = (md), .scwc = 4 }
#define M_ROUTE_BAD(c)          { .present = true, .ch = (c), .chunk = IDN_CHUNK_VOID, .cfg = true, \
                                  .cfl = ROUTING(0), .sid = 1, .mode = 1, .scwc = 4, .bad_cfg = true }
#define M_ROUTE_ALT(c)          { .present = true, .ch = (c), .chunk = IDN_CHUNK_VOID, .cfg = true, \
                                  .cfl = ROUTING(0), .sid = 1, .mode = 1, .scwc = 4, .alt_cfg = true }
#define M_ROUTE_TRUNC(c)        { .present = true, .ch = (c), .chunk = IDN_CHUNK_VOID, .cfg = true, \
                                  .cfl = ROUTING(0), .sid = 1, .mode = 1, .scwc = 4, .scwc_extra = 1 }
#define M_WAVE(c, sc)           { .present = true, .ch = (c), .chunk = IDN_CHUNK_WAVE, .nsamples = 20, .scm = (sc) }
#define M_ROUTE_WAVE(c, sdm)    { .present = true, .ch = (c), .chunk = IDN_CHUNK_WAVE, .cfg = true, \
                                  .cfl = ROUTING(sdm), .sid = 1, .mode = 1, .scwc = 4, .nsamples = 20, .scm = (sdm) }
#define M_CLOSE_CH(c)           { .present = true, .ch = (c), .chunk = IDN_CHUNK_VOID, .cfg = true, .cfl = CLOSE }

/* -- Scenario tables -------------------------------------------------------- */

enum { EP_A, EP_A_PORT2, EP_B, EP_A_IF1, EP_A_IF2, EP_COUNT };

static const idn_endpoint_t k_ep[EP_COUNT] = {
    [EP_A]       = { 0x0A000002u, 5000, 0 },
    [EP_A_PORT2] = { 0x0A000002u, 5001, 0 },
    [EP_B]       = { 0x0A000003u, 5000, 0 },
    [EP_A_IF1]   = { 0x0A000002u, 5000, 0xC0A80101u },
    [EP_A_IF2]   = { 0x0A000002u, 5000, 0x0A000001u },
};

typedef enum { EV_END = 0, EV_RT, EV_TICK, EV_GROUP, EV_CANCEL, EV_SET } ev_kind_t;

/* X(v): expect v. A zero field is not checked. Output deltas are always checked. */
#define X(v) ((v) + 1)

typedef struct {
    ev_kind_t  kind;
    int64_t    now;
    int        ep;          /* sender; EP_A unless given */
    uint8_t    cmd;         /* RT command; 0 is a channel message asking for an ACK */
    uint8_t    group;
    msg_spec_t msg;
    const uint8_t *raw;     /* EV_GROUP payload */
    size_t     raw_len;

    /* EV_SET knobs: X(0)/X(1) */
    int set_acquire_ok;
    int set_release_ok;
    int set_quiescent;

    int exp_result;     /* RT result / group response code */
    int exp_session;
    int exp_ch;         /* state of the message's channel (channel 0 for other steps) */
    int exp_iev;        /* ACK input events */
    int exp_pev;        /* ACK pipeline events */
    int exp_status;
    int exp_mask;       /* group response mask */
    int exp_reason;     /* last release reason */

    int d_compile, d_acquire, d_enqueue, d_release;
} step_t;

typedef struct {
    const char   *name;
    const step_t *steps;
} scenario_t;

#define RT(...)        { .kind = EV_RT, __VA_ARGS__ }
#define TICK(...)      { .kind = EV_TICK, __VA_ARGS__ }
#define GROUP(p, ...)  { .kind = EV_GROUP, .raw = (p), .raw_len = sizeof(p), __VA_ARGS__ }
#define CANCEL(...)    { .kind = EV_CANCEL, __VA_ARGS__ }
#define SET(...)       { .kind = EV_SET, __VA_ARGS__ }
#define END            { .kind = EV_END }

#define CLOSEACK IDN_CMD_RT_CLOSE_ACK
#define ABORT    IDN_CMD_RT_ABORT

/* Output actions of the first chunk on a newly routed channel */
#define OPENS       .d_compile = 1, .d_acquire = 1, .d_enqueue = 1
/* The usual first step: channel 0 routed and playing */
#define PLAYING     RT(.msg = M_ROUTE_WAVE(0, 0), OPENS)

/* Plan 02 transition table, row by row. */

static const step_t s_empty_allocates_only[] = {
    RT(.msg = M_EMPTY, .exp_result = X(IDN_RESULT_OK), .exp_session = X(IDN_SESSION_ACTIVE),
       .exp_iev = X(IDN_IEV_NEW), .exp_pev = X(0), .exp_status = X(0)),
    RT(.cmd = IDN_CMD_RT_CHANNEL, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OK), .exp_session = X(IDN_SESSION_ACTIVE)),
    END
};

static const step_t s_route_void_idle[] = {
    RT(.msg = M_ROUTE(0, 0), .exp_result = X(IDN_RESULT_OK), .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE),
       .exp_iev = X(IDN_IEV_NEW), .exp_pev = X(IDN_PEV_CNLRTED), .exp_status = X(IDN_STATUS_SOCNL), .d_compile = 1),
    END
};

static const step_t s_first_wave_acquires[] = {
    RT(.msg = M_ROUTE(0, 0), .d_compile = 1),
    RT(.msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(0), .d_acquire = 1, .d_enqueue = 1),
    RT(.msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(0), .d_enqueue = 1),
    END
};

static const step_t s_no_auto_open[] = {
    RT(.msg = M_WAVE(5, 0), .exp_result = X(IDN_RESULT_OK), .exp_ch = X(IDN_CHANNEL_CLOSED),
       .exp_iev = X(IDN_IEV_NEW | IDN_IEV_CCERR), .exp_pev = X(IDN_PEV_CNLCLSD), .exp_status = X(0)),
    /* Void messages on a closed channel are not CCErr */
    RT(.msg = M_VOID(5), .exp_iev = X(0), .exp_pev = X(IDN_PEV_CNLCLSD)),
    END
};

static const step_t s_second_channel_busy[] = {
    PLAYING,
    RT(.msg = M_ROUTE_WAVE(1, 0), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_BSYERR | IDN_PEV_CNLRTED)),
    /* Busy route is a NULL route: data is dropped without CCErr */
    RT(.msg = M_WAVE(1, 0), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_iev = X(0), .exp_pev = X(0)),
    /* The first channel keeps playing */
    RT(.msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .d_enqueue = 1),
    END
};

static const step_t s_unknown_service_mode[] = {
    RT(.msg = M_ROUTE_SVC(2, 7, 1), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_SMERR | IDN_PEV_CNLRTED),
       .exp_status = X(IDN_STATUS_SOCNL)),
    RT(.msg = M_WAVE(2, 0), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_iev = X(0), .exp_pev = X(0)),
    /* Discrete graphics on the laser service: unsupported, SMErr */
    RT(.msg = M_ROUTE_SVC(3, 1, 2), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_SMERR | IDN_PEV_CNLRTED)),
    /* Manufacturer-specific mode, default service */
    RT(.msg = M_ROUTE_SVC(4, 0, 0xC0), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_SMERR | IDN_PEV_CNLRTED)),
    END
};

static const step_t s_scm_mismatch[] = {
    RT(.msg = M_ROUTE_WAVE(0, 1), OPENS),
    RT(.msg = M_WAVE(0, 2), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_DCMERR),
       .exp_reason = X(IDN_STOP_ROUTE_CHANGE), .d_release = 1),
    /* The stale decoder is gone, even for data matching the old SDM */
    RT(.msg = M_WAVE(0, 1), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_CFGERR)),
    /* A new configuration recovers the channel */
    RT(.msg = M_ROUTE(0, 2), .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE), .exp_pev = X(IDN_PEV_CNLRTED), .d_compile = 1),
    RT(.msg = M_WAVE(0, 2), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(0), .d_acquire = 1, .d_enqueue = 1),
    END
};

static const step_t s_last_channel_close[] = {
    PLAYING,
    RT(.msg = M_CLOSE_CH(0), .exp_result = X(IDN_RESULT_OK), .exp_session = X(IDN_SESSION_ACTIVE),
       .exp_ch = X(IDN_CHANNEL_CLOSED), .exp_pev = X(IDN_PEV_CNLCLSD), .exp_status = X(0),
       .exp_reason = X(IDN_STOP_CHANNEL_CLOSE), .d_release = 1),
    RT(.msg = M_EMPTY, .exp_session = X(IDN_SESSION_ACTIVE), .exp_iev = X(0)),
    /* Reopen on the same connection */
    RT(.msg = M_ROUTE_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_iev = X(0), OPENS),
    END
};

static const step_t s_empty_close_no_connection[] = {
    RT(.cmd = CLOSEACK, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_EMPTY_CLOSE), .exp_session = X(IDN_SESSION_FREE)),
    RT(.cmd = IDN_CMD_RT_CLOSE, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_EMPTY_CLOSE),
       .exp_session = X(IDN_SESSION_FREE)),
    END
};

static const step_t s_same_ip_other_port[] = {
    PLAYING,
    RT(.ep = EP_A_PORT2, .msg = M_ROUTE_WAVE(0, 0), .exp_result = X(IDN_RESULT_OCCUPIED),
       .exp_session = X(IDN_SESSION_ACTIVE)),
    RT(.ep = EP_A_PORT2, .cmd = CLOSEACK, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_EMPTY_CLOSE),
       .exp_session = X(IDN_SESSION_ACTIVE)),
    RT(.ep = EP_A_PORT2, .cmd = ABORT, .msg = M_EMPTY, .exp_session = X(IDN_SESSION_ACTIVE)),
    RT(.ep = EP_B, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OCCUPIED)),
    RT(.msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_iev = X(0), .d_enqueue = 1),
    END
};

static const step_t s_abort[] = {
    PLAYING,
    RT(.cmd = ABORT, .msg = M_EMPTY, .exp_session = X(IDN_SESSION_FREE), .exp_ch = X(IDN_CHANNEL_CLOSED),
       .exp_reason = X(IDN_STOP_ABORT), .d_release = 1),
    /* Repeated abort does nothing */
    RT(.cmd = ABORT, .msg = M_EMPTY, .exp_session = X(IDN_SESSION_FREE)),
    /* Data after abort opens a new connection on a closed channel */
    RT(.msg = M_WAVE(0, 0), .exp_session = X(IDN_SESSION_ACTIVE), .exp_iev = X(IDN_IEV_NEW | IDN_IEV_CCERR)),
    END
};

static const step_t s_link_timeout[] = {
    PLAYING,
    RT(.now = 500000, .cmd = IDN_CMD_RT_CHANNEL, .msg = M_EMPTY),
    TICK(.now = 1499999, .exp_session = X(IDN_SESSION_ACTIVE)),
    TICK(.now = 1500000, .exp_session = X(IDN_SESSION_FREE), .exp_ch = X(IDN_CHANNEL_CLOSED),
         .exp_reason = X(IDN_STOP_TIMEOUT), .d_release = 1),
    END
};

static const uint8_t k_group_get[16] = { 16, 0x01 };

static const step_t s_management_while_streaming[] = {
    PLAYING,
    GROUP(k_group_get, .exp_result = X(IDN_GROUP_OK), .exp_mask = X(0xFFFF), .exp_session = X(IDN_SESSION_ACTIVE)),
    RT(.msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_iev = X(0), .d_enqueue = 1),
    END
};

/* Further cases from the plan's test list. */

static const step_t s_open_data_close_one_message[] = {
    RT(.msg = { .present = true, .ch = 9, .chunk = IDN_CHUNK_WAVE, .cfg = true, .cfl = ROUTING(0) | CLOSE,
                .sid = 1, .mode = 1, .scwc = 4, .nsamples = 20 },
       .exp_session = X(IDN_SESSION_ACTIVE), .exp_ch = X(IDN_CHANNEL_CLOSED),
       .exp_pev = X(IDN_PEV_CNLRTED | IDN_PEV_CNLCLSD), .exp_status = X(0),
       .exp_reason = X(IDN_STOP_CHANNEL_CLOSE), OPENS, .d_release = 1),
    END
};

static const step_t s_repeated_config[] = {
    PLAYING,
    RT(.msg = M_ROUTE_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(0), .d_enqueue = 1),
    RT(.msg = M_ROUTE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(0)),
    /* A changed, valid dictionary rebuilds the decoder but keeps the route */
    RT(.msg = M_ROUTE_ALT(0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(0), .d_compile = 1),
    END
};

static const step_t s_failed_replacement[] = {
    PLAYING,
    RT(.msg = M_ROUTE_BAD(0), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_CFGERR),
       .exp_reason = X(IDN_STOP_ROUTE_CHANGE), .d_compile = 1, .d_release = 1),
    RT(.msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_CFGERR)),
    /* The voided channel no longer blocks another channel */
    RT(.msg = M_ROUTE_WAVE(1, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(IDN_PEV_CNLRTED), OPENS),
    END
};

static const uint8_t k_group_set_no_g3[16] = { 16, 0x02, 0xFF, 0xF7 };
static const uint8_t k_group_set_bad_auth[16] = { 16, 0x02, 0xFF, 0xFF, 'x' };
static const uint8_t k_group_bad_op[16] = { 16, 0x07 };
static const uint8_t k_group_short[4] = { 4, 0x01 };

static const step_t s_group_exclusion[] = {
    RT(.group = 3, .msg = M_ROUTE_WAVE(0, 0), OPENS),
    GROUP(k_group_set_bad_auth, .exp_result = X(IDN_GROUP_AUTH_ERROR), .exp_mask = X(0xFFFF),
          .exp_session = X(IDN_SESSION_ACTIVE)),
    GROUP(k_group_bad_op, .exp_result = X(IDN_GROUP_UNSUPPORTED)),
    GROUP(k_group_short, .exp_result = X(IDN_GROUP_INVALID)),
    GROUP(k_group_set_no_g3, .exp_result = X(IDN_GROUP_OK), .exp_mask = X(0xFFF7), .exp_session = X(IDN_SESSION_FREE),
          .exp_reason = X(IDN_STOP_EXCLUDED), .d_release = 1),
    RT(.group = 3, .msg = M_ROUTE_WAVE(0, 0), .exp_result = X(IDN_RESULT_EXCLUDED), .exp_session = X(IDN_SESSION_FREE)),
    RT(.group = 0, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OK), .exp_session = X(IDN_SESSION_ACTIVE)),
    /* An excluded group on the live tuple is not processed either */
    RT(.group = 3, .msg = M_ROUTE(0, 0), .exp_result = X(IDN_RESULT_EXCLUDED)),
    END
};

static const step_t s_repeated_close[] = {
    PLAYING,
    RT(.cmd = CLOSEACK, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OK), .exp_session = X(IDN_SESSION_FREE),
       .exp_status = X(0), .exp_reason = X(IDN_STOP_GRACEFUL), .d_release = 1),
    RT(.cmd = CLOSEACK, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_EMPTY_CLOSE), .exp_session = X(IDN_SESSION_FREE)),
    END
};

static const step_t s_close_with_message[] = {
    RT(.cmd = CLOSEACK, .msg = M_ROUTE_WAVE(0, 0), .exp_result = X(IDN_RESULT_OK), .exp_session = X(IDN_SESSION_FREE),
       .exp_iev = X(IDN_IEV_NEW), .exp_pev = X(IDN_PEV_CNLRTED | IDN_PEV_CNLCLSD), .exp_status = X(0),
       .exp_reason = X(IDN_STOP_GRACEFUL), OPENS, .d_release = 1),
    END
};

static const step_t s_two_interfaces[] = {
    RT(.ep = EP_A_IF1, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OK)),
    RT(.ep = EP_A_IF2, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OCCUPIED)),
    RT(.ep = EP_A_IF1, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OK), .exp_iev = X(0)),
    END
};

static const step_t s_malformed[] = {
    /* Declared size one more than received: no session */
    RT(.msg = { .present = true, .size_delta = 1 }, .exp_result = X(IDN_RESULT_INVALID),
       .exp_session = X(IDN_SESSION_FREE)),
    /* Trailing bytes beyond the declared size */
    RT(.msg = { .present = true, .chunk = IDN_CHUNK_WAVE, .nsamples = 20, .size_delta = -8 },
       .exp_result = X(IDN_RESULT_INVALID), .exp_session = X(IDN_SESSION_FREE)),
    /* SCWC set without Routing */
    RT(.msg = { .present = true, .cfg = true, .scwc = 4 }, .exp_result = X(IDN_RESULT_INVALID),
       .exp_session = X(IDN_SESSION_FREE)),
    /* Not a channel message: well framed, reaches the session, MVErr */
    RT(.msg = { .present = true, .not_channel_msg = true }, .exp_result = X(IDN_RESULT_OK),
       .exp_iev = X(IDN_IEV_NEW | IDN_IEV_MVERR)),
    /* A malformed packet does not refresh the link */
    RT(.now = 900000, .msg = { .present = true, .size_delta = 1 }, .exp_result = X(IDN_RESULT_INVALID)),
    TICK(.now = 1000000, .exp_session = X(IDN_SESSION_FREE)),
    END
};

static const step_t s_release_deadline_fault[] = {
    SET(.set_release_ok = X(0), .set_quiescent = X(0)),
    PLAYING,
    RT(.cmd = ABORT, .msg = M_EMPTY, .exp_session = X(IDN_SESSION_FAULTED), .d_release = 1),
    RT(.ep = EP_B, .msg = M_EMPTY, .exp_result = X(IDN_RESULT_OCCUPIED), .exp_session = X(IDN_SESSION_FAULTED)),
    TICK(.now = 2000000, .exp_session = X(IDN_SESSION_FAULTED)),
    SET(.set_quiescent = X(1), .set_release_ok = X(1)),
    TICK(.now = 2100000, .exp_session = X(IDN_SESSION_FREE)),
    RT(.now = 2100000, .ep = EP_B, .msg = M_ROUTE_WAVE(0, 0), .exp_result = X(IDN_RESULT_OK),
       .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), OPENS),
    END
};

static const step_t s_output_busy_elsewhere[] = {
    SET(.set_acquire_ok = X(0)),
    RT(.msg = M_ROUTE_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE),
       .exp_pev = X(IDN_PEV_CNLRTED | IDN_PEV_BSYERR), .d_compile = 1, .d_acquire = 1),
    SET(.set_acquire_ok = X(1)),
    RT(.msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(0), .d_acquire = 1, .d_enqueue = 1),
    END
};

static const step_t s_void_mode[] = {
    RT(.msg = M_ROUTE_SVC(6, 0, IDN_MODE_VOID), .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE), .exp_pev = X(IDN_PEV_CNLRTED)),
    RT(.msg = M_WAVE(6, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE), .exp_iev = X(0), .exp_pev = X(0)),
    /* A Void-mode channel does not hold the laser */
    RT(.msg = M_ROUTE_WAVE(7, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), OPENS),
    /* Rerouting the laser channel to Void mode stops its output */
    RT(.msg = M_ROUTE_SVC(7, 1, IDN_MODE_VOID), .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE), .exp_pev = X(IDN_PEV_CNLRTED),
       .exp_reason = X(IDN_STOP_ROUTE_CHANGE), .d_release = 1),
    END
};

static const step_t s_default_service_id[] = {
    RT(.msg = M_ROUTE_SVC(0, 0, IDN_MODE_GRAPHIC_CONTINUOUS), .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE),
       .exp_pev = X(IDN_PEV_CNLRTED), .d_compile = 1),
    END
};

static const step_t s_wrong_chunk_type[] = {
    RT(.msg = M_ROUTE(0, 0), .d_compile = 1),
    /* Frame samples on a continuous route */
    RT(.msg = { .present = true, .chunk = 0x02 }, .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE), .exp_pev = X(IDN_PEV_CKTERR)),
    /* Under-minimum wave chunk: rejected before any output is taken */
    RT(.msg = { .present = true, .chunk = IDN_CHUNK_WAVE, .nsamples = 19 }, .exp_ch = X(IDN_CHANNEL_ROUTED_IDLE),
       .exp_pev = X(IDN_PEV_MCLERR)),
    END
};

static const step_t s_user_cancel[] = {
    PLAYING,
    CANCEL(.exp_session = X(IDN_SESSION_FREE), .exp_reason = X(IDN_STOP_CANCEL), .d_release = 1),
    CANCEL(.exp_session = X(IDN_SESSION_FREE)),
    END
};

static const step_t s_truncated_config[] = {
    /* From a new link: malformed, nothing allocated */
    RT(.msg = M_ROUTE_TRUNC(0), .exp_result = X(IDN_RESULT_INVALID), .exp_session = X(IDN_SESSION_FREE)),
    PLAYING,
    /* From another port: no effect on the live channel */
    RT(.ep = EP_A_PORT2, .msg = M_ROUTE_TRUNC(0), .exp_result = X(IDN_RESULT_INVALID),
       .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE)),
    /* From the connection: the decoder is void until a whole configuration arrives */
    RT(.now = 900000, .msg = M_ROUTE_TRUNC(0), .exp_result = X(IDN_RESULT_INVALID), .exp_ch = X(IDN_CHANNEL_VOIDED),
       .exp_reason = X(IDN_STOP_ROUTE_CHANGE), .d_release = 1),
    RT(.now = 900000, .msg = M_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_VOIDED), .exp_pev = X(IDN_PEV_CFGERR)),
    RT(.now = 900000, .msg = M_ROUTE_WAVE(0, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE), .exp_pev = X(IDN_PEV_CNLRTED),
       OPENS),
    /* The truncated packet did not refresh the link; the later valid ones did */
    TICK(.now = 1899999, .exp_session = X(IDN_SESSION_ACTIVE)),
    END
};

/* All 64 channel IDs route, play and close through the one physical route (filled in by main). */
static step_t s_all_channel_ids[2 * IDN_SESSION_CHANNELS + 1];

static void fill_all_channel_ids(void)
{
    for (uint8_t ch = 0; ch < IDN_SESSION_CHANNELS; ch++) {
        const step_t open = RT(.msg = M_ROUTE_WAVE(ch, 0), .exp_ch = X(IDN_CHANNEL_ROUTED_ACTIVE),
                               .exp_pev = X(IDN_PEV_CNLRTED), OPENS);
        const step_t close = RT(.msg = M_CLOSE_CH(ch), .exp_session = X(IDN_SESSION_ACTIVE),
                                .exp_ch = X(IDN_CHANNEL_CLOSED), .d_release = 1);
        s_all_channel_ids[2 * ch] = open;
        s_all_channel_ids[2 * ch + 1] = close;
    }
}

static const scenario_t k_scenarios[] = {
    { "empty RT allocates session only",            s_empty_allocates_only },
    { "routing + Void chunk -> ROUTED_IDLE",         s_route_void_idle },
    { "first matching wave chunk acquires output",   s_first_wave_acquires },
    { "data on CLOSED channel: no auto-open, CCErr", s_no_auto_open },
    { "second continuous channel -> BSYErr",         s_second_channel_busy },
    { "unknown service/mode -> SMErr",               s_unknown_service_mode },
    { "SCM != SDM voids decoder, DCMErr",            s_scm_mismatch },
    { "last channel close keeps connection",         s_last_channel_close },
    { "empty close without connection -> 0xEB",      s_empty_close_no_connection },
    { "same IP, other port is a distinct link",      s_same_ip_other_port },
    { "abort cancels output and closes",             s_abort },
    { "one-second link timeout",                     s_link_timeout },
    { "management request while streaming",          s_management_while_streaming },
    { "open + data + close in one message",          s_open_data_close_one_message },
    { "repeated identical configuration",            s_repeated_config },
    { "failed decoder replacement",                  s_failed_replacement },
    { "client group exclusion",                      s_group_exclusion },
    { "repeated close",                              s_repeated_close },
    { "close command carrying a message",            s_close_with_message },
    { "two destination interfaces",                  s_two_interfaces },
    { "malformed packets",                           s_malformed },
    { "release deadline -> FAULTED until quiescent", s_release_deadline_fault },
    { "output held by another producer",             s_output_busy_elsewhere },
    { "Void service mode",                           s_void_mode },
    { "service ID 0 selects default",                s_default_service_id },
    { "chunk type and minimum length",               s_wrong_chunk_type },
    { "local cancel (web Stop)",                     s_user_cancel },
    { "truncated configuration voids the decoder",   s_truncated_config },
    { "all 64 channel identifiers",                  s_all_channel_ids },
};

/* -- Runner ----------------------------------------------------------------- */

#define CHECK(cond, ...) do {                                         \
        if (!(cond)) {                                                \
            printf("  FAIL %s step %d: ", name, step_no);             \
            printf(__VA_ARGS__);                                      \
            printf("\n");                                             \
            failed = true;                                            \
        }                                                             \
    } while (0)

/* Checks `actual` against the step's X()-encoded expectation, if it has one. */
#define EXPECT(field, actual, what) do {                                                  \
        if (st->field)                                                                    \
            CHECK((unsigned)(actual) == (unsigned)(st->field - 1), "%s 0x%X, want 0x%X",  \
                  what, (unsigned)(actual), (unsigned)(st->field - 1));                   \
    } while (0)

/* The output actions a step caused, always checked. */
#define DELTA(n) CHECK(g_fake.n##s - before.n##s == st->d_##n, #n "s +%d, want +%d", \
                       g_fake.n##s - before.n##s, st->d_##n)

static bool run_scenario(const scenario_t *sc)
{
    static idn_session_t s;
    static uint8_t buf[0x10000];
    const char *name = sc->name;
    bool failed = false;

    memset(&g_fake, 0, sizeof(g_fake));
    g_fake.acquire_ok = g_fake.release_ok = g_fake.quiescent = true;
    idn_session_init(&s, &k_ops);

    int step_no = 0;
    for (const step_t *st = sc->steps; st->kind != EV_END; st++, step_no++) {
        fake_t before = g_fake;
        idn_rt_outcome_t out;
        uint8_t resp[IDN_GROUP_RESP_BYTES] = { 0 };
        bool have_out = false;

        switch (st->kind) {
            case EV_RT: {
                size_t n = build_msg(&st->msg, buf);
                idn_session_rt(&s, st->now, &k_ep[st->ep], st->cmd ? st->cmd : IDN_CMD_RT_CHANNEL_ACK, st->group,
                               (uint16_t)step_no, buf, n, &out);
                have_out = true;
                break;
            }
            case EV_TICK:
                idn_session_tick(&s, st->now);
                break;
            case EV_GROUP:
                idn_session_group_request(&s, st->raw, st->raw_len, resp);
                CHECK(resp[0] == IDN_GROUP_RESP_BYTES, "group response size %u", resp[0]);
                EXPECT(exp_result, resp[1], "group code");
                EXPECT(exp_mask, resp[2] << 8 | resp[3], "mask");
                break;
            case EV_CANCEL:
                idn_session_cancel(&s, IDN_STOP_CANCEL);
                break;
            case EV_SET:
                if (st->set_acquire_ok) g_fake.acquire_ok = st->set_acquire_ok - 1;
                if (st->set_release_ok) g_fake.release_ok = st->set_release_ok - 1;
                if (st->set_quiescent)  g_fake.quiescent = st->set_quiescent - 1;
                continue;
            default:
                break;
        }

        if (have_out) {
            if (st->exp_iev || st->exp_pev || st->exp_status) {
                CHECK(out.send_ack, "no ACK produced");
            }
            EXPECT(exp_result, out.result, "result");
            EXPECT(exp_iev, out.ack.input_events, "input events");
            EXPECT(exp_pev, out.ack.pipeline_events, "pipeline events");
            EXPECT(exp_status, out.ack.status, "status");
        }
        EXPECT(exp_session, s.state, "session state");
        EXPECT(exp_ch, s.ch[st->msg.ch & 0x3F].state, "channel state");
        EXPECT(exp_reason, g_fake.last_reason, "release reason");
        DELTA(compile);
        DELTA(acquire);
        DELTA(enqueue);
        DELTA(release);
        CHECK(g_fake.violations == 0, "ownership contract violated (%d)", g_fake.violations);

        /* Invariants after every step */
        CHECK(s.output_held == g_fake.held, "session/output hold disagree");
        if (s.state != IDN_SESSION_ACTIVE) {
            CHECK(!s.output_held, "output held without an active connection");
        }
        int routed_laser = 0;
        for (int i = 0; i < IDN_SESSION_CHANNELS; i++) {
            if (s.ch[i].route == IDN_ROUTE_LASER_CONTINUOUS) routed_laser++;
            if (s.ch[i].state == IDN_CHANNEL_ROUTED_ACTIVE) {
                CHECK(s.output_held && s.laser_channel == i, "channel %d active without output", i);
            }
        }
        CHECK(routed_laser <= 1, "%d channels hold the exclusive route", routed_laser);
    }

    return !failed;
}

int main(void)
{
    int failures = 0;

    fill_all_channel_ids();
    for (size_t i = 0; i < N(k_scenarios); i++) {
        bool ok = run_scenario(&k_scenarios[i]);
        printf("%s  %s\n", ok ? "ok  " : "FAIL", k_scenarios[i].name);
        failures += ok ? 0 : 1;
    }

    printf("\n%d/%d passed\n", (int)N(k_scenarios) - failures, (int)N(k_scenarios));
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}

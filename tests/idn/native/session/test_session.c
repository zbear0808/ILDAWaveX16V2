/*
 * Host tests for firmware/ESP32S3/main/idn_session.c, from plan 02's
 * transition table: only a routed, accepted configuration produces
 * samples, one channel owns the output at a time, and every way a
 * connection ends releases it.
 *
 * A scenario is a list of steps, one line each: an input (an RT packet,
 * a clock tick, a client group request, a local cancel) and what it must
 * cause. A step names only the fields it changes. Expectations it leaves
 * out are not checked, except the output actions (compiles, acquires,
 * enqueues, releases), which are zero unless given. The fake output also
 * checks the ownership contract after every step: one holder, a newer
 * generation for every acquire, enqueue and release only by the holder.
 */

#include <stdio.h>
#include <string.h>

#include "idn_session.h"

#define N(a) (sizeof(a) / sizeof((a)[0]))

/* -- Fake decoder and output ------------------------------------------------ */

static struct {
    int compiles, acquires, enqueues, releases, violations;
    bool acquire_ok, release_ok, quiescent;     /* what the next calls return */
    bool held, decoder[IDN_SESSION_CHANNELS];
    uint32_t held_gen, last_gen;
    idn_stop_reason_t last_reason;
} g;

static idn_cfg_result_t fake_compile(void *ctx, uint8_t ch, uint8_t mode, const uint8_t *cfg, uint16_t len)
{
    (void)ctx;
    g.compiles++;
    if (mode != IDN_MODE_GRAPHIC_CONTINUOUS) g.violations++;
    for (uint16_t i = 0; i + 1 < len; i += 2)
        if (cfg[i] == 0xFF && cfg[i + 1] == 0xFF) return IDN_CFG_INVALID;   /* tag 0xFFFF: rejected */
    g.decoder[ch] = true;
    return IDN_CFG_OK;
}

static void fake_discard(void *ctx, uint8_t ch)
{
    (void)ctx;
    g.decoder[ch] = false;
}

static idn_chunk_result_t fake_check(void *ctx, uint8_t ch, const uint8_t *chunk, uint16_t len)
{
    (void)ctx;
    (void)chunk;
    if (!g.decoder[ch]) g.violations++;
    return len >= 4 + 20 * 8 && (len - 4) % 8 == 0 ? IDN_CHUNK_OK : IDN_CHUNK_TOO_SHORT;
}

static bool fake_acquire(void *ctx, uint32_t gen)
{
    (void)ctx;
    g.acquires++;
    if (g.held || gen <= g.last_gen) g.violations++;
    if (!g.acquire_ok) return false;
    g.held = true;
    g.held_gen = g.last_gen = gen;
    return true;
}

static idn_chunk_result_t fake_enqueue(void *ctx, uint8_t ch, uint32_t gen, const uint8_t *chunk, uint16_t len)
{
    (void)ctx;
    (void)chunk;
    (void)len;
    g.enqueues++;
    if (!g.held || gen != g.held_gen || !g.decoder[ch]) g.violations++;
    return IDN_CHUNK_OK;
}

static bool fake_release(void *ctx, uint32_t gen, idn_stop_reason_t reason)
{
    (void)ctx;
    g.releases++;
    if (!g.held || gen != g.held_gen) g.violations++;
    g.held = false;
    g.last_reason = reason;
    return g.release_ok;
}

static bool fake_quiescent(void *ctx)
{
    (void)ctx;
    return g.quiescent;
}

static const idn_session_ops_t k_ops = {
    .config_compile = fake_compile, .config_discard = fake_discard, .wave_check = fake_check,
    .output_acquire = fake_acquire, .wave_enqueue = fake_enqueue, .output_release = fake_release,
    .output_quiescent = fake_quiescent,
};

/* -- Messages: a Void chunk on channel 0 plus what a step adds --------------- */

typedef struct {
    bool    empty;              /* RT packet without a message */
    uint8_t ch;
    bool    route;              /* configuration with Routing and the XYRGBI dictionary (Stream 2.2) */
    bool    close;              /* configuration with Close */
    uint8_t sdm, sid, mode;
    uint8_t samples;            /* wave chunk of 8-octet samples; 0: Void chunk */
    uint8_t scm;
    uint8_t chunk;              /* another chunk type */
    bool    bad_dict;           /* a dictionary the decoder rejects */
    bool    alt_dict;           /* another valid dictionary */
    bool    words_only;         /* dictionary words without Routing: malformed */
    uint8_t scwc_extra;         /* declares more configuration words than it sends */
    int     size_delta;         /* declared total size minus the real one */
    bool    not_channel_msg;
} msg_t;

#define EMPTY       { .empty = true }
#define VOID(...)   { .samples = 0, __VA_ARGS__ }
#define WAVE(...)   { .samples = 20, __VA_ARGS__ }
#define ROUTE(...)  { .route = true, .sid = 1, .mode = IDN_MODE_GRAPHIC_CONTINUOUS, __VA_ARGS__ }
#define OPEN(...)   ROUTE(.samples = 20, __VA_ARGS__)
#define CLOSE_CH(...) { .close = true, __VA_ARGS__ }

static size_t build(const msg_t *m, uint8_t *buf)
{
    static const uint8_t dict[16] = { 0x42,0x00, 0x40,0x10, 0x42,0x10, 0x40,0x10, 0x52,0x7E, 0x52,0x14, 0x51,0xCC, 0x5C,0x10 };
    const bool cfg = m->route || m->close || m->words_only, words = m->route || m->words_only;
    size_t n = 8;

    if (m->empty) return 0;
    buf[2] = (uint8_t)((m->not_channel_msg ? 0 : 0x80) | (cfg ? 0x40 : 0) | m->ch);
    buf[3] = m->chunk ? m->chunk : m->samples ? IDN_CHUNK_WAVE : IDN_CHUNK_VOID;
    memcpy(buf + 4, "\x00\x01\x00\x00", 4);                 /* timestamp */
    if (cfg) {
        buf[n++] = (uint8_t)((words ? 4 : 0) + m->scwc_extra);
        buf[n++] = (uint8_t)(m->sdm << 4 | (m->close ? 0x02 : 0) | (m->route ? 0x01 : 0));
        buf[n++] = m->sid;
        buf[n++] = m->mode;
        if (words) {
            memcpy(buf + n, dict, sizeof(dict));
            if (m->bad_dict) buf[n] = buf[n + 1] = 0xFF;
            if (m->alt_dict) buf[n + 14] = buf[n + 15] = 0x00;
            n += sizeof(dict);
        }
    }
    if (m->samples) {
        uint32_t duration = m->samples * 33u;
        buf[n++] = (uint8_t)(m->scm << 4);
        buf[n++] = (uint8_t)(duration >> 16);
        buf[n++] = (uint8_t)(duration >> 8);
        buf[n++] = (uint8_t)duration;
        memset(buf + n, 0, m->samples * 8u);
        n += m->samples * 8u;
    }
    buf[0] = (uint8_t)((n + m->size_delta) >> 8);
    buf[1] = (uint8_t)(n + m->size_delta);
    return n;
}

/* -- Steps ------------------------------------------------------------------ */

enum { EP_A, EP_A_PORT2, EP_B, EP_A_OTHER_IF };   /* senders: host A, its second port, host B, A on another interface */

static const idn_endpoint_t k_ep[] = {
    [EP_A]          = { 0x0A000002u, 5000, 0 },
    [EP_A_PORT2]    = { 0x0A000002u, 5001, 0 },
    [EP_B]          = { 0x0A000003u, 5000, 0 },
    [EP_A_OTHER_IF] = { 0x0A000002u, 5000, 0x0A000001u },
};

typedef enum { EV_END = 0, EV_RT, EV_TICK, EV_GROUP, EV_CANCEL, EV_SET } ev_kind_t;

#define ANY (-1)                /* not checked; for EV_SET, unchanged */

typedef struct {
    ev_kind_t kind;
    int64_t   now;              /* us */
    int       from;             /* sender */
    uint8_t   cmd, group;
    msg_t     msg;
    const uint8_t *req;         /* EV_GROUP payload */
    size_t    req_len;
    int       acquire_ok, release_ok, quiescent;     /* EV_SET: what the fake output returns from now on */

    int result, session, state, iev, pev, status, mask, reason;   /* expected; state is the message's channel */
    int compiles, acquires, enqueues, releases;                   /* output actions caused, always checked */
} step_t;

#define DEFAULTS    .result = ANY, .session = ANY, .state = ANY, .iev = ANY, .pev = ANY, .status = ANY, \
                    .mask = ANY, .reason = ANY, .acquire_ok = ANY, .release_ok = ANY, .quiescent = ANY
#define RT(...)     { .kind = EV_RT, .cmd = IDN_CMD_RT_CHANNEL_ACK, DEFAULTS, __VA_ARGS__ }
#define TICK(...)   { .kind = EV_TICK, DEFAULTS, __VA_ARGS__ }
#define GROUP(p, ...) { .kind = EV_GROUP, .req = (p), .req_len = sizeof(p), DEFAULTS, __VA_ARGS__ }
#define CANCEL(...) { .kind = EV_CANCEL, DEFAULTS, __VA_ARGS__ }
#define SET(...)    { .kind = EV_SET, DEFAULTS, __VA_ARGS__ }
#define END         { .kind = EV_END }

#define CLOSEACK    IDN_CMD_RT_CLOSE_ACK
#define ABORT       IDN_CMD_RT_ABORT
#define FREE        IDN_SESSION_FREE
#define ACTIVE      IDN_SESSION_ACTIVE
#define FAULTED     IDN_SESSION_FAULTED
#define HELD        IDN_SESSION_HELD
#define CLOSED      IDN_CHANNEL_CLOSED
#define VOIDED      IDN_CHANNEL_VOIDED
#define IDLE        IDN_CHANNEL_ROUTED_IDLE
#define PLAYING     IDN_CHANNEL_ROUTED_ACTIVE
#define RES(x)      IDN_RESULT_##x
#define IEV(x)      IDN_IEV_##x
#define PEV(x)      IDN_PEV_##x
#define STOP(x)     IDN_STOP_##x
#define OPENS       .compiles = 1, .acquires = 1, .enqueues = 1      /* a channel's first routed data */
#define PLAY_CH0    RT(.msg = OPEN(), OPENS)

/* -- Scenarios -------------------------------------------------------------- */

static const step_t s_route_then_play[] = {
    RT(.msg = EMPTY, .result = RES(OK), .session = ACTIVE, .iev = IEV(NEW), .pev = 0, .status = 0),
    RT(.msg = ROUTE(.sid = 0), .state = IDLE, .iev = 0, .pev = PEV(CNLRTED), .status = IDN_STATUS_SOCNL, .compiles = 1),
    RT(.msg = WAVE(.chunk = 0x02), .state = IDLE, .pev = PEV(CKTERR)),       /* frame samples: wrong chunk type */
    RT(.msg = WAVE(.samples = 19), .state = IDLE, .pev = PEV(MCLERR)),       /* too few, before taking the output */
    RT(.msg = WAVE(), .state = PLAYING, .pev = 0, .acquires = 1, .enqueues = 1),
    RT(.msg = WAVE(), .state = PLAYING, .enqueues = 1),
    END
};

static const step_t s_no_auto_open[] = {
    RT(.msg = WAVE(.ch = 5), .state = CLOSED, .iev = IEV(NEW) | IEV(CCERR), .pev = PEV(CNLCLSD)),
    RT(.msg = VOID(.ch = 5), .iev = 0, .pev = PEV(CNLCLSD)),                 /* a Void message is not CCErr */
    END
};

static const step_t s_one_channel_plays[] = {
    PLAY_CH0,
    RT(.msg = OPEN(.ch = 1), .state = VOIDED, .pev = PEV(BSYERR) | PEV(CNLRTED)),
    RT(.msg = WAVE(.ch = 1), .state = VOIDED, .iev = 0, .pev = 0),           /* dropped, not CCErr */
    RT(.msg = WAVE(), .state = PLAYING, .enqueues = 1),
    END
};

static const step_t s_unsupported_service_or_mode[] = {
    RT(.msg = ROUTE(.ch = 2, .sid = 7), .state = VOIDED, .pev = PEV(SMERR) | PEV(CNLRTED), .status = IDN_STATUS_SOCNL),
    RT(.msg = WAVE(.ch = 2), .state = VOIDED, .iev = 0, .pev = 0),
    RT(.msg = ROUTE(.ch = 3, .mode = 2), .state = VOIDED, .pev = PEV(SMERR) | PEV(CNLRTED)),   /* discrete graphics */
    RT(.msg = ROUTE(.ch = 4, .sid = 0, .mode = 0xC0), .state = VOIDED, .pev = PEV(SMERR) | PEV(CNLRTED)),
    END
};

static const step_t s_void_mode[] = {
    RT(.msg = ROUTE(.ch = 6, .sid = 0, .mode = IDN_MODE_VOID), .state = IDLE, .pev = PEV(CNLRTED)),
    RT(.msg = WAVE(.ch = 6), .state = IDLE, .iev = 0, .pev = 0),
    RT(.msg = OPEN(.ch = 7), .state = PLAYING, OPENS),                       /* a Void-mode channel holds nothing */
    RT(.msg = ROUTE(.ch = 7, .mode = IDN_MODE_VOID), .state = IDLE, .pev = PEV(CNLRTED),
       .reason = STOP(ROUTE_CHANGE), .releases = 1),
    END
};

static const step_t s_scm_mismatch[] = {
    RT(.msg = OPEN(.sdm = 1, .scm = 1), OPENS),
    RT(.msg = WAVE(.scm = 2), .state = VOIDED, .pev = PEV(DCMERR), .reason = STOP(ROUTE_CHANGE), .releases = 1),
    RT(.msg = WAVE(.scm = 1), .state = VOIDED, .pev = PEV(CFGERR)),          /* the old decoder is gone */
    RT(.msg = ROUTE(.sdm = 2), .state = IDLE, .pev = PEV(CNLRTED), .compiles = 1),
    RT(.msg = WAVE(.scm = 2), .state = PLAYING, .pev = 0, .acquires = 1, .enqueues = 1),
    END
};

static const step_t s_configuration_changes[] = {
    PLAY_CH0,
    RT(.msg = OPEN(), .state = PLAYING, .pev = 0, .enqueues = 1),            /* an identical repeat changes nothing */
    RT(.msg = ROUTE(.alt_dict = true), .state = PLAYING, .pev = 0, .compiles = 1),
    RT(.msg = ROUTE(.bad_dict = true), .state = VOIDED, .pev = PEV(CFGERR), .reason = STOP(ROUTE_CHANGE),
       .compiles = 1, .releases = 1),
    RT(.msg = WAVE(), .state = VOIDED, .pev = PEV(CFGERR)),                  /* the old decoder is not reused */
    RT(.msg = OPEN(.ch = 1), .state = PLAYING, .pev = PEV(CNLRTED), OPENS),  /* and the voided channel blocks no one */
    END
};

static const step_t s_channel_close[] = {
    RT(.msg = OPEN(.ch = 9, .close = true), .session = ACTIVE, .state = CLOSED, .pev = PEV(CNLRTED) | PEV(CNLCLSD),
       .reason = STOP(CHANNEL_CLOSE), OPENS, .releases = 1),                 /* route, data and close in one message */
    RT(.msg = OPEN(), .state = PLAYING, .iev = 0, OPENS),
    RT(.msg = CLOSE_CH(), .result = RES(OK), .session = ACTIVE, .state = CLOSED, .pev = PEV(CNLCLSD), .status = 0,
       .reason = STOP(CHANNEL_CLOSE), .releases = 1),
    RT(.msg = EMPTY, .session = ACTIVE, .iev = 0),                           /* the connection stays */
    END
};

static const step_t s_connection_close[] = {
    RT(.cmd = CLOSEACK, .msg = EMPTY, .result = RES(EMPTY_CLOSE), .session = FREE),     /* 0xEB, nothing allocated */
    RT(.cmd = IDN_CMD_RT_CLOSE, .msg = EMPTY, .result = RES(EMPTY_CLOSE), .session = FREE),
    PLAY_CH0,
    RT(.cmd = CLOSEACK, .msg = EMPTY, .result = RES(OK), .session = FREE, .status = 0, .reason = STOP(GRACEFUL),
       .releases = 1),
    RT(.cmd = CLOSEACK, .msg = EMPTY, .result = RES(EMPTY_CLOSE), .session = FREE),
    RT(.cmd = CLOSEACK, .msg = OPEN(), .result = RES(OK), .session = FREE, .iev = IEV(NEW),
       .pev = PEV(CNLRTED) | PEV(CNLCLSD), .reason = STOP(GRACEFUL), OPENS, .releases = 1),   /* data, then close */
    END
};

static const step_t s_abort_and_cancel[] = {
    PLAY_CH0,
    RT(.cmd = ABORT, .msg = EMPTY, .session = FREE, .state = CLOSED, .reason = STOP(ABORT), .releases = 1),
    RT(.cmd = ABORT, .msg = EMPTY, .session = FREE),
    RT(.msg = WAVE(), .iev = IEV(NEW) | IEV(CCERR), .pev = PEV(CNLCLSD)),   /* a new connection: channel closed */
    PLAY_CH0,
    CANCEL(.session = HELD, .reason = STOP(CANCEL), .releases = 1),          /* web Stop holds the sender */
    CANCEL(.session = HELD),
    RT(.now = 500000, .msg = OPEN(), .result = RES(OK), .session = HELD, .state = CLOSED, .pev = PEV(CNLCLSD),
       .status = 0),                                                         /* routed data plays nothing */
    RT(.now = 500000, .from = EP_B, .msg = OPEN(), .result = RES(OCCUPIED), .session = HELD),
    TICK(.now = 1499999, .session = HELD),                                   /* its packets keep the hold */
    TICK(.now = 1500000, .session = FREE),                                   /* until it goes quiet */
    RT(.now = 1500000, .msg = OPEN(), .iev = IEV(NEW), .state = PLAYING, OPENS),
    CANCEL(.session = HELD, .releases = 1),
    RT(.now = 1500000, .cmd = CLOSEACK, .msg = EMPTY, .result = RES(OK), .session = FREE),   /* or closes */
    END
};

static const step_t s_connection_identity[] = {
    PLAY_CH0,
    RT(.from = EP_A_PORT2, .msg = OPEN(), .result = RES(OCCUPIED), .session = ACTIVE),
    RT(.from = EP_A_PORT2, .cmd = CLOSEACK, .msg = EMPTY, .result = RES(EMPTY_CLOSE), .session = ACTIVE),
    RT(.from = EP_A_PORT2, .cmd = ABORT, .msg = EMPTY, .session = ACTIVE),
    RT(.from = EP_B, .msg = EMPTY, .result = RES(OCCUPIED)),
    RT(.from = EP_A_OTHER_IF, .msg = EMPTY, .result = RES(OCCUPIED)),
    RT(.msg = WAVE(), .state = PLAYING, .iev = 0, .enqueues = 1),
    END
};

static const step_t s_link_timeout[] = {
    PLAY_CH0,
    RT(.now = 500000, .cmd = IDN_CMD_RT_CHANNEL, .msg = EMPTY),
    TICK(.now = 1499999, .session = ACTIVE),
    TICK(.now = 1500000, .session = FREE, .state = CLOSED, .reason = STOP(TIMEOUT), .releases = 1),
    END
};

static const step_t s_malformed[] = {
    RT(.msg = VOID(.size_delta = 1), .result = RES(INVALID), .session = FREE),     /* declared longer than sent */
    RT(.msg = WAVE(.size_delta = -8), .result = RES(INVALID), .session = FREE),    /* octets past the declared end */
    RT(.msg = VOID(.words_only = true), .result = RES(INVALID), .session = FREE),  /* nonzero SCWC without Routing */
    RT(.msg = ROUTE(.scwc_extra = 1), .result = RES(INVALID), .session = FREE),    /* configuration cut short */
    RT(.msg = VOID(.not_channel_msg = true), .result = RES(OK), .iev = IEV(NEW) | IEV(MVERR), .pev = 0),
    RT(.now = 900000, .msg = VOID(.size_delta = 1), .result = RES(INVALID)),       /* does not refresh the link */
    TICK(.now = 1000000, .session = FREE),
    END
};

static const step_t s_truncated_configuration[] = {
    PLAY_CH0,
    RT(.from = EP_A_PORT2, .msg = ROUTE(.scwc_extra = 1), .result = RES(INVALID), .state = PLAYING),   /* not ours */
    RT(.msg = ROUTE(.scwc_extra = 1), .result = RES(INVALID), .state = VOIDED, .reason = STOP(ROUTE_CHANGE),
       .releases = 1),
    RT(.msg = WAVE(), .state = VOIDED, .pev = PEV(CFGERR)),
    RT(.msg = OPEN(), .state = PLAYING, .pev = PEV(CNLRTED), OPENS),
    END
};

static const step_t s_release_deadline[] = {
    SET(.release_ok = 0, .quiescent = 0),
    PLAY_CH0,
    RT(.cmd = ABORT, .msg = EMPTY, .session = FAULTED, .releases = 1),
    RT(.from = EP_B, .msg = EMPTY, .result = RES(OCCUPIED), .session = FAULTED),
    TICK(.now = 2000000, .session = FAULTED),
    SET(.release_ok = 1, .quiescent = 1),
    TICK(.now = 2100000, .session = FREE),                                   /* only once the output has stopped */
    RT(.now = 2100000, .from = EP_B, .msg = OPEN(), .result = RES(OK), .state = PLAYING, OPENS),
    END
};

static const step_t s_output_busy[] = {
    SET(.acquire_ok = 0),
    RT(.msg = OPEN(), .state = IDLE, .pev = PEV(CNLRTED) | PEV(BSYERR), .compiles = 1, .acquires = 1),
    SET(.acquire_ok = 1),
    RT(.msg = WAVE(), .state = PLAYING, .pev = 0, .acquires = 1, .enqueues = 1),
    END
};

static const uint8_t k_group_get[16] = { 16, 0x01 };
static const uint8_t k_group_all_but_0[16] = { 16, 0x02, 0xFF, 0xFE };
static const uint8_t k_group_all_but_3[16] = { 16, 0x02, 0xFF, 0xF7 };
static const uint8_t k_group_bad_auth[16] = { 16, 0x02, 0xFF, 0xFF, 'x' };
static const uint8_t k_group_bad_op[16] = { 16, 0x07 };
static const uint8_t k_group_short[4] = { 4, 0x01 };

static const step_t s_client_groups[] = {
    RT(.group = 3, .msg = OPEN(), OPENS),
    GROUP(k_group_get, .result = IDN_GROUP_OK, .mask = 0xFFFF, .session = ACTIVE),
    RT(.group = 3, .msg = WAVE(), .state = PLAYING, .enqueues = 1),
    GROUP(k_group_all_but_0, .result = IDN_GROUP_OK, .mask = 0xFFFE, .session = ACTIVE),   /* group 0 is unused */
    GROUP(k_group_bad_auth, .result = IDN_GROUP_AUTH_ERROR, .mask = 0xFFFE),
    GROUP(k_group_bad_op, .result = IDN_GROUP_UNSUPPORTED),
    GROUP(k_group_short, .result = IDN_GROUP_INVALID),
    GROUP(k_group_all_but_3, .result = IDN_GROUP_OK, .mask = 0xFFF7, .session = FREE, .reason = STOP(EXCLUDED),
          .releases = 1),
    RT(.group = 3, .msg = OPEN(), .result = RES(EXCLUDED), .session = FREE),
    RT(.group = 0, .msg = EMPTY, .result = RES(OK), .session = ACTIVE),
    RT(.group = 3, .msg = ROUTE(), .result = RES(EXCLUDED)),                 /* even on the live connection */
    END
};

static step_t s_all_channels[2 * IDN_SESSION_CHANNELS + 1];   /* each channel routes, plays and closes */

static void fill_all_channels(void)
{
    for (uint8_t ch = 0; ch < IDN_SESSION_CHANNELS; ch++) {
        const step_t open = RT(.msg = OPEN(.ch = ch), .state = PLAYING, .pev = PEV(CNLRTED), OPENS);
        const step_t close = RT(.msg = CLOSE_CH(.ch = ch), .session = ACTIVE, .state = CLOSED, .releases = 1);
        s_all_channels[2 * ch] = open;
        s_all_channels[2 * ch + 1] = close;
    }
}

/* -- Runner ----------------------------------------------------------------- */

static bool s_failed;

static void expect(const char *scenario, int step, const char *what, int got, int want)
{
    if (want != ANY && got != want) {
        printf("  FAIL %s, step %d: %s is 0x%X, expected 0x%X\n", scenario, step, what, (unsigned)got, (unsigned)want);
        s_failed = true;
    }
}

static bool run(const char *name, const step_t *steps)
{
    static idn_session_t s;
    static uint8_t buf[0x10000];

    memset(&g, 0, sizeof(g));
    g.acquire_ok = g.release_ok = g.quiescent = true;
    idn_session_init(&s, &k_ops);
    s_failed = false;

    for (int k = 0; steps[k].kind != EV_END; k++) {
        const step_t *st = &steps[k];
        const int compiles = g.compiles, acquires = g.acquires, enqueues = g.enqueues, releases = g.releases;
        idn_rt_outcome_t out;
        uint8_t resp[IDN_GROUP_RESP_BYTES] = { 0 };

        switch (st->kind) {
            case EV_RT:
                idn_session_rt(&s, st->now, &k_ep[st->from], st->cmd, st->group, (uint16_t)k, buf, build(&st->msg, buf),
                               &out);
                if (st->iev != ANY || st->pev != ANY || st->status != ANY)
                    expect(name, k, "ACK sent", out.send_ack, true);
                expect(name, k, "result", out.result, st->result);
                expect(name, k, "input events", out.ack.input_events, st->iev);
                expect(name, k, "pipeline events", out.ack.pipeline_events, st->pev);
                expect(name, k, "status", out.ack.status, st->status);
                break;
            case EV_TICK:
                idn_session_tick(&s, st->now);
                break;
            case EV_GROUP:
                idn_session_group_request(&s, st->req, st->req_len, resp);
                expect(name, k, "group response size", resp[0], IDN_GROUP_RESP_BYTES);
                expect(name, k, "group result", resp[1], st->result);
                expect(name, k, "group mask", resp[2] << 8 | resp[3], st->mask);
                break;
            case EV_CANCEL:
                idn_session_cancel(&s, IDN_STOP_CANCEL);
                break;
            case EV_SET:
                if (st->acquire_ok != ANY) g.acquire_ok = st->acquire_ok;
                if (st->release_ok != ANY) g.release_ok = st->release_ok;
                if (st->quiescent != ANY) g.quiescent = st->quiescent;
                continue;
            default:
                break;
        }
        expect(name, k, "session state", s.state, st->session);
        expect(name, k, "channel state", s.ch[st->msg.ch].state, st->state);
        expect(name, k, "stop reason", g.last_reason, st->reason);
        expect(name, k, "compiles", g.compiles - compiles, st->compiles);
        expect(name, k, "acquires", g.acquires - acquires, st->acquires);
        expect(name, k, "enqueues", g.enqueues - enqueues, st->enqueues);
        expect(name, k, "releases", g.releases - releases, st->releases);

        /* After every step: the contract holds, and at most one channel, the playing one, owns the output. */
        expect(name, k, "ownership violations", g.violations, 0);
        expect(name, k, "session holds the output", s.output_held, g.held);
        if (s.state != IDN_SESSION_ACTIVE)
            expect(name, k, "output held without a connection", s.output_held, false);
        int routed = 0;
        for (int i = 0; i < IDN_SESSION_CHANNELS; i++) {
            routed += s.ch[i].route == IDN_ROUTE_LASER_CONTINUOUS;
            if (s.ch[i].state == IDN_CHANNEL_ROUTED_ACTIVE)
                expect(name, k, "playing channel owns the output", s.output_held && s.laser_channel == i, true);
        }
        expect(name, k, "at most one channel routed to the laser", routed <= 1, true);
    }
    return !s_failed;
}

int main(void)
{
    static const struct { const char *name; const step_t *steps; } scenarios[] = {
        { "route, refuse bad chunks, then play", s_route_then_play },
        { "data on a closed channel: no auto-open", s_no_auto_open },
        { "one continuous channel at a time", s_one_channel_plays },
        { "unsupported service or mode", s_unsupported_service_or_mode },
        { "Void service mode", s_void_mode },
        { "SCM differs from SDM", s_scm_mismatch },
        { "configuration repeated, changed, rejected", s_configuration_changes },
        { "channel close", s_channel_close },
        { "connection close", s_connection_close },
        { "abort and local cancel", s_abort_and_cancel },
        { "connection identity", s_connection_identity },
        { "one-second link timeout", s_link_timeout },
        { "malformed packets", s_malformed },
        { "truncated configuration", s_truncated_configuration },
        { "missed release deadline", s_release_deadline },
        { "output held by another producer", s_output_busy },
        { "client groups", s_client_groups },
        { "all 64 channel IDs", s_all_channels },
    };
    int failures = 0;

    fill_all_channels();
    for (size_t i = 0; i < N(scenarios); i++) {
        bool ok = run(scenarios[i].name, scenarios[i].steps);
        printf("%s %s\n", ok ? "ok  " : "FAIL", scenarios[i].name);
        failures += !ok;
    }
    printf("%d of %d scenarios passed\n", (int)N(scenarios) - failures, (int)N(scenarios));
    return failures != 0;
}

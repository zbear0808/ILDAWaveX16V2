/*
 * Host-native tests for firmware/ESP32S3/main/idn_timing.c. Expected values are literals
 * worked out from the policies in idn_timing.h, IDN-Hello (2022-03-27) 6 / 6.3.3 and
 * IDN-Stream rev002 2.1 / 6.1, never recomputed with the module's own code.
 */

#include <stdio.h>

#include "idn_timing.h"

#define N(a)           (sizeof(a) / sizeof((a)[0]))
#define EQ(a, b)       expect(__LINE__, -1, #a, (unsigned long long)(a), (unsigned long long)(b))
#define EQ_AT(i, a, b) expect(__LINE__, (long)(i), #a, (unsigned long long)(a), (unsigned long long)(b))

static int checks, failures;

static void expect(int line, long row, const char *what, unsigned long long got, unsigned long long want)
{
    checks++;
    if (got != want) {
        failures++;
        printf("FAIL line %d row %ld: %s = %llu (0x%llx), expected %llu (0x%llx)\n",
               line, row, what, got, got, want, want);
    }
}

#define T1  IDN_INEV_SEQ_NOT_NEXT
#define T12 (IDN_INEV_SEQ_NOT_NEXT | IDN_INEV_SEQ_DUPLICATE)
#define T13 (IDN_INEV_SEQ_NOT_NEXT | IDN_INEV_SEQ_MISSING)
#define S(seq, cls, events, missing) { seq, IDN_SEQ_##cls, events, missing }
/* Not a packet: observe IDN_SEQ_WINDOW consecutive numbers from seq, pushing every earlier one
   out of the window; `total` is then the missing count of the whole case. */
#define FLUSH(seq, total)            { seq, (idn_seq_class_t)-1, 0, total }

/* Observe `count` numbers from `first`, `stride` apart; returns how many were accepted. */
static uint32_t seq_accepted(idn_seq_tracker_t *t, uint32_t first, uint32_t count, uint32_t stride)
{
    uint32_t n = 0;
    for (uint32_t k = 0; k < count; k++)
        n += idn_seq_observe(t, (uint16_t)(first + k * stride)).accept ? 1u : 0u;
    return n;
}

static void test_seq(void)
{
    /* Each case starts at a FIRST step, on a fresh tracker. */
    static const struct { uint16_t seq; idn_seq_class_t cls; uint16_t events; uint32_t missing; } s[] = {
        /* 65535 -> 0 is ordinary +1. */
        S(65535, FIRST, 0, 0), S(0, NEXT, 0, 0),
        /* Duplicates of the newest and of an older number: dropped, SeqErr types 1+2. */
        S(10, FIRST, 0, 0), S(11, NEXT, 0, 0), S(11, DUPLICATE, T12, 0), S(10, DUPLICATE, T12, 0),
        /* A gap, then a delayed older packet: dropped as late and never counted missing. The holes
           102 and 104 are counted once, when they leave the window. */
        S(100, FIRST, 0, 0), S(101, NEXT, 0, 0), S(105, GAP, T1, 0), S(103, LATE, T1, 0),
        S(103, DUPLICATE, T12, 0), S(106, NEXT, 0, 0), FLUSH(107, 2),
        /* A gap across the wrap is small: 65531..65535, 0 and 1. */
        S(65530, FIRST, 0, 0), S(2, GAP, T1, 0), FLUSH(3, 7),
        /* A big jump is counted once in total: beyond the window at once, the rest as they leave. */
        S(0, FIRST, 0, 0), S(1000, GAP, T13, 999 - (IDN_SEQ_WINDOW - 1)), FLUSH(1001, 999),
        /* Exactly half the range ahead (0x8000) is older; 0x7FFF ahead is new. */
        S(0, FIRST, 0, 0), S(0x8000, STALE, T1, 0), S(0x7FFF, GAP, T13, 0x7FFE - (IDN_SEQ_WINDOW - 1)),
        /* Window edges: a number before the baseline is stale. After a jump of exactly the window
           the number just outside is stale although it was seen, and every skipped one is late. */
        S(99, FIRST, 0, 0), S(100, NEXT, 0, 0), S(98, STALE, T1, 0), S(100 + IDN_SEQ_WINDOW, GAP, T1, 0),
        S(100, STALE, T1, 0), S(101, LATE, T1, 0), S(99 + IDN_SEQ_WINDOW, LATE, T1, 0),
    };
    idn_seq_tracker_t t;
    uint32_t total = 0;

    idn_seq_reset(&t);
    for (size_t i = 0; i < N(s); i++) {
        if (s[i].cls == IDN_SEQ_FIRST) { idn_seq_reset(&t); total = 0; }
        if (s[i].cls == (idn_seq_class_t)-1) {
            for (uint32_t k = 0; k < IDN_SEQ_WINDOW; k++)
                total += idn_seq_observe(&t, (uint16_t)(s[i].seq + k)).missing;
            EQ_AT(i, total, s[i].missing);
            continue;
        }
        idn_seq_result_t r = idn_seq_observe(&t, s[i].seq);
        bool dropped = s[i].cls == IDN_SEQ_DUPLICATE || s[i].cls == IDN_SEQ_LATE || s[i].cls == IDN_SEQ_STALE;
        total += r.missing;
        EQ_AT(i, r.cls, s[i].cls);                  EQ_AT(i, r.accept, !dropped);
        EQ_AT(i, r.input_events, s[i].events);      EQ_AT(i, r.missing, s[i].missing);
    }

    /* Stale numbers re-baseline only as an unbroken run of IDN_SEQ_RESYNC_RUN counting up by one,
       as from a sender that restarted at 0: a duplicate or an accepted packet restarts the run,
       and scattered stale numbers never form one. */
    idn_seq_reset(&t);
    idn_seq_observe(&t, 1000);
    EQ(seq_accepted(&t, 0, IDN_SEQ_RESYNC_RUN, 1), 1);                    /* the last one */
    EQ(idn_seq_observe(&t, IDN_SEQ_RESYNC_RUN).cls, IDN_SEQ_NEXT);         /* is the new baseline */
    EQ(seq_accepted(&t, 40000, IDN_SEQ_RESYNC_RUN - 1, 1), 0);
    EQ(idn_seq_observe(&t, IDN_SEQ_RESYNC_RUN).cls, IDN_SEQ_DUPLICATE);
    EQ(seq_accepted(&t, 40000 + (IDN_SEQ_RESYNC_RUN - 1), IDN_SEQ_RESYNC_RUN - 1, 1), 0);
    EQ(idn_seq_observe(&t, IDN_SEQ_RESYNC_RUN + 1).cls, IDN_SEQ_NEXT);
    EQ(seq_accepted(&t, 40000 + 2 * (IDN_SEQ_RESYNC_RUN - 1), IDN_SEQ_RESYNC_RUN - 1, 1), 0);
    EQ(seq_accepted(&t, 50000, 2 * IDN_SEQ_RESYNC_RUN, 2), 0);
}

/* Reset, and accept a first message at wire_us; returns its unwrapped time. */
static uint64_t chrono_start(idn_chronology_t *c, uint32_t wire_us)
{
    uint64_t u = 0;
    idn_chronology_reset(c);
    EQ(idn_chronology_check(c, wire_us, &u), IDN_TS_FIRST);
    idn_chronology_commit(c, u);
    return u;
}

static void test_chronology(void)
{
    idn_chronology_t c;
    uint64_t u = 0, u0 = chrono_start(&c, 0xFFFFFFF0u);
    uint32_t ts = 0;

    /* Forward across the 32-bit wrap; equal is legal; earlier is BEFORE and, not committed,
       leaves the baseline alone. */
    EQ(idn_chronology_check(&c, 0x10u, &u), IDN_TS_FORWARD);
    EQ(u - u0, 0x20);
    idn_chronology_commit(&c, u);
    EQ(idn_chronology_check(&c, 0x10u, &u), IDN_TS_EQUAL);
    EQ(idn_chronology_check(&c, 0x0Fu, &u), IDN_TS_BEFORE);
    EQ(idn_chronology_check(&c, 0x10u, &u), IDN_TS_EQUAL);

    /* Half the range: from 0, 2^31 - 1 us ahead is forward and 2^31 is earlier. */
    chrono_start(&c, 0);
    EQ(idn_chronology_check(&c, 0x7FFFFFFFu, &u), IDN_TS_FORWARD);
    EQ(idn_chronology_check(&c, 0x80000000u, &u), IDN_TS_BEFORE);

    /* A restarted sender clock: IDN_TS_RESYNC_RUN earlier, non-decreasing timestamps (0, 0, 100,
       100, ...) re-base the timeline on the last of them. */
    chrono_start(&c, 5000000);
    for (uint32_t k = 0; k < IDN_TS_RESYNC_RUN; k++) {
        ts = 100u * (k / 2u);
        EQ_AT(k, idn_chronology_before(&c, ts, &u0), k + 1u == IDN_TS_RESYNC_RUN);
    }
    EQ(idn_chronology_check(&c, ts + 100u, &u), IDN_TS_FORWARD);
    EQ(u - u0, 100);
    EQ(idn_chronology_check(&c, ts - 1000u, &u), IDN_TS_BEFORE);     /* across the wire wrap */

    /* An accepted message restarts the count, and scattered earlier messages never form a run. */
    chrono_start(&c, 5000000);
    for (uint32_t k = 0; k < 2u * (IDN_TS_RESYNC_RUN - 1u); k++) {
        if (k == IDN_TS_RESYNC_RUN - 1u) {
            idn_chronology_check(&c, 5000001, &u);
            idn_chronology_commit(&c, u);
        }
        EQ_AT(k, idn_chronology_before(&c, 1000u + k, &u), false);
    }
    for (uint32_t k = 0; k < IDN_TS_RESYNC_RUN; k++)
        EQ_AT(k, idn_chronology_before(&c, 900u - k, &u), false);

    /* Sequel fragments (chunk types 0xC0..0xFF) carry no timestamp. */
    EQ(idn_chunk_has_timestamp(0x00), true);    EQ(idn_chunk_has_timestamp(0xBF), true);
    EQ(idn_chunk_has_timestamp(0xC0), false);   EQ(idn_chunk_has_timestamp(0xFF), false);
}

static void test_wave(void)
{
    static const bool cty_err[] = { [IDN_WAVE_GAP] = true, [IDN_WAVE_OVERLAP] = true,
                                    [IDN_WAVE_AWAIT_RECOVERY] = true };
    idn_chronology_t c;
    idn_wave_timeline_t w;
    uint64_t t0 = chrono_start(&c, 0xFFFFFFF0u), t1 = 0;

    /* 0xFFFFFFF0 plus 32 us continues at 0x10, across the wire wrap. */
    idn_chronology_check(&c, 0x10u, &t1);
    idn_wave_reset(&w);
    EQ(idn_wave_check(&w, t0), IDN_WAVE_START);
    idn_wave_commit(&w, t0, 32);
    EQ(idn_wave_check(&w, t1), IDN_WAVE_CONTINUES);
    idn_wave_commit(&w, t1, 2133);
    /* A repeated configuration on a continuous channel changes nothing. Only the exact end of
       the last chunk continues: its own timestamp again (as after a Void message) overlaps. */
    idn_wave_recover(&w);
    EQ(idn_wave_check(&w, t1 + 2133), IDN_WAVE_CONTINUES);
    EQ(idn_wave_check(&w, t1 + 2134), IDN_WAVE_GAP);
    EQ(idn_wave_check(&w, t1), IDN_WAVE_OVERLAP);
    /* After a discontinuity chunks are dropped, even an exact one, until a recovery boundary
       (an accepted configuration); the next one starts a new epoch. */
    idn_wave_break(&w);
    EQ(idn_wave_check(&w, t1 + 2133), IDN_WAVE_AWAIT_RECOVERY);
    idn_wave_recover(&w);
    EQ(idn_wave_check(&w, t1 + 9000), IDN_WAVE_RESTART);
    /* reset() (a new connection) forgets earlier epochs; end() closes one without error. */
    idn_wave_reset(&w);
    EQ(idn_wave_check(&w, 0), IDN_WAVE_START);
    idn_wave_commit(&w, 0, 10);
    idn_wave_end(&w);
    EQ(idn_wave_check(&w, 10), IDN_WAVE_RESTART);

    for (int v = IDN_WAVE_START; v <= IDN_WAVE_AWAIT_RECOVERY; v++)
        EQ_AT(v, idn_wave_verdict_is_error((idn_wave_verdict_t)v), cty_err[v]);
}

static const idn_rate_policy_t P = { 1000, 30000, 1 };     /* min 1 kHz, limit 30 kHz, 1 us */
#define D(...) ((const uint32_t[]){ __VA_ARGS__, 0 })
#define R(n, d, locked, verdict) { n, d, locked, IDN_RATE_##verdict }
#define LK(limit, n0, d0, n1, d1, hz, verdict) { limit, n0, d0, n1, d1, hz, IDN_RATE_##verdict }

/* As a caller does: check and commit up to `chunks` chunks of n samples, durations cycling
   through the zero-terminated d[], stopping at the first one rejected. Returns how many passed. */
static uint32_t feed(idn_rate_epoch_t *e, uint32_t n, const uint32_t *d, uint32_t chunks)
{
    for (uint32_t k = 0, i = 0; k < chunks; k++, i = d[i + 1] ? i + 1 : 0) {
        if (idn_rate_check(&P, e, n, d[i], 0) != IDN_RATE_OK) return k;
        idn_rate_commit(e, n, d[i]);
    }
    return chunks;
}

static void test_rate(void)
{
    /* One chunk on an empty epoch: samples, duration_us, locked output rate (0: none), verdict. */
    static const struct { uint32_t n, d, locked; idn_rate_verdict_t v; } one[] = {
        R(0, 2133, 0, INVALID), R(64, 0, 0, INVALID),
        R(1, 1001, 0, OK), R(1, 1002, 0, BELOW_MIN),    /* the envelope reaches 1,000 Hz, or not */
        R(0xFFFFFFFFu, 1, 0, ABOVE_LIMIT), R(1, 0xFFFFFFFFu, 0, BELOW_MIN),     /* no overflow */
        R(64, 2133, 0, OK),                             /* 30,004.7 Hz: 30 kpps within 1 us */
        R(64, 2130, 0, ABOVE_LIMIT), R(64, 2126, 0, ABOVE_LIMIT), R(64, 1280, 0, ABOVE_LIMIT),
                                                        /* 30,050, 30,100 and 50,000 pps */
        /* A locked output rate: the chunk's envelope must come within 1 Hz of it. */
        R(64, 3200, 30000, CHANGED), R(64, 2133, 30000, OK), R(64, 2134, 30000, OK),
        R(20000, 1000000, 19999, OK), R(20000, 1000000, 20001, OK),
        R(20000, 1000000, 19998, CHANGED), R(20000, 1000000, 20002, CHANGED),
    };
    /* Up to a minute of 30 kpps in 64-sample chunks, well past the module's ~16.8 s estimation
       window: samples per chunk, durations, chunks accepted, selected rate. */
    const uint32_t minute = 60u * 30000u / 64u;
    const struct { uint32_t n; const uint32_t *d; uint32_t accepted, hz; } sender[] = {
        { 64, D(2133, 2133, 2134), minute, 30000 },     /* 30 kpps, cumulatively rounded */
        { 64, D(2133, 2134), minute, 29998 },           /* strict alternation: 29,997.7 pps */
        { 64, D(2133), minute, 30000 },                 /* every chunk rounded: 30,004.7, at the limit */
        { 20, D(667, 666, 667), minute, 30000 },        /* 30 kpps in 20-sample chunks */
        { 20, D(666, 665), 1, 30000 },                  /* 30,050 pps: one chunk fits, two do not */
    };
    /* 154 chunks of one size (3,080 samples: what the feed queues before START) lock the rate; then
       a chunk of another size, in the same epoch and in a new one. Each chunk is truncated or rounded
       on its own: limit, first chunks, later chunk, locked rate, verdict. */
    static const struct { uint32_t limit, n0, d0, n1, d1, hz; idn_rate_verdict_t v; } lock[] = {
        LK(40000, 20, 666, 166, 5533, 30030, OK),       /* 30 kpps truncated, under a 40 kpps limit */
        LK(30000, 20, 833, 166, 6916, 24010, OK),       /* 24 kpps truncated */
        LK(30000, 20, 667, 166, 5533, 29985, OK),       /* 30 kpps rounded */
        LK(40000, 20, 666, 166, 5514, 30030, CHANGED),  /* then 30,100 pps */
        LK(40000, 20, 666, 166, 5551, 30030, CHANGED),  /* then 29,900 pps */
    };
    idn_rate_epoch_t e;

    idn_rate_reset(&e);
    for (size_t i = 0; i < N(one); i++) {
        const idn_rate_lock_t point = { one[i].locked, one[i].locked, one[i].locked };
        EQ_AT(i, idn_rate_check(&P, &e, one[i].n, one[i].d, &point), one[i].v);
    }
    EQ(idn_rate_select(&P, &e), 0);                     /* nothing committed: no rate */
    EQ(idn_rate_requested_mhz(&e), 0);
    for (size_t i = 0; i < N(sender); i++) {
        idn_rate_reset(&e);
        EQ_AT(i, feed(&e, sender[i].n, sender[i].d, minute), sender[i].accepted);
        EQ_AT(i, idn_rate_select(&P, &e), sender[i].hz);
    }
    for (size_t i = 0; i < N(lock); i++) {
        const idn_rate_policy_t p = { 1000, lock[i].limit, 1 };
        idn_rate_epoch_t fresh;
        idn_rate_reset(&e);
        idn_rate_reset(&fresh);
        for (int k = 0; k < 154; k++)
            idn_rate_commit(&e, lock[i].n0, lock[i].d0);
        const idn_rate_lock_t lk = idn_rate_lock(&p, &e);
        EQ_AT(i, lk.hz, lock[i].hz);
        EQ_AT(i, idn_rate_check(&p, &e, lock[i].n1, lock[i].d1, &lk), lock[i].v);
        EQ_AT(i, idn_rate_check(&p, &fresh, lock[i].n1, lock[i].d1, &lk), lock[i].v);
    }

    /* After 100 chunks at 20 kpps a genuine change is CHANGED, up or down; 1 us either way is not. */
    idn_rate_reset(&e);
    EQ(feed(&e, 64, D(3200), 100), 100);
    EQ(idn_rate_requested_mhz(&e), 20000000);
    EQ(idn_rate_check(&P, &e, 64, 2560, 0), IDN_RATE_CHANGED);     /* 25,000 pps */
    EQ(idn_rate_check(&P, &e, 64, 3184, 0), IDN_RATE_CHANGED);     /* 20,100.5 pps */
    EQ(idn_rate_check(&P, &e, 64, 3300, 0), IDN_RATE_CHANGED);     /* 19,393.9 pps */
    EQ(idn_rate_check(&P, &e, 64, 3201, 0), IDN_RATE_OK);
    EQ(idn_rate_check(&P, &e, 64, 3199, 0), IDN_RATE_OK);
}

static void test_events(void)
{
    idn_event_latch_t ch[2];
    idn_event_report_t r, old;

    idn_event_latch_reset(&ch[0]); idn_event_latch_reset(&ch[1]);
    idn_event_latch_record(&ch[0], IDN_PLEV_CNLRTED);
    idn_event_latch_record(&ch[0], IDN_PLEV_CTYERR);
    idn_event_latch_record(&ch[1], IDN_PLEV_BSYERR);
    old = idn_event_latch_snapshot(&ch[0]);             /* every event since the last report, */
    r = idn_event_latch_snapshot(&ch[0]);               /* and that send failed: nothing cleared */
    EQ(r.flags, IDN_PLEV_CNLRTED | IDN_PLEV_CTYERR);
    idn_event_latch_reported(&ch[0], &old);             /* a report of an older snapshot clears nothing */
    r = idn_event_latch_snapshot(&ch[0]);
    EQ(r.flags, IDN_PLEV_CNLRTED | IDN_PLEV_CTYERR);
    idn_event_latch_record(&ch[0], IDN_PLEV_CTYERR | IDN_PLEV_RGUERR);     /* during the send */
    idn_event_latch_reported(&ch[0], &r);
    EQ(idn_event_latch_snapshot(&ch[0]).flags, IDN_PLEV_CTYERR | IDN_PLEV_RGUERR);  /* still latched */
    EQ(idn_event_latch_snapshot(&ch[1]).flags, IDN_PLEV_BSYERR);    /* another channel is untouched */
}

static void test_serialize(void)
{
    /* Result, input events, pipeline events, status, link quality, latency: no octet is zero. */
    const idn_ack_fields_t f = { .result_code = IDN_ACK_RESULT_OCCUPIED, .input_event_flags = 0x8031,
                                 .pipeline_event_flags = 0x0402, .status_flags = 0x05, .link_quality = 0x7F,
                                 .latency_us = 0x01020304u };
    static const uint8_t ack[] = { 0x0C, 0xEC, 0x80, 0x31, 0x04, 0x02, 0x05, 0x7F, 0x01, 0x02, 0x03, 0x04 };
    static const struct { idn_scan_state_t s; uint8_t flags; } scan[] = {
        { { .rt_supported = true, .sessions_total = 1 }, 0x01 },
        { { .rt_supported = true, .sessions_total = 1, .sessions_in_use = 1 }, 0x11 },  /* OCPD */
        { { .sessions_total = 1, .group_excluded = true }, 0x20 },
        { { .sessions_total = 1, .offline = true }, 0x40 },
        { { .sessions_total = 1, .malfunction = true }, 0x80 },
    };
    uint8_t out[IDN_ACK_STRUCT_SIZE] = { 0 };

    EQ(idn_ack_serialize(&f, out), 12);
    for (size_t i = 0; i < N(ack); i++)
        EQ_AT(i, out[i], ack[i]);
    /* Pipeline flags: none for an empty RT ACK (no channel); CnlClsd only from channel state. */
    EQ(idn_ack_pipeline_flags(IDN_PLEV_CTYERR, false, true), 0x0000);
    EQ(idn_ack_pipeline_flags(IDN_PLEV_CTYERR, true, true), 0x0402);
    EQ(idn_ack_pipeline_flags(IDN_PLEV_CTYERR | IDN_PLEV_CNLCLSD, true, false), 0x0400);
    /* A drained session whose device may still hold data: SNMsg clear, DOBuf set. */
    EQ(idn_ack_status_flags(true, false, true), 0x03);
    EQ(idn_ack_status_flags(false, true, false), 0x04);
    for (size_t i = 0; i < N(scan); i++)
        EQ_AT(i, idn_scan_status(&scan[i].s), scan[i].flags);
}

int main(void)
{
    test_seq();
    test_chronology();
    test_wave();
    test_rate();
    test_events();
    test_serialize();
    printf("idn_timing: %d of %d checks failed\n", failures, checks);
    return failures != 0;
}

/* eval_typing.c — replay a recorded typing session through the
 * decoder and grade its output against transcript.json.
 *
 * Usage:
 *   eval_typing <session-dir>
 *   eval_typing ~/spatial-os-recordings/typing/T-20260524-143012-abc123
 *
 * Output (stdout, one block per phrase + a session summary):
 *   PHRASE 0  target="the quick brown fox" output="the quick brown fox"
 *             CER=0.00 WPM=22.4 dur=2.10s
 *   ...
 *   SESSION  n=5 mean_CER=0.04 mean_WPM=19.8 total_chars=128
 *
 * The replay mirrors the wxrd-side pose writer exactly: same default
 * keyboard plane (origin / right / down / normal), same fingertip
 * indices (THUMB_TIP=4, INDEX_TIP=8, MIDDLE_TIP=12, RING_TIP=16,
 * PINKY_TIP=20), same per-finger arrangement (left 5 then right 5).
 *
 * SPDX-License-Identifier: MIT
 */

#define _POSIX_C_SOURCE 200809L

#include "decoder.h"
#include "keyboard_geom.h"
#include "lm.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

/* --- Plane constants (must match keyboard_pose_writer.c) ----- */
static const float kPlaneOrigin[3] = { -0.15f, -0.20f, -0.50f };
static const float kPlaneRight[3]  = {  1.0f,  0.0f,  0.0f };
static const float kPlaneDown[3]   = {  0.0f,  0.0f,  1.0f };
static const float kPlaneNormal[3] = {  0.0f,  1.0f,  0.0f };

/* --- Wire-protocol minimal parser (0x01 pose, 0x05 hand) ----- */

#define PKT_POSE 0x01
#define PKT_HAND 0x05
#define HAND_PKT_LEN 431

typedef struct {
    uint64_t ts_ns;
    uint8_t  hand_idx;
    float    joints[21][3];      /* xyz only (drop confidence/reserved) */
    float    conf[21];
} hand_packet_t;

static uint32_t
read_u32_le (const uint8_t *p)
{
    return (uint32_t) p[0] | ((uint32_t) p[1] << 8)
         | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}
static uint64_t
read_u64_le (const uint8_t *p)
{
    return (uint64_t) read_u32_le (p)
         | ((uint64_t) read_u32_le (p + 4) << 32);
}
static float
read_f32_le (const uint8_t *p)
{
    uint32_t u = read_u32_le (p);
    float f;
    memcpy (&f, &u, sizeof (f));
    return f;
}

/* Parse stream.bin into a flat array of hand packets, sorted by
 * timestamp. */
static hand_packet_t *
load_hand_packets (const char *path, size_t *out_n)
{
    *out_n = 0;
    FILE *f = fopen (path, "rb");
    if (!f) { perror (path); return NULL; }
    fseek (f, 0, SEEK_END);
    long sz = ftell (f);
    fseek (f, 0, SEEK_SET);
    uint8_t *buf = malloc ((size_t) sz);
    if (!buf) { fclose (f); return NULL; }
    if (fread (buf, 1, (size_t) sz, f) != (size_t) sz) {
        free (buf); fclose (f); return NULL;
    }
    fclose (f);

    size_t cap = 4096;
    hand_packet_t *hp = malloc (cap * sizeof (*hp));
    if (!hp) { free (buf); return NULL; }
    size_t n = 0;

    long off = 0;
    while (off + 4 <= sz) {
        uint32_t len = read_u32_le (buf + off);
        off += 4;
        if (len == 0 || (long) len > sz - off) break;
        const uint8_t *p = buf + off;
        off += len;
        if (len < 1) continue;
        if (p[0] == PKT_HAND && len >= HAND_PKT_LEN) {
            if (n == cap) {
                cap *= 2;
                hp = realloc (hp, cap * sizeof (*hp));
                if (!hp) { free (buf); return NULL; }
            }
            hand_packet_t *out = &hp[n++];
            out->ts_ns   = read_u64_le (p + 1);
            out->hand_idx = p[9];
            /* p[10] = joint count, always 21. */
            for (int j = 0; j < 21; ++j) {
                const uint8_t *jp = p + 11 + j * 20;
                out->joints[j][0] = read_f32_le (jp + 0);
                out->joints[j][1] = read_f32_le (jp + 4);
                out->joints[j][2] = read_f32_le (jp + 8);
                out->conf[j]      = read_f32_le (jp + 12);
            }
        }
    }
    free (buf);
    *out_n = n;
    return hp;
}

/* --- transcript.json parser ----------------------------------- */

typedef struct {
    int      phrase_idx;
    char     target[256];
    uint64_t start_ts_ns;
    uint64_t end_ts_ns;
} transcript_entry_t;

/* Tiny JSON scan — extracts entries from the canonical shape
 *   {"entries":[{"phraseIdx":..,"targetText":"..",
 *                 "startTsNs":..,"endTsNs":..}, ...]}
 * Hand-rolled; no JSON dep needed for the v1 tool. */
static transcript_entry_t *
load_transcript (const char *path, size_t *out_n)
{
    *out_n = 0;
    FILE *f = fopen (path, "rb");
    if (!f) { perror (path); return NULL; }
    fseek (f, 0, SEEK_END);
    long sz = ftell (f);
    fseek (f, 0, SEEK_SET);
    char *buf = malloc ((size_t) sz + 1);
    if (!buf) { fclose (f); return NULL; }
    if (fread (buf, 1, (size_t) sz, f) != (size_t) sz) {
        free (buf); fclose (f); return NULL;
    }
    buf[sz] = 0;
    fclose (f);

    size_t cap = 64;
    transcript_entry_t *out = calloc (cap, sizeof (*out));
    size_t n = 0;

    const char *p = buf;
    while ((p = strstr (p, "\"phraseIdx\"")) != NULL) {
        if (n == cap) { cap *= 2; out = realloc (out, cap * sizeof (*out)); }
        transcript_entry_t *e = &out[n++];
        memset (e, 0, sizeof (*e));
        const char *colon = strchr (p, ':');
        if (colon) e->phrase_idx = atoi (colon + 1);

        const char *t = strstr (p, "\"targetText\"");
        if (t) {
            const char *q1 = strchr (t + 12, '"');
            if (q1) {
                const char *q2 = strchr (q1 + 1, '"');
                if (q2) {
                    size_t L = (size_t) (q2 - q1 - 1);
                    if (L >= sizeof (e->target)) L = sizeof (e->target) - 1;
                    memcpy (e->target, q1 + 1, L);
                    e->target[L] = 0;
                }
            }
        }
        const char *s = strstr (p, "\"startTsNs\"");
        if (s) e->start_ts_ns = strtoull (strchr (s, ':') + 1, NULL, 10);
        const char *en = strstr (p, "\"endTsNs\"");
        if (en) e->end_ts_ns = strtoull (strchr (en, ':') + 1, NULL, 10);
        p++;
    }
    free (buf);
    *out_n = n;
    return out;
}

/* --- Metrics --------------------------------------------------- */

/* Levenshtein distance between two ASCII strings. */
static int
levenshtein (const char *a, const char *b)
{
    size_t la = strlen (a), lb = strlen (b);
    int *prev = calloc (lb + 1, sizeof (int));
    int *cur  = calloc (lb + 1, sizeof (int));
    for (size_t j = 0; j <= lb; ++j) prev[j] = (int) j;
    for (size_t i = 1; i <= la; ++i) {
        cur[0] = (int) i;
        for (size_t j = 1; j <= lb; ++j) {
            int sub = prev[j-1] + (a[i-1] == b[j-1] ? 0 : 1);
            int del = prev[j] + 1;
            int ins = cur[j-1] + 1;
            int v = sub;
            if (del < v) v = del;
            if (ins < v) v = ins;
            cur[j] = v;
        }
        int *tmp = prev; prev = cur; cur = tmp;
    }
    int d = prev[lb];
    free (prev); free (cur);
    return d;
}

/* --- Replay one phrase through the decoder -------------------- */

static int
project_and_decode (decoder_t *dec,
                    const hand_packet_t *pkts, size_t n,
                    uint64_t start_ts_ns, uint64_t end_ts_ns,
                    char *out_text, size_t out_cap)
{
    /* Group packets into pairs of (left, right) per frame.  Use 16ms
     * frame bins keyed off the first packet's timestamp. */
    size_t out_len = 0;
    out_text[0] = 0;

    /* Per-finger working state. */
    bool present[10];
    float fingertips_plane[10][3];
    /* Carry-over state so a hand without a packet this bin keeps its
     * last position with present=false. */
    float last_world[2][5][3] = {{{0}}};
    bool  hand_seen[2] = { false, false };
    uint64_t bin_size_ns = 16000000;  /* 16 ms */
    uint64_t bin_start = start_ts_ns;
    uint64_t prev_bin_ns = 0;

    static const int kTips[5] = { 4, 8, 12, 16, 20 };

    size_t i = 0;
    /* Skip leading packets earlier than start. */
    while (i < n && pkts[i].ts_ns < start_ts_ns) i++;

    while (i < n && pkts[i].ts_ns < end_ts_ns) {
        uint64_t bin_end = bin_start + bin_size_ns;
        bool hand_in_bin[2] = { false, false };
        while (i < n && pkts[i].ts_ns < bin_end) {
            int h = pkts[i].hand_idx & 1;
            hand_in_bin[h] = true;
            hand_seen[h] = true;
            for (int t = 0; t < 5; ++t) {
                int j = kTips[t];
                last_world[h][t][0] = pkts[i].joints[j][0];
                last_world[h][t][1] = pkts[i].joints[j][1];
                last_world[h][t][2] = pkts[i].joints[j][2];
            }
            i++;
        }
        /* Project + dispatch. */
        for (int h = 0; h < 2; ++h) {
            for (int t = 0; t < 5; ++t) {
                int idx = h * 5 + t;
                present[idx] = hand_seen[h] && hand_in_bin[h];
                if (!present[idx]) {
                    fingertips_plane[idx][0] = 0.5f;
                    fingertips_plane[idx][1] = 0.5f;
                    fingertips_plane[idx][2] = 0.2f;
                    continue;
                }
                float dx = last_world[h][t][0] - kPlaneOrigin[0];
                float dy = last_world[h][t][1] - kPlaneOrigin[1];
                float dz = last_world[h][t][2] - kPlaneOrigin[2];
                fingertips_plane[idx][0] =
                    dx*kPlaneRight[0]+dy*kPlaneRight[1]+dz*kPlaneRight[2];
                fingertips_plane[idx][1] =
                    dx*kPlaneDown[0]+dy*kPlaneDown[1]+dz*kPlaneDown[2];
                fingertips_plane[idx][2] =
                    dx*kPlaneNormal[0]+dy*kPlaneNormal[1]+dz*kPlaneNormal[2];
            }
        }
        float dt = prev_bin_ns == 0 ? (float) bin_size_ns * 1e-9f
                  : (float) (bin_start - prev_bin_ns) * 1e-9f;
        if (dt < 1e-4f) dt = 1e-4f;
        prev_bin_ns = bin_start;

        uint32_t k = decoder_update (dec, fingertips_plane, present, dt);
        if (k) {
            char ch = 0;
            if (k >= 'a' && k <= 'z') ch = (char) k;
            else if (k == 0x0020) ch = ' ';
            else if (k == 0xff08) {  /* backspace */
                if (out_len > 0) { out_len--; out_text[out_len] = 0; }
            } else if (k == 0xff0d) ch = '\n';
            if (ch && out_len + 1 < out_cap) {
                out_text[out_len++] = ch;
                out_text[out_len] = 0;
            }
        }
        bin_start = bin_end;
    }
    return (int) out_len;
}

/* --- Main ------------------------------------------------------ */

int
main (int argc, char **argv)
{
    if (argc != 2) {
        fprintf (stderr, "usage: eval_typing <session-dir>\n");
        return 2;
    }
    const char *dir = argv[1];

    char path_bin[512], path_trans[512];
    snprintf (path_bin,   sizeof (path_bin),   "%s/stream.bin",     dir);
    snprintf (path_trans, sizeof (path_trans), "%s/transcript.json", dir);

    size_t n_pkts;
    hand_packet_t *pkts = load_hand_packets (path_bin, &n_pkts);
    if (!pkts) return 1;

    size_t n_entries;
    transcript_entry_t *entries = load_transcript (path_trans, &n_entries);
    if (!entries) {
        fprintf (stderr, "eval_typing: failed to load transcript.json\n");
        free (pkts);
        return 1;
    }
    if (n_entries == 0) {
        fprintf (stderr, "eval_typing: 0 transcript entries — nothing to grade\n");
        free (pkts); free (entries);
        return 0;
    }

    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);

    double cer_sum = 0.0;
    double wpm_sum = 0.0;
    int total_chars = 0;

    for (size_t e = 0; e < n_entries; ++e) {
        const transcript_entry_t *te = &entries[e];
        /* Reset decoder state between phrases so prior context
         * doesn't leak.  Cheap — just destroy/recreate. */
        decoder_destroy (dec);
        dec = decoder_create (DECODER_MODE_SURFACE);

        char out[512] = {0};
        project_and_decode (dec, pkts, n_pkts,
                            te->start_ts_ns, te->end_ts_ns,
                            out, sizeof (out));

        int dist = levenshtein (out, te->target);
        size_t tlen = strlen (te->target);
        double cer = tlen == 0 ? 0.0 : (double) dist / (double) tlen;
        double dur_min =
            (double) (te->end_ts_ns - te->start_ts_ns) / 60.0e9;
        double net_chars = (double) tlen - (double) dist;
        if (net_chars < 0) net_chars = 0;
        double wpm = dur_min > 1e-6 ? (net_chars / 5.0) / dur_min : 0.0;

        printf ("PHRASE %d  target=\"%s\"  output=\"%s\"\n"
                "          CER=%.2f WPM=%.1f dur=%.2fs\n",
                te->phrase_idx, te->target, out,
                cer, wpm, dur_min * 60.0);
        cer_sum += cer;
        wpm_sum += wpm;
        total_chars += (int) tlen;
    }

    printf ("SESSION  n=%zu mean_CER=%.3f mean_WPM=%.2f total_chars=%d\n",
            n_entries,
            cer_sum / (double) n_entries,
            wpm_sum / (double) n_entries,
            total_chars);

    decoder_destroy (dec);
    free (pkts);
    free (entries);
    return 0;
}

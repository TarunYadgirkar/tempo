/* control_conn.c — see control_conn.h. */

#include "control_conn.h"

#include <stdlib.h>
#include <string.h>

struct ctl_conn {
    ctl_line_fn on_line;
    void       *user;
    void       *io;          /* control.c's per-conn fd bookkeeping */
    bool        subscribed;

    /* input line assembly */
    char   in[CTL_CONN_INPUT_MAX + 1];
    size_t in_len;
    bool   in_overflow;      /* current line already too long — drop to next '\n' */

    /* output: FIFO ring of whole lines (each malloc'd, NUL-terminated, with
     * its own '\n').  head = oldest, count = number queued.  front_off is
     * how many bytes of the head line have already been written out.
     * out_is_event[] tags each slot: events are droppable under backpressure,
     * replies are NOT (a reply is a response the agent is blocking on). */
    char    *out_lines[CTL_CONN_OUTPUT_LINES];
    bool     out_is_event[CTL_CONN_OUTPUT_LINES];
    size_t   out_head;
    size_t   out_count;
    size_t   front_off;
    uint64_t dropped;
    bool     reply_overflow;  /* a reply could not be queued — caller must close */
};

ctl_conn_t *
ctl_conn_create (ctl_line_fn on_line, void *user)
{
    ctl_conn_t *c = calloc (1, sizeof (*c));
    if (!c) return NULL;
    c->on_line = on_line;
    c->user    = user;
    return c;
}

void
ctl_conn_destroy (ctl_conn_t *conn)
{
    if (!conn) return;
    for (size_t i = 0; i < conn->out_count; i++) {
        size_t idx = (conn->out_head + i) % CTL_CONN_OUTPUT_LINES;
        free (conn->out_lines[idx]);
    }
    free (conn);
}

bool
ctl_conn_subscribed (const ctl_conn_t *conn)
{
    return conn && conn->subscribed;
}

void
ctl_conn_set_subscribed (ctl_conn_t *conn, bool sub)
{
    if (conn) conn->subscribed = sub;
}

void
ctl_conn_set_io (ctl_conn_t *conn, void *io)
{
    if (conn) conn->io = io;
}

void *
ctl_conn_get_io (const ctl_conn_t *conn)
{
    return conn ? conn->io : NULL;
}

void
ctl_conn_feed (ctl_conn_t *conn, const char *data, size_t len)
{
    if (!conn || !data) return;

    for (size_t i = 0; i < len; i++) {
        char ch = data[i];
        if (ch == '\n') {
            if (conn->in_overflow) {
                /* The over-length line ends here; drop it and resume cleanly
                 * on the next line.  Deterministic regardless of how the bytes
                 * were chunked — no connection close. */
                conn->in_len = 0;
                conn->in_overflow = false;
                continue;
            }
            conn->in[conn->in_len] = '\0';
            /* Strip a trailing '\r' for CRLF peers. */
            if (conn->in_len > 0 && conn->in[conn->in_len - 1] == '\r')
                conn->in[conn->in_len - 1] = '\0';
            if (conn->on_line)
                conn->on_line (conn, conn->in, conn->user);
            conn->in_len = 0;
            continue;
        }
        if (conn->in_overflow)
            continue;  /* swallow until newline */
        if (conn->in_len >= CTL_CONN_INPUT_MAX) {
            conn->in_overflow = true;  /* over-length: swallow to the next '\n' */
            continue;
        }
        conn->in[conn->in_len++] = ch;
    }
}

/* Index (ring slot) of the oldest droppable EVENT, or SIZE_MAX if none.
 * The in-flight head (front_off>0) is never droppable — evicting it would
 * splice a half-sent line into the next.  When front_off==0 even the head is
 * a clean line boundary and may be dropped if it is an event. */
static size_t
oldest_droppable_event (ctl_conn_t *conn)
{
    size_t start = (conn->front_off > 0) ? 1 : 0;
    for (size_t i = start; i < conn->out_count; i++) {
        size_t idx = (conn->out_head + i) % CTL_CONN_OUTPUT_LINES;
        if (conn->out_is_event[idx])
            return i; /* logical position from head */
    }
    return (size_t)-1;
}

/* Remove the entry at logical position `pos` from head (0 = head), shifting
 * the entries after it down by one.  Frees the removed line. */
static void
ring_remove_at (ctl_conn_t *conn, size_t pos)
{
    size_t victim = (conn->out_head + pos) % CTL_CONN_OUTPUT_LINES;
    free (conn->out_lines[victim]);
    for (size_t i = pos; i + 1 < conn->out_count; i++) {
        size_t a = (conn->out_head + i) % CTL_CONN_OUTPUT_LINES;
        size_t b = (conn->out_head + i + 1) % CTL_CONN_OUTPUT_LINES;
        conn->out_lines[a]    = conn->out_lines[b];
        conn->out_is_event[a] = conn->out_is_event[b];
    }
    conn->out_count--;
}

static void
conn_enqueue (ctl_conn_t *conn, const char *line, bool is_event)
{
    if (!conn || !line) return;

    if (conn->out_count == CTL_CONN_OUTPUT_LINES) {
        size_t victim = oldest_droppable_event (conn);
        if (victim != (size_t)-1) {
            ring_remove_at (conn, victim);
            conn->dropped++;
        } else if (is_event) {
            /* Full of replies / in-flight head — drop THIS event rather than a
             * pending reply or the in-flight line. */
            conn->dropped++;
            return;
        } else {
            /* A reply cannot be queued and nothing droppable exists: the peer
             * is not draining.  Signal the caller to close — never silently
             * drop a reply the agent is blocking on. */
            conn->reply_overflow = true;
            return;
        }
    }

    size_t llen = strlen (line);
    bool need_nl = (llen == 0 || line[llen - 1] != '\n');
    char *copy = malloc (llen + (need_nl ? 1 : 0) + 1);
    if (!copy) return;
    memcpy (copy, line, llen);
    if (need_nl) { copy[llen] = '\n'; copy[llen + 1] = '\0'; }
    else          copy[llen] = '\0';

    size_t tail = (conn->out_head + conn->out_count) % CTL_CONN_OUTPUT_LINES;
    conn->out_lines[tail]    = copy;
    conn->out_is_event[tail] = is_event;
    conn->out_count++;
}

void
ctl_conn_queue (ctl_conn_t *conn, const char *line)
{
    conn_enqueue (conn, line, /*is_event=*/false);  /* reply — never dropped */
}

void
ctl_conn_queue_event (ctl_conn_t *conn, const char *line)
{
    conn_enqueue (conn, line, /*is_event=*/true);   /* droppable under pressure */
}

bool
ctl_conn_reply_overflow (const ctl_conn_t *conn)
{
    return conn && conn->reply_overflow;
}

size_t
ctl_conn_pending (ctl_conn_t *conn, const char **out)
{
    if (!conn || conn->out_count == 0) { if (out) *out = NULL; return 0; }
    char *front = conn->out_lines[conn->out_head];
    size_t flen = strlen (front);
    if (out) *out = front + conn->front_off;
    return flen - conn->front_off;
}

void
ctl_conn_consumed (ctl_conn_t *conn, size_t n)
{
    if (!conn || conn->out_count == 0) return;
    char *front = conn->out_lines[conn->out_head];
    size_t flen = strlen (front);
    conn->front_off += n;
    if (conn->front_off >= flen) {
        free (front);
        conn->out_head = (conn->out_head + 1) % CTL_CONN_OUTPUT_LINES;
        conn->out_count--;
        conn->front_off = 0;
    }
}

bool
ctl_conn_has_output (const ctl_conn_t *conn)
{
    return conn && conn->out_count > 0;
}

uint64_t
ctl_conn_dropped (const ctl_conn_t *conn)
{
    return conn ? conn->dropped : 0;
}

/* control_conn.h — pure per-connection byte buffering for the control
 * socket, with NO dependency on sockets, wlroots, or the wl_event_loop.
 *
 * This is the unit-testable core of control.c's I/O: input-line assembly
 * across arbitrary chunk boundaries, and a bounded output queue with
 * drop-oldest backpressure so a stalled subscriber can never make the
 * compositor grow memory without bound.  control.c owns the fd and pumps
 * bytes through here; tests/test_control_conn.c drives it directly.
 */

#ifndef SPATIAL_CONTROL_CONN_H
#define SPATIAL_CONTROL_CONN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "control_protocol.h"  /* CTL_TEXT_MAX */

#ifdef __cplusplus
extern "C" {
#endif

/* Longest single request line we will assemble before declaring the peer
 * abusive and asking the caller to drop the connection. */
#define CTL_CONN_INPUT_MAX (CTL_TEXT_MAX + 128)

/* Number of whole output lines buffered before drop-oldest kicks in. */
#define CTL_CONN_OUTPUT_LINES 256

typedef struct ctl_conn ctl_conn_t;

/* Invoked once per complete '\n'-terminated input line.  `line` has the
 * trailing newline stripped and is NUL-terminated.  `user` is the pointer
 * passed to ctl_conn_create. */
typedef void (*ctl_line_fn) (ctl_conn_t *conn, const char *line, void *user);

ctl_conn_t *ctl_conn_create (ctl_line_fn on_line, void *user);
void        ctl_conn_destroy (ctl_conn_t *conn);

/* Public flag the dispatcher flips when this connection issues `subscribe`. */
bool ctl_conn_subscribed (const ctl_conn_t *conn);
void ctl_conn_set_subscribed (ctl_conn_t *conn, bool sub);

/* Opaque user pointer carried alongside the connection (control.c stores its
 * per-conn fd-source bookkeeping here). */
void  ctl_conn_set_io (ctl_conn_t *conn, void *io);
void *ctl_conn_get_io (const ctl_conn_t *conn);

/* Feed received bytes.  Invokes on_line for every complete line found.  An
 * over-length line (> CTL_CONN_INPUT_MAX with no newline) is swallowed up to
 * the next '\n' and dropped — deterministically, regardless of how the bytes
 * were chunked.  The connection is never closed by this call. */
void ctl_conn_feed (ctl_conn_t *conn, const char *data, size_t len);

/* Queue one output line (a trailing '\n' is added if absent).
 *
 * ctl_conn_queue       — a REPLY: never dropped.  Under backpressure it
 *   evicts the oldest droppable event; if none exists (ring full of replies /
 *   in-flight line), it sets the reply-overflow flag and the caller must
 *   close the connection (the peer is not draining).
 * ctl_conn_queue_event — an EVENT (subscribe stream): droppable.  Under
 *   backpressure the oldest event is discarded (dropped++); the in-flight
 *   head line and any pending replies are never disturbed. */
void ctl_conn_queue (ctl_conn_t *conn, const char *line);
void ctl_conn_queue_event (ctl_conn_t *conn, const char *line);

/* True once a reply could not be queued (ring full of undrained replies);
 * the caller should close this connection. */
bool ctl_conn_reply_overflow (const ctl_conn_t *conn);

/* Contiguous pending output: sets *out to the next bytes to write and
 * returns their count (0 if nothing pending). */
size_t ctl_conn_pending (ctl_conn_t *conn, const char **out);

/* Advance the output cursor after writing n bytes (n <= last pending). */
void ctl_conn_consumed (ctl_conn_t *conn, size_t n);

bool     ctl_conn_has_output (const ctl_conn_t *conn);
uint64_t ctl_conn_dropped (const ctl_conn_t *conn);

#ifdef __cplusplus
}
#endif

#endif /* SPATIAL_CONTROL_CONN_H */

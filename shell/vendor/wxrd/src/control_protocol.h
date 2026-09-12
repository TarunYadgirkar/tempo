/* control_protocol.h — Text wire format for the agent control plane at
 * $XDG_RUNTIME_DIR/spatial-os.sock.
 *
 * This is the generalised superset of the older anchor_protocol.{c,h}
 * (assign/clear/query), which it subsumes: the parser still accepts those
 * three legacy lines so the landed anchor_runtime test keeps passing.
 *
 * Like anchor_protocol, this header is the parser/serialiser ONLY — the
 * socket bind/accept/dispatch lives in control.c, and the per-connection
 * byte buffering lives in control_conn.c.  Keeping the wire layer pure makes
 * it unit-testable (tests/test_control_protocol.c) without standing up
 * wlroots, OpenXR, or a socket.
 *
 * Grammar (one request per line, '\n'-terminated; tokens space-separated):
 *
 *   Windows
 *     list-windows
 *     move      <handle> <x> <y> <z>          # absolute target position (m)
 *     move-rel  <handle> <dx> <dy> <dz>        # relative delta (m)
 *     anchor    <handle> <closest-wall|closest-horizontal|UUIDhex32>
 *     clear-anchor <handle>
 *     focus     <handle>
 *     close     <handle>
 *     resize    <handle> <w> <h>               # pixels
 *   Input
 *     type      <utf8 text to end of line>     # NB: rest-of-line, may contain spaces
 *     key       <keysym-name|0xNN>             # one keysym press+release
 *     scroll    <dx> <dy>                       # axis deltas
 *     click     <handle> <x> <y> [button]      # surface-local px; button default left
 *   Apps
 *     launch    <command to end of line>       # /bin/sh -c <command>
 *   World / state
 *     list-planes
 *     head-pose
 *     dump-state
 *     theme-reload
 *   Stream
 *     subscribe                                 # this conn receives `event ...` lines
 *   Observation (Phase 3 — parse now, dispatch may reply `err unimplemented`)
 *     screenshot <handle> [region=x,y,w,h] [scale=f]
 *     a11y       <handle>
 *     a11y-action <handle> <node-id>
 *   Meta
 *     version
 *   Legacy aliases (anchor_protocol compatibility)
 *     assign <handle> <UUIDhex32>   == anchor <handle> <UUIDhex32>
 *     clear  <handle>               == clear-anchor <handle>
 *     query  <handle>               (anchor-query; reply ok anchor=/ok no_anchor)
 *
 * Replies are one line:
 *     ok
 *     ok <key>=<value> ...          # e.g. `ok anchor=<uuid>`, `ok proto=1`
 *     ok no_anchor
 *     ok <json>                     # list-windows/list-planes/head-pose/dump-state
 *     err <one-word-code> <detail>
 * Asynchronous stream lines (only to subscribed conns) are prefixed `event `.
 */

#ifndef SPATIAL_CONTROL_PROTOCOL_H
#define SPATIAL_CONTROL_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPATIAL_CONTROL_PROTO_VERSION 1

/* Longest text/command payload we accept on a single line (the `type` and
 * `launch` verbs carry rest-of-line text). */
#define CTL_TEXT_MAX 1024

typedef enum {
    CTL_REQ_UNKNOWN = 0,
    /* windows */
    CTL_REQ_LIST_WINDOWS,
    CTL_REQ_MOVE,
    CTL_REQ_MOVE_REL,
    CTL_REQ_ANCHOR,
    CTL_REQ_CLEAR_ANCHOR,
    CTL_REQ_FOCUS,
    CTL_REQ_CLOSE,
    CTL_REQ_RESIZE,
    /* input */
    CTL_REQ_TYPE,
    CTL_REQ_KEY,
    CTL_REQ_SCROLL,
    CTL_REQ_CLICK,
    /* apps */
    CTL_REQ_LAUNCH,
    /* world / state */
    CTL_REQ_LIST_PLANES,
    CTL_REQ_HEAD_POSE,
    CTL_REQ_DUMP_STATE,
    CTL_REQ_THEME_RELOAD,
    /* stream */
    CTL_REQ_SUBSCRIBE,
    /* observation (Phase 3) */
    CTL_REQ_SCREENSHOT,
    CTL_REQ_A11Y,
    CTL_REQ_A11Y_ACTION,
    /* meta */
    CTL_REQ_VERSION,
    /* legacy anchor-query (assign/clear fold into ANCHOR/CLEAR_ANCHOR) */
    CTL_REQ_QUERY,
} ctl_req_kind_t;

/* How the ANCHOR verb names its target plane. */
typedef enum {
    CTL_ANCHOR_NONE = 0,
    CTL_ANCHOR_UUID,            /* explicit uuid in anchor_uuid[] */
    CTL_ANCHOR_CLOSEST_WALL,    /* nearest vertical plane (alignment==1) */
    CTL_ANCHOR_CLOSEST_HORIZ,   /* nearest horizontal plane (alignment==0) */
} ctl_anchor_mode_t;

typedef struct {
    ctl_req_kind_t kind;

    uint64_t handle;            /* window handle for handle-taking verbs */

    /* MOVE / MOVE_REL: vec is the target/delta position (metres). */
    float vec[3];

    /* ANCHOR */
    ctl_anchor_mode_t anchor_mode;
    uint8_t           anchor_uuid[16];  /* valid when anchor_mode==UUID */

    /* RESIZE */
    int width, height;

    /* SCROLL */
    float scroll[2];

    /* CLICK: surface-local pixel coords + button (0=left,1=right,2=middle) */
    float click_xy[2];
    int   button;

    /* TYPE text / LAUNCH command (rest-of-line, NUL-terminated) */
    char text[CTL_TEXT_MAX];

    /* KEY: parsed keysym (xkb keysym value). */
    uint32_t keysym;

    /* A11Y_ACTION node id (AT-SPI path token). */
    char node_id[128];

    /* SCREENSHOT options. region_valid → region[4] = {x,y,w,h}; scale>0. */
    bool  region_valid;
    int   region[4];
    float scale;
} ctl_request_t;

/* Parse one line into a request.  Returns true on success; on failure *out
 * is zeroed with kind==CTL_REQ_UNKNOWN.  Trailing CR/LF/space tolerated. */
bool ctl_request_parse (const char *line, ctl_request_t *out);

/* Serialise a request back to a single '\n'-terminated line.  Returns bytes
 * written (excluding NUL) or -1 on unsupported kind / short buffer.  Used by
 * spatialctl and by the round-trip test. */
int  ctl_request_format (const ctl_request_t *r, char *buf, size_t buf_len);

/* ------------------------------------------------------------------ */
/* Reply helpers — small builders so dispatch code never hand-rolls a
 * reply line (keeps the `ok`/`err` shape in one place).  Each returns
 * bytes written (excluding NUL) or -1 on short buffer.                 */
/* ------------------------------------------------------------------ */

int ctl_reply_ok        (char *buf, size_t n);
int ctl_reply_ok_kv     (char *buf, size_t n, const char *key, const char *val);
int ctl_reply_ok_anchor (char *buf, size_t n, const uint8_t uuid[16]);
int ctl_reply_ok_no_anchor (char *buf, size_t n);
/* Emit `ok <json>\n`.  json must not contain a newline. */
int ctl_reply_ok_json   (char *buf, size_t n, const char *json);
int ctl_reply_err       (char *buf, size_t n, const char *code,
                         const char *detail);

/* Hex<->uuid helpers shared with control.c (and identical to the ones in
 * anchor_protocol.c — exposed here so callers don't re-implement). */
bool ctl_hex_to_uuid (const char *s, uint8_t out[16]);
int  ctl_uuid_to_hex (const uint8_t uuid[16], char *out, size_t out_len);

#ifdef __cplusplus
}
#endif

#endif /* SPATIAL_CONTROL_PROTOCOL_H */

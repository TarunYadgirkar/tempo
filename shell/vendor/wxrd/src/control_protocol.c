/* control_protocol.c — see control_protocol.h. */

#include "control_protocol.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* hex / uuid                                                          */
/* ------------------------------------------------------------------ */

static int
hex_nibble (char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool
ctl_hex_to_uuid (const char *s, uint8_t out[16])
{
    if (!s || strlen (s) != 32) return false;
    for (int i = 0; i < 16; ++i) {
        int hi = hex_nibble (s[i * 2 + 0]);
        int lo = hex_nibble (s[i * 2 + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

int
ctl_uuid_to_hex (const uint8_t uuid[16], char *out, size_t out_len)
{
    if (out_len < 33) return -1;
    static const char hex[] = "0123456789abcdef";
    for (int i = 0; i < 16; ++i) {
        out[i * 2 + 0] = hex[(uuid[i] >> 4) & 0xf];
        out[i * 2 + 1] = hex[uuid[i] & 0xf];
    }
    out[32] = '\0';
    return 32;
}

/* ------------------------------------------------------------------ */
/* small token helpers                                                 */
/* ------------------------------------------------------------------ */

static const char *
skip_ws (const char *s)
{
    while (*s == ' ' || *s == '\t') ++s;
    return s;
}

/* Copy `line` into `buf`, stripping ONLY a trailing CR/LF.  Trailing spaces
 * and tabs are preserved so the rest-of-line verbs (`type`, `launch`) keep
 * payload whitespace intact; the per-verb tokenisers skip leading whitespace
 * and ignore trailing whitespace after their last argument on their own.
 * Returns false if the line is too long for buf. */
static bool
copy_trimmed (const char *line, char *buf, size_t buf_len)
{
    size_t n = strlen (line);
    if (n >= buf_len) return false;
    memcpy (buf, line, n);
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) {
        --n;
    }
    buf[n] = '\0';
    return true;
}

/* Parse a strict non-negative uint64.  Returns false on trailing junk. */
static bool
parse_u64 (const char *s, const char **endp, uint64_t *out)
{
    s = skip_ws (s);
    if (!isdigit ((unsigned char)*s)) return false;
    char *end = NULL;
    unsigned long long v = strtoull (s, &end, 10);
    if (end == s) return false;
    *out = (uint64_t)v;
    if (endp) *endp = end;
    return true;
}

static bool
parse_float (const char *s, const char **endp, float *out)
{
    s = skip_ws (s);
    char *end = NULL;
    float v = strtof (s, &end);
    if (end == s) return false;
    *out = v;
    if (endp) *endp = end;
    return true;
}

static bool
parse_int (const char *s, const char **endp, int *out)
{
    s = skip_ws (s);
    char *end = NULL;
    long v = strtol (s, &end, 10);
    if (end == s) return false;
    *out = (int)v;
    if (endp) *endp = end;
    return true;
}

/* Match a verb keyword at the start of p; on match return the cursor just
 * past the keyword AND a following space (or end-of-string).  Returns NULL
 * on no match.  Guarantees the keyword is whole (not a prefix of a longer
 * token). */
static const char *
match_verb (const char *p, const char *kw)
{
    size_t klen = strlen (kw);
    if (strncmp (p, kw, klen) != 0) return NULL;
    char after = p[klen];
    if (after == '\0') return p + klen;       /* bare verb */
    if (after == ' ' || after == '\t') return skip_ws (p + klen);
    return NULL;                               /* kw was a prefix of a longer token */
}

/* ------------------------------------------------------------------ */
/* parse                                                               */
/* ------------------------------------------------------------------ */

bool
ctl_request_parse (const char *line, ctl_request_t *out)
{
    if (out) memset (out, 0, sizeof (*out));
    if (!line || !out) return false;
    out->scale = 1.0f;
    out->button = 0;

    char buf[CTL_TEXT_MAX + 64];
    if (!copy_trimmed (line, buf, sizeof (buf))) return false;

    const char *p = skip_ws (buf);
    const char *a;

    /* ---- no-argument verbs ---- */
    if ((a = match_verb (p, "list-windows"))) { out->kind = CTL_REQ_LIST_WINDOWS; return true; }
    if ((a = match_verb (p, "list-planes")))  { out->kind = CTL_REQ_LIST_PLANES;  return true; }
    if ((a = match_verb (p, "head-pose")))    { out->kind = CTL_REQ_HEAD_POSE;    return true; }
    if ((a = match_verb (p, "dump-state")))   { out->kind = CTL_REQ_DUMP_STATE;   return true; }
    if ((a = match_verb (p, "theme-reload"))) { out->kind = CTL_REQ_THEME_RELOAD; return true; }
    if ((a = match_verb (p, "subscribe")))    { out->kind = CTL_REQ_SUBSCRIBE;    return true; }
    if ((a = match_verb (p, "version")))      { out->kind = CTL_REQ_VERSION;      return true; }

    /* ---- rest-of-line verbs (must come before generic tokenisers so the
     *      payload's internal spaces survive) ---- */
    if ((a = match_verb (p, "type"))) {
        if (a[0] == '\0') return false;
        if (strlen (a) >= sizeof (out->text)) return false;
        out->kind = CTL_REQ_TYPE;
        strncpy (out->text, a, sizeof (out->text) - 1);
        return true;
    }
    if ((a = match_verb (p, "launch"))) {
        if (a[0] == '\0') return false;
        if (strlen (a) >= sizeof (out->text)) return false;
        out->kind = CTL_REQ_LAUNCH;
        strncpy (out->text, a, sizeof (out->text) - 1);
        return true;
    }

    /* ---- handle-then-args verbs ---- */
    if ((a = match_verb (p, "move-rel"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        for (int i = 0; i < 3; i++)
            if (!parse_float (a, &a, &out->vec[i])) return false;
        out->kind = CTL_REQ_MOVE_REL;
        return true;
    }
    if ((a = match_verb (p, "move"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        for (int i = 0; i < 3; i++)
            if (!parse_float (a, &a, &out->vec[i])) return false;
        out->kind = CTL_REQ_MOVE;
        return true;
    }
    if ((a = match_verb (p, "anchor"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        a = skip_ws (a);
        if (strncmp (a, "closest-wall", 12) == 0) {
            out->anchor_mode = CTL_ANCHOR_CLOSEST_WALL;
        } else if (strncmp (a, "closest-horizontal", 18) == 0) {
            out->anchor_mode = CTL_ANCHOR_CLOSEST_HORIZ;
        } else if (ctl_hex_to_uuid (a, out->anchor_uuid)) {
            out->anchor_mode = CTL_ANCHOR_UUID;
        } else {
            return false;
        }
        out->kind = CTL_REQ_ANCHOR;
        return true;
    }
    if ((a = match_verb (p, "clear-anchor"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        out->kind = CTL_REQ_CLEAR_ANCHOR;
        return true;
    }
    if ((a = match_verb (p, "focus"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        out->kind = CTL_REQ_FOCUS;
        return true;
    }
    if ((a = match_verb (p, "close"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        out->kind = CTL_REQ_CLOSE;
        return true;
    }
    if ((a = match_verb (p, "resize"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        if (!parse_int (a, &a, &out->width))  return false;
        if (!parse_int (a, &a, &out->height)) return false;
        out->kind = CTL_REQ_RESIZE;
        return true;
    }
    if ((a = match_verb (p, "key"))) {
        a = skip_ws (a);
        if (a[0] == '\0') return false;
        /* numeric (0x.. or decimal) → keysym value; otherwise a keysym
         * name that control.c resolves via xkb_keysym_from_name. */
        char *end = NULL;
        unsigned long v = strtoul (a, &end, 0);
        if (end != a && (*end == '\0')) {
            out->keysym = (uint32_t)v;
        } else {
            if (strlen (a) >= sizeof (out->text)) return false;
            strncpy (out->text, a, sizeof (out->text) - 1);
            out->keysym = 0;
        }
        out->kind = CTL_REQ_KEY;
        return true;
    }
    if ((a = match_verb (p, "scroll"))) {
        if (!parse_float (a, &a, &out->scroll[0])) return false;
        if (!parse_float (a, &a, &out->scroll[1])) return false;
        out->kind = CTL_REQ_SCROLL;
        return true;
    }
    if ((a = match_verb (p, "click"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        if (!parse_float (a, &a, &out->click_xy[0])) return false;
        if (!parse_float (a, &a, &out->click_xy[1])) return false;
        a = skip_ws (a);
        if (*a) { if (!parse_int (a, &a, &out->button)) return false; }
        out->kind = CTL_REQ_CLICK;
        return true;
    }

    /* ---- observation (Phase 3) ---- */
    if ((a = match_verb (p, "screenshot"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        /* optional region=x,y,w,h and scale=f tokens, any order */
        while (1) {
            a = skip_ws (a);
            if (*a == '\0') break;
            if (strncmp (a, "region=", 7) == 0) {
                if (sscanf (a + 7, "%d,%d,%d,%d", &out->region[0], &out->region[1],
                            &out->region[2], &out->region[3]) != 4)
                    return false;
                out->region_valid = true;
                while (*a && *a != ' ' && *a != '\t') ++a;
            } else if (strncmp (a, "scale=", 6) == 0) {
                if (!parse_float (a + 6, NULL, &out->scale)) return false;
                while (*a && *a != ' ' && *a != '\t') ++a;
            } else {
                return false;
            }
        }
        out->kind = CTL_REQ_SCREENSHOT;
        return true;
    }
    if ((a = match_verb (p, "a11y-action"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        a = skip_ws (a);
        if (*a == '\0' || strlen (a) >= sizeof (out->node_id)) return false;
        strncpy (out->node_id, a, sizeof (out->node_id) - 1);
        out->kind = CTL_REQ_A11Y_ACTION;
        return true;
    }
    if ((a = match_verb (p, "a11y"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        out->kind = CTL_REQ_A11Y;
        return true;
    }

    /* ---- legacy anchor_protocol aliases ---- */
    if ((a = match_verb (p, "assign"))) {
        char uuid_hex[64] = {0};
        if (!parse_u64 (a, &a, &out->handle)) return false;
        if (sscanf (a, "%63s", uuid_hex) != 1) return false;
        if (!ctl_hex_to_uuid (uuid_hex, out->anchor_uuid)) return false;
        out->anchor_mode = CTL_ANCHOR_UUID;
        out->kind = CTL_REQ_ANCHOR;
        return true;
    }
    if ((a = match_verb (p, "clear"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        out->kind = CTL_REQ_CLEAR_ANCHOR;
        return true;
    }
    if ((a = match_verb (p, "query"))) {
        if (!parse_u64 (a, &a, &out->handle)) return false;
        out->kind = CTL_REQ_QUERY;
        return true;
    }

    return false;
}

/* ------------------------------------------------------------------ */
/* format (round-trip; used by spatialctl + tests)                     */
/* ------------------------------------------------------------------ */

int
ctl_request_format (const ctl_request_t *r, char *buf, size_t buf_len)
{
    if (!r || !buf || buf_len < 16) return -1;
    char uuid_hex[33];
    switch (r->kind) {
    case CTL_REQ_LIST_WINDOWS: return snprintf (buf, buf_len, "list-windows\n");
    case CTL_REQ_LIST_PLANES:  return snprintf (buf, buf_len, "list-planes\n");
    case CTL_REQ_HEAD_POSE:    return snprintf (buf, buf_len, "head-pose\n");
    case CTL_REQ_DUMP_STATE:   return snprintf (buf, buf_len, "dump-state\n");
    case CTL_REQ_THEME_RELOAD: return snprintf (buf, buf_len, "theme-reload\n");
    case CTL_REQ_SUBSCRIBE:    return snprintf (buf, buf_len, "subscribe\n");
    case CTL_REQ_VERSION:      return snprintf (buf, buf_len, "version\n");
    case CTL_REQ_MOVE:
        return snprintf (buf, buf_len, "move %llu %g %g %g\n",
                         (unsigned long long)r->handle,
                         (double)r->vec[0], (double)r->vec[1], (double)r->vec[2]);
    case CTL_REQ_MOVE_REL:
        return snprintf (buf, buf_len, "move-rel %llu %g %g %g\n",
                         (unsigned long long)r->handle,
                         (double)r->vec[0], (double)r->vec[1], (double)r->vec[2]);
    case CTL_REQ_ANCHOR:
        if (r->anchor_mode == CTL_ANCHOR_CLOSEST_WALL)
            return snprintf (buf, buf_len, "anchor %llu closest-wall\n",
                             (unsigned long long)r->handle);
        if (r->anchor_mode == CTL_ANCHOR_CLOSEST_HORIZ)
            return snprintf (buf, buf_len, "anchor %llu closest-horizontal\n",
                             (unsigned long long)r->handle);
        if (ctl_uuid_to_hex (r->anchor_uuid, uuid_hex, sizeof (uuid_hex)) < 0)
            return -1;
        return snprintf (buf, buf_len, "anchor %llu %s\n",
                         (unsigned long long)r->handle, uuid_hex);
    case CTL_REQ_CLEAR_ANCHOR:
        return snprintf (buf, buf_len, "clear-anchor %llu\n",
                         (unsigned long long)r->handle);
    case CTL_REQ_FOCUS:
        return snprintf (buf, buf_len, "focus %llu\n",
                         (unsigned long long)r->handle);
    case CTL_REQ_CLOSE:
        return snprintf (buf, buf_len, "close %llu\n",
                         (unsigned long long)r->handle);
    case CTL_REQ_RESIZE:
        return snprintf (buf, buf_len, "resize %llu %d %d\n",
                         (unsigned long long)r->handle, r->width, r->height);
    case CTL_REQ_TYPE:
        return snprintf (buf, buf_len, "type %s\n", r->text);
    case CTL_REQ_LAUNCH:
        return snprintf (buf, buf_len, "launch %s\n", r->text);
    case CTL_REQ_KEY:
        if (r->keysym)
            return snprintf (buf, buf_len, "key 0x%x\n", r->keysym);
        return snprintf (buf, buf_len, "key %s\n", r->text);
    case CTL_REQ_SCROLL:
        return snprintf (buf, buf_len, "scroll %g %g\n",
                         (double)r->scroll[0], (double)r->scroll[1]);
    case CTL_REQ_CLICK:
        return snprintf (buf, buf_len, "click %llu %g %g %d\n",
                         (unsigned long long)r->handle,
                         (double)r->click_xy[0], (double)r->click_xy[1],
                         r->button);
    case CTL_REQ_SCREENSHOT:
        if (r->region_valid)
            return snprintf (buf, buf_len, "screenshot %llu region=%d,%d,%d,%d scale=%g\n",
                             (unsigned long long)r->handle, r->region[0], r->region[1],
                             r->region[2], r->region[3], (double)r->scale);
        return snprintf (buf, buf_len, "screenshot %llu scale=%g\n",
                         (unsigned long long)r->handle, (double)r->scale);
    case CTL_REQ_A11Y:
        return snprintf (buf, buf_len, "a11y %llu\n",
                         (unsigned long long)r->handle);
    case CTL_REQ_A11Y_ACTION:
        return snprintf (buf, buf_len, "a11y-action %llu %s\n",
                         (unsigned long long)r->handle, r->node_id);
    case CTL_REQ_QUERY:
        return snprintf (buf, buf_len, "query %llu\n",
                         (unsigned long long)r->handle);
    default:
        return -1;
    }
}

/* ------------------------------------------------------------------ */
/* reply builders                                                      */
/* ------------------------------------------------------------------ */

int
ctl_reply_ok (char *buf, size_t n)
{
    if (!buf || n < 4) return -1;
    return snprintf (buf, n, "ok\n");
}

int
ctl_reply_ok_kv (char *buf, size_t n, const char *key, const char *val)
{
    if (!buf) return -1;
    return snprintf (buf, n, "ok %s=%s\n", key ? key : "", val ? val : "");
}

int
ctl_reply_ok_anchor (char *buf, size_t n, const uint8_t uuid[16])
{
    char hex[33];
    if (ctl_uuid_to_hex (uuid, hex, sizeof (hex)) < 0) return -1;
    return snprintf (buf, n, "ok anchor=%s\n", hex);
}

int
ctl_reply_ok_no_anchor (char *buf, size_t n)
{
    if (!buf || n < 13) return -1;
    return snprintf (buf, n, "ok no_anchor\n");
}

int
ctl_reply_ok_json (char *buf, size_t n, const char *json)
{
    if (!buf || !json) return -1;
    return snprintf (buf, n, "ok %s\n", json);
}

int
ctl_reply_err (char *buf, size_t n, const char *code, const char *detail)
{
    if (!buf) return -1;
    if (detail && detail[0])
        return snprintf (buf, n, "err %s %s\n", code ? code : "unknown", detail);
    return snprintf (buf, n, "err %s\n", code ? code : "unknown");
}

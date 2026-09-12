/* menubar_items.c — see menubar_items.h.
 *
 * SPDX-License-Identifier: MIT
 */

#include "menubar_items.h"

#include <string.h>

/* Action cookies for status-bar items.  The status-bar's commit
 * handler reads these via the cookie field. */
enum mbi_action {
    MBI_ACTION_NONE = 0,

    /* Clock */
    MBI_ACTION_CLOCK_OPEN_CALENDAR = 100,
    MBI_ACTION_CLOCK_SET_TIME      = 101,
    MBI_ACTION_CLOCK_TIMEZONE      = 102,

    /* Wifi */
    MBI_ACTION_WIFI_TOGGLE         = 200,
    MBI_ACTION_WIFI_PICK_NETWORK   = 201,
    MBI_ACTION_WIFI_DISCONNECT     = 202,
    MBI_ACTION_WIFI_SETTINGS       = 203,

    /* iPhone */
    MBI_ACTION_IPHONE_RECONNECT    = 300,
    MBI_ACTION_IPHONE_DISCONNECT   = 301,
    MBI_ACTION_IPHONE_REPAIR       = 302,

    /* Tracking */
    MBI_ACTION_TRACKING_RELOCALISE = 400,
    MBI_ACTION_TRACKING_RESET      = 401,
    MBI_ACTION_TRACKING_DIAGNOSTICS= 402,
};

static const rm_item_t IT_CALENDAR    = { "Calendar",       NULL, (void*) MBI_ACTION_CLOCK_OPEN_CALENDAR };
static const rm_item_t IT_SET_TIME    = { "Set time",       NULL, (void*) MBI_ACTION_CLOCK_SET_TIME      };
static const rm_item_t IT_TIMEZONE    = { "Timezone",       NULL, (void*) MBI_ACTION_CLOCK_TIMEZONE      };

static const rm_item_t IT_WIFI_TOGGLE = { "Toggle Wi-Fi",   NULL, (void*) MBI_ACTION_WIFI_TOGGLE         };
static const rm_item_t IT_WIFI_PICK   = { "Pick network",   NULL, (void*) MBI_ACTION_WIFI_PICK_NETWORK   };
static const rm_item_t IT_WIFI_DC     = { "Disconnect",     NULL, (void*) MBI_ACTION_WIFI_DISCONNECT     };
static const rm_item_t IT_WIFI_SET    = { "Wi-Fi settings", NULL, (void*) MBI_ACTION_WIFI_SETTINGS       };

static const rm_item_t IT_IP_RC       = { "Reconnect",      NULL, (void*) MBI_ACTION_IPHONE_RECONNECT    };
static const rm_item_t IT_IP_DC       = { "Disconnect",     NULL, (void*) MBI_ACTION_IPHONE_DISCONNECT   };
static const rm_item_t IT_IP_REPAIR   = { "Re-pair",        NULL, (void*) MBI_ACTION_IPHONE_REPAIR       };

static const rm_item_t IT_TR_RELOC    = { "Relocalise",     NULL, (void*) MBI_ACTION_TRACKING_RELOCALISE };
static const rm_item_t IT_TR_RESET    = { "Reset map",      NULL, (void*) MBI_ACTION_TRACKING_RESET      };
static const rm_item_t IT_TR_DIAG     = { "Diagnostics",    NULL, (void*) MBI_ACTION_TRACKING_DIAGNOSTICS};

static size_t
push (rm_item_t *out, size_t cap, size_t n, rm_item_t item)
{
    if (n < cap) out[n] = item;
    return n + 1;
}

size_t
mbi_status_bar_items (mbi_source_t source,
                       const mbi_state_t *state,
                       rm_item_t *out,
                       size_t out_max)
{
    mbi_state_t empty = {0};
    if (!state) state = &empty;
    if (!out) out_max = 0;

    size_t n = 0;
    switch (source) {
    case MBI_SOURCE_CLOCK:
        n = push (out, out_max, n, IT_CALENDAR);
        n = push (out, out_max, n, IT_SET_TIME);
        n = push (out, out_max, n, IT_TIMEZONE);
        break;

    case MBI_SOURCE_WIFI:
        n = push (out, out_max, n, IT_WIFI_TOGGLE);
        n = push (out, out_max, n, IT_WIFI_PICK);
        if (state->wifi_connected) {
            n = push (out, out_max, n, IT_WIFI_DC);
        }
        n = push (out, out_max, n, IT_WIFI_SET);
        break;

    case MBI_SOURCE_IPHONE:
        if (state->iphone_connected) {
            n = push (out, out_max, n, IT_IP_DC);
        } else {
            n = push (out, out_max, n, IT_IP_RC);
        }
        n = push (out, out_max, n, IT_IP_REPAIR);
        break;

    case MBI_SOURCE_TRACKING:
        n = push (out, out_max, n, IT_TR_RELOC);
        n = push (out, out_max, n, IT_TR_RESET);
        n = push (out, out_max, n, IT_TR_DIAG);
        break;

    case MBI_SOURCE_UNKNOWN:
        /* Empty list — caller knows the source isn't supported. */
        break;
    }

    /* Return logical count even when out_max truncated; callers can
     * detect "would have written more" by checking n > out_max. */
    return n;
}

size_t
mbi_from_labels (const char *const *labels,
                  void *const       *actions,
                  size_t             n,
                  rm_item_t         *out,
                  size_t             out_max)
{
    if (!labels || !actions || !out || out_max == 0) return 0;
    size_t written = 0;
    for (size_t i = 0; i < n && written < out_max; i++) {
        out[written].label     = labels[i];
        out[written].icon_path = NULL;
        out[written].action    = actions[i];
        written++;
    }
    return written;
}

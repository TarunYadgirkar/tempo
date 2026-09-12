/* menubar_items.h — Build radial-menu item lists for menubar dropdowns.
 *
 * Status-bar "dropdowns" (clock, wifi, iphone, tracking) and arbitrary
 * GTK appmenu items used to open vertical popovers — desktop chrome.
 * Per the radial-menu task family, they should open radial menus
 * instead.  The actual GTK click → radial_menu_show() wiring is GTK4
 * and runtime-driven; this header is the *pure* data layer the wiring
 * consumes.  Separating them keeps the item builder unit-testable
 * without spinning up a Wayland display or a GMenuModel proxy.
 *
 * Two builders:
 *
 *   * mbi_status_bar_items() — known dropdown sources in the spatial
 *     status bar.  Each source returns a small, fixed item list that
 *     the radial widget can show on the spot.
 *
 *   * mbi_from_labels()      — generic adapter for arbitrary item
 *     vectors (used by the GMenuModel walker — when an upstream app
 *     publishes an appmenu, the walker iterates the model into a flat
 *     (label, action) list and hands that to this helper).
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_MENUBAR_ITEMS_H
#define SPATIAL_MENUBAR_ITEMS_H

#include <stdbool.h>
#include <stddef.h>

/* Locally-mirrored radial-menu item struct.  The authoritative
 * definition lives in radial_menu.h alongside this file, but we copy
 * it here so the menubar items layer can build in isolation — useful
 * when the two are landed on separate branches that haven't yet
 * merged.  Field layout matches verbatim. */
#ifndef SPATIAL_RADIAL_MENU_H
typedef struct rm_item {
    const char *label;
    const char *icon_path;
    void       *action;
} rm_item_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MBI_SOURCE_UNKNOWN  = 0,
    MBI_SOURCE_CLOCK    = 1,
    MBI_SOURCE_WIFI     = 2,
    MBI_SOURCE_IPHONE   = 3,
    MBI_SOURCE_TRACKING = 4,
} mbi_source_t;

/* Look up the canonical status-bar item list for `source` and copy up
 * to `out_max` items into out[].  Returns the number of items the
 * source defines (may exceed out_max — the caller should check and
 * decide whether to truncate or grow the buffer).  Items use static
 * storage; do not free.
 *
 * The `state` argument is consulted for state-dependent items
 * (e.g. wifi: "Disconnect" only when connected). */
typedef struct {
    bool wifi_connected;
    bool iphone_connected;
    bool tracking_normal;
} mbi_state_t;

size_t mbi_status_bar_items (mbi_source_t source,
                              const mbi_state_t *state,
                              rm_item_t *out,
                              size_t out_max);

/* Generic adapter: copy `n` (label, action_cookie) pairs into out[].
 * The labels are *not* copied — the caller must keep them alive for
 * as long as the radial menu is shown.  icon_path is set to NULL.
 *
 * Returns min(n, out_max). */
size_t mbi_from_labels (const char *const *labels,
                         void *const       *actions,
                         size_t             n,
                         rm_item_t         *out,
                         size_t             out_max);

#ifdef __cplusplus
}
#endif

#endif /* SPATIAL_MENUBAR_ITEMS_H */

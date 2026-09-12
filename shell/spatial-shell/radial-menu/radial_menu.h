/* radial_menu.h — Reusable radial-menu widget API for spatial-shell.
 *
 * Why a radial menu, not a vertical list?  Vertical dropdown menus
 * are desktop chrome transplanted into XR.  Radial menus match
 * Fitts's-law-in-3D-space (every segment is equidistant from the
 * gesture origin), pair naturally with thumb-along-curled-index
 * scroll for paging, and avoid the "pointer travels down the list"
 * fatigue that vertical menus impose.
 *
 * Design rules (decision recorded 2026-05-15 — see BACKLOG.md
 * `radial-menu-widget`):
 *   * Up to 6 items per page render as a fan around `origin`.
 *   * >6 items page; small dots indicator above/below the fan.
 *   * Page changes consume `scroll` gestures (left=prev, right=next).
 *   * Segment commit happens when the user's thumb extends past the
 *     segment's outer arc (the existing `thumb_slider` mechanic).
 *   * Cancel by retracting / opening the hand; never auto-commits.
 *   * Style (colors, radii, glow) sources from theme.toml via the
 *     design-tokens system — no hard-coded look here.
 *
 * The header splits the API into two layers:
 *   - `rm_model_*`    : pure C state machine (testable without GTK).
 *   - `radial_menu_*` : GTK4 + cairo presentation layer (uses the
 *     model + the gesture event stream).  Deferred to in-person
 *     iteration; the model is the contract this header pins down.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_RADIAL_MENU_H
#define SPATIAL_RADIAL_MENU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RM_ITEMS_PER_PAGE 6

/* ----------------------------------------------------------------- */
/* Item                                                              */
/* ----------------------------------------------------------------- */

typedef struct rm_item {
    const char *label;       /* user-visible text       */
    const char *icon_path;   /* GTK icon-name OR file path; may be NULL */
    void       *action;      /* opaque cookie surfaced via the on_commit cb */
} rm_item_t;

/* ----------------------------------------------------------------- */
/* Model — paging + segment focus + commit logic                     */
/*
 *  States the model tracks:
 *    - all items the menu was shown with,
 *    - current page (0..page_count-1),
 *    - currently focused segment within the current page (or -1 for none),
 *    - "thumb_extension" scalar 0..1 used to drive commit.
 *
 *  All inputs are deterministic; tests drive the model directly.        */
/* ----------------------------------------------------------------- */

typedef struct rm_model {
    const rm_item_t *items;        /* not owned */
    size_t           item_count;
    size_t           page;         /* 0..page_count - 1 */
    int              focused;      /* -1 (no focus) or 0..items-on-page - 1 */
    float            thumb_extension; /* 0..1; commit fires when >= 1.0   */
    bool             committed;    /* set true once on commit; consumers
                                      can poll, then call rm_model_reset_commit() */
    int              committed_index; /* absolute index into items[]
                                         when committed=true              */
} rm_model_t;

/* Initialise the model around the given item list (not copied).
 * Sets page=0, focused=-1, thumb_extension=0. */
void rm_model_init (rm_model_t *m, const rm_item_t *items, size_t item_count);

/* Number of pages = ceil(item_count / RM_ITEMS_PER_PAGE).  Always >= 1. */
size_t rm_model_page_count (const rm_model_t *m);

/* Items on the current page (1..RM_ITEMS_PER_PAGE — the last page can
 * be partial). */
size_t rm_model_items_on_page (const rm_model_t *m);

/* Returns a pointer into the items[] array for the n-th item on the
 * current page, or NULL if n is out of range. */
const rm_item_t *rm_model_item_at (const rm_model_t *m, size_t n);

/* Page navigation (no-op if already at the edge — pages don't wrap).
 * Resets focused to -1 on a page change. */
void rm_model_page_next (rm_model_t *m);
void rm_model_page_prev (rm_model_t *m);

/* Set the currently-focused segment index (within the current page,
 * 0..items_on_page-1, or -1 to clear). */
void rm_model_set_focus (rm_model_t *m, int segment_on_page);

/* Set the thumb-extension scalar (clamped to 0..1).  When the value
 * crosses >= 1.0 AND a segment is focused, commit fires: the model's
 * `committed` becomes true and `committed_index` is set to the
 * absolute items[] index.  Subsequent calls do nothing until
 * rm_model_reset_commit() is called or the model is hidden. */
void rm_model_set_thumb (rm_model_t *m, float t);

/* Acknowledge a commit (call after handling it). */
void rm_model_reset_commit (rm_model_t *m);

/* ----------------------------------------------------------------- */
/* Presentation                                                     */
/*
 *  Deferred — the GTK4 + cairo rendering and gesture-event wiring
 *  land in the in-person session that designs the visual feel.
 *  These functions are declared so consumers can call the future
 *  implementation; until it lands, they no-op.                    */
/* ----------------------------------------------------------------- */

struct radial_menu_view;
typedef struct radial_menu_view radial_menu_view_t;

typedef void (*radial_menu_commit_cb)(const rm_item_t *committed_item,
                                      void *user_data);

typedef struct radial_menu_show_opts {
    const rm_item_t       *items;
    size_t                 item_count;
    float                  origin_xyz[3];  /* world-space gesture origin */
    radial_menu_commit_cb  on_commit;
    void                  *user_data;
} radial_menu_show_opts_t;

/* Allocate, show, and return a view.  Owns its internal model.
 * The deferred GTK4 implementation will subscribe to scroll +
 * thumb_slider events for the lifetime of the view.  Returns NULL
 * until that lands. */
radial_menu_view_t *radial_menu_show (const radial_menu_show_opts_t *opts);

/* Hide and destroy the view.  Safe to call with NULL. */
void radial_menu_hide (radial_menu_view_t *view);

#ifdef __cplusplus
}
#endif

#endif /* SPATIAL_RADIAL_MENU_H */

/* radial_menu.c — see radial_menu.h.
 *
 * SPDX-License-Identifier: MIT
 */

#include "radial_menu.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* ----------------------------------------------------------------- */
/* Model                                                             */
/* ----------------------------------------------------------------- */

static size_t base_for_page (size_t page) {
    return page * RM_ITEMS_PER_PAGE;
}

void
rm_model_init (rm_model_t *m, const rm_item_t *items, size_t item_count)
{
    if (!m) return;
    m->items            = items;
    m->item_count       = item_count;
    m->page             = 0;
    m->focused          = -1;
    m->thumb_extension  = 0.0f;
    m->committed        = false;
    m->committed_index  = -1;
}

size_t
rm_model_page_count (const rm_model_t *m)
{
    if (!m || m->item_count == 0) return 1;
    return (m->item_count + RM_ITEMS_PER_PAGE - 1) / RM_ITEMS_PER_PAGE;
}

size_t
rm_model_items_on_page (const rm_model_t *m)
{
    if (!m || m->item_count == 0) return 0;
    size_t base = base_for_page (m->page);
    if (base >= m->item_count) return 0;
    size_t rem = m->item_count - base;
    return rem < RM_ITEMS_PER_PAGE ? rem : RM_ITEMS_PER_PAGE;
}

const rm_item_t *
rm_model_item_at (const rm_model_t *m, size_t n)
{
    if (!m || !m->items) return NULL;
    size_t base = base_for_page (m->page);
    if (n >= rm_model_items_on_page (m)) return NULL;
    return &m->items[base + n];
}

void
rm_model_page_next (rm_model_t *m)
{
    if (!m) return;
    size_t pc = rm_model_page_count (m);
    if (m->page + 1 < pc) {
        m->page++;
        m->focused = -1;
        m->thumb_extension = 0.0f;
    }
}

void
rm_model_page_prev (rm_model_t *m)
{
    if (!m) return;
    if (m->page > 0) {
        m->page--;
        m->focused = -1;
        m->thumb_extension = 0.0f;
    }
}

void
rm_model_set_focus (rm_model_t *m, int segment_on_page)
{
    if (!m) return;
    int n = (int) rm_model_items_on_page (m);
    if (segment_on_page < 0 || segment_on_page >= n) {
        m->focused = -1;
    } else {
        m->focused = segment_on_page;
    }
}

void
rm_model_set_thumb (rm_model_t *m, float t)
{
    if (!m) return;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    m->thumb_extension = t;

    if (m->committed) return;
    if (m->focused < 0) return;
    if (t >= 1.0f) {
        m->committed       = true;
        m->committed_index = (int)(base_for_page (m->page) + (size_t)m->focused);
    }
}

void
rm_model_reset_commit (rm_model_t *m)
{
    if (!m) return;
    m->committed        = false;
    m->committed_index  = -1;
    m->thumb_extension  = 0.0f;
}

/* ----------------------------------------------------------------- */
/* Presentation — deferred to in-person session                      */
/* ----------------------------------------------------------------- */

radial_menu_view_t *
radial_menu_show (const radial_menu_show_opts_t *opts)
{
    (void) opts;
    /* GTK4 + cairo + scroll/thumb_slider wiring lands with the
     * hardware-iterated visual tuning.  For now consumers get NULL
     * back; the model API is what the wiring will consume. */
    return NULL;
}

void
radial_menu_hide (radial_menu_view_t *view)
{
    (void) view;
}

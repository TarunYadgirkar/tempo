/* test_radial_menu — Synthetic-input tests for the radial-menu model.
 *
 * Covers:
 *   - 5-item menu pages once (≤ RM_ITEMS_PER_PAGE); 9-item menu has 2 pages
 *     with the last page partial (3 items).
 *   - rm_model_page_next / page_prev respect bounds and clear focus.
 *   - Set focus → thumb extension crossing 1.0 commits with the right
 *     absolute index.
 *   - Commit fires once; further thumb activity is ignored until reset.
 *   - Setting thumb without a focused segment is harmless.
 *
 * SPDX-License-Identifier: MIT
 */

#include "radial_menu.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define EXPECT(cond) do {                                            \
    if (!(cond)) {                                                    \
        fprintf (stderr, "FAIL: %s @ %s:%d\n", #cond, __FILE__, __LINE__); \
        return 1;                                                     \
    }                                                                 \
} while (0)

static int test_five_items_single_page(void) {
    static const rm_item_t items[5] = {
        { "Copy",   NULL, (void*) 1 },
        { "Cut",    NULL, (void*) 2 },
        { "Paste",  NULL, (void*) 3 },
        { "Delete", NULL, (void*) 4 },
        { "Props",  NULL, (void*) 5 },
    };
    rm_model_t m;
    rm_model_init (&m, items, 5);

    EXPECT (rm_model_page_count (&m)    == 1);
    EXPECT (rm_model_items_on_page (&m) == 5);
    EXPECT (rm_model_item_at (&m, 0)->label[0] == 'C');
    EXPECT (rm_model_item_at (&m, 4)->label[0] == 'P');
    EXPECT (rm_model_item_at (&m, 5) == NULL);

    /* Paging at edges is a no-op. */
    rm_model_page_next (&m);
    EXPECT (m.page == 0);
    rm_model_page_prev (&m);
    EXPECT (m.page == 0);

    /* Setting an out-of-range focus clears it. */
    rm_model_set_focus (&m, 5);
    EXPECT (m.focused == -1);
    rm_model_set_focus (&m, -1);
    EXPECT (m.focused == -1);
    rm_model_set_focus (&m, 2);
    EXPECT (m.focused == 2);

    /* Thumb-extension < 1.0 doesn't commit. */
    rm_model_set_thumb (&m, 0.3f);
    EXPECT (!m.committed);
    rm_model_set_thumb (&m, 0.99f);
    EXPECT (!m.committed);

    /* Crossing 1.0 commits with absolute index 2 ("Paste"). */
    rm_model_set_thumb (&m, 1.0f);
    EXPECT (m.committed);
    EXPECT (m.committed_index == 2);
    const rm_item_t *committed = &items[m.committed_index];
    EXPECT (strcmp (committed->label, "Paste") == 0);

    /* Further thumb activity is a no-op (idempotent). */
    rm_model_set_thumb (&m, 1.0f);
    EXPECT (m.committed_index == 2);

    /* Reset releases the commit. */
    rm_model_reset_commit (&m);
    EXPECT (!m.committed);
    EXPECT (m.committed_index == -1);
    EXPECT (m.thumb_extension == 0.0f);
    return 0;
}

static int test_nine_items_two_pages(void) {
    static const rm_item_t items[9] = {
        {"a", NULL, (void*) 1}, {"b", NULL, (void*) 2}, {"c", NULL, (void*) 3},
        {"d", NULL, (void*) 4}, {"e", NULL, (void*) 5}, {"f", NULL, (void*) 6},
        {"g", NULL, (void*) 7}, {"h", NULL, (void*) 8}, {"i", NULL, (void*) 9},
    };
    rm_model_t m;
    rm_model_init (&m, items, 9);

    EXPECT (rm_model_page_count (&m)    == 2);
    EXPECT (rm_model_items_on_page (&m) == 6);
    EXPECT (rm_model_item_at (&m, 5)->label[0] == 'f');

    /* page_next advances and clears focus. */
    rm_model_set_focus (&m, 0);
    EXPECT (m.focused == 0);
    rm_model_page_next (&m);
    EXPECT (m.page == 1);
    EXPECT (m.focused == -1);
    EXPECT (rm_model_items_on_page (&m) == 3);
    EXPECT (rm_model_item_at (&m, 0)->label[0] == 'g');
    EXPECT (rm_model_item_at (&m, 2)->label[0] == 'i');
    EXPECT (rm_model_item_at (&m, 3) == NULL);

    /* Past-the-end page_next is a no-op. */
    rm_model_page_next (&m);
    EXPECT (m.page == 1);

    /* page_prev goes back. */
    rm_model_set_focus (&m, 1);
    rm_model_page_prev (&m);
    EXPECT (m.page == 0);
    EXPECT (m.focused == -1);

    /* Commit on page 1, focus 2 ("i") → absolute index 6 + 2 = 8. */
    rm_model_page_next (&m);
    rm_model_set_focus (&m, 2);
    rm_model_set_thumb (&m, 1.0f);
    EXPECT (m.committed);
    EXPECT (m.committed_index == 8);
    EXPECT (strcmp (items[m.committed_index].label, "i") == 0);
    return 0;
}

static int test_empty_and_no_focus(void) {
    rm_model_t m;

    /* Empty menu: 1 page, 0 items on page. */
    rm_model_init (&m, NULL, 0);
    EXPECT (rm_model_page_count (&m)    == 1);
    EXPECT (rm_model_items_on_page (&m) == 0);
    EXPECT (rm_model_item_at (&m, 0)    == NULL);

    /* Thumb extension without focus never commits. */
    static const rm_item_t items[3] = {
        {"x", NULL, NULL}, {"y", NULL, NULL}, {"z", NULL, NULL},
    };
    rm_model_init (&m, items, 3);
    rm_model_set_thumb (&m, 1.0f);
    EXPECT (!m.committed);
    EXPECT (m.committed_index == -1);

    /* Thumb clamping: > 1.0 is clamped before triggering commit only
     * with focus. */
    rm_model_set_focus (&m, 1);
    rm_model_set_thumb (&m, 5.0f);
    EXPECT (m.thumb_extension == 1.0f);
    EXPECT (m.committed);
    return 0;
}

int main(void) {
    int rc = 0;
    rc |= test_five_items_single_page ();
    rc |= test_nine_items_two_pages ();
    rc |= test_empty_and_no_focus ();
    if (rc == 0) printf ("ok\n");
    return rc;
}

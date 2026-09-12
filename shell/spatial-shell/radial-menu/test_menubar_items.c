/* test_menubar_items — Tests for the menubar-dropdown item builder.
 *
 * Verifies the per-source item lists and the state-dependent gating
 * (wifi disconnect only when connected; iphone reconnect vs disconnect)
 * plus the generic from-labels adapter.
 *
 * SPDX-License-Identifier: MIT
 */

#include "menubar_items.h"

#include <stdio.h>
#include <string.h>

#define EXPECT(cond) do { \
    if (!(cond)) { fprintf(stderr, "FAIL: %s @ %s:%d\n", #cond, __FILE__, __LINE__); return 1; } \
} while (0)

static int contains(const rm_item_t *items, size_t n, const char *label) {
    for (size_t i = 0; i < n; i++) {
        if (items[i].label && strcmp(items[i].label, label) == 0) return 1;
    }
    return 0;
}

static int test_clock(void) {
    rm_item_t out[6] = {0};
    size_t n = mbi_status_bar_items(MBI_SOURCE_CLOCK, NULL, out, 6);
    EXPECT(n == 3);
    EXPECT(contains(out, n, "Calendar"));
    EXPECT(contains(out, n, "Set time"));
    EXPECT(contains(out, n, "Timezone"));
    return 0;
}

static int test_wifi_connected(void) {
    mbi_state_t s = { .wifi_connected = true };
    rm_item_t out[6] = {0};
    size_t n = mbi_status_bar_items(MBI_SOURCE_WIFI, &s, out, 6);
    EXPECT(n == 4);
    EXPECT(contains(out, n, "Toggle Wi-Fi"));
    EXPECT(contains(out, n, "Pick network"));
    EXPECT(contains(out, n, "Disconnect"));
    EXPECT(contains(out, n, "Wi-Fi settings"));
    return 0;
}

static int test_wifi_not_connected(void) {
    mbi_state_t s = {0};
    rm_item_t out[6] = {0};
    size_t n = mbi_status_bar_items(MBI_SOURCE_WIFI, &s, out, 6);
    EXPECT(n == 3);
    EXPECT(!contains(out, n, "Disconnect"));
    return 0;
}

static int test_iphone_states(void) {
    rm_item_t out[6] = {0};
    mbi_state_t connected = { .iphone_connected = true };
    size_t n = mbi_status_bar_items(MBI_SOURCE_IPHONE, &connected, out, 6);
    EXPECT(n == 2);
    EXPECT(contains(out, n, "Disconnect"));
    EXPECT(!contains(out, n, "Reconnect"));
    EXPECT(contains(out, n, "Re-pair"));

    mbi_state_t disc = {0};
    n = mbi_status_bar_items(MBI_SOURCE_IPHONE, &disc, out, 6);
    EXPECT(n == 2);
    EXPECT(contains(out, n, "Reconnect"));
    EXPECT(!contains(out, n, "Disconnect"));
    return 0;
}

static int test_tracking(void) {
    rm_item_t out[6] = {0};
    size_t n = mbi_status_bar_items(MBI_SOURCE_TRACKING, NULL, out, 6);
    EXPECT(n == 3);
    EXPECT(contains(out, n, "Relocalise"));
    EXPECT(contains(out, n, "Reset map"));
    EXPECT(contains(out, n, "Diagnostics"));
    return 0;
}

static int test_unknown(void) {
    rm_item_t out[6] = {0};
    size_t n = mbi_status_bar_items(MBI_SOURCE_UNKNOWN, NULL, out, 6);
    EXPECT(n == 0);
    return 0;
}

static int test_truncation(void) {
    /* out_max smaller than the source's logical count.  The function
     * writes what fits and returns the logical count so the caller can
     * detect truncation. */
    rm_item_t out[2] = {0};
    size_t n = mbi_status_bar_items(MBI_SOURCE_CLOCK, NULL, out, 2);
    EXPECT(n == 3);
    EXPECT(strcmp(out[0].label, "Calendar") == 0);
    EXPECT(strcmp(out[1].label, "Set time") == 0);
    return 0;
}

static int test_from_labels(void) {
    const char *labels[3] = { "Open", "Save", "Close" };
    void       *actions[3] = { (void*) 1, (void*) 2, (void*) 3 };
    rm_item_t out[10] = {0};
    size_t n = mbi_from_labels(labels, actions, 3, out, 10);
    EXPECT(n == 3);
    EXPECT(strcmp(out[0].label, "Open") == 0);
    EXPECT(out[0].action == (void*) 1);
    EXPECT(strcmp(out[2].label, "Close") == 0);
    EXPECT(out[2].action == (void*) 3);

    /* Empty list. */
    EXPECT(mbi_from_labels(NULL, NULL, 0, out, 10) == 0);

    /* Truncation: only fits 2 of 3. */
    n = mbi_from_labels(labels, actions, 3, out, 2);
    EXPECT(n == 2);
    return 0;
}

int main(void) {
    int rc = 0;
    rc |= test_clock();
    rc |= test_wifi_connected();
    rc |= test_wifi_not_connected();
    rc |= test_iphone_states();
    rc |= test_tracking();
    rc |= test_unknown();
    rc |= test_truncation();
    rc |= test_from_labels();
    if (rc == 0) printf("ok\n");
    return rc;
}

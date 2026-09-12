/* lm.c — see lm.h.
 *
 * Implements the bigram model.  Counts are computed once at startup
 * from the embedded `lm_corpus.h` text.  P(c | prev_c) uses
 * Laplace smoothing with α = 0.5 so unseen bigrams still get a
 * small but nonzero probability.
 */

#include "lm.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "lm_corpus.h"

#define XK_a         0x0061
#define XK_z         0x007a
#define XK_space     0x0020
#define XK_BackSpace 0xff08
#define XK_Return    0xff0d

#define LAPLACE_ALPHA 0.5f

struct lm_t {
    /* Bigram log-probs: [prev_c][c]. */
    float logp[LM_VOCAB][LM_VOCAB];
    /* Unigram log-probs. */
    float logp_unigram[LM_VOCAB];
};

static int
char_to_idx (char ch)
{
    if (ch >= 'a' && ch <= 'z') return ch - 'a';
    if (ch >= 'A' && ch <= 'Z') return ch - 'A';
    if (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r')
        return LM_IDX_SPACE;
    /* Skip punctuation by returning a sentinel. */
    return -1;
}

uint32_t
lm_idx_to_keysym (int idx)
{
    if (idx >= 0 && idx < 26) return (uint32_t) (XK_a + idx);
    if (idx == LM_IDX_SPACE)  return XK_space;
    if (idx == LM_IDX_BS)     return XK_BackSpace;
    if (idx == LM_IDX_RET)    return XK_Return;
    return 0;
}

int
lm_keysym_to_idx (uint32_t keysym)
{
    if (keysym >= (uint32_t) XK_a && keysym <= (uint32_t) XK_z)
        return (int) (keysym - XK_a);
    if (keysym == XK_space)     return LM_IDX_SPACE;
    if (keysym == XK_BackSpace) return LM_IDX_BS;
    if (keysym == XK_Return)    return LM_IDX_RET;
    return -1;
}

bool
lm_is_character (uint32_t keysym)
{
    return (keysym >= (uint32_t) XK_a && keysym <= (uint32_t) XK_z)
           || keysym == (uint32_t) XK_space;
}

float
lm_log_prob_neutral (void)
{
    return logf (1.0f / (float) LM_VOCAB);
}

lm_t *
lm_create (void)
{
    lm_t *self = calloc (1, sizeof (*self));
    if (!self) return NULL;

    /* Count bigrams + unigrams from the corpus. */
    uint32_t bigram[LM_VOCAB][LM_VOCAB] = {{0}};
    uint32_t unigram[LM_VOCAB] = {0};
    int prev = -1;
    for (size_t i = 0; i < kLmCorpusLen; ++i) {
        int c = char_to_idx (kLmCorpus[i]);
        if (c < 0) {
            prev = -1;   /* punctuation resets context */
            continue;
        }
        unigram[c]++;
        if (prev >= 0) bigram[prev][c]++;
        prev = c;
    }

    /* Apply Laplace smoothing and convert to log-probs. */
    const float alpha = LAPLACE_ALPHA;
    const float vocab_alpha = alpha * (float) LM_VOCAB;
    uint64_t total_unigram = 0;
    for (int c = 0; c < LM_VOCAB; ++c) total_unigram += unigram[c];
    for (int c = 0; c < LM_VOCAB; ++c) {
        float p = ((float) unigram[c] + alpha)
                  / ((float) total_unigram + vocab_alpha);
        self->logp_unigram[c] = logf (p);
    }
    for (int prev_c = 0; prev_c < LM_VOCAB; ++prev_c) {
        uint64_t row_total = 0;
        for (int c = 0; c < LM_VOCAB; ++c) row_total += bigram[prev_c][c];
        for (int c = 0; c < LM_VOCAB; ++c) {
            float p = ((float) bigram[prev_c][c] + alpha)
                      / ((float) row_total + vocab_alpha);
            self->logp[prev_c][c] = logf (p);
        }
    }
    return self;
}

void
lm_destroy (lm_t *self)
{
    free (self);
}

float
lm_log_prob (const lm_t *self, int prev_c, int c)
{
    if (!self || prev_c < 0 || prev_c >= LM_VOCAB
              || c < 0 || c >= LM_VOCAB)
        return -50.0f;   /* effectively -inf */
    return self->logp[prev_c][c];
}

float
lm_log_prob_unigram (const lm_t *self, int c)
{
    if (!self || c < 0 || c >= LM_VOCAB) return -50.0f;
    return self->logp_unigram[c];
}

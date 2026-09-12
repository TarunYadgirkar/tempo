/* lm.h — character n-gram language model for the typing decoder.
 *
 * v1: bigram model trained from an embedded ~5KB English corpus.
 * Returns log-probabilities P(c | prev_c) with Laplace smoothing.
 *
 * The model intentionally keeps a small public API so the agent
 * iteration loop can swap it for a higher-quality LM (KenLM-built
 * 5-gram, trigram word model, etc.) without touching the decoder.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_LM_H
#define SPATIAL_KEYBOARD_DECODER_LM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Alphabet: 26 lowercase letters + space + backspace + return.
 * Indices 0..25 are 'a'..'z'; 26 is space; 27 is backspace; 28 is
 * return.  Indices >= LM_VOCAB are unknown. */
#define LM_VOCAB    29
#define LM_IDX_SPACE 26
#define LM_IDX_BS    27
#define LM_IDX_RET   28

typedef struct lm_t lm_t;

/* Build the bigram model from the embedded corpus.  Returns NULL on
 * allocation failure.  Idempotent and threadsafe — call once at
 * startup, share the pointer. */
lm_t *lm_create (void);
void lm_destroy (lm_t *self);

/* Map a vocab index back to its X11 keysym (XK_a, XK_space, etc.). */
uint32_t lm_idx_to_keysym (int idx);
/* Map a keysym to a vocab index, or -1 if not in vocab. */
int lm_keysym_to_idx (uint32_t keysym);

/* True when `keysym` is a character the corpus can actually contain (a-z or
 * space).  BackSpace and Return hold vocabulary slots but appear zero times in
 * the corpus, and shift / the layer toggle are not in the vocabulary at all —
 * so a unigram lookup hands all four a probability ~1e-4 of a letter's, which
 * is smoothing noise, not evidence.  Score those with lm_log_prob_neutral. */
bool lm_is_character (uint32_t keysym);

/* The max-entropy "the model has no opinion" log-prob, log(1 / LM_VOCAB).
 * Keeps a control key competitive with the letters it neighbours instead of
 * losing to them on a prior that was never fitted for it. */
float lm_log_prob_neutral (void);

/* log P(c | prev_c).  Both indices in [0, LM_VOCAB).  Returns a
 * large negative number for unseen bigrams (Laplace-smoothed). */
float lm_log_prob (const lm_t *self, int prev_c, int c);

/* log P(c) — uniform prior fallback when no history is available
 * (start of utterance). */
float lm_log_prob_unigram (const lm_t *self, int c);

#ifdef __cplusplus
}
#endif

#endif

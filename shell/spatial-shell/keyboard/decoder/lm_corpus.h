/* lm_corpus.h — embedded English corpus for the v1 bigram LM.
 *
 * ~5 KB of common English text drawn from:
 *   - the MacKenzie & Soukoreff phrase-set spirit (common-word
 *     pangrams + everyday declaratives), and
 *   - widely-used English text fragments (public domain Aesop's
 *     fables snippets).
 *
 * Composition rules:
 *   - lowercase only
 *   - apostrophes / commas / periods stripped (punctuation breaks
 *     the bigram chain in lm.c via char_to_idx returning -1)
 *   - spaces between sentences
 *
 * Replaced by the agent iteration loop later with a real corpus
 * (Wikipedia + OpenSubtitles, built into KenLM .binary files).  Until
 * then, this gives the decoder a reasonable English prior — letters
 * like t/h/e are weighted as common, q is rare, q-u bigram is heavy.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_LM_CORPUS_H
#define SPATIAL_KEYBOARD_DECODER_LM_CORPUS_H

#include <stddef.h>

static const char kLmCorpus[] =
"the quick brown fox jumps over the lazy dog "
"a quick movement of the enemy will jeopardize six gunboats "
"sphinx of black quartz judge my vow "
"pack my box with five dozen liquor jugs "
"how vexingly quick daft zebras jump "
"the five boxing wizards jump quickly "
"jackdaws love my big sphinx of quartz "
"bright vixens jump dozy fowl quack "
"a wizard with a black cap and a fluffy beard "
"my watch fell in the water this morning "
"prevailing wind from the east "
"never too rich and never too thin "
"breathing is difficult under the weight of the snow "
"i can see the rings on saturn through the telescope "
"physics and chemistry are hard but rewarding "
"my favorite place to visit is the small bakery on main "
"three two one zero blast off into the bright black sky "
"my preferred treat is dark chocolate from belgium "
"the storm lasted all night and into the next afternoon "
"old saint nicholas comes tonight bringing gifts to all "
"always carry an umbrella with you when the sky looks gray "
"my flight leaves in the morning and arrives at noon "
"this is a very good idea and i hope you try it soon "
"we ran around the playground until our legs were tired "
"did you have a good time at the party last weekend "
"the museum opens at ten and the exhibits are wonderful "
"we missed the bus by seconds and had to wait an hour "
"please put the kettle on so we can have some tea "
"a clever bird once tried to drink from a tall pitcher "
"a stork once was invited to dinner by a fox "
"the lion and the mouse became unlikely friends one day "
"a hungry wolf saw a small kid grazing on a green hill "
"the boy who cried wolf had no one believe his shouts "
"slow and steady wins the race said the wise old tortoise "
"a piece of bread fell from a window of the bakery "
"the wind and the sun had a long argument one morning "
"the north wind blew with all its might but the man held on "
"the sun warmed the traveler gently and he removed his coat "
"a frog and an ox had a discussion about size and pride "
"the country mouse visited the city and quickly went home "
"a thirsty crow saw a pitcher with a little water inside "
"the milkmaid dreamed of riches as she walked to market "
"the wolves admitted the dogs to their company one day "
"a fly settled on the head of a bald man who was sleeping "
"a fisherman cast his net into the sea and waited "
"the wind whistled through the trees of the dark forest "
"sometimes the simplest path leads to the greatest reward "
"if you keep practicing each day your skills will grow "
"the morning fog lifted slowly from the surface of the lake "
"birds sang from every branch as the sun rose above the hills "
"a small child carried a basket of apples down the long path "
"the village square filled with music as the festival began "
"hard work and patience are the keys to most lasting success "
"the river flowed quietly past the village every morning "
"a kind word costs nothing yet means a great deal to the hearer "
"better to light a single candle than to curse the darkness "
"the only way to do great work is to love what you do "
"in the middle of difficulty lies opportunity for those who look "
"some say the world ends in fire some say in ice "
"two roads diverged in a wood and i took the one less traveled "
"so we beat on boats against the current borne back into the past "
"all happy families are alike each unhappy family is unhappy its own way "
"it was the best of times it was the worst of times "
"call me ishmael and let us begin our voyage on the wide sea "
"there once lived a king who ruled over a small and peaceful land "
"the cat sat on the mat watching birds through the window for hours "
"a slow loris climbed branch by branch toward the moon "
"the kettle whistled and the cups clinked on the saucers "
"please pass the salt and the pepper and the bread basket too "
"my grandmother told me stories about the old country every winter "
"the children gathered around the fireplace to hear about long ago "
"the rain tapped softly on the windowpane all through the night "
"i love the smell of fresh bread on a quiet sunday morning "
"the wind in the willows by the river spoke of summer ending "
"a careful reader will find many surprises in this short book "
"the kitten chased a moth across the moonlit garden path "
"to be or not to be that is the question for many of us "
"all that we are is the result of what we have thought before "
"the journey of a thousand miles begins with a single step "
"every cloud carries with it the silver edge of hope tomorrow "
"a friend who listens well is worth more than a stranger who lectures "
"learn to write before you learn to speak in a crowded room "
"keep your face always toward the sunshine and shadows fall behind "
"the early bird catches the worm but the second mouse gets the cheese "
"a stitch in time saves nine reads the old proverb on the wall "
"birds of a feather often flock together against the wind "
"do not put off until tomorrow what can be done quietly today "
"the pen is mightier than the sword in the hands of a careful writer "
"actions speak louder than words and silence speaks louder still "
"absence makes the heart grow fonder but presence keeps it warm "
"a chain is only as strong as its weakest link in the long run "
"beauty is in the eye of the beholder said the old painter laughing "
"better safe than sorry said the careful traveler to the bold one "
"curiosity may have killed the cat but satisfaction brought it back "
"every dog has its day even the small one curled by the fire "
"good things come to those who wait but better things to those who work "
"hope for the best but prepare for the worst said the captain "
"if at first you do not succeed try and try and try again ";

static const size_t kLmCorpusLen = sizeof (kLmCorpus) - 1;

#endif

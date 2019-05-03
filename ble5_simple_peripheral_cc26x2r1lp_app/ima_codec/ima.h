/*
 * (c) redsh, 2010
 * mailto: californ251@gmail.com
 */

/* -------------------------------------------------------------------------- */

#ifndef __ima_h__
#define __ima_h__

/* -------------------------------------------------------------------------- */

#include <stdlib.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */

typedef struct
{
	int current;
	int stepindex;
} ima_state;

/* -------------------------------------------------------------------------- */

void ima_encode_mono(ima_state *state, void *dest, void *src, size_t length);
void ima_decode_mono(ima_state *state, void *dest, void *src, size_t length);

/* -------------------------------------------------------------------------- */

#endif

/* -------------------------------------------------------------------------- */

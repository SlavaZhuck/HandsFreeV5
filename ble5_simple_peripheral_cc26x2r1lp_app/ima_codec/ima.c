/*
 * (c) redsh, 2010
 * mailto: californ251@gmail.com
 */

/* -------------------------------------------------------------------------- */

#include "ima.h"

/* -------------------------------------------------------------------------- */

int ima_index_table[] =
{
    -1, -1, -1, -1, 2, 4, 6, 8
};

/* -------------------------------------------------------------------------- */

int ima_step_table[] =
{
        7,     8,     9,    10,    11,    12,    13,    14,
       16,    17,    19,    21,    23,    25,    28,    31,
       34,    37,    41,    45,    50,    55,    60,    66,
       73,    80,    88,    97,   107,   118,   130,   143,
      157,   173,   190,   209,   230,   253,   279,   307,
      337,   371,   408,   449,   494,   544,   598,   658,
      724,   796,   876,   963,  1060,  1166,  1282,  1411,
     1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,
     3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
     7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767
};

/* -------------------------------------------------------------------------- */

__inline int ima_encode_sample(ima_state *state, int sample)
{
    int current, step;
    uint8_t stepindex;
    int delta;
    int diff;
    int value = 0;

    current = state->current;
    stepindex = state->stepindex;
    step = ima_step_table[stepindex];

    //calculate delta
    delta = sample - current;

    //value = delta / step
    //diff = (value + 0.5) * step / 4
    if(delta < 0)
    {
        value |= 8;
        delta = -delta;
    }
    diff = step >> 3;
    if(delta > step)
    {
        value |= 4;
        delta -= step;
        diff += step;
    }
    step >>= 1;
    if(delta > step)
    {
        value |= 2;
        delta -= step;
        diff += step;
    }
    step >>= 1;
    if(delta > step)
    {
        value |= 1;
        diff += step;
    }

    //update current sample
    if(value & 8)
    {
        current -= diff;
        if(current < -32768)
            current = -32768;
    }
    else
    {
        current += diff;
        if(current > 32767)
            current = 32767;
    }

    //update step
    stepindex += ima_index_table[value & 7];
    if(stepindex < 0)
        stepindex = 0;
    if(stepindex > 88)
        stepindex = 88;

    state->current = current;
    state->stepindex = stepindex;

    return value;
}

/* -------------------------------------------------------------------------- */

__inline int ima_decode_sample(ima_state *state, int value)
{
    int current, step, diff;
    uint8_t stepindex;

    current = state->current;
    stepindex = state->stepindex;
    step = ima_step_table[stepindex];

    //diff = (value + 0.5) * step / 4
    diff = step >> 3;
    if(value & 1) diff += step >> 2;
    if(value & 2) diff += step >> 1;
    if(value & 4) diff += step;

    //update sample
    if(value & 8)
    {
        current -= diff;
        if(current < -32768)
            current = -32768;
    }
    else
    {
        current += diff;
        if(current > 32767)
            current = 32767;
    }

    //update step
    stepindex += ima_index_table[value & 7];
    if(stepindex < 0)
        stepindex = 0;
    if(stepindex > 88)
        stepindex = 88;

    state->current = current;
    state->stepindex = stepindex;

    return current;
}

/* -------------------------------------------------------------------------- */

void ima_encode_mono(ima_state *state, void *dest, void *src, size_t length)
{
    short *ps = src;
    unsigned char *pd = dest;

    while(length >= 4)
    {
        *pd = ima_encode_sample(state, *(ps++));
        *pd |= ima_encode_sample(state, *(ps++)) << 4;
        pd++;
        length -= 4;
    }
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

void ima_decode_mono(ima_state *state, void *dest, void *src, size_t length)
{
    unsigned char *ps = src;
    short *pd = dest;

    while(length >= 1)
    {
        *(pd++) = ima_decode_sample(state, *ps & 0x0f);
        *(pd++) = ima_decode_sample(state, *ps >> 4);
        ps++;
        length--;
    }
}

/* -------------------------------------------------------------------------- */

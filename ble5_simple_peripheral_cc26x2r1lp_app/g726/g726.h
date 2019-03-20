#include <stdint.h>

void g726_16_encode(int16_t *pcm,
                 int8_t *bitstream);

void g726_16_decode(int8_t *bitstream,
                 int16_t *pcm);

void g726_24_encode(int16_t *pcm,
                 int8_t *bitstream);

void g726_24_decode(int8_t *bitstream,
                 int16_t *pcm);

void g726_32_encode(int16_t *pcm,
                 int8_t *bitstream);

void g726_32_decode(int8_t *bitstream,
                 int16_t *pcm);

void g726_40_encode(int16_t *pcm,
                 int8_t *bitstream);

void g726_40_decode(int8_t *bitstream,
                 int16_t *pcm);

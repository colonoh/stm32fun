#pragma once

#define AUDIO_CLIP_COUNT 21

typedef struct {
  uint32_t start;
  uint32_t length;
} AudioClip;

static const AudioClip audio_clips[AUDIO_CLIP_COUNT] = {
  // Zareyoustillthere
  { 0x000000, 107034 },
  // Zcoffeeinmycoffeehole
  { 0x01A21A, 160530 },
  // Zgetmad
  { 0x04152C, 78610 },
  // Zi_like_your_style
  { 0x05483E, 287590 },
  // Zradio-cropped-2
  { 0x09ABA4, 1412738 },
  // abcdelicious
  { 0x1F3A26, 281122 },
  // abcsingwithme
  { 0x238448, 1322530 },
  // bubbles
  { 0x37B26A, 125986 },
  // chimesounds
  { 0x399E8C, 115234 },
  // creamandsugar
  { 0x3B60AE, 410146 },
  // goodjob
  { 0x41A2D0, 136738 },
  // happyandyouknowit
  { 0x43B8F2, 473122 },
  // hyper
  { 0x4AF114, 98338 },
  // iknewyoucouldbrewit
  { 0x4C7136, 219682 },
  // onetwothreemoresugarplease
  { 0x4FCB58, 224290 },
  // onetwothreesingwithme
  { 0x53377A, 685090 },
  // pipessound
  { 0x5DAB9C, 96802 },
  // redorangeyellowgreenandblue
  { 0x5F25BE, 883234 },
  // sipbleepahhh
  { 0x6C9FE0, 195106 },
  // someotherpipesthing
  { 0x6F9A02, 122914 },
  // thanksalatte
  { 0x717A24, 253474 },
};

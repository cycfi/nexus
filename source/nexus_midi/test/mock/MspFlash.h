// Host mock of Energia's MspFlash: two RAM-backed 64-byte segments that
// behave like flash (erased = 0xff, write-once-per-bit is not enforced —
// the firmware ring-buffer logic doesn't need it for functional testing).
#pragma once
#include <stdint.h>
#include <string.h>

struct _MockFlashMem
{
   unsigned char seg_b[64];
   unsigned char seg_c[64];
   _MockFlashMem()
   {
      memset(seg_b, 0xff, sizeof(seg_b));   // power-on = erased
      memset(seg_c, 0xff, sizeof(seg_c));
   }
};
extern _MockFlashMem _flashmem;
#define SEGMENT_B (_flashmem.seg_b)
#define SEGMENT_C (_flashmem.seg_c)

struct _FlashT
{
   void erase(unsigned char* seg) { memset(seg, 0xff, 64); }
   void write(unsigned char* dst, unsigned char* src, int n)
   {
      memcpy(dst, src, n);
   }
};
extern _FlashT Flash;

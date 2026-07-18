# LZSA1 / LZSA2 license

LZAN ships the LZSA1/LZSA2 encoders (`src/lzsa1.rs`, `src/lzsa2.rs`) and the ported 6502 decoders (`decrunchers/lzsa1-marty*.s`, `decrunchers/lzsa2-marty*.s`). The upstream work is by Emmanuel Marty, under Zlib.

The 6502 decoders (`decompress_small_v1.asm`, `decompress_small_v2.asm`) carry the same Zlib license, Copyright (c) 2019 Emmanuel Marty. Upstream `matchfinder.c` is CC0 but was not used here; the match finder is reimplemented.

Upstream: https://github.com/emmanuel-marty/lzsa/blob/master/LICENSE.zlib.md

---

Copyright (c) 2019 Emmanuel Marty

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

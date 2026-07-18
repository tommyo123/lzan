# Shrinkler license

LZAN ships the Shrinkler encoder (`src/shrinkler.rs`) and the ported 6502 unShrinkler decoder (`decrunchers/shrinkler-atari8xxl-unshrinkler*.s`). The upstream work is by Aske Simon Christensen, under Shrinkler license (permissive).

The Shrinkler license (Aske Simon Christensen) follows. The 6502 unShrinkler decoder is a separate work by Krzysztof "XXL" Dudek and Piotr Fusik under the Zlib license, included after it.

Upstream: https://raw.githubusercontent.com/askeksa/Shrinkler/master/LICENSE.txt

---

Shrinkler executable file compressor for Amiga

Copyright 1999-2022 Aske Simon Christensen, with exceptions noted below.

Permission is hereby granted to anyone obtaining a copy of this software
package (including accompanying documentation) to compile, use, copy,
modify, merge and/or distribute it, in whole or in part, subject to the
following conditions:

- Distribution in source code form must include a copy of this license.

- Distribution in binary form must not be misattributed, i.e. you must
  not claim (implicitly or explicitly) that you wrote it yourself.

- Distribution of the decrunch headers (Header.S, MiniHeader.S,
  OverlapHeader.S, and the .bin and .dat files generated from them) in
  binary form as part of an Amiga executable is not restricted by this
  license and does not require attribution.
  In particular, output executables from Shrinkler (which contain code
  from the decrunch headers) are to be considered original works of the
  author(s) of the corresponding input executables.

- The data decompression code (ShrinklerDecompress.S) is distributed
  alongside the Shrinkler binaries in the official archives and has its
  own license stated inside the file.

Exceptions:

- doshunks.h is part of the Amiga SDK and is Copyright 1989-1993
  Commodore-Amiga, Inc.

---

## Zlib: 6502 unShrinkler decoder

Copyright (c) 2021 Krzysztof 'XXL' Dudek and Piotr '0xF' Fusik

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

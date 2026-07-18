# Exomizer license

LZAN ships the ported 6502 decruncher (`decrunchers/exomizer-lind-mem*.s`); the Exomizer v3 encoder in `src/exo3.rs` is an independent reimplementation of the format. The upstream work is by Magnus Lind, under Zlib.

This is the license of the Exomizer decruncher (`exodecrunch.s`), which grants use for any purpose including commercial applications. The Exomizer compressor tool sources carry a different, more restrictive license, and the upstream Z80 decruncher (LGPL) and bison-generated parser (GPL) are not used in this project.

Upstream: https://raw.githubusercontent.com/bitshifters/exomizer/master/exomizer/decruncher/exodecrunch.s

---

Copyright (c) 2002, 2003 Magnus Lind.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from
the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
  claim that you wrote the original software. If you use this software in a
  product, an acknowledgment in the product documentation would be
  appreciated but is not required.

  2. Altered source versions must be plainly marked as such, and must not
  be misrepresented as being the original software.

  3. This notice may not be removed or altered from any distribution.

  4. The names of this software and/or it's copyright holders may not be
  used to endorse or promote products derived from this software without
  specific prior written permission.

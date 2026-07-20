//! Shared synthetic corpora for match-finder / encoder equivalence tests and the
//! `finderbench` binary. Deterministic, std-only.

/// Small xorshift-free LCG so every generator is reproducible.
#[inline]
fn lcg(state: &mut u32) -> u32 {
    *state = state.wrapping_mul(1664525).wrapping_add(1013904223);
    *state
}

/// All zeros.
pub fn zeros(n: usize) -> Vec<u8> {
    vec![0u8; n]
}

/// The C64 power-on RAM pattern: alternating 64-byte runs of $00 and $FF.
pub fn power_on(n: usize) -> Vec<u8> {
    (0..n)
        .map(|i| if (i / 64) % 2 == 0 { 0x00 } else { 0xff })
        .collect()
}

/// Incompressible pseudo-random bytes.
pub fn noise(n: usize) -> Vec<u8> {
    let mut s = 0x1234_5678u32;
    (0..n).map(|_| (lcg(&mut s) >> 24) as u8).collect()
}

/// Mixed realistic-ish content: code-like, tables, text, zero pockets, noise.
pub fn mixed(n: usize) -> Vec<u8> {
    let mut s = 0xdead_beefu32;
    let mut out = Vec::with_capacity(n);
    let text = b"the quick brown fox jumps over the lazy dog. ";
    let ops: [u8; 16] = [
        0xa9, 0x8d, 0xad, 0x20, 0x60, 0xa2, 0xbd, 0x9d, 0xe8, 0xd0, 0x4c, 0xa0, 0xb1, 0x91, 0xc8,
        0xea,
    ];
    while out.len() < n {
        match (lcg(&mut s) >> 28) % 6 {
            0 => {
                let k = 1 + (lcg(&mut s) as usize % 400);
                out.extend(std::iter::repeat(0u8).take(k));
            }
            1 => {
                for _ in 0..(1 + lcg(&mut s) as usize % 200) {
                    out.push((lcg(&mut s) >> 24) as u8);
                }
            }
            2 => {
                for _ in 0..(1 + lcg(&mut s) as usize % 8) {
                    out.extend_from_slice(text);
                }
            }
            3 => {
                for _ in 0..(1 + lcg(&mut s) as usize % 300) {
                    out.push(ops[(lcg(&mut s) >> 24) as usize % ops.len()]);
                    if (lcg(&mut s) >> 30) == 0 {
                        out.push((lcg(&mut s) >> 24) as u8);
                        out.push((lcg(&mut s) >> 24) as u8);
                    }
                }
            }
            4 => {
                let p = 1 + (lcg(&mut s) as usize % 17);
                for k in 0..(1 + lcg(&mut s) as usize % 500) {
                    out.push((k % p) as u8);
                }
            }
            _ => {
                let b = (lcg(&mut s) >> 24) as u8;
                out.extend(std::iter::repeat(b).take(1 + lcg(&mut s) as usize % 100));
            }
        }
    }
    out.truncate(n);
    out
}

/// 6502-code-like bytes: opcodes from a small set plus operands.
pub fn code_like(n: usize) -> Vec<u8> {
    let mut s = 0x0bad_c0deu32;
    let ops: [u8; 12] = [
        0xa9, 0x8d, 0xad, 0x20, 0x60, 0xa2, 0xbd, 0x9d, 0xe8, 0xd0, 0x4c, 0xea,
    ];
    let mut out = Vec::with_capacity(n + 4);
    while out.len() < n {
        out.push(ops[(lcg(&mut s) >> 24) as usize % ops.len()]);
        let k = (lcg(&mut s) >> 30) as usize;
        for _ in 0..k {
            out.push((lcg(&mut s) >> 20) as u8);
        }
    }
    out.truncate(n);
    out
}

/// Sparse: mostly zero with small pockets of data.
pub fn sparse(n: usize) -> Vec<u8> {
    let mut s = 0x5555_aaaau32;
    let mut out = vec![0u8; n];
    let mut i = 0usize;
    while i < n {
        i += 1 + lcg(&mut s) as usize % 300;
        let k = 1 + lcg(&mut s) as usize % 20;
        for j in 0..k {
            if i + j < n {
                out[i + j] = (lcg(&mut s) >> 24) as u8;
            }
        }
        i += k;
    }
    out
}

/// Highly periodic data (short repeating cycle).
pub fn periodic(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i % 13) as u8).collect()
}

/// Repeated fixed phrases with occasional variation.
pub fn phrases(n: usize) -> Vec<u8> {
    let mut s = 0x1357_9bdfu32;
    let bank: [&[u8]; 4] = [
        b"LOAD\"*\",8,1:RUN",
        b"THE QUICK BROWN FOX",
        b"0123456789ABCDEF",
        b"..........",
    ];
    let mut out = Vec::with_capacity(n + 32);
    while out.len() < n {
        out.extend_from_slice(bank[(lcg(&mut s) >> 24) as usize % bank.len()]);
        if (lcg(&mut s) >> 29) == 0 {
            out.push((lcg(&mut s) >> 24) as u8);
        }
    }
    out.truncate(n);
    out
}

/// Two-symbol alphabet: worst case for suffix-tree depth after all-zeros.
pub fn binary_ish(n: usize) -> Vec<u8> {
    let mut s = 0x2468_ace0u32;
    (0..n).map(|_| ((lcg(&mut s) >> 31) as u8) * 0xff).collect()
}

/// Long runs of a single byte separated by short markers.
pub fn runs(n: usize) -> Vec<u8> {
    let mut s = 0x0f0f_1e1eu32;
    let mut out = Vec::with_capacity(n + 8);
    while out.len() < n {
        let b = (lcg(&mut s) >> 30) as u8;
        let k = 1 + lcg(&mut s) as usize % 500;
        out.extend(std::iter::repeat(b).take(k));
        out.push(0xa5);
    }
    out.truncate(n);
    out
}

/// Fibonacci word over {0,1} — pathological LCP structure.
pub fn fibonacci(n: usize) -> Vec<u8> {
    let mut a: Vec<u8> = vec![0];
    let mut b: Vec<u8> = vec![0, 1];
    while b.len() < n {
        let mut c = b.clone();
        c.extend_from_slice(&a);
        a = b;
        b = c;
    }
    b.truncate(n);
    b
}

/// The named 65008-byte benchmark cases.
pub fn bench_cases() -> Vec<(&'static str, Vec<u8>)> {
    const N: usize = 65008;
    vec![
        ("zeros", zeros(N)),
        ("power-on", power_on(N)),
        ("noise", noise(N)),
        ("mixed", mixed(N)),
    ]
}

/// A varied corpus of (name, data) at many sizes, for equivalence testing.
pub fn corpus(sizes: &[usize]) -> Vec<(String, Vec<u8>)> {
    let gens: [(&str, fn(usize) -> Vec<u8>); 11] = [
        ("zeros", zeros),
        ("power-on", power_on),
        ("noise", noise),
        ("mixed", mixed),
        ("code-like", code_like),
        ("sparse", sparse),
        ("periodic", periodic),
        ("phrases", phrases),
        ("binary-ish", binary_ish),
        ("runs", runs),
        ("fibonacci", fibonacci),
    ];
    let mut out = Vec::new();
    for &sz in sizes {
        for (name, f) in gens.iter() {
            out.push((format!("{name}/{sz}"), f(sz)));
        }
    }
    out
}

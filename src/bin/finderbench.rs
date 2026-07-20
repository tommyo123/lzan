//! Brute-force vs output-sensitive LZSA1 match finder: wall clock, compressed
//! size and byte-identity, over the four named 65008-byte cases.
//!
//! Run with `cargo run --release --bin finderbench`.

use std::time::Instant;

use lzan::lzsa1::compress_lzsa1_with;
use lzan::matchfinder::{find_matches_exact, find_matches_fast};
use lzan::testcorpus;

fn main() {
    const MIN_MATCH: usize = 3;
    const MAX_OFFSET: usize = 0xffff;

    println!(
        "{:<10} {:>7} {:>11} {:>11} {:>8} {:>10} {:>10} {:>10}",
        "case", "bytes", "brute ms", "fast ms", "speedup", "brute out", "fast out", "identical"
    );
    println!("{}", "-".repeat(84));

    let mut all_identical = true;
    let mut all_faster = true;

    for (name, data) in testcorpus::bench_cases() {
        let n = data.len();
        let max_len = 0xffffusize.min(n.max(1));

        // Match-finder only.
        let t0 = Instant::now();
        let brute = find_matches_exact(&data, MIN_MATCH, MAX_OFFSET, max_len);
        let brute_ms = t0.elapsed().as_secs_f64() * 1000.0;

        let t1 = Instant::now();
        let fast = find_matches_fast(&data, MIN_MATCH, MAX_OFFSET, max_len);
        let fast_ms = t1.elapsed().as_secs_f64() * 1000.0;

        // Identity of the match sets themselves.
        let mut sets_equal = brute.starts_slice() == fast.starts_slice();
        if sets_equal {
            let a = brute.cands_slice();
            let b = fast.cands_slice();
            sets_equal = a.len() == b.len()
                && a.iter()
                    .zip(b.iter())
                    .all(|(x, y)| x.offset == y.offset && x.length == y.length);
        }

        // Whole-encoder output.
        let out_brute = compress_lzsa1_with(&data, true);
        let out_fast = compress_lzsa1_with(&data, false);
        let identical = out_brute == out_fast;

        all_identical &= identical && sets_equal;
        all_faster &= fast_ms < brute_ms;

        println!(
            "{:<10} {:>7} {:>11.1} {:>11.1} {:>7.1}x {:>10} {:>10} {:>10}",
            name,
            n,
            brute_ms,
            fast_ms,
            brute_ms / fast_ms.max(1e-9),
            out_brute.len(),
            out_fast.len(),
            if identical && sets_equal {
                "yes"
            } else if identical {
                "bytes only"
            } else if out_fast.len() < out_brute.len() {
                "NO (smaller)"
            } else {
                "NO (LARGER)"
            }
        );
    }

    println!();
    println!(
        "all byte-identical: {}   fast wins every case: {}",
        if all_identical { "yes" } else { "NO" },
        if all_faster { "yes" } else { "NO" }
    );
}

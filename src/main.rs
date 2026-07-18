//! LZAN command-line tool.
//!
//!   lzan c <in> <out>         compress
//!   lzan d <in> <out>         decompress
//!   lzan rt <in>              roundtrip-test one file (compress, decompress, compare)
//!   lzan bench <dir>          compress every file in <dir>, verify roundtrip, report
//!                             totals (and compare to docs/lzsa2-baseline.csv if present)

use std::env;
use std::fs;
use std::path::Path;
use std::process::ExitCode;

use lzan::{compress_with, decompress, Options};

/// Build Options, applying env overrides:
///   LZAN_CHAIN, LZAN_ITERS, LZAN_WINDOW, LZAN_NOREP=1, LZAN_NOENT=1
fn opts() -> Options {
    // c64-only build: always the 6510 ZX0-style target.
    let mut o = Options::c64();
    if let Ok(v) = env::var("LZAN_CHAIN") {
        if let Ok(n) = v.parse() {
            o.max_chain = n;
        }
    }
    if let Ok(v) = env::var("LZAN_ITERS") {
        if let Ok(n) = v.parse() {
            o.parse_iters = n;
        }
    }
    if let Ok(v) = env::var("LZAN_WINDOW") {
        if let Ok(n) = v.parse() {
            o.window = n;
        }
    }
    if env::var("LZAN_NOREP").is_ok() {
        o.use_rep = false;
    }
    if env::var("LZAN_NOENT").is_ok() {
        o.use_entropy = false;
    }
    // LZAN_EFFORT selects the encoder effort tier (1=fast / 2=balanced / 3=optimal); default 3.
    // A CLI flag (-O1/-O2/-O3, --effort N) overrides it.
    if let Ok(v) = env::var("LZAN_EFFORT") {
        if let Some(e) = parse_effort(v.trim()) {
            o.effort = e;
        }
    }
    o
}

/// Parse an effort tier from a string: accepts "1".."3", "fast"/"balanced"/"optimal", or the
/// flag forms "-O1"/"O2"/"--effort=3". Returns `None` if unrecognized (caller keeps the default).
fn parse_effort(s: &str) -> Option<u8> {
    let s = s
        .trim()
        .trim_start_matches("--effort")
        .trim_start_matches('=');
    let s = s.trim_start_matches("-O").trim_start_matches('O');
    match s {
        "1" | "fast" => Some(1),
        "2" | "balanced" | "bal" => Some(2),
        "3" | "optimal" | "opt" => Some(3),
        _ => None,
    }
}

/// Pull an effort flag (`-O1`/`-O2`/`-O3`, `--effort N`, `--effort=N`) out of an argument list,
/// returning the parsed tier (if any) and the args with the flag removed. Lets the effort flag
/// appear anywhere among a subcommand's positional args.
fn extract_effort(args: &[String]) -> (Option<u8>, Vec<String>) {
    let mut effort = None;
    let mut rest = Vec::new();
    let mut i = 0;
    while i < args.len() {
        let a = &args[i];
        if a == "--effort" {
            // value in the next arg
            if i + 1 < args.len() {
                if let Some(e) = parse_effort(&args[i + 1]) {
                    effort = Some(e);
                    i += 2;
                    continue;
                }
            }
        } else if a.starts_with("--effort=") || a.starts_with("-O") {
            if let Some(e) = parse_effort(a) {
                effort = Some(e);
                i += 1;
                continue;
            }
        }
        rest.push(a.clone());
        i += 1;
    }
    (effort, rest)
}

fn read_file(p: &str) -> Vec<u8> {
    fs::read(p).unwrap_or_else(|e| {
        eprintln!("error reading {}: {}", p, e);
        std::process::exit(2);
    })
}

fn main() -> ExitCode {
    let raw_args: Vec<String> = env::args().collect();
    // Pull any effort flag (-O1/-O2/-O3 or --effort N) out of the args, wherever it appears. A CLI
    // flag overrides LZAN_EFFORT; publish it back into the env so `opts()` and any thread reading it
    // see the same tier. Remaining args keep their positions.
    let (cli_effort, args) = extract_effort(&raw_args);
    if let Some(e) = cli_effort {
        env::set_var("LZAN_EFFORT", e.to_string());
    }
    if args.len() < 2 {
        usage();
        return ExitCode::from(2);
    }
    match args[1].as_str() {
        "c" | "compress" => {
            if args.len() != 4 {
                usage();
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let out = compress_with(&input, &opts());
            fs::write(&args[3], &out).unwrap();
            eprintln!(
                "{} -> {} bytes ({:.4})",
                input.len(),
                out.len(),
                ratio(out.len(), input.len())
            );
            ExitCode::SUCCESS
        }
        "d" | "decompress" => {
            if args.len() != 4 {
                usage();
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            match decompress(&input) {
                Ok(out) => {
                    fs::write(&args[3], &out).unwrap();
                    eprintln!("{} -> {} bytes", input.len(), out.len());
                    ExitCode::SUCCESS
                }
                Err(e) => {
                    eprintln!("decompress error: {}", e);
                    ExitCode::from(1)
                }
            }
        }
        "zx0c" => {
            // Raw ZX0 v2 stream (no LZAN container), byte-identical to zx0.exe and decodable by
            // dzx0 and the 6502 ZX0 decoders.
            if args.len() != 4 {
                eprintln!("usage: lzan zx0c <in> <out.zx0>");
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let out = lzan::zx0compat::compress_zx0_compatible(&input);
            fs::write(&args[3], &out).unwrap();
            eprintln!(
                "{} -> {} bytes ({:.4}) [ZX0 v2]",
                input.len(),
                out.len(),
                ratio(out.len(), input.len())
            );
            ExitCode::SUCCESS
        }
        "bb2c" => {
            // ByteBoozer2 .b2 container, byte-identical to `b2 <file.prg>`. Input is a C64 .prg
            // (2-byte load address + data); output is [start_addr][load_addr][crunched stream].
            if args.len() != 4 {
                eprintln!("usage: lzan bb2c <in.prg> <out.b2>");
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let out = lzan::bb2::b2_container(&input).unwrap_or_else(|| {
                panic!("bb2c: input too small (need 2-byte load address + data)")
            });
            fs::write(&args[3], &out).unwrap();
            eprintln!(
                "{} -> {} bytes ({:.4}) [ByteBoozer2 .b2]",
                input.len(),
                out.len(),
                ratio(out.len(), input.len())
            );
            ExitCode::SUCCESS
        }
        "tscc" | "tsccb" => {
            // TSCrunch, byte-identical to `tscrunch -p` (tscc) / `tscrunch -p -i` (tsccb). Input is a
            // C64 .prg; the 2-byte load address is stripped and threaded into the in-place wrapper.
            if args.len() != 4 {
                eprintln!("usage: lzan {} <in.prg> <out>", args[1]);
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            if input.len() < 2 {
                eprintln!("input too small for PRG (need a 2-byte load address)");
                return ExitCode::from(2);
            }
            let addr = [input[0], input[1]];
            let data = &input[2..];
            let (out, label) = if args[1] == "tsccb" {
                (
                    lzan::tscrunch::compress_tscrunch_backward_with_addr(data, addr),
                    "TSCrunch in-place",
                )
            } else {
                (lzan::tscrunch::compress_tscrunch(data), "TSCrunch forward")
            };
            fs::write(&args[3], &out).unwrap();
            eprintln!(
                "{} -> {} bytes ({:.4}) [{}]",
                input.len(),
                out.len(),
                ratio(out.len(), input.len()),
                label
            );
            ExitCode::SUCCESS
        }
        // Foreign-format output (raw blocks/streams, decodable by the reference tools), forward and
        // backward (in-place) variants:
        //   lzsa1c/lzsa2c/exo3c -> forward ; zx0cb/lzsa1cb/lzsa2cb/exo3cb -> backward ;
        //   upkrc/upkrcr -> upkr forward/reverse.
        "lzsa1c" | "lzsa2c" | "exo3c" | "zx0cb" | "lzsa1cb" | "lzsa2cb" | "exo3cb" | "upkrc"
        | "upkrcr" => {
            if args.len() != 4 {
                eprintln!("usage: lzan {} <in> <out>", args[1]);
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let (out, label) = match args[1].as_str() {
                "lzsa1c" => (lzan::lzsa1::compress_lzsa1(&input), "LZSA1 raw"),
                "lzsa2c" => (lzan::lzsa2::compress_lzsa2(&input), "LZSA2 raw"),
                "exo3c" => (lzan::exo3::compress_exo3(&input), "Exomizer 3 raw"),
                "zx0cb" => (
                    lzan::zx0compat::compress_zx0_compatible_backward(&input),
                    "ZX0 v2 backward",
                ),
                "lzsa1cb" => (
                    lzan::lzsa1::compress_lzsa1_backward(&input),
                    "LZSA1 raw backward",
                ),
                "lzsa2cb" => (
                    lzan::lzsa2::compress_lzsa2_backward(&input),
                    "LZSA2 raw backward",
                ),
                "exo3cb" => (
                    lzan::exo3::compress_exo3_backward(&input),
                    "Exomizer 3 raw backward",
                ),
                "upkrc" => (lzan::upkr::compress_upkr(&input), "upkr forward"),
                _ => (lzan::upkr::compress_upkr_reverse(&input), "upkr reverse"),
            };
            fs::write(&args[3], &out).unwrap();
            eprintln!(
                "{} -> {} bytes ({:.4}) [{}]",
                input.len(),
                out.len(),
                ratio(out.len(), input.len()),
                label
            );
            ExitCode::SUCCESS
        }
        // Backward (in-place) ZX variant. Raw backward blob layout:
        // [orig_len: u32 LE][mode_byte][reversed payload...]; the orig_len prefix makes the blob
        // self-contained for decode and roundtrip. `zxcb` = compress backward, `zxdb` = decode
        // backward, `zxrtb` = backward roundtrip-test a single file.
        "zxcb" => {
            if args.len() != 4 {
                eprintln!("usage: lzan zxcb <in> <out>");
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let blob = lzan::zx::compress_backward_best_of(&input, opts().effort);
            let mut out = Vec::with_capacity(4 + blob.len());
            out.extend_from_slice(&(input.len() as u32).to_le_bytes());
            out.extend_from_slice(&blob);
            fs::write(&args[3], &out).unwrap();
            eprintln!(
                "{} -> {} bytes ({:.4}) [ZX backward]",
                input.len(),
                out.len(),
                ratio(out.len(), input.len())
            );
            ExitCode::SUCCESS
        }
        "zxdb" => {
            if args.len() != 4 {
                eprintln!("usage: lzan zxdb <in.zxb> <out>");
                return ExitCode::from(2);
            }
            let blob = read_file(&args[2]);
            if blob.len() < 5 {
                eprintln!("zxdb: backward blob too short");
                return ExitCode::from(1);
            }
            let orig_len = u32::from_le_bytes([blob[0], blob[1], blob[2], blob[3]]) as usize;
            let out = lzan::zx::decode_backward(&blob[4..], orig_len);
            fs::write(&args[3], &out).unwrap();
            eprintln!("{} -> {} bytes [ZX backward]", blob.len(), out.len());
            ExitCode::SUCCESS
        }
        "zxrtb" => {
            if args.len() != 3 {
                eprintln!("usage: lzan zxrtb <in>");
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let blob = lzan::zx::compress_backward_best_of(&input, opts().effort);
            let out = lzan::zx::decode_backward(&blob, input.len());
            if out == input {
                println!(
                    "OK  {:>10} -> {:>10}  ({:.4})  {}",
                    input.len(),
                    blob.len(),
                    ratio(blob.len(), input.len()),
                    args[2]
                );
                ExitCode::SUCCESS
            } else {
                println!("FAIL backward roundtrip mismatch: {}", args[2]);
                ExitCode::from(1)
            }
        }
        // Backward self-roundtrip over a directory (plus edge cases) with a forward-vs-backward
        // size delta report.
        "zxbenchb" => {
            if args.len() < 3 {
                eprintln!("usage: lzan zxbenchb <dir>");
                return ExitCode::from(2);
            }
            zxbenchb(&args[2])
        }
        "fmtcheck" => {
            // Roundtrip every format, forward and backward, through the polymorphic API.
            if args.len() != 3 {
                eprintln!("usage: lzan fmtcheck <in>");
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let mut all_ok = true;
            for fmt in lzan::format::Format::all() {
                for &backward in &[false, true] {
                    let c = lzan::format::compress(fmt, &input, fmt.max_level(), backward);
                    // An empty result for non-empty input means the format cannot represent it
                    // (LZSA raw blocks cap at 64 KB).
                    let status = if c.is_empty() && !input.is_empty() {
                        "n/a"
                    } else if lzan::format::decompress(fmt, &c, backward) == input {
                        "OK"
                    } else {
                        all_ok = false;
                        "FAIL"
                    };
                    println!(
                        "  {:9} {:8} {:>8} -> {:>8}  {}",
                        fmt.name(),
                        if backward { "reverse" } else { "forward" },
                        input.len(),
                        c.len(),
                        status
                    );
                }
            }
            if all_ok {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            }
        }
        "levels" => {
            // Show the forward size at each effort tier (1..=max_level) per format, and check the
            // tiers are monotone non-increasing (level 1 = fastest, level max = best) and the max
            // tier roundtrips - all through the polymorphic API.
            if args.len() != 3 {
                eprintln!("usage: lzan levels <in>");
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let mut all_ok = true;
            for fmt in lzan::format::Format::all() {
                let max = fmt.max_level();
                let sizes: Vec<usize> = (1..=max)
                    .map(|lvl| lzan::format::compress(fmt, &input, lvl, false).len())
                    .collect();
                let monotone = sizes.windows(2).all(|w| w[1] <= w[0]);
                let best = lzan::format::compress(fmt, &input, max, false);
                let rt = lzan::format::decompress(fmt, &best, false) == input;
                if !monotone || !rt {
                    all_ok = false;
                }
                let list = sizes
                    .iter()
                    .enumerate()
                    .map(|(i, s)| format!("L{}={}", i + 1, s))
                    .collect::<Vec<_>>()
                    .join(" ");
                println!(
                    "  {:9} max={}  {:30}  {}{}",
                    fmt.name(),
                    max,
                    list,
                    if monotone { "monotone" } else { "NON-MONOTONE" },
                    if rt { "" } else { " RT-FAIL" }
                );
            }
            if all_ok {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            }
        }
        "rt" => {
            if args.len() != 3 {
                usage();
                return ExitCode::from(2);
            }
            let input = read_file(&args[2]);
            let c = compress_with(&input, &opts());
            let d = decompress(&c).expect("decompress");
            if d == input {
                println!(
                    "OK  {:>10} -> {:>10}  ({:.4})  {}",
                    input.len(),
                    c.len(),
                    ratio(c.len(), input.len()),
                    args[2]
                );
                ExitCode::SUCCESS
            } else {
                println!("FAIL roundtrip mismatch: {}", args[2]);
                ExitCode::from(1)
            }
        }
        "bench" => {
            if args.len() < 3 {
                usage();
                return ExitCode::from(2);
            }
            bench(&args[2])
        }
        "zxbench" => {
            if args.len() < 3 {
                usage();
                return ExitCode::from(2);
            }
            zxbench(&args[2])
        }
        "zxverify" => {
            if args.len() < 3 {
                usage();
                return ExitCode::from(2);
            }
            zxverify(&args[2])
        }
        "zxstats" => {
            if args.len() < 3 {
                usage();
                return ExitCode::from(2);
            }
            zxstats(&args[2])
        }
        "zxcmp" => {
            // Compare the heuristic best-of vs the complete-candidate best-of, per file.
            if args.len() < 3 {
                usage();
                return ExitCode::from(2);
            }
            zxcmp(&args[2])
        }
        "zxopt1" => {
            // Single-file diagnostic: ZX0 ground-truth bits vs the internal bits vs the encoded
            // bits. Separates parse residual from encode-replay residual.
            if args.len() < 4 {
                eprintln!("usage: lzan zxopt1 <file> <zx0i.exe> [offset_limit]");
                return ExitCode::from(2);
            }
            let data = read_file(&args[2]);
            let ol = args
                .get(4)
                .and_then(|s| s.parse::<usize>().ok())
                .unwrap_or(32640);
            let (cmds, internal_bits) = lzan::zx0opt::optimize_zx0_with_bits(&data, 0, ol);
            let enc_bits = lzan::zx::predicted_payload_bits(&data, &cmds, 1, false, false);
            let blob = lzan::zx::encode_with(&data, &cmds, 1);
            let rt = lzan::zx::decode(&blob, data.len()) == data;
            // run ZX0
            use std::process::Command;
            let tmp_out = std::env::temp_dir().join("zxopt1.zx0");
            let out = Command::new(&args[3])
                .arg("-f")
                .arg(&args[2])
                .arg(&tmp_out)
                .output();
            let zx0_bits: Option<i64> = out.ok().and_then(|o| {
                let s = String::from_utf8_lossy(&o.stderr).into_owned();
                s.lines().find_map(|l| {
                    l.strip_prefix("ZX0_OPTIMAL_BITS ")
                        .and_then(|v| v.trim().parse::<i64>().ok())
                })
            });
            println!(
                "file: {}  ({} bytes)  offset_limit={}",
                args[2],
                data.len(),
                ol
            );
            println!("  ZX0 optimal->bits : {:?}", zx0_bits);
            println!("  port internal bits: {}", internal_bits);
            println!("  port encoded bits : {}", enc_bits);
            println!("  ncmds: {}  roundtrip: {}", cmds.len(), rt);
            if let Some(zb) = zx0_bits {
                println!(
                    "  parse residual (port_internal - zx0): {:+}",
                    internal_bits - zb
                );
                println!(
                    "  encode residual (encoded - port_internal): {:+}",
                    enc_bits as i64 - internal_bits
                );
            }
            ExitCode::SUCCESS
        }
        "zxopt" => {
            // Compare the LZAN rep0-only parse cost (bits) against the instrumented ZX0 reference.
            // args[2] = corpus dir, args[3] = path to instrumented zx0 (prints ZX0_OPTIMAL_BITS).
            if args.len() < 4 {
                eprintln!("usage: lzan zxopt <dir> <zx0i.exe>");
                return ExitCode::from(2);
            }
            zxopt(&args[2], &args[3])
        }
        "fuzz" => {
            let iters: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(200_000);
            fuzz(iters)
        }
        _ => {
            usage();
            ExitCode::from(2)
        }
    }
}

/// Roundtrip fuzzer. Generates pathological and random inputs that target format boundaries (slot
/// edges, ll/ml extension thresholds, rep MTF, overlap copies, window edges) and asserts lossless
/// roundtrip.
fn fuzz(iters: u64) -> ExitCode {
    let mut state: u64 = 0x9E3779B97F4A7C15;
    let mut rng = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };

    let check = |label: &str, data: &[u8]| -> bool {
        // Exercise several option profiles of the ZX0-style backend.
        let profiles = vec![
            Options::default(),
            Options {
                use_rep: false,
                ..Options::default()
            },
            Options {
                use_entropy: false,
                parse_iters: 1,
                ..Options::default()
            },
            Options {
                window: 64,
                ..Options::default()
            },
        ];
        for o in profiles {
            let c = compress_with(data, &o);
            match decompress(&c) {
                Ok(d) if d == data => {}
                _ => {
                    println!(
                        "FUZZ FAIL [{}] len={} opts(rep={},ent={},win={})",
                        label,
                        data.len(),
                        o.use_rep,
                        o.use_entropy,
                        o.window
                    );
                    return false;
                }
            }
        }
        true
    };

    // Deterministic corner cases first.
    let mut ok = true;
    for n in 0..300usize {
        ok &= check("empty/seq", &(0..n).map(|i| i as u8).collect::<Vec<_>>());
        ok &= check("same", &vec![(n & 0xff) as u8; n]);
    }
    // boundary-targeted patterns
    for &period in &[1usize, 2, 3, 4, 5, 7, 8, 16, 17, 255, 256, 257] {
        let v: Vec<u8> = (0..2000).map(|i| (i % period) as u8).collect();
        ok &= check("period", &v);
    }
    for &off in &[
        1usize, 2, 3, 4, 5, 8, 9, 256, 257, 8192, 65535, 65536, 65537,
    ] {
        // a literal prelude of `off` then a long copy from distance `off`
        let mut v: Vec<u8> = (0..off).map(|i| (i * 131 % 251) as u8).collect();
        let pre = v.clone();
        for k in 0..3000 {
            v.push(pre[k % pre.len()]);
        }
        ok &= check("offset", &v);
    }
    if !ok {
        return ExitCode::from(1);
    }

    // Random structured fuzzing.
    for it in 0..iters {
        let mode = rng() % 6;
        let n = (rng() % 4000) as usize + 1;
        let mut v = Vec::with_capacity(n);
        match mode {
            0 => {
                // low-alphabet noise (lots of short matches)
                let alpha = (rng() % 6) as u8 + 1;
                for _ in 0..n {
                    v.push((rng() % alpha as u64) as u8);
                }
            }
            1 => {
                // full-entropy
                for _ in 0..n {
                    v.push((rng() >> 24) as u8);
                }
            }
            2 => {
                // runs of repeats with random copies (rep-heavy)
                while v.len() < n {
                    if v.is_empty() || rng() % 2 == 0 {
                        let run = (rng() % 50) as usize + 1;
                        let b = (rng() >> 16) as u8;
                        for _ in 0..run {
                            v.push(b);
                        }
                    } else {
                        let back = 1 + (rng() as usize % v.len());
                        let len = (rng() % 40) as usize + 1;
                        let src = v.len() - back;
                        for k in 0..len {
                            v.push(v[src + (k % back)]);
                        }
                    }
                }
                v.truncate(n);
            }
            3 => {
                // text-ish
                let words: [&[u8]; 4] = [b"the ", b"quick ", b"fox ", b"lazy "];
                while v.len() < n {
                    v.extend_from_slice(words[(rng() % 4) as usize]);
                }
                v.truncate(n);
            }
            4 => {
                // sparse high bytes + zeros
                for _ in 0..n {
                    v.push(if rng() % 8 == 0 {
                        (rng() >> 20) as u8
                    } else {
                        0
                    });
                }
            }
            _ => {
                // mixed: random segments concatenated
                while v.len() < n {
                    let seg = (rng() % 100) as usize + 1;
                    let b = (rng() >> 24) as u8;
                    let kind = rng() % 3;
                    for k in 0..seg {
                        v.push(match kind {
                            0 => b,
                            1 => b.wrapping_add(k as u8),
                            _ => (rng() >> 24) as u8,
                        });
                    }
                }
                v.truncate(n);
            }
        }
        if !check("rand", &v) {
            return ExitCode::from(1);
        }
        if it % 20000 == 0 && it > 0 {
            eprintln!("fuzz: {} / {} ok", it, iters);
        }
    }
    println!(
        "FUZZ OK: all roundtrips lossless ({} random + corner cases)",
        iters
    );
    ExitCode::SUCCESS
}

fn ratio(a: usize, b: usize) -> f64 {
    if b == 0 {
        0.0
    } else {
        a as f64 / b as f64
    }
}

/// Backward (in-place) ZX self-roundtrip over a directory plus edge cases, with a
/// forward-vs-backward size delta report. For each file
/// `decode_backward(compress_backward_best_of(x)) == x` must hold; the forward best-of
/// (`lzan::zx_best_of`) is also run so the report shows the size delta. Edge cases (empty, 1 byte,
/// repetitive, incompressible) are tested first.
fn zxbenchb(dir: &str) -> ExitCode {
    let effort = opts().effort;
    let mut all_ok = true;

    // ----- edge cases -----
    let mut edge: Vec<(String, Vec<u8>)> = Vec::new();
    edge.push(("empty".into(), Vec::new()));
    edge.push(("one-byte".into(), vec![0x42]));
    edge.push(("repetitive".into(), vec![0xAB; 4096]));
    edge.push((
        "two-byte-period".into(),
        (0..4096).map(|i| (i % 2) as u8).collect(),
    ));
    {
        // incompressible (deterministic LCG)
        let mut s = 0x1234_5678u32;
        let v: Vec<u8> = (0..4096)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        edge.push(("incompressible".into(), v));
    }
    eprintln!("backward edge cases:");
    for (name, data) in &edge {
        let blob = lzan::zx::compress_backward_best_of(data, effort);
        let out = lzan::zx::decode_backward(&blob, data.len());
        let ok = out == *data;
        all_ok &= ok;
        eprintln!(
            "  {:<16} {:>7} -> {:>7}  {}",
            name,
            data.len(),
            blob.len(),
            if ok { "OK" } else { "FAIL" }
        );
    }

    // ----- corpus -----
    let mut entries: Vec<_> = match fs::read_dir(dir) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(e) => {
            eprintln!("cannot read dir {}: {}", dir, e);
            return ExitCode::from(2);
        }
    };
    entries.retain(|p| p.is_file());
    entries.sort();

    let mut tot_in: u64 = 0;
    let mut tot_fwd: u64 = 0;
    let mut tot_bwd: u64 = 0;
    println!(
        "{:<46} {:>9} {:>9} {:>9} {:>7} {:>5}",
        "file", "in", "fwd", "bwd", "delta", "rt"
    );
    for p in &entries {
        let data = match fs::read(p) {
            Ok(d) => d,
            Err(_) => continue,
        };
        let fwd = lzan::zx_best_of(&data, effort);
        let bwd = lzan::zx::compress_backward_best_of(&data, effort);
        let out = lzan::zx::decode_backward(&bwd, data.len());
        let ok = out == data;
        all_ok &= ok;
        tot_in += data.len() as u64;
        tot_fwd += fwd.len() as u64;
        tot_bwd += bwd.len() as u64;
        let name = p.file_name().and_then(|s| s.to_str()).unwrap_or("?");
        let name = if name.len() > 46 { &name[..46] } else { name };
        println!(
            "{:<46} {:>9} {:>9} {:>9} {:>+7} {:>5}",
            name,
            data.len(),
            fwd.len(),
            bwd.len(),
            bwd.len() as i64 - fwd.len() as i64,
            if ok { "OK" } else { "FAIL" }
        );
    }
    println!(
        "{:<46} {:>9} {:>9} {:>9} {:>+7} {:>5}",
        "TOTAL",
        tot_in,
        tot_fwd,
        tot_bwd,
        tot_bwd as i64 - tot_fwd as i64,
        if all_ok { "OK" } else { "FAIL" }
    );
    println!(
        "backward self-roundtrip: {}",
        if all_ok { "ALL-OK" } else { "FAILURES PRESENT" }
    );
    if all_ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

fn usage() {
    eprintln!("usage (lzan - c64 ZX0-style codec):");
    eprintln!("  lzan c  <in> <out>    compress");
    eprintln!("  lzan d  <in> <out>    decompress");
    eprintln!("  lzan rt <in>          roundtrip-test one file");
    eprintln!("  lzan bench   <dir>    compress+verify all files in dir, report totals");
    eprintln!("  lzan zxbench <dir>    ZX best-of benchmark (honors encoder effort)");
    eprintln!();
    eprintln!("backward (in-place) ZX variant - reverse/in-place decompression for C64:");
    eprintln!("  lzan zxcb  <in> <out>  compress backward (raw backward blob)");
    eprintln!("  lzan zxdb  <in> <out>  decode a backward blob");
    eprintln!("  lzan zxrtb <in>        backward roundtrip-test one file");
    eprintln!("  lzan zxbenchb <dir>    backward self-roundtrip + fwd-vs-bwd size delta (ALL-OK)");
    eprintln!();
    eprintln!("foreign-format compatible output (raw blocks, decodable by the reference tools):");
    eprintln!("  lzan zx0c   <in> <out.zx0>   ZX0 v2 (byte-identical to zx0.exe / dzx0)");
    eprintln!("  lzan lzsa1c <in> <out>       LZSA1 raw (lzsa -d -f1 -r)");
    eprintln!("  lzan lzsa2c <in> <out>       LZSA2 raw (lzsa -d -f2 -r)");
    eprintln!("  lzan exo3c  <in> <out>       Exomizer 3 raw");
    eprintln!();
    eprintln!("encoder effort (general - applies to ALL ZX modes; decoder is identical):");
    eprintln!("  -O1 / --effort 1      FAST     single DP pass, no rep-seeding/reparse/reduce");
    eprintln!("  -O2 / --effort 2      BALANCED salvador-class (seeding + reparse + reduce)");
    eprintln!("  -O3 / --effort 3      OPTIMAL  brute-force complete-candidate parse (default)");
    eprintln!("                        (also via LZAN_EFFORT=1|2|3; CLI flag overrides the env)");
    eprintln!("  e.g.  lzan c -O1 in out      lzan zxbench --effort 2 <dir>");
    eprintln!();
    eprintln!(
        "env: LZAN_EFFORT=1|2|3  LZAN_CHAIN  LZAN_ITERS  LZAN_WINDOW  LZAN_NOREP=1  LZAN_NOENT=1"
    );
}

/// Load lzsa2 baseline (per-file best sizes) keyed by input length, from the CSV.
/// CSV columns: idx,name,insz,stream,raw,best
fn load_baseline() -> Option<Vec<(u64, u64)>> {
    let p = Path::new("docs/lzsa2-baseline.csv");
    let text = fs::read_to_string(p).ok()?;
    let mut rows = Vec::new();
    for (i, line) in text.lines().enumerate() {
        if i == 0 {
            continue; // header
        }
        // naive CSV: fields may be quoted; we only need insz (col 2) and best (col 5)
        let cols = split_csv(line);
        if cols.len() >= 6 {
            if let (Ok(insz), Ok(best)) = (cols[2].parse::<u64>(), cols[5].parse::<u64>()) {
                rows.push((insz, best));
            }
        }
    }
    Some(rows)
}

fn split_csv(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut in_q = false;
    for ch in line.chars() {
        match ch {
            '"' => in_q = !in_q,
            ',' if !in_q => {
                out.push(cur.clone());
                cur.clear();
            }
            _ => cur.push(ch),
        }
    }
    out.push(cur);
    out
}

/// ZX-codec benchmark: for each file, compress with the table-free Elias-gamma backend (rep0-only
/// and rep0-3 variants), verify roundtrip, and sum sizes. Prints totals bucketed by input size
/// (<=32 KB, <=64 KB, all), with salvador and LZAN-tANS reference totals for comparison. Reports the
/// raw blob size; the LZAN container header (~10 bytes/file) is excluded.
fn zxbench(dir: &str) -> ExitCode {
    let mut entries: Vec<_> = match fs::read_dir(dir) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(e) => {
            eprintln!("cannot read dir {}: {}", dir, e);
            return ExitCode::from(2);
        }
    };
    entries.retain(|p| p.is_file());
    entries.sort();

    // Encoder effort tier (1=fast / 2=balanced / 3=optimal), from LZAN_EFFORT / -O flag via opts().
    let effort = opts().effort;
    eprintln!(
        "zxbench effort = {} ({})",
        effort,
        match effort {
            1 => "fast",
            2 => "balanced",
            _ => "optimal",
        }
    );

    // reference totals (<=32 KB / <=64 KB / all)
    let sal = (132_768u64, 418_505u64, 1_477_322u64);
    let tans = (138_976u64, 422_025u64, 1_320_269u64);

    // accumulators: [rep0-only, rep0-3, best-of] x [<=32K, <=64K, all]
    let mut sum_r1 = [0u64; 3];
    let mut sum_r4 = [0u64; 3];
    let mut sum_nr = [0u64; 3]; // rep0-3 + near-rep
    let mut sum_am = [0u64; 3]; // rep0-3 + after-match near-rep
    let mut sum_best = [0u64; 3]; // best-of {rep0, rep0-3, +near-rep, +am-near-rep} per file
    let mut sum_best2 = [0u64; 3]; // best-of {rep0, rep0-3} per file (no near-rep)
    let mut sum_best3 = [0u64; 3]; // best-of {rep0, rep0-3, +near-rep} (no am-near-rep)
    let mut all_ok = true;

    println!(
        "{:<40} {:>8} {:>9} {:>9} {:>9} {:>6}",
        "file", "in", "zx-rep0", "zx-rep0-3", "zx+nrep", "rt"
    );

    // Per-file work is independent, so fan the files out across scratch threads; every blob is
    // byte-identical to the sequential version. Each file runs its 5 ZX variants and decode-verify;
    // rows are collected, then printed/accumulated below in deterministic entry order.
    struct Row {
        short: String,
        n: u64,
        s1: u64,
        s4: u64,
        snr: u64,
        sam: u64,
        ok: bool,
    }
    let rows: Vec<Row> = std::thread::scope(|s| {
        let handles: Vec<_> = entries
            .iter()
            .map(|p| {
                s.spawn(move || {
                    let data = fs::read(p).unwrap();
                    let n = data.len() as u64;
                    let blob1 = lzan::zx::compress_e(&data, 1, effort);
                    let blob4 = lzan::zx::compress_e(&data, 4, effort);
                    let blobnr = lzan::zx::compress3_e(&data, 4, true, false, effort);
                    // after-match near-rep: try am-only and am + after-lit near-rep, keep the smaller.
                    let blobam_a = lzan::zx::compress3_e(&data, 4, false, true, effort);
                    let blobam_b = lzan::zx::compress3_e(&data, 4, true, true, effort);
                    let d1 = lzan::zx::decode(&blob1, data.len());
                    let d4 = lzan::zx::decode(&blob4, data.len());
                    let dnr = lzan::zx::decode(&blobnr, data.len());
                    let dam_a = lzan::zx::decode(&blobam_a, data.len());
                    let dam_b = lzan::zx::decode(&blobam_b, data.len());
                    let ok =
                        d1 == data && d4 == data && dnr == data && dam_a == data && dam_b == data;
                    let name = p.file_name().unwrap().to_string_lossy();
                    Row {
                        short: name.chars().take(40).collect(),
                        n,
                        s1: blob1.len() as u64,
                        s4: blob4.len() as u64,
                        snr: blobnr.len() as u64,
                        sam: (blobam_a.len() as u64).min(blobam_b.len() as u64),
                        ok,
                    }
                })
            })
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("zxbench file thread panicked"))
            .collect()
    });

    for row in &rows {
        let Row {
            short,
            n,
            s1,
            s4,
            snr,
            sam,
            ok,
        } = row;
        let (n, s1, s4, snr, sam, ok) = (*n, *s1, *s4, *snr, *sam, *ok);
        if !ok {
            all_ok = false;
        }
        let buckets: &[usize] = if n <= 32 * 1024 {
            &[0, 1, 2]
        } else if n <= 64 * 1024 {
            &[1, 2]
        } else {
            &[2]
        };
        let sb = s1.min(s4).min(snr).min(sam);
        let sb2 = s1.min(s4);
        let sb3 = s1.min(s4).min(snr); // best-of without am-near-rep
        for &b in buckets {
            sum_r1[b] += s1;
            sum_r4[b] += s4;
            sum_nr[b] += snr;
            sum_am[b] += sam;
            sum_best[b] += sb;
            sum_best2[b] += sb2;
            sum_best3[b] += sb3;
        }
        println!(
            "{:<40} {:>8} {:>9} {:>9} {:>9} {:>6}",
            short,
            n,
            s1,
            s4,
            snr,
            if ok { "ok" } else { "FAIL" }
        );
    }

    let pct = |a: u64, b: u64| -> f64 {
        if b == 0 {
            0.0
        } else {
            100.0 * (a as i64 - b as i64) as f64 / b as f64
        }
    };

    println!("{}", "=".repeat(106));
    println!(
        "{:<14} {:>14} {:>14} {:>14} {:>14}",
        "bucket", "zx-rep0", "zx-rep0-3", "zx+nrep", "best-of/file"
    );
    let labels = ["<=32 KB", "<=64 KB", "all 39"];
    for b in 0..3 {
        println!(
            "{:<14} {:>14} {:>14} {:>14} {:>14}",
            labels[b], sum_r1[b], sum_r4[b], sum_nr[b], sum_best[b]
        );
    }
    println!("{}", "-".repeat(106));
    println!("near-rep delta vs rep0-3 best-of-2/file (negative = near-rep helps the headline):");
    for b in 0..3 {
        println!(
            "  {}: best(all)={} vs best(2)={}  delta={}",
            labels[b],
            sum_best[b],
            sum_best2[b],
            sum_best[b] as i64 - sum_best2[b] as i64
        );
    }
    println!("{}", "-".repeat(106));
    println!("after-match near-rep (idea 3) delta vs best-of WITHOUT it (negative = am helps):");
    for b in 0..3 {
        println!(
            "  {}: best(all)={} vs best(no-am)={}  delta={}   [zx+amrep alone={}]",
            labels[b],
            sum_best[b],
            sum_best3[b],
            sum_best[b] as i64 - sum_best3[b] as i64,
            sum_am[b]
        );
    }
    println!("{}", "-".repeat(106));
    println!("REFERENCES (<=32 KB / <=64 KB / all):");
    println!("  salvador (ZX0):  {} / {} / {}", sal.0, sal.1, sal.2);
    println!("  LZAN-tANS:       {} / {} / {}", tans.0, tans.1, tans.2);
    println!("{}", "-".repeat(92));
    // best-of-per-file: the codec picks the smallest variant per file (the choice costs 1 mode
    // bit/file).
    println!("ZX (best-of/file) vs salvador:");
    println!(
        "  <=32 KB: {} vs {}  ({:+.2}%)  {}",
        sum_best[0],
        sal.0,
        pct(sum_best[0], sal.0),
        if sum_best[0] < sal.0 {
            "*** ZX BEATS SALVADOR ***"
        } else {
            "salvador ahead"
        }
    );
    println!(
        "  <=64 KB: {} vs {}  ({:+.2}%)",
        sum_best[1],
        sal.1,
        pct(sum_best[1], sal.1)
    );
    println!(
        "  all 39:  {} vs {}  ({:+.2}%)",
        sum_best[2],
        sal.2,
        pct(sum_best[2], sal.2)
    );
    println!("ZX (best-of/file) vs LZAN-tANS:");
    println!(
        "  <=32 KB: {} vs {}  ({:+.2}%)  {}",
        sum_best[0],
        tans.0,
        pct(sum_best[0], tans.0),
        if sum_best[0] < tans.0 {
            "*** beats tANS ***"
        } else {
            "(tANS ahead)"
        }
    );
    println!(
        "  <=64 KB: {} vs {}  ({:+.2}%)",
        sum_best[1],
        tans.1,
        pct(sum_best[1], tans.1)
    );
    println!(
        "  all 39:  {} vs {}  ({:+.2}%)",
        sum_best[2],
        tans.2,
        pct(sum_best[2], tans.2)
    );
    println!("rep0-3 vs rep0-only (negative = rep0-3 smaller):");
    for b in 0..3 {
        println!(
            "  {}: rep0={} rep0-3={}  delta={} ({:+.3}%)",
            labels[b],
            sum_r1[b],
            sum_r4[b],
            sum_r4[b] as i64 - sum_r1[b] as i64,
            pct(sum_r4[b], sum_r1[b])
        );
    }
    println!("roundtrip: {}", if all_ok { "ALL-OK" } else { "FAILURES!" });

    if all_ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

/// For every corpus file and ZX mode, check that (a) the blob roundtrips and (b) the parser's
/// predicted payload cost is bit-exact against the real encoded payload (ceil(bits/8) bytes). A cost
/// mismatch means the DP optimizes against a model that diverges from the encoder.
fn zxverify(dir: &str) -> ExitCode {
    let mut entries: Vec<_> = match fs::read_dir(dir) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(e) => {
            eprintln!("cannot read dir {}: {}", dir, e);
            return ExitCode::from(2);
        }
    };
    entries.retain(|p| p.is_file());
    entries.sort();
    let modes: &[(usize, bool, bool, &str)] = &[
        (1, false, false, "rep0"),
        (4, false, false, "rep0-3"),
        (4, true, false, "rep0-3+nrep"),
        (4, false, true, "rep0-3+am"),
        (4, true, true, "rep0-3+nrep+am"),
    ];
    // Per-file verification is independent, so fan out across scratch threads; results are
    // deterministic. Each file checks all modes and returns its messages and counters.
    let results: Vec<(bool, u64, Vec<String>)> = std::thread::scope(|s| {
        let handles: Vec<_> = entries
            .iter()
            .map(|p| {
                s.spawn(move || {
                    let data = fs::read(p).unwrap();
                    let mut ok_all = true;
                    let mut mm = 0u64;
                    let mut msgs: Vec<String> = Vec::new();
                    let name = p.file_name().unwrap().to_string_lossy().to_string();
                    for &(rs, nr, am, label) in modes {
                        let (blob, cmds) = lzan::zx::compress3_with_cmds(&data, rs, nr, am);
                        let out = lzan::zx::decode(&blob, data.len());
                        if out != data {
                            ok_all = false;
                            msgs.push(format!("ROUNDTRIP FAIL {} [{}]", name, label));
                        }
                        // predicted payload bits vs actual (blob = 1 mode byte + ceil(bits/8) bytes)
                        let pred = lzan::zx::predicted_payload_bits(&data, &cmds, rs, nr, am);
                        let pred_bytes = ((pred + 7) / 8) as usize;
                        let actual_payload = blob.len().saturating_sub(1);
                        if pred_bytes != actual_payload {
                            mm += 1;
                            msgs.push(format!(
                                "COST MISMATCH {} [{}]: predicted {} bits = {} B, actual payload {} B",
                                name, label, pred, pred_bytes, actual_payload));
                        }
                    }
                    (ok_all, mm, msgs)
                })
            })
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("zxverify file thread panicked"))
            .collect()
    });
    let mut all_ok = true;
    let mut mismatches = 0u64;
    for (ok, mm, msgs) in &results {
        if !ok {
            all_ok = false;
        }
        mismatches += mm;
        for m in msgs {
            println!("{}", m);
        }
    }
    println!("roundtrip: {}", if all_ok { "ALL-OK" } else { "FAILURES" });
    println!(
        "cost bit-exactness: {} ({} mismatches over {} files x {} modes)",
        if mismatches == 0 { "EXACT" } else { "MISMATCH" },
        mismatches,
        entries.len(),
        modes.len()
    );
    if all_ok && mismatches == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

/// Compare the heuristic best-of vs the complete-candidate best-of, per file and per bucket.
/// Heuristic = best-of {compress(1), compress(4), compress2(4,nr), compress3(4,*,am)}.
/// Complete  = best-of {compress_complete(1), _(4,F,F), _(4,T,F), _(4,F,T), _(4,T,T)}.
/// Checks every variant roundtrips. Reports per-bucket totals and the delta.
fn zxcmp(dir: &str) -> ExitCode {
    let mut entries: Vec<_> = match fs::read_dir(dir) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(e) => {
            eprintln!("cannot read dir {}: {}", dir, e);
            return ExitCode::from(2);
        }
    };
    entries.retain(|p| p.is_file());
    entries.sort();
    println!(
        "{:<42} {:>8} {:>9} {:>9} {:>7} {:>4}",
        "file", "in", "old-best", "new-best", "delta", "rt"
    );
    // buckets: <=32K, <=64K, all
    let mut old_sum = [0u64; 3];
    let mut new_sum = [0u64; 3];
    let mut all_ok = true;
    let rows: Vec<(String, u64, u64, u64, bool)> = std::thread::scope(|s| {
        let handles: Vec<_> = entries
            .iter()
            .map(|p| {
                s.spawn(move || {
                    let data = fs::read(p).unwrap();
                    let n = data.len() as u64;
                    // heuristic best-of (the codec's variant choices).
                    let old_blobs = [
                        lzan::zx::compress(&data, 1),
                        lzan::zx::compress(&data, 4),
                        lzan::zx::compress2(&data, 4, true),
                        lzan::zx::compress3(&data, 4, false, true),
                        lzan::zx::compress3(&data, 4, true, true),
                    ];
                    // complete-candidate best-of.
                    let new_blobs = [
                        lzan::zx::compress_complete(&data, 1, false, false),
                        lzan::zx::compress_complete(&data, 4, false, false),
                        lzan::zx::compress_complete(&data, 4, true, false),
                        lzan::zx::compress_complete(&data, 4, false, true),
                        lzan::zx::compress_complete(&data, 4, true, true),
                    ];
                    let mut ok = true;
                    for b in old_blobs.iter().chain(new_blobs.iter()) {
                        if lzan::zx::decode(b, data.len()) != data {
                            ok = false;
                        }
                    }
                    let old_best = old_blobs.iter().map(|b| b.len() as u64).min().unwrap();
                    let new_best = new_blobs.iter().map(|b| b.len() as u64).min().unwrap();
                    let name: String = p
                        .file_name()
                        .unwrap()
                        .to_string_lossy()
                        .chars()
                        .take(42)
                        .collect();
                    (name, n, old_best, new_best, ok)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for (name, n, old_best, new_best, ok) in &rows {
        if !ok {
            all_ok = false;
        }
        let buckets: &[usize] = if *n <= 32 * 1024 {
            &[0, 1, 2]
        } else if *n <= 64 * 1024 {
            &[1, 2]
        } else {
            &[2]
        };
        for &b in buckets {
            old_sum[b] += old_best;
            new_sum[b] += new_best;
        }
        println!(
            "{:<42} {:>8} {:>9} {:>9} {:>+7} {:>4}",
            name,
            n,
            old_best,
            new_best,
            *new_best as i64 - *old_best as i64,
            if *ok { "ok" } else { "FAIL" }
        );
    }
    println!("{}", "=".repeat(84));
    let labels = ["<=32 KB", "<=64 KB", "all"];
    for b in 0..3 {
        println!(
            "{:<10} old-best={:>10}  new-best={:>10}  delta={:+}",
            labels[b],
            old_sum[b],
            new_sum[b],
            new_sum[b] as i64 - old_sum[b] as i64
        );
    }
    println!("roundtrip: {}", if all_ok { "ALL-OK" } else { "FAILURES" });
    if all_ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

/// Compare LZAN's rep0-only parse cost (bits) against the ZX0 optimum (instrumented ZX0 prints
/// `ZX0_OPTIMAL_BITS <n>` to stderr). LZAN's window is capped to ZX0's MAX_OFFSET_ZX0=32640 so both
/// search the same offset space; the gap is 0 when LZAN's rep0-only parse is exact.
fn zxopt(dir: &str, zx0exe: &str) -> ExitCode {
    use std::process::Command;
    const ZX0_WINDOW: usize = 32640; // MAX_OFFSET_ZX0
    let mut entries: Vec<_> = match fs::read_dir(dir) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(e) => {
            eprintln!("cannot read dir {}: {}", dir, e);
            return ExitCode::from(2);
        }
    };
    entries.retain(|p| p.is_file());
    entries.sort();
    // Columns: zx0-bits (ground truth), old-DP gap, new exact gap.
    println!(
        "{:<42} {:>8} {:>10} {:>8} {:>8} {:>4}",
        "file", "in", "zx0-bits", "dp-gap", "exact-gap", "rt"
    );
    let mut total_gap_dp: i64 = 0;
    let mut total_gap_ex: i64 = 0;
    let mut total_files = 0u64;
    let mut exact_files = 0u64;
    let mut all_ok = true;
    for p in &entries {
        let data = fs::read(p).unwrap();
        if data.is_empty() {
            continue;
        }
        // Ground truth: instrumented ZX0. Write to a temp output (force overwrite).
        let tmp_out = std::env::temp_dir().join("zxopt_tmp.zx0");
        let out = Command::new(zx0exe).arg("-f").arg(p).arg(&tmp_out).output();
        let zx0_bits: Option<i64> = match out {
            Ok(o) => {
                let s = String::from_utf8_lossy(&o.stderr);
                s.lines().find_map(|l| {
                    l.strip_prefix("ZX0_OPTIMAL_BITS ")
                        .and_then(|v| v.trim().parse::<i64>().ok())
                })
            }
            Err(_) => None,
        };
        let (dp_bits, _bl1, ok1) = lzan::zx::rep0_cost_with_window(&data, ZX0_WINDOW);
        let (ex_bits, _bl2, ok2) = lzan::zx::rep0_zx0exact_cost(&data, ZX0_WINDOW);
        let ok = ok1 && ok2;
        if !ok {
            all_ok = false;
        }
        let name: String = p
            .file_name()
            .unwrap()
            .to_string_lossy()
            .chars()
            .take(42)
            .collect();
        match zx0_bits {
            Some(zb) => {
                let gap_dp = dp_bits as i64 - zb;
                let gap_ex = ex_bits as i64 - zb;
                total_gap_dp += gap_dp;
                total_gap_ex += gap_ex;
                total_files += 1;
                if gap_ex == 0 {
                    exact_files += 1;
                }
                println!(
                    "{:<42} {:>8} {:>10} {:>+8} {:>+8} {:>4}",
                    name,
                    data.len(),
                    zb,
                    gap_dp,
                    gap_ex,
                    if ok { "ok" } else { "BAD" }
                );
            }
            None => {
                println!(
                    "{:<42} {:>8} {:>10} {:>8} {:>8} {:>4}",
                    name,
                    data.len(),
                    "ZX0-FAIL",
                    "?",
                    "?",
                    if ok { "ok" } else { "BAD" }
                );
            }
        }
    }
    println!("{}", "=".repeat(86));
    println!("files: {}   EXACT-port gap==0 on: {}/{}   old-DP total gap: {:+}   exact-port total gap: {:+}",
        total_files, exact_files, total_files, total_gap_dp, total_gap_ex);
    println!("roundtrip: {}", if all_ok { "ALL-OK" } else { "FAILURES" });
    if total_gap_ex == 0 && all_ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

/// Aggregate the ZX bitstream cost breakdown across all <=32 KB files (rep0-3), showing where the
/// bytes go.
fn zxstats(dir: &str) -> ExitCode {
    let mut entries: Vec<_> = match fs::read_dir(dir) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(e) => {
            eprintln!("cannot read dir {}: {}", dir, e);
            return ExitCode::from(2);
        }
    };
    entries.retain(|p| p.is_file());
    entries.sort();

    let mut agg = lzan::zx::ZxStats::default();
    let mut nfiles = 0u64;
    for p in &entries {
        let data = fs::read(p).unwrap();
        if data.len() as u64 > 32 * 1024 {
            continue;
        }
        nfiles += 1;
        let (_blob, cmds) = lzan::zx::compress_with_cmds(&data, 4);
        let s = lzan::zx::stats(&data, &cmds, 4);
        agg.n_lit_runs += s.n_lit_runs;
        agg.lit_bytes += s.lit_bytes;
        agg.lit_frame_bits += s.lit_frame_bits;
        agg.n_newoff += s.n_newoff;
        agg.newoff_off_bits += s.newoff_off_bits;
        agg.newoff_len_bits += s.newoff_len_bits;
        for i in 0..4 {
            agg.rep_counts[i] += s.rep_counts[i];
        }
        agg.rep_index_bits += s.rep_index_bits;
        agg.rep_len_bits += s.rep_len_bits;
        agg.flag_bits += s.flag_bits;
        agg.total_bits += s.total_bits;
    }
    let b2 = |bits: u64| bits / 8;
    println!(
        "=== ZX cost breakdown, {} files <=32 KB (rep0-3) ===",
        nfiles
    );
    println!(
        "literal bytes     : {:>10} bytes ({} lit runs)",
        agg.lit_bytes, agg.n_lit_runs
    );
    println!("  raw byte bits   : {:>10} B", b2(agg.lit_bytes * 8));
    println!("  run-gamma bits  : {:>10} B", b2(agg.lit_frame_bits));
    println!("new-offset matches: {:>10}", agg.n_newoff);
    println!("  offset bits     : {:>10} B", b2(agg.newoff_off_bits));
    println!("  length bits     : {:>10} B", b2(agg.newoff_len_bits));
    println!(
        "rep matches       : rep0={} rep1={} rep2={} rep3={}",
        agg.rep_counts[0], agg.rep_counts[1], agg.rep_counts[2], agg.rep_counts[3]
    );
    println!("  index bits      : {:>10} B", b2(agg.rep_index_bits));
    println!("  length bits     : {:>10} B", b2(agg.rep_len_bits));
    println!("flag bits         : {:>10} B", b2(agg.flag_bits));
    println!(
        "TOTAL             : {:>10} B  (vs salvador 132768)",
        b2(agg.total_bits)
    );
    ExitCode::SUCCESS
}

fn bench(dir: &str) -> ExitCode {
    let opts = opts();
    let mut entries: Vec<_> = match fs::read_dir(dir) {
        Ok(rd) => rd.filter_map(|e| e.ok()).map(|e| e.path()).collect(),
        Err(e) => {
            eprintln!("cannot read dir {}: {}", dir, e);
            return ExitCode::from(2);
        }
    };
    entries.retain(|p| p.is_file());
    entries.sort();

    let baseline = load_baseline();
    // baseline rows are keyed by input size; build a multiset lookup
    let mut base_by_size: std::collections::HashMap<u64, Vec<u64>> =
        std::collections::HashMap::new();
    if let Some(rows) = &baseline {
        for (insz, best) in rows {
            base_by_size.entry(*insz).or_default().push(*best);
        }
    }

    let mut tot_in = 0u64;
    let mut tot_out = 0u64;
    let mut tot_base = 0u64;
    let mut all_ok = true;
    let mut wins = 0u32;
    let mut losses = 0u32;

    println!(
        "{:<52} {:>9} {:>9} {:>9} {:>8} {:>5}",
        "file", "in", "lzan", "lzsa2", "ratio", "vs"
    );
    for p in &entries {
        let data = fs::read(p).unwrap();
        let c = compress_with(&data, &opts);
        let d = decompress(&c).expect("decompress");
        let ok = d == data;
        if !ok {
            all_ok = false;
        }
        tot_in += data.len() as u64;
        tot_out += c.len() as u64;

        let base = base_by_size
            .get(&(data.len() as u64))
            .and_then(|v| v.first().copied());
        let base_str = match base {
            Some(b) => {
                tot_base += b;
                if (c.len() as u64) < b {
                    wins += 1;
                    format!("{:>9}", b)
                } else {
                    losses += 1;
                    format!("{:>9}", b)
                }
            }
            None => "        -".to_string(),
        };
        let vs = match base {
            Some(b) => {
                if (c.len() as u64) < b {
                    "WIN"
                } else {
                    "lose"
                }
            }
            None => "-",
        };
        let name = p.file_name().unwrap().to_string_lossy();
        let short: String = name.chars().take(52).collect();
        println!(
            "{:<52} {:>9} {:>9} {} {:>8.4} {:>5}{}",
            short,
            data.len(),
            c.len(),
            base_str,
            ratio(c.len(), data.len()),
            vs,
            if ok { "" } else { "  !!ROUNDTRIP-FAIL" }
        );
    }

    println!("{}", "-".repeat(100));
    println!(
        "TOTAL in={} lzan={} ({:.4})   lzsa2_best={} ({:.4})   wins={} losses={}  roundtrip={}",
        tot_in,
        tot_out,
        ratio(tot_out as usize, tot_in as usize),
        tot_base,
        ratio(tot_base as usize, tot_in as usize),
        wins,
        losses,
        if all_ok { "ALL-OK" } else { "FAILURES!" }
    );
    if tot_base > 0 {
        let delta = tot_base as i64 - tot_out as i64;
        println!(
            "LZAN vs lzsa2_best: {} bytes ({:+.2}%)  {}",
            delta,
            -100.0 * delta as f64 / tot_base as f64 * -1.0,
            if tot_out < tot_base {
                "*** LZAN WINS OVERALL ***"
            } else {
                "lzsa2 still ahead"
            }
        );
    }

    if all_ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

//! Standalone pucrunch-format CLI: compress a PRG into a `p`,`u` file or
//! decompress one back.
//!
//!   cargo run --release --example pucrunch -- c <in.prg> <out.pu>
//!   cargo run --release --example pucrunch -- d <in.pu> <out.prg>

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let usage = "usage: pucrunch c|d <infile> <outfile>";
    let (mode, inf, outf) = match args.as_slice() {
        [_, m, i, o] => (m.as_str(), i, o),
        _ => {
            eprintln!("{usage}");
            std::process::exit(2);
        }
    };
    let data = std::fs::read(inf).unwrap_or_else(|e| panic!("{inf}: {e}"));
    let out = match mode {
        "c" => {
            let packed = lzan::pucrunch::compress_pucrunch_prg(&data);
            eprintln!("in {} bytes, out {} bytes", data.len(), packed.len());
            packed
        }
        "d" => {
            let prg =
                lzan::pucrunch::decompress_pucrunch(&data).unwrap_or_else(|e| panic!("{inf}: {e}"));
            eprintln!("in {} bytes, out {} bytes", data.len(), prg.len());
            prg
        }
        _ => {
            eprintln!("{usage}");
            std::process::exit(2);
        }
    };
    std::fs::write(outf, &out).unwrap_or_else(|e| panic!("{outf}: {e}"));
}

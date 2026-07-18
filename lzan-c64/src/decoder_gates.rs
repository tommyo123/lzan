//! Per-crunch tailored decoder composition.
//!
//! A decoder source may carry *gate* annotations - comment lines the
//! assembler ignores - that mark sections gated on a stream *feature*. The
//! static source assembles with every gated section in its "feature present"
//! form (the annotation markers and the alternative lines are plain comments),
//! so an un-composed source assembles byte-identically to the fully-featured
//! decoder. Given the set of features a specific compressed stream actually
//! uses, [`compose`] emits a trimmed source where every gate whose feature is
//! ABSENT is replaced by its smaller alternative - the tailored decoder.
//!
//! asm6502 has no conditional-assembly directives, so this composition happens
//! Rust-side as line selection. Because the markers and alternative lines are
//! comments, the two hard invariants hold by construction:
//!
//! 1. **Anchor** - `compose(source, <all gated features>)` assembles to the
//!    same bytes as the raw static source (both keep exactly the if-present
//!    lines as code; markers and `;g` lines are zero-byte comments either way).
//! 2. **Address independence** - a gate only ever selects between line sets;
//!    it never depends on an address, so a body composed for a stream's traits
//!    has a size fixed at composition time.
//!
//! Grammar (each marker is a whole line; leading whitespace before the marker
//! is allowed):
//!
//! ```text
//!   ;>>> gate <feature>
//!         <lines kept when <feature> is present>
//!   ;=== else                     (optional)
//!   ;g <line kept when <feature> is absent>
//!   ;g <...>
//!   ;<<< gate
//! ```
//!
//! The `;g ` prefix (semicolon, `g`, one separating space) keeps the
//! alternative lines inert in the static build; [`compose`] strips it when it
//! activates them. Gates do not nest.

use std::collections::BTreeSet;

const BEGIN: &str = ";>>> gate ";
const ELSE: &str = ";=== else";
const END: &str = ";<<< gate";
const ALT: &str = ";g";

/// List the distinct feature names gated in `source`, in first-appearance
/// order. `compose(source, &all_of_these)` is the anchor (== static source).
pub fn gates(source: &str) -> Vec<String> {
    let mut v: Vec<String> = Vec::new();
    for line in source.lines() {
        if let Some(rest) = line.trim_start().strip_prefix(BEGIN) {
            let f = rest.trim().to_string();
            if !f.is_empty() && !v.contains(&f) {
                v.push(f);
            }
        }
    }
    v
}

/// Compose a tailored source for the given set of *present* features. Every
/// gate whose feature is in `present` keeps its if-present lines; every gate
/// whose feature is absent emits its `;g` alternative lines (prefix stripped).
/// Marker lines are dropped. Returns an error string describing the first
/// malformed annotation (unbalanced / nested markers, a non-`;g` line in an
/// else branch, an unterminated gate).
pub fn compose(source: &str, present: &BTreeSet<String>) -> Result<String, String> {
    let mut out = String::with_capacity(source.len());
    // Gate state: None outside a gate; Some((feature_present, in_else_branch)).
    let mut gate: Option<(bool, bool)> = None;

    for (i, line) in source.lines().enumerate() {
        let lineno = i + 1;
        let t = line.trim_start();

        if let Some(rest) = t.strip_prefix(BEGIN) {
            if gate.is_some() {
                return Err(format!("gate: nested gate at line {lineno}"));
            }
            let feat = rest.trim();
            if feat.is_empty() {
                return Err(format!("gate: empty gate feature at line {lineno}"));
            }
            gate = Some((present.contains(feat), false));
            continue; // marker line: dropped
        }
        if t.starts_with(ELSE) {
            match gate {
                Some((keep_if, false)) => gate = Some((keep_if, true)),
                Some((_, true)) => {
                    return Err(format!("gate: duplicate else at line {lineno}"))
                }
                None => return Err(format!("gate: else outside a gate at line {lineno}")),
            }
            continue; // marker line: dropped
        }
        if t.starts_with(END) {
            if gate.is_none() {
                return Err(format!("gate: gate end outside a gate at line {lineno}"));
            }
            gate = None;
            continue; // marker line: dropped
        }

        match gate {
            None => {
                out.push_str(line);
                out.push('\n');
            }
            Some((keep_if, false)) => {
                // if-present branch: kept only when the feature is present.
                if keep_if {
                    out.push_str(line);
                    out.push('\n');
                }
            }
            Some((keep_if, true)) => {
                // else branch: the alternative used when the feature is ABSENT.
                // Every line MUST be `;g`-prefixed so the static build ignores
                // it; anything else would assemble in the static path and break
                // the anchor invariant.
                if let Some(act) = activate_alt(line) {
                    if !keep_if {
                        out.push_str(&act);
                        out.push('\n');
                    }
                } else {
                    return Err(format!(
                        "gate: non-`;g` line in else branch at line {lineno}: {line:?}"
                    ));
                }
            }
        }
    }

    if gate.is_some() {
        return Err("gate: unterminated gate (missing `;<<< gate`)".to_string());
    }
    Ok(out)
}

/// Strip the `;g` alternative-line prefix (and one optional separating space).
/// `";g         RTS"` -> `"        RTS"`, `";g exit:"` -> `"exit:"`, `";g"` ->
/// `""`. Returns `None` if `line` is not `;g`-prefixed (leading whitespace is
/// tolerated so the annotation can be indented with the block it replaces).
fn activate_alt(line: &str) -> Option<String> {
    let t = line.trim_start();
    let rest = t.strip_prefix(ALT)?;
    let rest = rest.strip_prefix(' ').unwrap_or(rest);
    Some(rest.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(items: &[&str]) -> BTreeSet<String> {
        items.iter().map(|s| s.to_string()).collect()
    }

    const SAMPLE: &str = "\
head
;>>> gate litseq
        keep_a
        keep_b
;=== else
;g        alt_a
;<<< gate
mid
;>>> gate len16
        only_when_present
;<<< gate
tail";

    #[test]
    fn gates_lists_features_in_order() {
        assert_eq!(gates(SAMPLE), vec!["litseq".to_string(), "len16".to_string()]);
    }

    #[test]
    fn compose_all_present_is_the_static_body_minus_markers() {
        // compose(ALL) keeps every if-present line, drops markers and the `;g`
        // alternative - i.e. exactly the code lines of the static source.
        let all = compose(SAMPLE, &set(&["litseq", "len16"])).unwrap();
        assert_eq!(
            all,
            "head\n        keep_a\n        keep_b\nmid\n        only_when_present\ntail\n"
        );
    }

    #[test]
    fn compose_absent_uses_the_alternative() {
        let none = compose(SAMPLE, &set(&[])).unwrap();
        // litseq absent -> alt_a (`;g ` prefix stripped: the 8 spaces after `;g`
        // lose one separator, leaving 7); len16 absent -> its (empty) else, so
        // the gated line is dropped.
        assert_eq!(none, "head\n       alt_a\nmid\ntail\n");
    }

    #[test]
    fn compose_mixed() {
        let m = compose(SAMPLE, &set(&["len16"])).unwrap();
        assert_eq!(m, "head\n       alt_a\nmid\n        only_when_present\ntail\n");
    }

    #[test]
    fn activate_strips_prefix_preserving_indent() {
        assert_eq!(activate_alt(";g         RTS").unwrap(), "        RTS");
        assert_eq!(activate_alt(";g exit_or_lit_seq:").unwrap(), "exit_or_lit_seq:");
        assert_eq!(activate_alt(";g").unwrap(), "");
        assert!(activate_alt("        RTS").is_none());
    }

    #[test]
    fn unterminated_gate_errors() {
        assert!(compose("head\n;>>> gate x\n  body\n", &set(&[])).is_err());
    }

    #[test]
    fn nested_gate_errors() {
        assert!(compose(";>>> gate a\n;>>> gate b\n;<<< gate\n;<<< gate\n", &set(&[])).is_err());
    }

    #[test]
    fn non_alt_line_in_else_errors() {
        let s = ";>>> gate a\n  keep\n;=== else\n  not_prefixed\n;<<< gate\n";
        assert!(compose(s, &set(&[])).is_err());
    }
}

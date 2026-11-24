#!/usr/bin/env python3
"""
model_bench.py - side-by-side benchmark of the PLASMA evidence decoder.

Every test case is a list of 1-3 QR "evidence fragments" with a known answer (red/blue/green/unknown).
Each case is run through
  (a) the rule-based decoder in plasma_vault.py   (analyse_fragment -> color_from_evidence)
  (b) every Ollama model named on the command line, via plasma_ai.Gemma.analyse_evidence
      (same prompt, same JSON format, same timeout the robot uses)
and the script prints a per-case grid plus a summary: accuracy, latency, failures/timeouts, and the
accuracy of the real robot pipeline (rules + model, combined exactly like plasma_vault.chosen_core).

Usage (Python 3.12 is not on PATH on the team laptop):
  set PY=%LOCALAPPDATA%\\Programs\\Python\\Python312\\python.exe
  %PY% model_bench.py --rule-only                                  # baseline, no network at all
  %PY% model_bench.py --models gemma4 qwen3:4b llama3.2:3b phi4-mini
  %PY% model_bench.py --models gemma4:e4b -v --out bench.json        # show decodes/reasoning, save JSON
  %PY% model_bench.py --models gemma4 --only rot13,riddle-sky       # a few cases only

Only the local Ollama server is contacted (OLLAMA_HOST, default 127.0.0.1:11434). The robot is never touched.
"""

import argparse
import base64
import json
import os
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plasma_vault as pv   # noqa: E402  (no side effects at import)
import plasma_ai            # noqa: E402

COLORS = ("red", "blue", "green")


# =====================================================================================
# Encoders (independent of plasma_vault, so a decoder bug cannot hide in a shared table)
# =====================================================================================
def caesar(s, k):
    out = []
    for c in s:
        if "A" <= c <= "Z":
            out.append(chr((ord(c) - 65 + k) % 26 + 65))
        elif "a" <= c <= "z":
            out.append(chr((ord(c) - 97 + k) % 26 + 97))
        else:
            out.append(c)
    return "".join(out)


def rot13(s):
    return caesar(s, 13)


def atbash(s):
    return "".join(chr(155 - ord(c)) if "A" <= c <= "Z" else chr(219 - ord(c)) if "a" <= c <= "z" else c for c in s)


def vigenere(s, key):
    k = [ord(ch) - 65 for ch in key.upper()]
    out, i = [], 0
    for c in s:
        if "A" <= c <= "Z" or "a" <= c <= "z":
            base = 65 if c <= "Z" else 97
            out.append(chr((ord(c) - base + k[i % len(k)]) % 26 + base))
            i += 1
        else:
            out.append(c)
    return "".join(out)


MORSE_ENC = {"A": ".-", "B": "-...", "C": "-.-.", "D": "-..", "E": ".", "F": "..-.", "G": "--.", "H": "....",
             "I": "..", "J": ".---", "K": "-.-", "L": ".-..", "M": "--", "N": "-.", "O": "---", "P": ".--.",
             "Q": "--.-", "R": ".-.", "S": "...", "T": "-", "U": "..-", "V": "...-", "W": ".--", "X": "-..-",
             "Y": "-.--", "Z": "--.."}


def morse(s):
    return " / ".join(" ".join(MORSE_ENC[c] for c in w) for w in s.upper().split())


def binary(s):
    return " ".join(format(ord(c), "08b") for c in s)


def a1z26(s, word_sep=" / "):
    return word_sep.join(" ".join(str(ord(c) - 64) for c in w) for w in s.upper().split())


def b64(s):
    return base64.b64encode(s.encode()).decode()


def hex_text(s):
    return s.encode().hex().upper()


def url_pct(s):
    return "".join("%%%02X" % ord(c) for c in s)


# =====================================================================================
# Test set: id, category, fragments (1-3), expected answer
# =====================================================================================
CASES = [
    dict(id="plain", cat="plain text", want="blue",
         frags=["THE BLUE CORE IS SAFE"]),
    dict(id="plain-lower", cat="plain text", want="green",
         frags=["Evidence log 7: the green core is the correct one"]),
    dict(id="base64", cat="base64", want="red",
         frags=[b64("THE RED CORE IS STABLE")]),
    dict(id="rot13", cat="ROT13", want="green",
         frags=[rot13("THE GREEN CORE IS SAFE")]),
    dict(id="caesar-3", cat="Caesar +3", want="blue",
         frags=[caesar("THE BLUE CORE IS CORRECT", 3)]),
    dict(id="caesar-19", cat="Caesar +19", want="red",
         frags=[caesar("ONLY THE RED CORE CAN RESTART THE VAULT", 19)]),
    dict(id="vigenere", cat="Vigenere + key hint", want="green",
         frags=["VIGENERE KEY IS VAULT: " + vigenere("THE GREEN CORE IS SAFE", "VAULT")]),
    dict(id="reversed", cat="reversed", want="red",
         frags=["THE RED CORE IS SAFE"[::-1]]),
    dict(id="atbash", cat="Atbash", want="green",
         frags=[atbash("THE GREEN CORE IS SAFE")]),
    dict(id="morse", cat="Morse", want="green",
         frags=[morse("GREEN IS SAFE")]),
    dict(id="binary", cat="binary", want="blue",
         frags=[binary("BLUE CORE")]),
    dict(id="a1z26-word", cat="A1Z26 (one word)", want="green",
         frags=[a1z26("GREEN")]),
    dict(id="a1z26-sentence", cat="A1Z26 (words / sep)", want="blue",
         frags=[a1z26("BLUE CORE IS SAFE")]),
    dict(id="hex-text", cat="hex-encoded text", want="blue",
         frags=[hex_text("BLUE IS CORRECT")]),
    dict(id="url-encoded", cat="URL %-encoding", want="green",
         frags=[url_pct("GREEN CORE")]),
    dict(id="hexcolour", cat="hex colour code", want="green",
         frags=["ENERGY SIGNATURE #00FF00"]),
    dict(id="hexcolour-0x", cat="hex colour code", want="blue",
         frags=["CORE SIGNATURE 0x1E90FF"]),
    dict(id="foreign", cat="foreign word", want="green",
         frags=["LE NOYAU VERT EST LE BON"]),
    dict(id="negation", cat="negation", want="blue",
         frags=["RED IS A DECOY. THE BLUE CORE IS SAFE."]),
    dict(id="negation-nopunct", cat="negation, no punctuation", want="blue",
         frags=["RED IS A DECOY TRUST THE BLUE CORE"]),
    dict(id="negation-elim", cat="negation -> elimination", want="blue",
         frags=["NOT RED. NOT GREEN."]),
    dict(id="multi-elim", cat="multi: elimination", want="blue",
         frags=["RED IS A DECOY", rot13("GREEN IS NOT SAFE")]),
    dict(id="multi-majority", cat="multi: majority", want="red",
         frags=[b64("THE RED CORE IS SAFE"), "BLUE IS SAFE", morse("RED IS CORRECT")]),
    dict(id="multi-mixed", cat="multi: decoy+reversed+hex", want="blue",
         frags=["THE CORE IS NOT GREEN", "THE BLUE CORE IS STABLE"[::-1], "#0000FF"]),
    dict(id="layered", cat="base64 of ROT13", want="red",
         frags=[b64(rot13("THE RED CORE IS SAFE"))]),
    dict(id="riddle-sky", cat="riddle", want="blue",
         frags=["THE TRUE CORE HAS THE COLOUR OF A CLEAR DAYTIME SKY"]),
    dict(id="riddle-grass", cat="riddle", want="green",
         frags=["SEEK THE CORE THE COLOUR OF FRESH GRASS"]),
    dict(id="no-colour", cat="no colour info", want="unknown",
         frags=["THE VAULT REMEMBERS EVERY VISITOR"]),
    dict(id="no-colour-b64", cat="no colour info (base64)", want="unknown",
         frags=[b64("THE VAULT DOOR OPENS AT DAWN")]),
]


# =====================================================================================
# Engines
# =====================================================================================
def run_rules(analysed):
    t0 = time.perf_counter()
    choice, votes, reasons = pv.color_from_evidence(analysed)
    return {"core": choice or "unknown", "votes": votes, "reasons": reasons,
            "latency": time.perf_counter() - t0}


def pipeline(rule_core, g):
    """plasma_vault.chosen_core without --core/config: Gemma if confident (>=0.6) or rules found nothing,
    else rules. Production then defaults to 'red' - counted here as 'unknown'."""
    rc = None if rule_core == "unknown" else rule_core
    if g and g.get("core") in COLORS and (g.get("confidence", 0) >= 0.6 or not rc):
        return g["core"]
    return rc or "unknown"


class LogTap:
    """Captures plasma_ai._log so 'Gemma call failed: timed out' can be classified per call."""

    def __init__(self):
        self.lines = []

    def __call__(self, msg):
        self.lines.append(str(msg))


def ollama_tags():
    with urllib.request.urlopen(plasma_ai.AI_CFG["GEMMA_URL"] + "/api/tags", timeout=3) as r:
        return [m["name"] for m in json.loads(r.read())["models"]]


def resolve(name, tags):
    """Exact tag, then name:latest, then prefix (like Gemma.__init__) - but never Gemma.__init__'s fallback to
    'any gemma model', which would silently benchmark gemma under another model's name."""
    n = name.lower()
    for t in tags:
        if t.lower() == n or t.lower() == n + ":latest":
            return t
    pref = [t for t in tags if t.lower().startswith(n)]
    return pref[0] if pref else None


def run_model(name, resolved, cases, analysed, rules, args, tap):
    plasma_ai.AI_CFG["GEMMA_MODEL"] = name
    g = plasma_ai.Gemma(enabled=False)          # no auto-detect / fallback; we pick the exact tag ourselves
    g.model = resolved
    print("\n[%s] loading weights (warm-up, not timed in results)..." % resolved, flush=True)
    tap.lines.clear()
    t0 = time.perf_counter()
    warm = g.ask("Reply with OK.", as_json=False, timeout=args.warmup_timeout)
    load_s = time.perf_counter() - t0
    out = {"model": resolved, "requested": name, "load_s": load_s, "cases": {}}
    if warm is None:
        out["skipped"] = "warm-up failed: %s" % (tap.lines[-1] if tap.lines else "no reply")
        print("   " + out["skipped"])
        return out
    print("   loaded in %.1f s" % load_s)
    for c in cases:
        tap.lines.clear()
        t0 = time.perf_counter()
        try:
            res = g.analyse_evidence(analysed[c["id"]])
        except Exception as e:                   # analyse_evidence itself crashed on a weird reply
            res = None
            tap.lines.append("exception in analyse_evidence: %r" % e)
        dt = time.perf_counter() - t0
        if res is None:
            status = "timeout" if any("timed out" in ln.lower() for ln in tap.lines) else "fail"
            r = {"status": status, "answer": status.upper(), "latency": dt,
                 "error": tap.lines[-1] if tap.lines else "no/invalid JSON reply"}
            pipe = pipeline(rules[c["id"]]["core"], None)
        else:
            ans = res["core"] or "unknown"
            r = {"status": "ok", "answer": ans, "latency": dt, "confidence": res["confidence"],
                 "decoded": res["decoded"], "reasoning": res["reasoning"]}
            pipe = pipeline(rules[c["id"]]["core"], res)
        r["correct"] = r["answer"] == c["want"]
        r["pipeline"] = pipe
        r["pipeline_correct"] = pipe == c["want"]
        out["cases"][c["id"]] = r
        print("   %-18s want %-7s got %-8s %5.1f s%s" % (
            c["id"], c["want"], r["answer"], dt, "" if r["correct"] else "   <-- wrong"), flush=True)
    return out


# =====================================================================================
# Report
# =====================================================================================
def fmt_lat(sec):
    return "%.1f ms" % (sec * 1000) if sec < 1 else "%.1f s" % sec


def print_report(cases, analysed, rules, models, verbose):
    names = [m["model"] for m in models if "skipped" not in m]
    w_id = max(len(c["id"]) for c in cases) + 2
    w_cat = max(len(c["cat"]) for c in cases) + 2
    cols = ["rules"] + names
    w = [max(9, len(n) + 2) for n in cols]

    print("\n" + "=" * 100)
    print("PER-CASE RESULTS   (* = wrong)")
    print("=" * 100)
    print("%-*s%-*s%-9s" % (w_id, "case", w_cat, "category", "want") + "".join("%-*s" % (wi, n) for n, wi in zip(cols, w)))
    for c in cases:
        cells = []
        r = rules[c["id"]]
        cells.append(r["core"] + ("" if r["core"] == c["want"] else "*"))
        for m in models:
            if "skipped" in m:
                continue
            mr = m["cases"][c["id"]]
            cells.append(mr["answer"] + ("" if mr["correct"] else "*"))
        print("%-*s%-*s%-9s" % (w_id, c["id"], w_cat, c["cat"], c["want"]) +
              "".join("%-*s" % (wi, x) for x, wi in zip(cells, w)))

    n = len(cases)
    print("\n" + "=" * 100)
    print("SUMMARY   (%d cases; pipeline = rules + model combined like plasma_vault.chosen_core)" % n)
    print("=" * 100)
    hdr = "%-26s%-10s%-10s%-12s%-12s%-7s%-9s%-10s%s" % (
        "engine", "correct", "accuracy", "avg lat", "max lat", "fail", "timeout", "pipeline", "load")
    print(hdr)
    ok = sum(rules[c["id"]]["core"] == c["want"] for c in cases)
    lat = [rules[c["id"]]["latency"] + analysed["_lat"][c["id"]] for c in cases]
    print("%-26s%-10s%-10s%-12s%-12s%-7s%-9s%-10s%s" % (
        "rules (plasma_vault)", "%d/%d" % (ok, n), "%.0f%%" % (100.0 * ok / n), fmt_lat(sum(lat) / n),
        fmt_lat(max(lat)), "0", "0", "-", "-"))
    for m in models:
        if "skipped" in m:
            print("%-26s%s" % (m["model"][:25], "SKIPPED: " + m["skipped"]))
            continue
        rs = list(m["cases"].values())
        ok = sum(r["correct"] for r in rs)
        pok = sum(r["pipeline_correct"] for r in rs)
        lats = [r["latency"] for r in rs]
        print("%-26s%-10s%-10s%-12s%-12s%-7s%-9s%-10s%s" % (
            m["model"][:25], "%d/%d" % (ok, n), "%.0f%%" % (100.0 * ok / n), fmt_lat(sum(lats) / n),
            fmt_lat(max(lats)), sum(r["status"] == "fail" for r in rs), sum(r["status"] == "timeout" for r in rs),
            "%.0f%%" % (100.0 * pok / n), "%.0f s" % m["load_s"]))

    print("\nRULE-BASED MISSES")
    misses = [c for c in cases if rules[c["id"]]["core"] != c["want"]]
    if not misses:
        print("   none")
    for c in misses:
        r = rules[c["id"]]
        print("   %s: want %s, got %s   votes %s" % (c["id"], c["want"], r["core"],
                                                     {k: v for k, v in r["votes"].items() if v}))
        for f in analysed[c["id"]]:
            print("      raw %r\n      -> %s: %r" % (f["raw"][:90], f["method"], str(f["decoded"])[:90]))

    if verbose:
        for m in models:
            if "skipped" in m:
                continue
            print("\nDETAIL %s" % m["model"])
            for c in cases:
                r = m["cases"][c["id"]]
                flag = "ok " if r["correct"] else "BAD"
                if r["status"] != "ok":
                    print("   [%s] %-18s %s: %s" % (flag, c["id"], r["status"], r["error"][:120]))
                else:
                    print("   [%s] %-18s %s conf=%.2f decoded=%s\n          reasoning: %s" % (
                        flag, c["id"], r["answer"], r["confidence"], [d[:60] for d in r["decoded"]],
                        r["reasoning"][:200]))


# =====================================================================================
def main():
    for s in (sys.stdout, sys.stderr):           # unicode fragments must not crash a Windows console
        try:
            s.reconfigure(errors="backslashreplace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description="Benchmark the PLASMA evidence decoder: rules vs Ollama models")
    ap.add_argument("--models", nargs="+", default=[], metavar="MODEL", help="Ollama tags, e.g. gemma4 qwen3:4b")
    ap.add_argument("--rule-only", action="store_true", help="only the rule-based baseline (no Ollama)")
    ap.add_argument("--timeout", type=float, default=plasma_ai.AI_CFG["GEMMA_TIMEOUT"],
                    help="per-call LLM timeout in s (default %(default)s = plasma_ai GEMMA_TIMEOUT, as on the robot)")
    ap.add_argument("--warmup-timeout", type=float, default=300, help="time allowed to load a model (s)")
    ap.add_argument("--only", help="comma-separated case ids to run")
    ap.add_argument("-v", "--verbose", action="store_true", help="print decodes, votes and model reasoning")
    ap.add_argument("--out", help="also write all results to this JSON file")
    args = ap.parse_args()

    if "10.10.53." in plasma_ai.AI_CFG["GEMMA_URL"]:
        sys.exit("OLLAMA_HOST points at the robot network (%s) - refusing." % plasma_ai.AI_CFG["GEMMA_URL"])
    cases = CASES
    if args.only:
        want = {x.strip() for x in args.only.split(",") if x.strip()}
        cases = [c for c in CASES if c["id"] in want]
        if not cases:
            sys.exit("no case matches --only %s (ids: %s)" % (args.only, ", ".join(c["id"] for c in CASES)))
    if args.rule_only and args.models:
        print("--rule-only given: ignoring --models %s" % " ".join(args.models))
    model_names = [] if args.rule_only else args.models
    if not model_names and not args.rule_only:
        print("no --models given: running the rule-based baseline only")

    # analyse every fragment once (deterministic), exactly like add_fragment() does on the robot
    analysed, rules = {"_lat": {}}, {}
    for c in cases:
        t0 = time.perf_counter()
        analysed[c["id"]] = [pv.analyse_fragment(t) for t in c["frags"]]
        analysed["_lat"][c["id"]] = time.perf_counter() - t0
        rules[c["id"]] = run_rules(analysed[c["id"]])
        if args.verbose:
            print("%-18s want %-7s rules %-7s votes %s" % (c["id"], c["want"], rules[c["id"]]["core"],
                                                          rules[c["id"]]["votes"]))
            for f in analysed[c["id"]]:
                print("    raw %r -> %s: %r" % (f["raw"][:80], f["method"], str(f["decoded"])[:80]))

    models = []
    if model_names:
        plasma_ai.AI_CFG["GEMMA_TIMEOUT"] = args.timeout
        tap = LogTap()
        plasma_ai._log = tap
        try:
            tags = ollama_tags()
            print("Ollama at %s has: %s" % (plasma_ai.AI_CFG["GEMMA_URL"], ", ".join(tags) or "(no models)"))
        except Exception as e:
            tags = None
            print("Ollama not reachable at %s (%s). Install it / run 'ollama serve', then pull the models."
                  % (plasma_ai.AI_CFG["GEMMA_URL"], e))
        for name in model_names:
            if tags is None:
                models.append({"model": name, "skipped": "Ollama not reachable"})
                continue
            resolved = resolve(name, tags)
            if not resolved:
                models.append({"model": name, "skipped": "not installed -> ollama pull %s" % name})
                continue
            models.append(run_model(name, resolved, cases, analysed, rules, args, tap))

    print_report(cases, analysed, rules, models, args.verbose)

    if args.out:
        dump = {"timeout_s": args.timeout,
                "cases": [{"id": c["id"], "category": c["cat"], "fragments": c["frags"], "want": c["want"],
                           "rules": {"core": rules[c["id"]]["core"], "votes": rules[c["id"]]["votes"],
                                     "decoded": [(f["method"], f["decoded"]) for f in analysed[c["id"]]]}}
                          for c in cases],
                "models": models}
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(dump, f, indent=2, ensure_ascii=False)
        print("\nresults written to %s" % args.out)


if __name__ == "__main__":
    main()

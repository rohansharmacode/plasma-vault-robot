# Evidence decoder benchmark

`model_bench.py` runs 29 evidence cases with known answers (1-3 QR fragments each) through:

- the rule-based decoder (`plasma_vault.analyse_fragment` and `color_from_evidence`), and
- any Ollama model, through `plasma_ai.Gemma.analyse_evidence`. That is the same prompt, JSON format and 25 s timeout the robot uses.

It reports per-model accuracy, average and maximum latency, failures, timeouts and **pipeline** accuracy. Pipeline accuracy combines rules and model the way `chosen_core` does: the model wins if its confidence is at least 0.6 or the rules found nothing. That is what the robot actually commits to.

The test set covers:

- plain text, base64, ROT13, Caesar +3 and +19, Vigenere with the key given, reversed text, Atbash, Morse and binary
- A1Z26 (a single word, and a sentence with `/` between words), hex-encoded text and URL %-encoding
- hex colour codes, a foreign word, negations, elimination and majority across several fragments
- base64 of ROT13, two riddles, and two cases with no colour at all (expected: unknown)

## Baseline (rule-based only), 2026-10-25

`--rule-only`: **22/29 correct (76%)**, about 2 ms per case.

| Miss | Input | Expected | Got | Why |
|---|---|---|---|---|
| **Bug** | `RED IS A DECOY TRUST THE BLUE CORE` | blue | **green** | Clauses are split only on `.,;:!?`, BUT and WHILE. A negation word anywhere in a clause counts against every colour in it, so red and blue both get -1 and "elimination" picks green. These also return green: `BLUE IS SAFE NOT RED`, `THE SAFE CORE IS BLUE AND RED IS UNSAFE`, `AVOID RED - CHOOSE BLUE`. |
| **Bug** | `LE NOYAU VERT EST LE BON` | green | unknown | `score()` rewards any candidate that contains a colour word. caesar-10 turns it into `BU DEOQK LUHJ UIJ BU RED`, which beats the raw text. Red +1 then ties green's VERT hint. Same root cause: `THE CORE IS IN THE HUT` gives **red** (caesar-16 turns HUT into RED). |
| **Bug** | `2 12 21 5 / 3 15 18 5 / 9 19 / 19 1 6 5` | blue | unknown | The A1Z26 regex `[\d\s,]+` rejects `/`, and dashes too (`2-12-21-5`). With spaces only, `2 12 21 5 3 15 18 5` decodes to `BLUECORE` with no spaces, so `\bBLUE\b` never matches. A single word (`2 12 21 5`) works. |
| Gap | `VIGENERE KEY IS VAULT: OHY RKZEH NHME CD LVFY` | green | unknown | Vigenere is not implemented. |
| Gap | base64 of ROT13 (`R1VSIEVSUSBQQkVSIFZGIEZOU1I=`) | red | unknown | The second layer only base64-decodes after Caesar or reversal. It never applies Caesar after base64. |
| Gap | riddles ("colour of a clear daytime sky", "fresh grass") | blue / green | unknown | No world knowledge. This is the LLM's job. |

Minor: German `GRUNE` (an inflected form) is missed, because only GRUN and GRÜN are listed.

## Ollama status on this laptop

Ollama is **not installed**: there is no `ollama` on PATH, no install folder, and nothing answers on 127.0.0.1:11434. Nothing was installed or pulled.

**Install and pull on another network** (phone hotspot or home), not on the hackathon Wi-Fi the robot shares. The total download is about 17 GB:

```powershell
winget install --id Ollama.Ollama -e        # or OllamaSetup.exe from https://ollama.com/download/windows
ollama --version
ollama pull gemma4:e4b          # 6.6 GB  Gemma 4 E4B (= gemma4 / gemma4:latest). Best small Gemma 4 for a laptop
ollama pull gemma4:e2b          # 4.6 GB  optional: 8 GB RAM / CPU-only fallback
ollama pull qwen3:4b-instruct   # 2.5 GB  Qwen3 4B 2507, non-thinking (qwen3:4b is the *thinking* variant)
ollama pull llama3.2:3b         # 2.0 GB
ollama pull phi4-mini           # 2.5 GB  (3.8B)
ollama pull ministral-3:3b      # 3.0 GB  Mistral's small edge model; mistral-small3.2 is 24B / 15 GB, too big here
```

All of these tags were checked on ollama.com/library on 2026-10-25.

## Running the benchmark

```powershell
$PY = "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe"
& $PY model_bench.py --rule-only
& $PY model_bench.py --models gemma4:e4b qwen3:4b-instruct llama3.2:3b phi4-mini ministral-3:3b --out bench.json
& $PY model_bench.py --models gemma4:e4b -v --only riddle-sky,negation-nopunct   # show reasoning on a few cases
```

Model loading is warmed up first and is not counted in the latency figures. `--timeout` defaults to `GEMMA_TIMEOUT` (25 s). The script picks the exact tag itself, because `Gemma.__init__` would silently fall back to *any* gemma model if the requested one is missing.

## How to choose a model

1. **Reliability gate.** Drop any model with more than one fail or timeout at the default 25 s. On the robot, a timeout silently falls back to the rule-based answer.
2. **Accuracy first.** Rank by accuracy, then by pipeline accuracy. Look closely at `no-colour*` and `negation*`: a confident wrong answer (confidence 0.6 or higher) overrides correct rules.
3. **Then latency.** Among models within about 1 case of the best, take the fastest one whose **average is under ~20 s** and whose maximum stays under 25 s. The model call is inside the 3-minute run.
4. **Keep Gemma 4 as the default** (it is what the prize is for) unless a rival beats it clearly, for example by 3 or more cases. Run each model twice (temperature 0.2 varies a little), on the laptop as it will be at the event (plugged in, other apps closed).
5. **Pin the chosen tag** before the run: `$env:GEMMA_MODEL="gemma4:e4b"`. `plasma_ai` takes the first installed tag that starts with `GEMMA_MODEL`, so with several gemma4 tags installed the choice is otherwise arbitrary.

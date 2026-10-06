# Fault State Machine with Structured Recovery (ESP32 + Wokwi)

A deterministic fault state machine for an ESP32 battery and relay system. It has four
states (NORMAL, DEGRADED, FAILSAFE, SHUTDOWN) driven by one transition table. Every fault
has an ID and a source (CELL, RELAY, COMM or ADC) and is remembered for the whole run.
Recovery from FAILSAFE goes through a verification sequence, never straight back to NORMAL.
Every state transition is logged as one JSON line.

▶ **Run it live:** <https://wokwi.com/projects/477122498605360129>
🎥 **Demo video:** <https://lnkd.in/p/geRKw9bV>

## Features
- `enum` states and a single table `TRANS[state][event]` that defines every transition
- Fault isolation: each fault ID carries a source (CELL / RELAY / COMM / ADC) and a severity
- Fault registry: count, first time and last time for each fault, kept all run (button dumps it)
- Frozen-ADC detection (identical readings in a row) and relay-mismatch detection (commanded state vs feedback)
- Communication monitoring over a real UART loopback with CRC, timeout and sequence checks
- Verified recovery: stable wait, relay test, ADC liveness check, comm health check
- Bounded behaviour: FAILSAFE has a timeout and a retry limit, so it cannot get stuck
- Structured JSON-lines log with timestamp, previous state, new state and fault ID
- No `delay()`; everything runs from `millis()` timers

## Files
| File | Purpose |
|---|---|
| `sketch.ino` | Firmware |
| `diagram.json` | Wokwi circuit |
| `libraries.txt` | Wokwi library list (`LiquidCrystal I2C`) |
| `REPORT.md` | Design report |

## Hardware (Wokwi)
| Part | Pin | Role |
|---|---|---|
| Pot 1, 2, 3 | GPIO 34, 35, 32 | Battery cells 1 to 3 (2.8 to 4.4 V) |
| Relay module (active LOW) | IN GPIO 19 | Load switch |
| Relay feedback | COM to 3V3, NO to GPIO 33 | Tells the sketch if the contact really closed |
| UART loopback wire | GPIO 17 to GPIO 16 | Heartbeat link (Serial2) |
| I2C LCD 16x2 | SDA 21, SCL 22 | State, load limit, fault or verify step |
| Green / yellow / red LED | GPIO 4 / 5 / 2 | NORMAL / DEGRADED / FAILSAFE or SHUTDOWN |
| Buzzer | GPIO 18 | Slow pulse in FAILSAFE, fast in SHUTDOWN |
| Button RESET | GPIO 25 | Leave SHUTDOWN |
| Button INJECT | GPIO 26 | Cycle the injected fault |
| Button REPORT | GPIO 27 | Print the fault registry |

## How to run
1. Open the Wokwi link, or create an ESP32 project and paste in `sketch.ino`,
   `diagram.json` and `libraries.txt`.
2. Press Play and open the Serial Monitor (115200 baud). Each line is a JSON object.

## States
| State | Relay | Load limit | LED | Meaning |
|---|---|---|---|---|
| NORMAL | closed | 100% | Green | Healthy |
| DEGRADED | closed | 50% | Yellow | Minor fault, reduced operation |
| FAILSAFE | open | 0% | Red blink | Major fault, safe state, verifying recovery |
| SHUTDOWN | open | 0% | Red solid | Fatal fault or failed recovery, needs the RESET button |

## The transition table
Next state = `TRANS[current state][event]`.

| | CLEAR | MINOR | MAJOR | FATAL | VERIFIED | EXHAUSTED | RESET |
|---|---|---|---|---|---|---|---|
| NORMAL | NORMAL | DEGRADED | FAILSAFE | SHUTDOWN | NORMAL | NORMAL | NORMAL |
| DEGRADED | NORMAL | DEGRADED | FAILSAFE | SHUTDOWN | DEGRADED | DEGRADED | DEGRADED |
| FAILSAFE | FAILSAFE | FAILSAFE | FAILSAFE | SHUTDOWN | DEGRADED | SHUTDOWN | FAILSAFE |
| SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | FAILSAFE |

No row leads from FAILSAFE or SHUTDOWN straight to NORMAL. Recovery always goes
FAILSAFE, then DEGRADED (5 s probation), then NORMAL.

## Fault table
| ID | Name | Source | Severity | Detected by |
|---|---|---|---|---|
| 1 to 3 | C1_LOW to C3_LOW | CELL | DEGRADED | Cell below 3.30 V (clears above 3.40 V) |
| 4 to 6 | C1_CRIT to C3_CRIT | CELL | FAILSAFE | Cell below 3.00 V or above 4.25 V |
| 7 | IMBAL | CELL | DEGRADED | Cell spread above 0.25 V |
| 8 | RLY_OPEN | RELAY | FAILSAFE | Commanded closed, feedback still open |
| 9 | RLY_WELD | RELAY | SHUTDOWN | Commanded open, feedback still closed |
| 10 | COMM_CRC | COMM | DEGRADED | Repeated bad CRC frames |
| 11 | COMM_TMO | COMM | FAILSAFE | No good frame for 500 ms |
| 12 to 14 | ADC1_FRZ to ADC3_FRZ | ADC | FAILSAFE | 20 identical readings in a row (1 s) |

Each fault is debounced: it sets after 3 bad samples (150 ms) and clears after 20 good
ones (1 s). If an ADC channel is frozen, that cell is isolated: its stale value is no
longer used for cell or imbalance checks.

## FAILSAFE recovery (verification)
After a fault clears, the system stays in FAILSAFE and runs these steps in order:

| Step | What it checks | Time |
|---|---|---|
| WAIT_CLEAR | No active faults | until clear |
| STABLE | Still no faults | 3 s |
| RELAY_TEST | Close the relay and check feedback, then open it and check again | 0.8 s |
| ADC_CHECK | Every ADC channel shows live variation | 1 s |
| COMM_CHECK | 8 consecutive in-sequence good frames | under 2 s |

Passing all steps moves FAILSAFE to DEGRADED, then NORMAL after 5 s clean. A failed step
counts as an attempt. After 3 failed attempts, or 45 s in FAILSAFE, the system goes to
SHUTDOWN. A relapse from DEGRADED also counts as an attempt.

## Log format
Every line is one JSON object. State transitions look like this:
```
{"t_ms":21100,"ev":"STATE","seq":1,"from":"NORMAL","to":"FAILSAFE","fault_id":13,"fault":"ADC2_FRZ","source":"ADC","reason":"major fault"}
{"t_ms":29000,"ev":"STATE","seq":2,"from":"FAILSAFE","to":"DEGRADED","fault_id":0,"fault":"NONE","source":"NONE","reason":"verification passed"}
{"t_ms":34000,"ev":"STATE","seq":3,"from":"DEGRADED","to":"NORMAL","fault_id":0,"fault":"NONE","source":"NONE","reason":"stable, hold complete"}
```
`seq` rises by 1 on each transition, so a missed transition shows up as a gap. Other event
types: `FAULT` (set or clear), `VERIFY`, `RELAY`, `INJECT`, `REGISTRY` and `STAT`.

## Demo script (INJECT button cycles the modes)
| Mode | Result |
|---|---|
| ADC_FREEZE | Cell 2 ADC value stuck: ADC2_FRZ, FAILSAFE |
| RELAY_OPEN | Feedback stuck open: RLY_OPEN, FAILSAFE. Recovery fails the relay test 3 times, then SHUTDOWN |
| RELAY_WELD | Not visible while the relay is closed. Turn a pot below 3.0 V so the system opens the relay and the weld is found: SHUTDOWN |
| COMM_LOSS | No heartbeat: COMM_TMO, FAILSAFE |
| COMM_NOISE | Every other frame corrupted: COMM_CRC, DEGRADED only |

Also try the pots: below 3.3 V gives DEGRADED, below 3.0 V or above 4.25 V gives FAILSAFE.
Select NONE to remove an injection and watch the verification steps in the log. Press
RESET to leave SHUTDOWN, and REPORT to print the fault registry.

## Test results
Tested on a PC with stubbed Arduino functions, a simulated relay with feedback delay and a
UART loopback (not on Wokwi or real hardware):
- Table checks: every entry is a valid state, all four states reachable, every state has an exit, no direct path to NORMAL from FAILSAFE or SHUTDOWN
- Each fault type detected, with the right ID and source in the log
- Recovery ran the verification steps in order; DEGRADED lasted exactly 5.000 s before NORMAL
- A failing relay test ended in SHUTDOWN after 3 attempts; RESET led back through FAILSAFE
- 60 simulated minutes of random injections, cell values and button presses: 411 transitions, all legal, none missed, log count equal to the firmware counter, SHUTDOWN only left by RESET, and the longest FAILSAFE was exactly the 45 s limit

## Limitations
- ADC noise (about 3 counts) is simulated in the sketch; a real ADC supplies its own, which is what makes a frozen value detectable
- The relay feedback and UART loopback depend on how Wokwi simulates them; I have not run this on Wokwi. If the relay looks wrong at start-up, change `RELAY_ACTIVE_LOW`
- The relay test closes the relay for 0.4 s; on real equipment do this with the load disconnected or protected
- DEGRADED can last as long as a minor fault stays; it is an operating mode, not a stuck state
- SHUTDOWN is terminal on purpose and needs the RESET button



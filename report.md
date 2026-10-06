# Fault State Machine with Structured Recovery: Project Report

## 1. Objective
Implement a deterministic state machine for an ESP32 battery and relay system that:
- has four operating states, NORMAL, DEGRADED, FAILSAFE and SHUTDOWN, using an `enum` and a clearly defined transition table;
- isolates faults by source (battery cells, relay, communication, ADC) and remembers them for the whole run;
- detects frozen ADC values and relay mismatches;
- logs every transition with timestamp, previous state, new state and fault ID in a structured format;
- recovers from FAILSAFE through a verification process, with reliable, deadlock-free behaviour.

## 2. Hardware (Wokwi)
| Part | Pin | Purpose |
|---|---|---|
| ESP32 DevKit | n/a | Controller |
| 3 potentiometers | GPIO 34, 35, 32 | Battery cell voltages |
| Relay module (active LOW) | IN 19, contacts COM to 3V3 and NO to GPIO 33 | Load switch with real feedback |
| Wire GPIO 17 to GPIO 16 | Serial2 | UART loopback for the heartbeat link |
| I2C LCD 16x2 | SDA 21, SCL 22 | State, load limit, fault or verify step |
| Red / green / yellow LED, buzzer | GPIO 2 / 4 / 5, 18 | Status |
| 3 buttons | GPIO 25 / 26 / 27 | Reset, inject fault, print registry |

## 3. States and what each one does
| State | Relay | Load limit | Meaning |
|---|---|---|---|
| NORMAL | closed | 100% | Healthy |
| DEGRADED | closed | 50% | A minor fault is present, or recovery probation is running |
| FAILSAFE | open | 0% | Major fault. The system is in its safe state and verifying recovery |
| SHUTDOWN | open | 0% | Fatal fault or failed recovery. Needs an operator reset |

The state is stored in a `uint8_t` with named `enum` values. All changes go through one
function, `enterState()`, which logs the change, counts it and runs the entry actions
(relay command and load limit).

## 4. The transition table
Events are produced once per 50 ms sample. A fault event comes from the worst active
fault severity (MINOR, MAJOR or FATAL). `CLEAR` means the DEGRADED probation finished.
`VERIFIED` and `EXHAUSTED` come from the FAILSAFE verification engine. `RESET` is the button.

The next state is a plain table lookup, `TRANS[state][event]`:

| | CLEAR | MINOR | MAJOR | FATAL | VERIFIED | EXHAUSTED | RESET |
|---|---|---|---|---|---|---|---|
| NORMAL | NORMAL | DEGRADED | FAILSAFE | SHUTDOWN | NORMAL | NORMAL | NORMAL |
| DEGRADED | NORMAL | DEGRADED | FAILSAFE | SHUTDOWN | DEGRADED | DEGRADED | DEGRADED |
| FAILSAFE | FAILSAFE | FAILSAFE | FAILSAFE | SHUTDOWN | DEGRADED | SHUTDOWN | FAILSAFE |
| SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | SHUTDOWN | FAILSAFE |

Why a table: all behaviour is visible in one place, every combination is defined (no
missing case), and it can be checked automatically. The sketch verifies at start-up that
each entry is a valid state, and the tests confirmed that:
- all four states are reachable from NORMAL;
- every state has at least one exit;
- no entry leads from FAILSAFE or SHUTDOWN directly to NORMAL.

An event that maps to the current state does nothing. In FAILSAFE, MINOR and MAJOR events
change nothing because the verification engine owns that state.

## 5. Fault identification and isolation
### 5.1 Fault table
Each fault has an ID, a name, a source and a severity, held in one constant table.

| ID | Name | Source | Severity |
|---|---|---|---|
| 1 to 3 | C1_LOW to C3_LOW | CELL | DEGRADED |
| 4 to 6 | C1_CRIT to C3_CRIT | CELL | FAILSAFE |
| 7 | IMBAL | CELL | DEGRADED |
| 8 | RLY_OPEN | RELAY | FAILSAFE |
| 9 | RLY_WELD | RELAY | SHUTDOWN |
| 10 | COMM_CRC | COMM | DEGRADED |
| 11 | COMM_TMO | COMM | FAILSAFE |
| 12 to 14 | ADC1_FRZ to ADC3_FRZ | ADC | FAILSAFE |

Severity follows consequence. A weak cell is a warning. An out-of-range cell, a frozen
measurement or a lost link means the system cannot be trusted, so it goes to the safe state.
A relay welded shut is the worst case, because the load cannot be disconnected at all.

### 5.2 Isolation
- **By source:** every fault carries its source in the state log, on the LCD and in the registry, so it is clear whether the cells, the relay, the link or an ADC channel is at fault.
- **By data:** when an ADC channel is frozen, that cell's stale reading is excluded from the cell and imbalance checks, so one bad channel does not create false cell faults.
- **By severity:** a minor communication problem (CRC errors) only degrades the system, while a communication timeout triggers FAILSAFE. The same source can have different outcomes.

### 5.3 Fault registry
Faults active at any moment are a 16-bit mask. For each fault the registry also keeps:
- the number of times it occurred;
- the time it first appeared;
- the time it was last set or cleared.

This survives state changes for the whole run. The REPORT button prints it as JSON lines.

### 5.4 Debounce
Each fault sets after 3 consecutive bad samples (150 ms) and clears after 20 good ones
(1 s). Cell limits also use hysteresis (for example, a weak cell sets below 3.30 V and
clears above 3.40 V). This prevents a value on a threshold from making a fault flicker
and causing state changes.

## 6. Detection methods
- **Frozen ADC.** A working ADC is noisy, so a reading that does not change for 20
  samples in a row (1 s) means the channel is stuck. The sketch adds about 3 counts of
  simulated noise to each reading, since the simulator's pots are perfectly stable.
- **Relay mismatch.** The relay's COM contact is wired to 3V3 and NO to a GPIO, so the
  sketch reads whether the contact is really closed. After a 150 ms settle time:
  - commanded closed but feedback open means RLY_OPEN (stuck open);
  - commanded open but feedback closed means RLY_WELD (welded).
  A weld can only be seen when the relay is told to open, which is why the relay test during
  recovery matters.
- **Communication.** A 4-byte frame (start byte, sequence number, status, CRC-8) is sent
  every 100 ms through a UART loopback and parsed by a non-blocking receiver. A bad CRC
  raises a leaky error score. No good frame for 500 ms means a timeout. Sequence numbers
  let the sketch require frames to arrive in order.
- **Cells.** Under-voltage, over-voltage and spread limits with hysteresis, on voltages
  from a 12-bit ADC.

## 7. Structured logging
Every log line is one JSON object, so it can be read by a script or loaded into a tool.
State transitions have these fields:

| Field | Meaning |
|---|---|
| `t_ms` | Timestamp in milliseconds since start |
| `ev` | `STATE` |
| `seq` | Transition number, rising by 1 each time |
| `from`, `to` | Previous and new state |
| `fault_id`, `fault`, `source` | The triggering fault (ID 0, NONE for recovery transitions) |
| `reason` | Why: minor fault, major fault, fatal fault, verification passed, stable hold complete, operator reset, failsafe timeout and so on |

```
{"t_ms":21100,"ev":"STATE","seq":1,"from":"NORMAL","to":"FAILSAFE","fault_id":13,"fault":"ADC2_FRZ","source":"ADC","reason":"major fault"}
{"t_ms":29000,"ev":"STATE","seq":2,"from":"FAILSAFE","to":"DEGRADED","fault_id":0,"fault":"NONE","source":"NONE","reason":"verification passed"}
{"t_ms":34000,"ev":"STATE","seq":3,"from":"DEGRADED","to":"NORMAL","fault_id":0,"fault":"NONE","source":"NONE","reason":"stable, hold complete"}
```
Because `seq` increases by exactly 1 per transition, a missed or unlogged transition would
show as a gap. Other event types in the same format are `FAULT` (set and clear), `VERIFY`,
`RELAY`, `INJECT`, `REGISTRY` and a `STAT` line every 5 s with counters.

## 8. Recovery from FAILSAFE
A cleared fault does not by itself return the system to service. FAILSAFE runs a fixed
verification sequence:

| Step | Check | Duration |
|---|---|---|
| WAIT_CLEAR | No fault is active | until clear |
| STABLE | No fault for a continuous period | 3 s |
| RELAY_TEST | Close the relay and read the feedback, then open it and read it again | 2 x 0.4 s |
| ADC_CHECK | Every ADC channel shows live variation (not frozen) | 1 s |
| COMM_CHECK | 8 consecutive in-sequence good frames | up to 2 s |

- A fault appearing during STABLE sends the sequence back to WAIT_CLEAR.
- A failed step counts as one attempt, opens the relay and restarts the sequence.
- All steps passing raises `VERIFIED`, which leads to **DEGRADED**, not NORMAL.
- DEGRADED is a probation: 50% load limit, with the relay closed, for 5 s with no faults.
  Only then does the system reach NORMAL.

A typical clean recovery from the injected ADC fault:
fault clears at 24.1 s, FAILSAFE to DEGRADED at 29.0 s, DEGRADED to NORMAL at 34.0 s.

## 9. Reliability and deadlock freedom
Every state has a way out, and each wait has a bound:

| State | How it is left | Bound |
|---|---|---|
| NORMAL | A fault event | Immediate |
| DEGRADED | Fault worsens, or 5 s probation completes | 5 s once clean |
| FAILSAFE | Verification succeeds, 3 failed attempts, or timeout | At most 45 s |
| SHUTDOWN | Operator RESET button, always accepted | By design |

- **No indefinite waiting inside the system.** FAILSAFE cannot last longer than 45 s, and
  repeated relapses are counted (a relapse from DEGRADED costs one attempt), so a flapping
  fault cannot loop forever; it ends in SHUTDOWN.
- **No blocking calls.** There is no `delay()`. All waits are timestamps checked each pass.
- **One path for every change.** All transitions go through `enterState()`, so none can
  happen without being logged and counted.
- **SHUTDOWN is terminal on purpose.** It is a safe resting state that needs a person to
  act. RESET leads to FAILSAFE, so even a manual reset must pass verification.
- **DEGRADED can persist** while a minor fault remains. That is a valid operating mode,
  not a stuck state.

## 10. Test results
Tests ran on a PC with stubbed Arduino functions, a simulated relay with a 20 ms feedback
delay, and a UART loopback. This was not run on Wokwi or real hardware.

| Test | Result |
|---|---|
| Table valid; 4 states reachable; every state has an exit; no direct path to NORMAL from FAILSAFE or SHUTDOWN | Pass |
| 20 s healthy run with noisy ADC | No faults, comm frames flowing |
| ADC freeze | FAILSAFE, fault 13 ADC2_FRZ, source ADC |
| Fault cleared | Stays in FAILSAFE, runs all verification steps in order, then DEGRADED, then NORMAL after exactly 5.000 s |
| Relay stuck open | FAILSAFE (RLY_OPEN); relay test fails 3 times; SHUTDOWN; stays there until RESET |
| RESET | FAILSAFE (not NORMAL), verified, back to NORMAL |
| Welded relay | Not visible while closed; found when a cell fault forces an open; SHUTDOWN |
| Communication loss | FAILSAFE (COMM_TMO), recovered after verification |
| Communication noise | DEGRADED only (COMM_CRC), no timeout fault |
| Weak cell, then over-voltage cell | DEGRADED, then FAILSAFE, with both faults remembered |
| Fault registry | Counts and first/last times correct |

**60-minute random run** (random injections, cell voltages and button presses):

| Property | Result |
|---|---|
| Transitions logged | 411, equal to the firmware's own counter |
| Illegal transitions | 0 |
| Sequence gaps (missed transitions) | 0 |
| Direct jumps to NORMAL from FAILSAFE or SHUTDOWN | 0 |
| SHUTDOWN exits other than operator reset | 0 |
| Longest FAILSAFE | 45.0 s (the timeout) |
| Loop stalls | 0 |

**A problem found while testing.** The first version let DEGRADED return to NORMAL only
0.15 s after verification, because the 5 s probation was counting the time already spent
verifying. It now counts from entry into DEGRADED. The result above reflects the fix.

## 11. Limitations and future work
- Simulated ADC noise is added in software; a real ADC provides its own.
- Relay feedback and UART loopback depend on Wokwi's behaviour, which I have not tested.
  Relay polarity can be changed with `RELAY_ACTIVE_LOW`.
- The relay test closes the relay for 0.4 s. On real equipment, run it with the load
  disconnected or use a pre-charge circuit.
- Thresholds, timings and limits are demo values and need tuning for real hardware.
- Future work: store the registry and last state in non-volatile memory so it survives a
  reboot, add a hardware watchdog, and send the JSON log over a network.

## 12. Conclusion
The project meets the brief. Four states, one transition table and one transition function
make behaviour deterministic. Every fault has an ID, a source and a remembered history.
Frozen ADCs and relay mismatches are detected. Each transition is logged as a numbered
JSON line. Recovery from FAILSAFE passes through verification and a probation before
NORMAL, and every state has a bounded or operator-driven exit, so the system cannot
deadlock. A 60-minute random test produced no illegal or missed transitions.

# Project Background — 3-Stage DC-DC Power Supply (EK-TM4C1294XL)

> **Purpose of this document.** This repo collects raw, heterogeneous material for a
> three-stage DC-DC switching-power-supply project: course requirement PDFs, grading rules,
> TivaWare driverlib, chip datasheets, a *past-years* reference implementation on different
> hardware, and one working demo (`EXP0`). This file ties that material together into a single
> reference so that any later development session (human or AI) starts from the same facts.
>
> **Status:** compiled from the repository as of 2026-10-14. Everything below is sourced from
> files in this repo; uncertain items are called out explicitly in
> [§10 Known gaps](#10-known-gaps--open-questions). Verify anything marked *unverified* before
> relying on it.

---

## 1. Project overview

**Course:** Shanghai Jiao Tong University — *Engineering Practice and Technological Innovation [3A]*
(工程实践与科技创新[3A]). The project is a **switching DC-DC power supply** built around the
**TI EK-TM4C1294XL LaunchPad**, where analog buck power modules do the power conversion and the
MCU supervises and steers them.

The project is split into three stages, each evaluated separately:

| Stage | Deliverable | Status |
|---|---|---|
| **1 — Constant Voltage (稳压)** | TPS40200 buck module, 5 V / 1 A, + MCU that measures and displays output voltage and current | Mandatory |
| **2 — Constant Current (稳流)** | TPS40200 CC module, 0.1–1.0 A adjustable, **custom PCB**, + MCU that sets the current setpoint via an I²C DAC and monitors V/I | Mandatory |
| **3 — Parallel with Current Sharing (并联均流)** | Stage-1 and Stage-2 modules in parallel, output behaves as a 5 V / 2 A CV source, MCU performs current sharing | **Optional (选做)** |

**Overall group-grade weightings** (from the grading-rules PDF, Table 1):

| Component | Weight |
|---|---|
| Daily work & participation (平时作业及参与度) | 20% (≤2 late submissions allowed; **−3 pts** each beyond that) |
| Project works — Stage 1 | 25–30 pts (deducted if abandoned) |
| Project works — Stage 2 | 25–30 pts (deducted if abandoned) |
| Project works — Stage 3 | 8 pts (optional; no deduction if abandoned) |
| Major lab report #1 | 5 pts |
| Major lab report #2 (final) | 12 pts |

Individual score = [group-internal contribution% / group size] × group score + weighting × group score ± teacher adjustment.

**Report scoring breakdown** (grading PDF, Table 5): system framework ≈10, hardware design ≈50,
software design ≈30, other ≈10. Deductions apply for a missing specification list, missing block
diagram, missing design calculations, missing schematic, missing software flowchart, etc.

**Test logistics:** three evaluation sessions, one per stage (Stage 1 in weeks 9–10, Stage 2 in
weeks 12–13, Stage 3 + creative function in weeks 14–15). ≤15 min preparation; **+10 min over →
−5 pts**; evaluation must finish within 15 min, **−5 pts per 10 min of team-caused delay**.

---

## 2. System architecture

The three stages map to the three block diagrams (Fig. 1/2/3) in the task PDF.

### Stage 1 — DC-DC constant-voltage module + MCU monitoring (Fig. 1)
A **TPS40200-based switching buck** converts **10–20 V in → 5 V out, 1 A rated**, with output
current-limiting. A current-sense and voltage-sense conditioning network feeds two ADC channels of
the MCU (`ADC1` = current, `ADC2` = voltage). The MCU **only measures and displays** — it takes no
part in the voltage regulation.

### Stage 2 — DC-DC constant-current module + MCU setpoint control (Fig. 2)
A **TPS40200-based switching CC module**, **10–20 V in → 0.1–1.0 A out**, with both current-limit
and voltage-limit protection. This module must be realised on a **custom-fabricated PCB**. The MCU:
- reads output current and voltage over `ADC1`/`ADC2`;
- drives the **current setpoint** through an external **DAC6571 (10-bit, I²C)**;
- reads a keypad to change the setpoint and displays setpoint / output current / voltage.

### Stage 3 — Parallel output with current sharing (Fig. 3)
The Stage-1 (CV) and Stage-2 (CC) modules are **connected in parallel** and fed from a shared
10–20 V input. The MCU runs a **current-sharing (均流)** loop: it reads both branch currents
(`ADC1`, `ADC2`) and the shared output voltage (`ADC3`) and adjusts the CC module's current
setpoint through the DAC so the two branches carry comparable current. The combined output must
behave as a **CV source: 5 V DC, 2 A rated**.

> A revision note in the task PDF states that an isolation diode originally drawn in Fig. 3 was
> **deleted** to avoid misunderstanding — i.e. no isolating diode between the two module outputs.

### ⚠ Critical architectural note — the MCU does **not** generate the buck PWM
The switching loop lives **inside the TPS40200** (analog voltage-mode controller with feed-forward).
The MCU's only control action is the **DAC setpoint** plus the slow current-sharing adjustment.
Do **not** add MCU/`M0PWM` gate-drive code for the main rail — it is not part of this design.

**Signal chain summary:**

```
10–20 V ──► TPS40200 buck module ──► 5 V output ──► load
                    ▲                      │
                    │ (FB/COMP, analog)    │
              ┌─────┴──────── MCU (EK-TM4C1294XL) ────────┐
              │   ADC ◄── 0.1 Ω shunt + diff-amp (current) │
              │   ADC ◄── resistor divider (voltage)       │
              │   I²C  ──► DAC6571 ──► current setpoint     │
              │   GPIO ──► TM1638 (7-seg + LEDs + keypad)   │
              └─────────────────────────────────────────────┘
```

---

## 3. Hardware platform

### 3.1 MCU — TI EK-TM4C1294XL LaunchPad
- Part: **TM4C1294NCPDT** — ARM **Cortex-M4F**, **120 MHz**, **1 MB flash / 256 KB SRAM**,
  hardware FPU.
- Build define: `PART_TM4C1294NCPDT`.
- Project device in Keil: `TM4C129CNCPDT` (TivaWare DFP naming); headers from `inc/tm4c1294ncpdt.h`.

### 3.2 Buck controller — TI TPS40200
Source: `tps40200.pdf`. *Wide-Input-Range Non-Synchronous Voltage-Mode Controller.*

| Parameter | Value |
|---|---|
| Input voltage range | **4.5 V – 52 V** |
| Output voltage range | 700 mV – 90% V_IN |
| Gate driver | internal **200-mA P-channel FET** driver |
| Control | voltage-mode with input **feed-forward** compensation |
| Reference | **700 mV**, ±1% (datasheet description: trimmed to 2%) |
| Switching frequency | programmable **35 kHz – 500 kHz** (typ. **100 kHz** at `R_RC` = 200 kΩ, `C_RC` = 470 pF) |
| Overcurrent threshold (ISNS) | **100 mV** typ (65–140 mV) |
| Overcurrent reset threshold | 150 mV |
| Overcurrent behaviour | **hiccup** fault recovery |
| Soft-start | internal pull-up 105 kΩ / pull-down 305 kΩ; reset threshold 150 mV |
| UVLO | turn-on 4.25 V typ, turn-off 4.05 V typ, hysteresis ~200 mV |
| Package | 8-pin SOIC (D) or VSON (DRB) |

Pin map: `1 = RC`, `2 = SS`, `3 = COMP`, `4 = FB`, `5 = GND`, `6 = GDRV`, `7 = ISNS`, `8 = VDD`.

> **Note:** the *past-years* design used a TPS40200 clocked near **~300 kHz** with `R = 68 kΩ`,
> `C = 470 pF`. The datasheet's typ. operating point is `R = 200 kΩ` / `C = 470 pF` → 100 kHz.
> Confirm the actual values on the current board's schematic.

### 3.3 Current-setpoint DAC — TI DAC6571
- **10-bit**, **I²C** interface, address `0x98` (7-bit `0x4C`) in the past design.
- Sets the CC module's current reference. **The new design should use the TM4C hardware I²C
  module** (`driverlib/i2c.c`) rather than the past design's bit-banged GPIO I²C.

### 3.4 Current sensing
- **0.1 Ω shunt resistor** in series with the output.
- **All currents in this project are specified and measured as the voltage across that shunt**
  (`I [A] = V_shunt [V] / 0.1 Ω`) — e.g. **1 A ≡ 100 mV**, 0.5 A ≡ 50 mV.
- The shunt voltage goes to a **differential amplifier** before the ADC.
- The grading PDF states explicitly: measurements assume the 0.1 Ω sense resistor is exact.

### 3.5 Display / keypad / UI — TM1638 (U2)
- **TM1638** drives the whole user interface: an **8-digit 7-segment display** (two 4-digit
  common-anode modules L1, L2), **8 indicator LEDs**, and a **3×3 keypad** (SW1–SW9, key lines
  K1–K3) — all over a 3-wire bus (STB / CLK / DIO).
- **MCU wiring (verified on the A2000TM4 schematic):** **STB = PK4 (pin 42)**,
  **DIO = PK5 (pin 43)**, **CLK = PM0 (pin 44)**; R1/R2/R3 = 10 kΩ pull-ups to +3.3 V.
  This matches `EXP0`'s wiring exactly.
- `TM1638_Readkeyboard()` returns the scanned keys — use it for the Stage-2 setpoint buttons.
- Two buzzers are present: **B1** (active) and **B2** (passive).

### 3.6 MCU carrier board — "A2000TM4"
The project's own carrier board, `A2000TM4电原理图_v3.pdf` (dated 2025-09-29). It is an
**MCU + UI + power** board and does **not** contain the buck converters.

| Ref | Part | Role |
|---|---|---|
| U1 | **TM4C1294** | MCU |
| U2 | **TM1638** | display / LED / keypad driver |
| U3 | **TPS60400** | charge-pump inverter → **−5 V** rail |
| P2 | **AMS1117-5V** | 5 V LDO |
| P1 / J1 | USB / Mini-USB 2.0 Type B | USB — uses the MCU's **internal USB**, there is no external USB-UART chip |
| L1, L2 | 4LED_ANODE | 8-digit 7-segment display |
| LED1–LED8 | LEDs | indicator LEDs |
| SW1–SW9, S1 | push buttons / Switch3x1 | 3×3 keypad + a 3-position switch |
| B1, B2 | buzzers | active + passive |
| P_GND, P_GND1 | banana jacks | power / ground terminals |
| A, B, Header 5 | headers | power distribution (5 V / +5 V / −5 V / GND) |
| D1, D10–12, F1 | Schottky / diodes / fuse | input protection, keypad-matrix diodes, fuse |

> **Not on this board:** the TPS40200 CV/CC modules, the DAC6571, and the current-sense
> shunt / amplifier. Those belong to the **external power modules** the student builds and wires
> to this board.

### 3.7 MCU net / pin map (as drawn)
| MCU pin | Port | Net |
|---|---|---|
| 42 | PK4 | TM1638 **STB** |
| 43 | PK5 | TM1638 **DIO** |
| 44 | PM0 | TM1638 **CLK** |
| 63 | PF3 | net `PF3` (broken out) |
| 46 | PM2 | net `PM2` (broken out) |

These are the **only** MCU pins that carry an explicit net label on the schematic. The ADC-input
and I²C-to-DAC lines are **not labelled** because they route to the external modules rather than to
on-board parts — still an open item, see [§10](#10-known-gaps--open-questions).

---

## 4. Requirements & acceptance numbers

> Source: `problem/工程实践与科技创新[3A]课程任务要求.pdf` (§2) and
> `…测试方法和评分规则.pdf` (Tables 2–4). These numbers **are** the acceptance criteria — target
> them directly.

### 4.1 Stage-1 constant-voltage module (task §2.1)
| Item | Requirement |
|---|---|
| Input voltage | DC **10–20 V** |
| Rated output voltage | DC **5 V** |
| Absolute voltage accuracy | **5 V ± 3%** (4.85–5.15 V) |
| Line regulation | ΔV from input change ≤ **0.5%** |
| Load regulation | ΔV from load change ≤ **0.5%** |
| Output ripple | ≤ **100 mV p-p** |
| Rated output current | **1 A** |
| Overcurrent-protection trip | **1.1 A – 1.9 A** |
| Conversion efficiency (full load) | ≥ **80%** |
| No-load | output voltage must still be measurable |

### 4.2 MCU monitoring accuracy (task §2.2)
| Item | Pass | Full credit |
|---|---|---|
| Voltage measurement & display error | ≤ **0.05 V** | ≤ 0.02 V |
| Current measurement & display error | ≤ **0.05 A** | ≤ 0.01 A |

### 4.3 Stage-2 constant-current module (task §2.3)
| Item | Requirement |
|---|---|
| Input voltage | DC **10–20 V** |
| Output current | DC **0.1 – 1.0 A adjustable**, step **0.1 A**, power-on default **0.8 A** |
| Output voltage limit (限压值) | **5.25 V ± 0.1 V** (5.15–5.35 V) |
| Conversion efficiency (full load) | ≥ **80%** |
| Protection | output overcurrent / short-circuit protection |
| No-load | must auto-enter voltage-limit protection |

> Revision history: the voltage-limit value was changed **5.2 V ±0.2 V → 5.25 V ±0.1 V** (2025-12-11).

### 4.4 Stage-3 load balancing (task §2.4)
When the load exceeds **120% of rated load**, the **load-balancing error must be ≤ 15%**:

```
load-balance error = |I_CV − I_CC| / (0.5 × (I_CV + I_CC)) × 100%
```

where `I_CV` = constant-voltage-module output current, `I_CC` = constant-current-module output
current, and total output current = `I_CV + I_CC`.

### 4.5 Grading tables

**Stage 1 — `Table 2` (100 pts)**

| Item | Test condition | Scoring |
|---|---|---|
| Output voltage accuracy (10) | V_IN 10.0 V; R_load = 5.1 Ω | 4.85–5.15 V → **5**, else 0 |
| | R_load = ∞ | 4.85–5.15 V → **5**, else 0 |
| Overcurrent protection & recovery (10) | R_load = 5.1 Ω ∥ 5.1 Ω | output effectively off → **5**; recovery at 5.1 Ω → **5** |
| Load regulation (5) | R_load = 5.1 Ω and 10 Ω | ΔV < 0.01 V → **5**; 0.01–0.02 V → **2**; else 0 |
| Conversion efficiency (40) | V_IN 10.0 V; R_load = 5.1 Ω | η > 92% → **40**; 80–92% → formula; < 80% → 0 |
| Line regulation (5) | R_load = 5.1 Ω; V_IN 10→20 V | ΔV < 0.01 V → **5**; 0.01–0.025 V → **2**; else 0 |
| Output ripple (20) | V_IN 20.0 V; R_load = 5.1 Ω; 20 MHz BW limit | U_pp ≤ 30 mV → **20**; 30–100 mV → formula; > 100 mV → 0 |
| Illegal potentiometer | visual | **−6 pts each** |
| MCU V/I monitoring (10) | V_IN 10.0 V; not no-load | V: ≤0.02 V → **5**, ≤0.05 V → **2**; I: ≤0.01 A → **5**, ≤0.05 A → **2** |

Efficiency score = `40 − (92% − η) / 12% × 20` (1 decimal).
Ripple score = `8 + (100 mV − U_pp) / 70 mV × 12` (1 decimal).

**Stage 2 — `Table 3` (100 pts)**

| Item | Test condition | Scoring |
|---|---|---|
| Output current accuracy (50) | setpoint 0.1 A, R_load 5.1/10 Ω | 10 ±0.5 mV → **10**; ±1.0 mV → **5** |
| | 0.2 A (no power-down) | 20 ±0.5 mV → **10**; ±1.0 mV → **5** |
| | 0.3 A | 30 ±0.5 mV → **10**; ±1.0 mV → **5** |
| | 0.5 A (R_load 3/5.1 Ω) | 49–51 mV → **10**; 47–53 mV → **5** |
| | 0.8 A (R_load 3/5.1 Ω) | 78–82 mV → **10**; 74–86 mV → **5** |
| Voltage-limit protection & recovery (15) | setpoint 1.0 A; R_load 10 Ω | 5.15–5.35 V → **5**; no-load → **5**; back to 3 Ω auto-recovers → **5** |
| Conversion efficiency (25) | V_IN 10.0 V; R_load 5.1 Ω; setpoint 1.0 A | η > 91% → **25**; 80–91% → formula; < 80% → 0 |
| Custom PCB | visual | non-custom PCB → **−20**; custom PCB without group mark → **−15**; each illegal potentiometer → **−6** |
| MCU V/I monitoring (10) | V_IN 10.0 V; output ≥ 0.5 A | V: ≤0.02 V → **5**; I: ≤0.01 A → **5** (else scaled) |

Efficiency score = `25 − (91% − η) / 11% × 15` (1 decimal).
Current is observed **only through the MCU's UI** — change the setpoint *before* the load; no
power-down / reboot during the test.

**Stage 3 — `Table 4` (100 pts)**

| Item | Test condition | Scoring |
|---|---|---|
| Load balance (60) | V_IN 10.0 V; three loads (10 Ω, 5.1 Ω, a third ≤ rated current) | parallel V must stay 4.85–5.15 V else 0. Then: error ≤5% **and** each path ripple ≤2 mV → **20**; ≤10% / ≤8 mV → **10**; ≤15% / ≤24 mV → **5**; else 0 |
| MCU V/I monitoring (15) | total output > 0.8 A | V ≤0.02 V → **5**; path-1 I ≤0.01 A → **5**; path-2 I ≤0.01 A → **5** |
| Scope-measured balancing response time (10) | step R_load 5.1 Ω → 5.1 Ω ∥ 5.1 Ω | correct scope operation → **10** (tests instrument skill) |
| Short-circuit protection & recovery (15) | short the output, then remove | limits output, no bench-supply alarm → **5**; auto-recovers → **10** |

---

## 5. Software environment & toolchain

| Item | Value |
|---|---|
| IDE | **Keil MDK-ARM (µVision)** — project `EXP0/EXP0.uvprojx` |
| Vendor library | **TivaWare for C Series 2.2.0.295** |
| Driverlib source | bundled at `EXP0/driverlib/` (`adc.c`, `gpio.c`, `i2c.c`, `pwm.c`, `timer.c`, `uart.c`, `systick.c`, `interrupt.c`, `sysctl.c`, …) |
| Prebuilt driverlib | linked as `EXP0/driverlib/rvmdk/driverlib.lib` |
| Register headers | `EXP0/inc/` (`hw_*.h`, `tm4c1294ncpdt.h`) |
| Include paths | `./inc;./driverlib` |
| Startup / CMSIS | RTE component `RTE/Device/TM4C129CNCPDT/startup_TM4C129.s` + `system_TM4C129.c`; CMSIS:CORE 5.6.0 |
| Memory | IRAM `0x20000000`, 256 KB; IROM `0x0`, 1 MB |

**Repository hygiene caveat.** A full TivaWare tree is checked in under `ti1294/`, and
`git status` shows many modified Keil project/user files there and in `EXP0` (`.uvguix.*`,
`.uvoptx`, `Objects/*.o`, `build_log.htm`). This churn is **IDE/XML bookkeeping and build output**,
not project work — don't treat it as meaningful changes.

---

## 6. `EXP0` base-code analysis

`EXP0` is the SJTU-provided *adc_demo* and serves as the starting scaffold. Sources:
`EXP0/exp0.c`, `EXP0/tm1638.c`, `EXP0/tm1638.h`. **It samples two ADC channels at 25 Hz and shows
both raw 12-bit values (0–4095) on the TM1638.**

### 6.1 Clock & timing
- `DevicesInit()` → `SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
  SYSCTL_CFG_VCO_480, 20000000)` → **SystemClock = 20 MHz** from the 25 MHz crystal via PLL
  (VCO 480 MHz).
- `SysTickPeriodSet(ui32SysClock / 50)` → **20 ms** tick; `SysTick_Handler` runs the display refresh.
- Soft timers on the 20 ms tick: `V_T40ms = 2` → `ADC_Sample()` every **40 ms** (25 Hz);
  `V_T100ms = 5` → a 100 ms flag.

### 6.2 ADC (the pattern to extend)
```c
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);      // PE3=AIN0(CH0), PE2=AIN1(CH1)
ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);  // sequencer 1, software trigger
ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 | ADC_CTL_END | ADC_CTL_IE);
ADCSequenceEnable(ADC0_BASE, 1);
ADCIntClear(ADC0_BASE, 1);
```
Read path in `ADC_Sample()`: `ADCProcessorTrigger()` → poll `while(!ADCIntStatus(...))` →
`ADCIntClear()` → `ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value)`; `pui32ADC0Value[0]` = CH0,
`[1]` = CH1. ADC0 is a **12-bit** SAR.

> **Extension needed for later stages:** Stage 3 needs **three** channels (V_out + two branch
> currents) — add a third step (`ADC_CTL_CHx`) before the `ADC_CTL_END` step and size the result
> array ≥ the number of steps. Note that the A2000TM4 schematic does **not** label the
> voltage-sense / current-sense channels (they route to the external modules), so the Stage-2/3
> channel-to-pin mapping is **not** in `EXP0` and must be confirmed on the bench — see [§10](#10-known-gaps--open-questions).

### 6.3 Display / keypad
- `TM1638_Init()` writes the display-config command (`0x8A`); `main()` waits ~60 ms for power-up.
- `SysTick_Handler` calls `TM1638_RefreshDIGIandLED(digit, pnt, led)` (16 bytes serialised
  bit-banged over STB/DIO/CLK).
- Digit buffer index vs board position: **digits are ordered 4,5,6,7,0,1,2,3** left→right;
  **LEDs 7,6,5,4,3,2,1,0** left→right.
- Display split in the demo: left 4 digits = CH0 (PE3), right 4 digits = CH1 (PE2).

### 6.4 Files in `EXP0` and what they provide
| File | Role |
|---|---|
| `exp0.c` | `main()` + `GPIOInit`, `ADCInit`, `ADC_Sample`, `SysTickInit`, `DevicesInit`, `SysTick_Handler` |
| `tm1638.c/.h` | TM1638 driver: `TM1638_Init`, `TM1638_RefreshDIGIandLED`, `TM1638_Readkeyboard`, `TM1638_DigiSegment`, low-level serial in/out |

*(Note: `tm1638.h` is GBK-encoded, so its Chinese comments appear as mojibake in a UTF-8 editor —
cosmetic only; the code is fine.)*

---

## 7. Resource maps

### 7.1 ADC0 external channels (from `spmu365c.pdf` §2.1.6)
| AIN | Pin | LaunchPad location |
|---|---|---|
| AIN0 | PE3 | BP1 B1-6 |
| AIN1 | PE2 | BP1 B1-5 |
| AIN2 | PE1 | BP1 B1-4 |
| AIN3 | PE0 | BP1 B1-3 |
| AIN4 | PD7 | BP1 B1-7 |
| AIN5 | PD6 | X11 pad 44 |
| AIN6 | PD5 | BP2 A2-6 / X11 pad 38 |
| AIN7 | PD4 | BP2 A2-5 / X11 pad 40 |
| AIN8 | PE5 | BP1 A1-6 |
| AIN9 | PE4 | BP1 A1-2 |
| AIN10 | PB4 | BP2 B2-3 / X11 pad 5 |
| AIN11 | PB5 | BP2 B2-4 / X11 pad 7 |
| AIN12 | PD3 | BP1 A1-7 / X11 pad 46 |
| AIN13 | PD2 | BP2 A2-2 / X11 pad 52 |
| AIN14 | PD1 | BP1 D1-6 / X11 pad 48 |
| AIN15 | PD0 | BP1 D1-7 / X11 pad 50 |
| AIN16 | PK0 | BP2 B2-5 / X11 pad 26 |
| AIN17 | PK1 | BP2 B2-6 / X11 pad 28 |
| AIN18 | PK2 | BP2 B2-7 / X11 pad 30 |
| AIN19 | PK3 | BP2 B2-8 / X11 pad 32 |

⚠ Port-D ordering is **inverted**: `PD7 = AIN4` … `PD0 = AIN15`, and `PD3 = AIN12`.

### 7.2 PWM outputs (available, but **unused** by this design)
`PF0`=M0PWM0, `PF1`=M0PWM1, `PF2`=M0PWM2, `PF3`=M0PWM3, `PG0`=M0PWM4, `PG1`=M0PWM5,
`PK4`=M0PWM6, `PK5`=M0PWM7. Note **PK4/PK5 are used by the TM1638 bus** in `EXP0`, so they are not
free if the same display wiring is kept.

### 7.3 Driverlib modules available in `EXP0/driverlib`
`adc`, `gpio`, `i2c`, `pwm`, `timer`, `uart`, `ssi`, `systick`, `interrupt`, `sysctl`, `epi`,
`emac`, `usb`, `can`, `qei`, `onewire`, `hibernate`, `watchdog`, `eeprom`, `flash`, `aes`, `des`,
`shamd5`, `crc`, `comp`, `lcd`, `mpu`, `udma`, …

---

## 8. Reference implementation — `Pastcode/`

`Pastcode/` holds a complete past-years implementation (2019, group 33). **It runs on
MSP430G2553 (MSP-EXP430G2), not the TM4C1294, and its hardware differs — use it for strategy only.**

| File | Content |
|---|---|
| `3aCodeOfAllStages.c` | All three stages in one file, switched by a global `stage` variable |
| `工程实践与科技创新[3A]-第33组(于前程)设计报告(1).docx` | Full design report (hardware + software) — richest structural reference |
| `定标.xlsx` | Calibration table (measured vs. displayed) used to fit the `k, b` coefficients |
| `阶段1/2/3.png` | Stage block diagrams |

### 8.1 Reusable *conceptually*
- **Current sharing (master-slave, midpoint-iterative)**: the CV module is the master; the MCU
  nudges the CC module's setpoint toward half the summed currents, updating every 100 ms:
  ```c
  int target = (I_CC + I_CV) / 2 + 2;                 // +2 mA bias
  if (abs(setpoint - target) > 4)                     // 4 mA deadband
      setpoint = (target + setpoint) / 2;             // move halfway
  ```
  The deadband + half-step prevents limit-cycling ("跷跷板").
- **Calibration by linear fit**: `measured = k × raw + b`, coefficients fitted from
  measured-vs-displayed data.
- **ADC averaging**: sample many times and average before display/control.
- **Keypad setpoint scheme**: −10 / −100 / +100 / +10 mA.

### 8.2 Does **not** transfer — do not copy
- The **calibration coefficients** (board-specific; the new board needs its own `定标`).
- All **MSP430 register code**, the **bit-banged I²C**, and the 20 ms Timer_A scheme.
- The past design's external-DAC **scaling** (`4096 / (max+1)` over codes 0…4999) — that span is
  internally inconsistent with a 10-bit DAC part; treat it as a bug, not a template.
- The **TPS40200 analog loop and the DAC-setpoint idea are architectural** and *do* carry over;
  the *values* (switching frequency, gains) do not.

---

## 9. Development roadmap (suggested)

1. **Port the monitoring stack first.** Extend `EXP0` to read the correct channels for V and I,
   apply the `k·x+b` calibration, and display real units. This unblocks the monitoring score on
   every stage.
2. **Stage 2 control.** Add the hardware-I²C driver for the DAC6571, wire the TM1638
   `TM1638_Readkeyboard()` keys to the setpoint, clamp to 0.1–1.0 A with default 0.8 A.
3. **Stage 3 sharing.** Add the third ADC channel, implement the midpoint-iterative sharing loop
   with a deadband, and verify balance ≤5% / ripple ≤2 mV for full marks.
4. **Protection display.** Surface overcurrent / voltage-limit / short-circuit states on the LEDs.

---

## 10. Known gaps & open questions

These items are **not yet verified** — resolve them before depending on them:

1. **How the external CV/CC modules attach to the MCU.** The A2000TM4 schematic labels only
   `PK4`/`PK5`/`PM0` (TM1638), `PF3` and `PM2` on the MCU; the **ADC-input and I²C-to-DAC lines
   are not labelled** (they route off-board). The pin assignments for **voltage-sense,
   current-sense and the DAC I²C bus are therefore still unknown** and must be confirmed on the
   bench or from the module schematics.
2. The TPS40200 **switching frequency, soft-start, and current-limit components** actually fitted
   on the current modules.
3. The **DAC6571 I²C address and output scaling** on the current board (the past design used
   address `0x98`).

> **Resolved (was open):** the A2000TM4 schematic *is* extractable — the original
> `A2000TM4电原理图_v3.pdf` carries a **text layer** readable with `pdftotext` (only the IDE's
> built-in PDF reader fails on it). The board is an MCU/UI/power carrier, and its TM1638 wiring
> (**STB = PK4, DIO = PK5, CLK = PM0**) is **confirmed** and matches `EXP0` — see
> [§3.6](#36-mcu-carrier-board--a2000tm4). (Symbol quirks: pins 76 and 56 are both labelled
> `*RESET`, and pins 25/26 are labelled `See_JP4`/`See_JP5` — artefacts of the custom symbol.)

---

## 11. File & document index

| Path | What it is |
|---|---|
| `problem/工程实践与科技创新[3A]课程任务要求.pdf` | Task requirements — the three stages (§1), technical indicators (§2.1–2.4), revision history |
| `problem/工程实践与科技创新[3A]测试方法和评分规则.pdf` | Test methods & grading rules — Tables 1–5, exact test conditions & scores |
| `spmu365c.pdf` | TM4C1294NCPDT device datasheet — pin muxing (AIN/PWM), electrical specs |
| `tps40200.pdf` | TPS40200 buck-controller datasheet |
| `A2000TM4电原理图_v3.pdf` | MCU/UI/power carrier-board schematic (text layer extractable with `pdftotext`) — see §3.6 |
| `SW-TM4C-DRL-UG-2.2.0.295.pdf` (root & `ti1294/docs/`) | TivaWare **Driver Library user guide** — the API reference |
| `ti1294/docs/SW-EK-TM4C1294XL-UG-2.2.0.295.pdf` | EK-TM4C1294XL board/package user guide |
| `ti1294/docs/SW-TM4C-EXAMPLES-UG-2.2.0.295.pdf` | TivaWare examples guide |
| `ti1294/docs/*` (other PDFs) | Boot loader, grlib, sensorlib, usblib, utils, tools, release notes |
| `ti1294/` | Full TivaWare 2.2.0.295 distribution (driverlib, inc, examples, utils, docs) |
| `EXP0/exp0.c` | Base demo: dual-channel ADC + TM1638 display |
| `EXP0/tm1638.c` / `.h` | TM1638 driver |
| `EXP0/driverlib/`, `EXP0/inc/` | Driverlib + register headers used by the Keil project |
| `EXP0/EXP0.uvprojx` | Keil MDK project |
| `Pastcode/3aCodeOfAllStages.c` | Past-years (MSP430) reference implementation |
| `Pastcode/…设计报告(1).docx` | Past-years design report |
| `Pastcode/定标.xlsx`, `阶段*.png` | Past calibration data & stage diagrams |
| `ADC_DEMO.rar`, `工科创3a.zip`, `my.code-workspace` | Archived/demo material and VS Code workspace file |

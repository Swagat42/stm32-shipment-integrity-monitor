# 🧮 Algorithms

Everything below runs on the STM32F411's Cortex-M4F, using its hardware FPU, without any maths library.

---

## Per-sample pipeline (once per second, `IN_TRANSIT`)

```
 Sense ──▶ Filter ──▶ Compute ──▶ Motion ──▶ Check ──▶ Act ──▶ Log ──▶ Send
 SHT31     EMA T      MKT         vib RMS    profile   fan     SEQ+CHK  HC-05
 LDR×5     median     trend/ETA   fusion     limits    LED
 FIFO                 excursion   drop       latch     buzzer
 RTC, GPS                         tilt       bits      LCD
```

Between samples, a background task drains the accelerometer FIFO every 80 ms (the FIFO holds 320 ms at 100 Hz, so no samples are lost) and parses GPS bytes.

---

## 1 · Filtering
- **Temperature:** exponential moving average with weight ¼
  $T_f[k] = T_f[k-1] + \dfrac{T[k] - T_f[k-1]}{4}$
- **Light:** median of five back-to-back ADC conversions, so a single spike cannot look like light in the box.

## 2 · Mean Kinetic Temperature (MKT)
The single steady temperature that would cause the same total degradation as the recorded history:

$$T_{MKT} = \frac{\Delta H / R}{-\ln\left(\frac{1}{n}\sum_{i=1}^{n} e^{-\Delta H/(R\,T_i)}\right)}, \qquad \Delta H/R = 10\,000\ \text{K}$$

- Temperatures are in kelvin. The sum is taken relative to 25 °C so it stays near 1.
- **Kahan summation** limits rounding drift over long trips.
- Example: 23 h at 20 °C + 1 h at 40 °C gives a mean of 20.83 °C but an **MKT of 22.45 °C**.
- MKT is reported **alongside** the limit checks, never instead of them (USP ⟨1079.2⟩).

## 3 · Trend and time to limit
A ring buffer of 60 points (one every second sample, about 2 min) holds real timestamps, so uneven spacing doesn't matter:

$$m = \frac{\sum (t_i - \bar t)(T_i - \bar T)}{\sum (t_i - \bar t)^2}, \qquad \text{ETA} = \frac{T_{lim} - T}{m}$$

- At least 15 points are required, and slopes below 0.10 °C/min are ignored.
- If the box is warming and ETA < 120 s, the **fan starts early** and the LED turns amber.

## 4 · Excursion budget
The first crossing logs only `TEMP_WARNING`. A latched `TEMP_EXCURSION` occurs when any one budget runs out:

| Budget | Value | Logged as |
|---|---|---|
| Time outside the window | 30 s | `WHY=TIME` |
| Area outside the window | 0.50 °C·min | `WHY=AREA` |
| Distance beyond the limit | 3.0 °C | `WHY=HARD` |

<sub>These are demonstration values. Real budgets must come from product stability data.</sub>

## 5 · Accelerometer (ADXL345, 100 Hz, ±16 g, FIFO stream)
- **Calibration at sealing:** the calmest 16 samples set 1 g (their magnitude) and the reference direction.
- **Vibration:** RMS of $(\lvert a\rvert - 1g)$ over each sample interval, logged as `VIB` in mg.
- **Drop:** magnitude < 0.4 g for 4 to 60 samples, then a 10-sample impact window.
  $h = \tfrac12 g t^2$; for example, 20 samples (0.2 s) gives **196 mm**. A drop of 10 cm or more is logged as `DROP` and latches a shock breach.
- **Tilt:** $\theta = \operatorname{atan2}(\lVert \mathbf a \times \mathbf r \rVert,\ \mathbf a \cdot \mathbf r)$ using quasi-static samples only (0.8 to 1.2 g). Two consecutive checks must agree. Above 135° the event is `INVERTED`.

## 6 · Shock fusion

| Observed | Result |
|---|---|
| ADXL345 over the limit **and** an SW-420 edge within 300 ms | `SHOCK_CONFIRMED` → breach |
| ADXL345 over the limit only | `SHOCK_ACCEL` → breach |
| SW-420 only | `VIBRATION_SUSPECT` (at most once per 5 s), no breach |
| SW-420 only, ADXL345 failed | `SHOCK_SWITCH` → breach |

## 7 · Integrity score
The score starts at 100 and loses: temperature 10 per °C outside the window (max 40), humidity 10, shock 25, tilt 15, light 30. It is clamped to 0 to 100, and the trip's **worst** score goes into the summary.

## 8 · Predictive fan control
- **ON** when $T > T_{max} - 1\,°C$ **or** the predicted ETA is under 120 s
- **OFF** when $T < T_{max} - 2.5\,°C$ (hysteresis)

## 9 · Hash chain
CRC-32 (polynomial `0x04C11DB7`) on the hardware CRC unit is seeded with `LOG_KEY` at sealing and **never reset between lines**, so each `CHK` covers the full history. A boot self-test compares the hardware result with a software model and falls back to software if they differ.

## 10 · Hand-written maths

| Routine | Method | Max error vs reference |
|---|---|---|
| `sqrt` | Newton iteration from a bit-trick start | 1.2 × 10⁻⁷ |
| `exp` | Range reduction + degree-6 polynomial | 4 × 10⁻⁶ |
| `ln` | atanh series | 2 × 10⁻⁶ |
| `atan` (degrees) | Minimax polynomial | 0.0007° |

---

## Verification on a PC
The pure-maths part of the firmware was compiled and tested on a PC before flashing:

| Test | Result |
|---|---|
| Compilation | Zero warnings at several optimisation levels, hard and soft float |
| MKT, constant 25 °C | 25.000 °C |
| MKT, 23 h @ 20 °C + 1 h @ 40 °C | 22.45 °C (reference 22.448) |
| Trend on a 0.500 ramp | 0.503 |
| 20-sample free fall | 196 mm, 4.95 g peak |
| Tilt 30° / 60° (limit 45°) | not flagged / flagged |
| Hash chain | All 159 test lines verified by the dashboard; an edit is caught |

## Processor cost
The DWT cycle counter times six operations (MKT update, trend fit, full step, accelerometer sample, tilt and log CRC) and prints them in the `BENCH` line at delivery. At 16 MHz, 1 µs = 16 cycles. *Fill in real numbers from your board; the dashboard's sample values are illustrative.*

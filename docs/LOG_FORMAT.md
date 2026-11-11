# 📜 Log Format

The HC-05 sends one comma-separated line per sample and per event, at **9600 baud, 8N1**. A full example is in [`../samples/sample_log.txt`](../samples/sample_log.txt).

---

## Line types

| Type | When | Example (shortened) |
|---|---|---|
| Boot | Power-on | `SHT31 OK` · `ADXL345 OK` · `RTC OK` · `CRC HW OK` |
| `SEALED` | Lid closed; starts the chain | `09/10/26 14:00:10,EVENT=SEALED,PROFILE=Pharma-Cold,TMIN=23.4,TMAX=29.9,…,SEQ=0,CHK=…` |
| Sample | Every second in transit | `14:00:11,T=28.4,H=72.0,G=0.99,LDR=309,…,SEQ=1,CHK=A513EDA5` |
| Event | When something happens | `14:02:08,EVENT=DROP,HEIGHT_CM=19,FALL_MS=200,PEAK=4.95,SEQ=121,CHK=…` |
| `SUMMARY` | At delivery | `…,DURATION_S=166,WORST=70,BREACH=60,…,VERDICT=FAIL,SEQ=174,CHK=…` |
| `BENCH` | At delivery | `…,CLK_MHZ=16,AVG_MKT=…,MAX_MKT=…,AVG_TREND=…,SEQ=175,CHK=…` |

---

## Sample-line fields

| Field | Meaning |
|---|---|
| `T`, `H` | Filtered temperature (°C), relative humidity (%RH) |
| `G`, `PEAK` | Current and peak acceleration magnitude (g) |
| `LDR` | Light reading (ADC counts, 0 to 4095) |
| `FIX`, `LAT`, `LON` | GPS fix flag, latitude and longitude (decimal degrees) |
| `SCORE` | Integrity score, 0 to 100 |
| `BREACH` | Latched breach bit mask (see below) |
| `ERR` | Failed sensors: `1` SHT31, `2` ADXL345 (`4` reserved for the RTC) |
| `MKT` | Mean Kinetic Temperature so far (°C) |
| `RATE`, `ETA` | Trend (°C/min) and seconds to the limit (`0` = no prediction) |
| `EXC`, `DEG` | Seconds and area (°C·min) outside the window |
| `VIB`, `TILT` | Vibration RMS (mg), tilt from the sealed position (°) |
| `SEQ`, `CHK` | Sequence number and hash-chain CRC-32 |

## Breach bits

| Bit | Value | Breach |
|:-:|:-:|---|
| 0 | 1 | Temperature |
| 1 | 2 | Humidity |
| 2 | 4 | Shock or drop |
| 3 | 8 | Light |
| 4 | 16 | Lid opened |
| 5 | 32 | Tilt |

For example, `BREACH=60` = 4 + 8 + 16 + 32, meaning shock, light, lid and tilt all latched.

## Events

| Event | Meaning | Latches |
|---|---|:-:|
| `TEMP_WARNING` | First crossing of the temperature window | – |
| `TEMP_EXCURSION` | A budget is used up (`WHY` = TIME / AREA / HARD) | 1 |
| `HUMIDITY_EXCURSION` | Humidity above the profile limit | 2 |
| `SHOCK_CONFIRMED` | ADXL345 and SW-420 agree within 300 ms | 4 |
| `SHOCK_ACCEL` | ADXL345 only | 4 |
| `SHOCK_SWITCH` | SW-420 only while the ADXL345 has failed | 4 |
| `DROP` | Free fall followed by impact, with `HEIGHT_CM`, `FALL_MS` and `PEAK` | 4 |
| `VIBRATION_SUSPECT` | SW-420 only (rate-limited) | – |
| `LIGHT_TAMPER` | LDR above the sealed baseline + `LIGHT_DELTA` | 8 |
| `UNAUTHORIZED_OPEN` | Reed switch opened in transit | 16 |
| `TILT_EXCEEDED` / `INVERTED` | Tilt above the limit / above 135° | 32 |
| `DELIVERED` | Both buttons held for 2 s | – |

---

## Verifying the chain

```
CHK[n] = CRC32( LOG_KEY ‖ line[0] ‖ line[1] ‖ … ‖ line[n] )   (CRC unit never reset)
```

- Each line's text, up to `,CHK=`, is fed into the CRC as little-endian 32-bit words, with the last word zero-padded.
- **Edited line** → its `CHK` and every later `CHK` fail.
- **Deleted line** → gap in `SEQ`.
- The dashboard restarts the chain at every `SEALED` line and reports *lines checked / missing / mismatched* and the first bad line.

> Tamper-**evident**, not tamper-**proof**: a CRC detects accidental and casual edits, but anyone who knows `LOG_KEY` and the algorithm could rebuild the chain.

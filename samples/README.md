# Sample log

`sample_log.txt` is the complete log of the dashboard's **Play sample run**, saved with **Save log**.

- Profile: Pharma-Cold (demo mode), sealed 14:00:10, delivered 14:02:56
- Events: temperature warning, confirmed shock, tilt, 19 cm drop, suspect vibration, lid opened, light tamper
- Verdict: **FAIL** · `BREACH=60` (shock + light + lid + tilt)
- Chain: 176 chained lines, all intact (`python tools/verify_log.py samples/sample_log.txt`)

> This is simulated data from the dashboard, not a recording from the board. Its `BENCH` cycle counts are illustrative. Replace it with a real log from your hardware when you have one.

# 🔌 Hardware

## Bill of materials

| # | Part | Role | Interface |
|:-:|---|---|---|
| 1 | STM32F411CEU6 "Black Pill" | Main controller (Cortex-M4F, FPU, hardware CRC) | — |
| 2 | SHT31 | Temperature and humidity (factory-calibrated, CRC-protected) | I²C `0x44` |
| 3 | ADXL345 | 3-axis accelerometer, ±16 g, 32-sample FIFO | I²C `0x53` |
| 4 | SW-420 | Vibration switch, the second opinion for fusion | EXTI0 · PB0 |
| 5 | LDR + resistor | Light inside the box | ADC1 · PA0 |
| 6 | Reed switch + magnet | Lid state | EXTI1 · PA1 |
| 7 | DS1307 / DS3231 | Real-time clock | I²C `0x68` |
| 8 | NEO-6M | GPS position | USART1 · PA9/PA10 |
| 9 | 16×2 LCD + PCF8574 | Local display | I²C `0x27` |
| 10 | Common-cathode RGB LED | Status colour | PWM · PB4/PB5/PB3 |
| 11 | Buzzer | Breach alert | GPIO · PB8 |
| 12 | Relay module + 5 V fan | Corrective cooling | GPIO · PB9 (active-low) |
| 13 | HC-05 | Bluetooth serial log | USART2 · PA2/PA3 |
| 14 | 2 push buttons | Profile, confirm/seal/deliver | GPIO · PB12/PB13 |
| 15 | USB power bank | Supply | — |

## Why these parts

| Function | Chosen | Alternatives | Reason |
|---|---|---|---|
| MCU | STM32F411 (M4F) | STM32F103 (no FPU), ESP32 (not ARM) | FPU for the maths, hardware CRC, plenty of timers and UARTs |
| Temp/RH | SHT31 | DHT22, DS18B20, LM35 | Digital, calibrated, CRC on every reading |
| Motion | ADXL345 | MPU6050, analogue accelerometer | ±16 g for impacts, FIFO for burst reads |
| Shock switch | SW-420 | Piezo film | Cheap, independent second sensor |
| Lid | Reed switch | Micro-switch, Hall sensor | No contact wear; interrupt on both edges |
| Display | I²C LCD | OLED | Two shared wires instead of six |
| Link | HC-05 | BLE, LoRa, cellular | Simplest serial link to a laptop |

## Design decisions

- **One shared I²C bus** (SHT31, ADXL345, RTC, LCD) saves pins. Every wait has a time-out and bus recovery, so one stuck device cannot freeze the others.
- **Open-drain AF pins** on PB6/PB7, as the I²C specification requires.
- **Safe power-up:** the idle levels of the buzzer and relay pins are written *before* the pins become outputs, so there is no chirp or fan pulse at boot.
- **Interrupt inputs** for the reed switch and SW-420, so a lid opening or knock is caught instantly even while the loop is busy.
- **Internal pull-ups** on the buttons, so no external resistors are needed.
- **Graceful degradation:** a failed SHT31 skips temperature and humidity checks; a failed ADXL345 skips tilt and lets the SW-420 latch shocks; the LCD is retried every 10 s after three failures.

## Reference documents
- [STM32F411xC/xE datasheet (DS10314)](https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html)
- [RM0383 reference manual](https://www.st.com/resource/en/reference_manual/dm00119316.pdf)
- [UM10204 I²C-bus specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
- Sensirion SHT3x-DIS, Analog Devices ADXL345, u-blox NEO-6 and Maxim DS1307 datasheets

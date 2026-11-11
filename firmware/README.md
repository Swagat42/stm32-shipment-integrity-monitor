# 🔧 Firmware

`main.c` is the complete firmware: one bare-metal C file, register-level, with **no HAL and no RTOS**. The only external file it needs is the CMSIS device header `stm32f411xe.h`, which provides the register names.

| | |
|---|---|
| Target | STM32F411CEU6 "Black Pill" (Cortex-M4F with FPU) |
| Clock | Internal HSI **16 MHz**, no PLL. All baud, I²C and timer values assume this clock. |
| Structure | Super-loop with a state variable and 4 interrupt handlers |
| Size | ≈ 2,200 lines |

---

## Build and flash

### STM32CubeIDE (recommended)
1. **File → New → STM32 Project**, select **STM32F411CEUx**, and choose **Empty** as the project type. This gives you the startup file and linker script, with no HAL.
2. Replace `Src/main.c` with this `main.c`.
3. Add the CMSIS headers, which an Empty project does not include. Copy them from the STM32CubeF4 package or from [STMicroelectronics/cmsis_device_f4](https://github.com/STMicroelectronics/cmsis_device_f4) plus [ARM CMSIS Core](https://github.com/ARM-software/CMSIS_5/tree/develop/CMSIS/Core/Include):
   - `Include/stm32f4xx.h`, `stm32f411xe.h` and `system_stm32f4xx.h`
   - `core_cm4.h` and its companion `cmsis_*.h` files

   Add both folders to the include path, and add `STM32F411xE` to the preprocessor symbols.
4. Build, then flash with an **ST-Link** over SWD (or USB DFU with BOOT0 held high).

> `main()` enables the FPU itself (`SCB->CPACR`) and the firmware runs on the reset-default 16 MHz HSI, so no `SystemInit` clock setup is needed. If your startup file calls `SystemInit`, add the stock `system_stm32f4xx.c` or an empty `void SystemInit(void){}`. Don't use a HAL-generated project as-is: its `stm32f4xx_it.c` also defines `SysTick_Handler`, which clashes with the one in this file.

### arm-none-eabi-gcc (Makefile projects)
Use any STM32F411 template that provides `startup_stm32f411xe.s`, a linker script and the CMSIS headers, then compile `main.c` with:
```
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DSTM32F411xE
```
No maths library is needed, because `sqrt`, `exp`, `ln` and `atan2` are implemented in the file.

---

## User settings (top of `main.c`)

| Setting | Default | Meaning |
|---|---|---|
| `DEMO_MODE` | `1` | Re-centre the Pharma-Cold and Produce limits on room conditions at sealing |
| `RTC_SET_TIME` | `0` | Set to `1` **once** to write `RTC_INIT_*`, flash, then set back to `0` and flash again |
| `RELAY_ACTIVE_HIGH` | `0` | `0` for active-low relay modules |
| `RGB_COMMON_ANODE` | `0` | `1` if the LED's common pin goes to 3.3 V |
| `LIGHT_DELTA` | `500` | ADC counts above the sealed baseline that count as light in the box |
| `SAMPLE_MS` | `1000` | Sample period |
| `ARM_DELAY_MS` | `3000` | Grace period after sealing, during which no breach can latch |
| `EXC_TIME_BUDGET_S` | `30` | Seconds allowed outside the window |
| `EXC_AREA_BUDGET` | `50` | Area budget in 0.01 °C·min (`50` = 0.50 °C·min) |
| `EXC_HARD_MARGIN` | `300` | Immediate breach at 3.00 °C beyond the limit |
| `PRED_FAN_S` | `120` | Start the fan when a breach is predicted within this many seconds |
| `FUSION_MS` | `300` | ADXL345 and SW-420 coincidence window |
| `FF_THRESH_MG` | `400` | Free-fall threshold (mg) |
| `DROP_MIN_CM` | `10` | Equivalent drop height that latches a shock breach |
| `LOG_KEY` | `0xA5C3E1B7` | Seed word of the hash chain. **Change it for your own unit.** |
| `CRC_USE_HW` | `1` | Use the hardware CRC unit (with a self-test and software fallback) |

---

## Pin map

| Peripheral | Pins | Device |
|---|---|---|
| I2C1 (100 kHz, open-drain, AF4) | PB6 SCL, PB7 SDA | SHT31 `0x44`, ADXL345 `0x53`, RTC `0x68`, LCD `0x27` |
| ADC1 ch0 (median of 5) | PA0 | LDR divider |
| EXTI1 (both edges, pull-up) | PA1 | Reed switch (magnet near = closed = LOW) |
| EXTI0 (both edges) | PB0 | SW-420 |
| USART1 9600 8N1, RX interrupt | PA9 TX, PA10 RX | NEO-6M (GPS TX → PA10) |
| USART2 9600 8N1 | PA2 TX, PA3 RX | HC-05 |
| TIM3 / TIM2 PWM, 1 kHz | PB4 R, PB5 G, PB3 B | RGB LED |
| GPIO out | PB8, PB9 | Buzzer, relay (fan) |
| GPIO in, pull-up | PB12, PB13 | Profile button, confirm/seal/deliver button |

---

## Operating the unit

| Action | How |
|---|---|
| Choose a profile | **PB12** cycles Pharma-Cold → Electronics → Produce |
| Confirm | **PB13** |
| Seal | Close the lid (reed closed for > 0.5 s). The firmware calibrates the accelerometer and light baseline, then starts the chain. |
| Deliver | Hold **both** buttons for 2 s to write `DELIVERED`, `SUMMARY` and `BENCH` |
| New shipment | **PB13** after delivery, or a power cycle |

**LED colours:** 🟢 OK · 🟠 warning or predicted breach · 🔴 latched breach · 🔵 delivered

**Boot self-test:** the LCD shows `S:Y A:Y R:Y` (SHT31, ADXL345, RTC), and the serial output reports `CRC HW OK` when the hardware CRC unit matches the software model.

---

## Firmware map

| Section | Contents |
|---|---|
| 1 · Time base | SysTick 1 ms, `millis()`, `delay_ms()`, which keeps background tasks running |
| 2 · GPS | USART1 IRQ → 512-byte ring buffer → NMEA GGA parser |
| I²C + drivers | Time-out-guarded I²C1 with bus recovery; SHT31, ADXL345 (FIFO stream), RTC, LCD |
| Maths | `f_sqrt`, `f_exp`, `f_ln`, `atan2`; MKT (Kahan), regression ring, tilt |
| CRC + bench | Hardware CRC-32 chain, software fallback, DWT cycle timing |
| Accelerometer | 100 Hz FIFO drain every 80 ms, free-fall and impact, vibration RMS, tilt, calibration |
| I/O | ADC/LDR, relay, buzzer, RGB PWM, EXTI, buttons |
| Logging | Allocation-free string builder, `SEALED`, sample, event, `SUMMARY` and `BENCH` lines |
| Application | `Sample_Sensors`, `Motion_Step`, `Do_Seal`, `Show_Status`, the state machine in `app_main` |

Interrupt handlers only set flags or store bytes; every decision is made in the main loop.

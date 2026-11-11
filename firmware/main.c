/* ============================================================================
   ARM-BASED ADAPTIVE MULTI-SENSOR SHIPMENT INTEGRITY MONITORING SYSTEM
   Board : STM32F411CEU6 "Black Pill"  (bare-metal, register level, no HAL)
   Clock : default HSI = 16 MHz  (no PLL)  -> all baud/I2C maths assume 16 MHz

   PIN MAP (your wiring)
     I2C1  PB6 SCL, PB7 SDA    : SHT31 (0x44), ADXL345 (0x53), RTC (0x68), LCD (0x27)
     ADC1  PA0                 : LDR (voltage divider)
     EXTI1 PA1                 : reed switch  (magnet near = closed = LOW)
     EXTI0 PB0                 : SW-420 vibration sensor
     USART2 PA2 TX, PA3 RX     : HC-05 Bluetooth  (log output)
     USART1 PA9 TX, PA10 RX    : NEO-6M GPS       (GPS TX -> PA10)
     TIM3  PB4 (R), PB5 (G)    : RGB LED PWM
     TIM2  PB3 (B)             : RGB LED PWM
     GPIO  PB8 buzzer, PB9 relay (fan)
     GPIO  PB12 profile button, PB13 confirm/seal/deliver button (to GND)

   HOW IT WORKS
     MENU -> choose profile (PB12), confirm (PB13)
     WAIT_SEAL -> close the lid; sensors are calibrated, shipment starts
     IN_TRANSIT -> every second: sense, score, latch breaches, drive outputs, log
                   interrupts catch shock / lid-open instantly
     DELIVERED -> hold BOTH buttons for 2 s; summary is logged

   COMPUTATION (all on the Cortex-M4F, no libraries beyond CMSIS)
     filtering     moving average on temperature, median-of-5 on the LDR
     MKT           mean kinetic temperature (Arrhenius-weighted), pharma compliance metric
     trend + ETA   least-squares line over the last ~2 min -> warming rate and time to the limit
     excursion     time and degree-seconds outside the window, warning first, breach when the budget is used up
     accelerometer 100 Hz FIFO: drop height from free-fall time (h = 1/2 g t^2), impact peak, vibration RMS,
                   tilt angle from the sealed orientation (atan2), 1 g calibration at sealing
     fusion        ADXL345 + SW-420 must agree to confirm a shock; SW-420 alone is only "suspect"
     hash chain    hardware CRC-32 unit chains every log line (SEQ + CHK), so edited logs are detectable
     cycle counts  DWT counter times each algorithm (BENCH line at delivery)
   ============================================================================ */

#include "stm32f411xe.h"
#include <stdint.h>

/* ============================ USER SETTINGS ============================== */
#define DEMO_MODE          1   /* 1 = limits re-centred on room conditions at seal (see report note) */
#define RTC_SET_TIME       0   /* set to 1 ONCE, flash, then set back to 0 and flash again          */
#define RTC_INIT_HOUR      16
#define RTC_INIT_MIN       35
#define RTC_INIT_SEC       0
#define RTC_INIT_DATE      6
#define RTC_INIT_MONTH     10
#define RTC_INIT_YEAR      26

#define RELAY_ACTIVE_HIGH  0   /* 1 = relay ON when pin HIGH, 0 = relay ON when pin LOW (your module) */
#define RGB_COMMON_ANODE   0   /* 1 if your RGB LED common pin goes to 3.3 V                        */
#define LIGHT_DELTA        500 /* ADC counts above the sealed baseline that count as "light in box" */
#define SAMPLE_MS          1000
#define ARM_DELAY_MS       3000 /* ignore events this long after sealing (box is still being moved) */

/* ---- computation settings (explained in the report, "Computing" section) ---- */
#define EXCURSION_BUDGET   1    /* 1 = warning first, breach when the budget is used up; 0 = old rule (breach on first crossing) */
#define EXC_TIME_BUDGET_S  30   /* seconds allowed outside the window (scaled-down demo value)                */
#define EXC_AREA_BUDGET    50   /* allowed area outside the window, in 0.01 degC*min (50 = 0.50 degC*min)      */
#define EXC_HARD_MARGIN    300  /* breach at once when this far outside the window, in 0.01 degC (300 = 3 degC) */
#define REG_N              60   /* trend window: the last 60 points (one point every 2nd sample, about 2 min)  */
#define REG_EVERY          2    /* take a trend point every N-th sample                                       */
#define REG_MIN_POINTS     15   /* need this many points before a prediction is made                          */
#define RATE_MIN_100       10   /* ignore trends slower than 0.10 degC per minute                             */
#define PRED_FAN_S         120  /* start the fan early when the predicted breach is closer than this (s)      */
#define FUSION_MS          300  /* ADXL345 and SW-420 must agree within this time to confirm a shock          */
#define FUSION_SW420_ALONE_LATCHES 0 /* 1 = the SW-420 alone can latch a shock breach (old behaviour)       */
#define FF_THRESH_MG       400  /* below this magnitude (mg) the box is in free fall                          */
#define FF_MIN_SAMPLES     4    /* shortest free fall that is tracked (4 x 10 ms)                             */
#define FF_MAX_SAMPLES     60   /* longer than 0.6 s is treated as a sensor fault, not a drop                 */
#define DROP_MIN_CM        10   /* equivalent drop height that counts as a handling breach                    */
#define LOG_KEY            0xA5C3E1B7UL /* secret word mixed into the start of the log hash chain            */
#define CRC_USE_HW         1    /* 1 = STM32 hardware CRC unit (falls back to software if self-test fails)    */
#define MKT_K              10000.0f /* delta-H / R in kelvin (83.14 kJ/mol), the pharma standard value        */
#define MKT_TREF           298.15f  /* reference temperature used to keep the sums in a safe range            */

#define LCD_ADDR           (0x27 << 1)
#define SHT31_ADDR         0x44
#define ADXL_ADDR          0x53
#define RTC_ADDR           0x68

/* ========================================================================
   1. TIME BASE (SysTick interrupt, 1 ms)
   ======================================================================== */
static volatile uint32_t g_ms = 0;

static void Background_Task(void);   /* GPS drain + accelerometer FIFO drain; defined below */

void SysTick_Handler(void)
{
    g_ms++;
}

static void Time_Init(void)
{
    SysTick->LOAD = 16000 - 1;   /* 16 MHz / 1000 */
    SysTick->VAL  = 0;
    SysTick->CTRL = 7;           /* enable | interrupt | processor clock */
}

static uint32_t millis(void)
{
    return g_ms;
}

/* Blocking delay that keeps draining the GPS buffer and the accelerometer FIFO while it waits */
static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms;
    while ((g_ms - start) < ms)
        Background_Task();
}

/* ========================================================================
   2. GPS RECEIVE (USART1 interrupt + ring buffer + NMEA parser)
   ======================================================================== */
#define RB_SIZE 512

static volatile uint16_t rb_head = 0;
static volatile uint16_t rb_tail = 0;
static uint8_t           rb_buf[RB_SIZE];

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE))
    {
        uint8_t b = (uint8_t)USART1->DR;       /* reading DR clears the flags */
        uint16_t n = (uint16_t)((rb_head + 1) % RB_SIZE);
        if (n != rb_tail)
        {
            rb_buf[rb_head] = b;
            rb_head = n;
        }
    }
}

static int rb_get(void)
{
    if (rb_head == rb_tail) return -1;
    uint8_t b = rb_buf[rb_tail];
    rb_tail = (uint16_t)((rb_tail + 1) % RB_SIZE);
    return b;
}

typedef struct
{
    int32_t lat5;     /* degrees x 100000, negative = S */
    int32_t lon5;     /* degrees x 100000, negative = W */
    uint8_t fix;      /* 0 = no fix                     */
} GpsData;

static GpsData g_gps = {0, 0, 0};

/* "ddmm.mmmm" + hemisphere  ->  degrees x 100000 (integer maths only) */
static int32_t nmea_to_deg5(const char *s, char hemi)
{
    int32_t ip = 0, frac = 0;
    uint8_t fd = 0;

    if (*s == 0) return 0;

    while (*s >= '0' && *s <= '9') { ip = ip * 10 + (*s - '0'); s++; }

    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9' && fd < 5) { frac = frac * 10 + (*s - '0'); s++; fd++; }
    }
    while (fd < 5) { frac *= 10; fd++; }

    int32_t deg     = ip / 100;
    int32_t min_int = ip % 100;
    int32_t min5    = min_int * 100000 + frac;           /* minutes x 100000 */
    int32_t v       = deg * 100000 + min5 / 60;

    return (hemi == 'S' || hemi == 'W') ? -v : v;
}

/* Handles $GPGGA / $GNGGA, keeps empty fields in place (no strtok) */
static void gps_parse(char *line, uint8_t len)
{
    char *f[8];
    uint8_t n = 0;
    char *p = line;

    if (len < 20) return;
    if (line[0] != '$' || line[3] != 'G' || line[4] != 'G' || line[5] != 'A') return;

    f[n++] = p;
    while (*p && n < 8)
    {
        if (*p == ',') { *p = 0; f[n++] = p + 1; }
        p++;
    }
    if (n < 7) return;

    /* f[2]=lat f[3]=N/S f[4]=lon f[5]=E/W f[6]=fix quality */
    if (f[6][0] >= '1' && f[6][0] <= '9')
    {
        g_gps.fix  = 1;
        g_gps.lat5 = nmea_to_deg5(f[2], f[3][0]);
        g_gps.lon5 = nmea_to_deg5(f[4], f[5][0]);
    }
    else
    {
        g_gps.fix = 0;
    }
}

static char    gps_line[100];
static uint8_t gps_len = 0;

static void GPS_Task(void)
{
    int c;
    while ((c = rb_get()) >= 0)
    {
        if (c == '$') gps_len = 0;
        if (gps_len < sizeof(gps_line) - 1) gps_line[gps_len++] = (char)c;
        if (c == '\n')
        {
            gps_line[gps_len] = 0;
            gps_parse(gps_line, gps_len);
            gps_len = 0;
        }
    }
}

/* ========================================================================
   3. UARTs
   ======================================================================== */
static void USART2_Init(void)            /* HC-05, 9600 baud */
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER  &= ~((3U << 4) | (3U << 6));
    GPIOA->MODER  |=  ((2U << 4) | (2U << 6));
    GPIOA->AFR[0] &= ~((0xFU << 8) | (0xFU << 12));
    GPIOA->AFR[0] |=  ((7U << 8)  | (7U << 12));     /* AF7 = USART2 */

    USART2->BRR = 1667;                               /* 16 MHz / 9600 */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void USART2_Write(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

static void USART1_Init(void)            /* GPS, 9600 baud, RX interrupt */
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER  &= ~((3U << 18) | (3U << 20));
    GPIOA->MODER  |=  ((2U << 18) | (2U << 20));
    GPIOA->AFR[1] &= ~((0xFU << 4) | (0xFU << 8));
    GPIOA->AFR[1] |=  ((7U << 4)  | (7U << 8));       /* AF7 = USART1 (PA9, PA10) */

    USART1->BRR = 1667;                               /* 16 MHz / 9600 */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_EnableIRQ(USART1_IRQn);
}

/* ========================================================================
   4. I2C1 (with time-outs, so a loose wire cannot freeze the system)
   ======================================================================== */
#define I2C_TO 30000UL

static void I2C1_Config(void)
{
    I2C1->CR1  &= ~I2C_CR1_PE;
    I2C1->CR2   = 16;            /* APB1 = 16 MHz */
    I2C1->CCR   = 80;            /* 100 kHz       */
    I2C1->TRISE = 17;
    I2C1->CR1  |= I2C_CR1_PE;
    I2C1->CR1  |= I2C_CR1_ACK;
}

static void I2C1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER   &= ~((3U << 12) | (3U << 14));
    GPIOB->MODER   |=  ((2U << 12) | (2U << 14));
    GPIOB->OTYPER  |=  (1U << 6) | (1U << 7);
    GPIOB->OSPEEDR |=  (3U << 12) | (3U << 14);
    GPIOB->AFR[0]  &= ~((0xFU << 24) | (0xFU << 28));
    GPIOB->AFR[0]  |=  ((4U << 24)  | (4U << 28));    /* AF4 = I2C1 */

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1_Config();
}

static void I2C_Recover(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    I2C1_Config();
}

static uint8_t wait_sr1(uint32_t mask)
{
    uint32_t t = I2C_TO;
    while (!(I2C1->SR1 & mask))
    {
        if (I2C1->SR1 & I2C_SR1_AF)
        {
            I2C1->SR1 &= ~I2C_SR1_AF;
            return 0;
        }
        if (--t == 0) return 0;
    }
    return 1;
}

static uint8_t I2C_Start(void)
{
    uint32_t t = I2C_TO;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && --t);
    if (t == 0) return 0;
    I2C1->CR1 |= I2C_CR1_START;
    return wait_sr1(I2C_SR1_SB);
}

static uint8_t I2C_RepStart(void)
{
    I2C1->CR1 |= I2C_CR1_START;
    return wait_sr1(I2C_SR1_SB);
}

static uint8_t I2C_Address(uint8_t addr_rw)
{
    I2C1->DR = addr_rw;
    if (!wait_sr1(I2C_SR1_ADDR)) return 0;
    (void)I2C1->SR2;                       /* clears ADDR */
    return 1;
}

static uint8_t I2C_Write(uint8_t d)
{
    if (!wait_sr1(I2C_SR1_TXE)) return 0;
    I2C1->DR = d;
    return wait_sr1(I2C_SR1_BTF);
}

static void I2C_Stop(void)
{
    uint32_t t = I2C_TO;
    I2C1->CR1 |= I2C_CR1_STOP;
    while ((I2C1->CR1 & I2C_CR1_STOP) && --t);
}

/* Reads len bytes after the read-address phase; NACK + STOP on the last */
static uint8_t I2C_ReadTail(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    I2C1->CR1 |= I2C_CR1_ACK;
    for (i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        if (!wait_sr1(I2C_SR1_RXNE)) return 0;
        buf[i] = (uint8_t)I2C1->DR;
    }
    return 1;
}

static uint8_t I2C_WriteBytes(uint8_t a7, const uint8_t *d, uint8_t n)
{
    uint8_t i;
    if (!I2C_Start()) goto fail;
    if (!I2C_Address((uint8_t)(a7 << 1))) goto fail;
    for (i = 0; i < n; i++)
        if (!I2C_Write(d[i])) goto fail;
    I2C_Stop();
    return 1;
fail:
    I2C_Recover();
    return 0;
}

static uint8_t I2C_ReadRegs(uint8_t a7, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!I2C_Start()) goto fail;
    if (!I2C_Address((uint8_t)(a7 << 1))) goto fail;
    if (!I2C_Write(reg)) goto fail;
    if (!I2C_RepStart()) goto fail;
    if (!I2C_Address((uint8_t)((a7 << 1) | 1))) goto fail;
    if (!I2C_ReadTail(buf, len)) goto fail;
    return 1;
fail:
    I2C_Recover();
    return 0;
}

static uint8_t I2C_ReadRaw(uint8_t a7, uint8_t *buf, uint8_t len)
{
    if (!I2C_Start()) goto fail;
    if (!I2C_Address((uint8_t)((a7 << 1) | 1))) goto fail;
    if (!I2C_ReadTail(buf, len)) goto fail;
    return 1;
fail:
    I2C_Recover();
    return 0;
}

/* ========================================================================
   5. LCD 16x2 (PCF8574 backpack, 4-bit mode)
   ======================================================================== */
static uint8_t g_lcd_fail = 0;

static void LCD_Write4(uint8_t data)
{
    uint8_t seq[3];
    if (g_lcd_fail >= 3) return;                /* LCD missing: do not slow the system */
    seq[0] = (uint8_t)(data | 0x08);            /* backlight */
    seq[1] = (uint8_t)(data | 0x0C);            /* EN = 1    */
    seq[2] = (uint8_t)(data | 0x08);            /* EN = 0    */
    if (I2C_WriteBytes(LCD_ADDR >> 1, seq, 3)) g_lcd_fail = 0;
    else g_lcd_fail++;
}

static void LCD_Send(uint8_t data, uint8_t rs)
{
    uint8_t high = data & 0xF0;
    uint8_t low  = (uint8_t)((data << 4) & 0xF0);
    if (rs) { high |= 1; low |= 1; }
    LCD_Write4(high);
    LCD_Write4(low);
}

static void LCD_Command(uint8_t cmd) { LCD_Send(cmd, 0); delay_ms(2); }
static void LCD_Data(uint8_t d)      { LCD_Send(d, 1); }

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
    LCD_Command((uint8_t)((row == 0 ? 0x80 : 0xC0) + col));
}

static void LCD_Clear(void)
{
    LCD_Command(0x01);
    delay_ms(3);
}

static void LCD_Init(void)
{
    delay_ms(50);
    LCD_Write4(0x30); delay_ms(5);
    LCD_Write4(0x30); delay_ms(1);
    LCD_Write4(0x30); delay_ms(1);
    LCD_Write4(0x20); delay_ms(1);
    LCD_Command(0x28);   /* 4-bit, 2 lines */
    LCD_Command(0x0C);   /* display on     */
    LCD_Command(0x06);   /* entry mode     */
    LCD_Command(0x01);   /* clear          */
    delay_ms(5);
}

/* Print one full 16-character row (pads with spaces) */
static void LCD_Print(uint8_t row, const char *s)
{
    uint8_t n = 0;
    LCD_SetCursor(row, 0);
    while (*s && n < 16) { LCD_Data((uint8_t)*s++); n++; }
    while (n < 16)       { LCD_Data(' '); n++; }
}

/* ========================================================================
   5b. COMPUTE LIBRARY - pure maths, no hardware access (unit-tested on a PC).
       Own float helpers, so nothing from libm is needed at link time.
   ======================================================================== */
/* COMPUTE-BEGIN */
static uint32_t isqrt32(uint32_t v)            /* integer square root */
{
    uint32_t r = 0, bit = 1UL << 30;
    while (bit > v) bit >>= 2;
    while (bit)
    {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

typedef union { float f; uint32_t u; } FBits;

static float f_sqrt(float x)                   /* Newton-Raphson, start value from the exponent bits */
{
    FBits b;
    float y;
    uint8_t i;
    if (x <= 0.0f) return 0.0f;
    b.f = x;
    b.u = (b.u >> 1) + 0x1FBD3F7DUL;
    y = b.f;
    for (i = 0; i < 4; i++) y = 0.5f * (y + x / y);
    return y;
}

static float f_exp(float x)                    /* e^x: x = k*ln2 + r, polynomial for e^r, then scale by 2^k */
{
    FBits b;
    float r, p;
    int32_t k;
    if (x > 60.0f)  x = 60.0f;
    if (x < -60.0f) return 0.0f;
    k = (int32_t)(x * 1.44269504f + (x >= 0.0f ? 0.5f : -0.5f));
    r = x - (float)k * 0.693147181f;
    p = 1.0f + r * (1.0f + r * (0.5f + r * (0.166666667f + r * (0.0416666667f
        + r * (0.00833333333f + r * 0.00138888889f)))));
    b.u = (uint32_t)(k + 127) << 23;
    return p * b.f;
}

static float f_ln(float x)                     /* ln x: x = m*2^e, ln m from the atanh series */
{
    FBits b;
    int32_t e;
    float m, z, z2;
    if (x <= 0.0f) return -87.0f;
    b.f = x;
    e = (int32_t)((b.u >> 23) & 0xFFUL) - 127;
    b.u = (b.u & 0x007FFFFFUL) | 0x3F800000UL;
    m = b.f;
    if (m > 1.41421356f) { m *= 0.5f; e++; }
    z  = (m - 1.0f) / (m + 1.0f);
    z2 = z * z;
    return (float)e * 0.693147181f
         + 2.0f * z * (1.0f + z2 * (0.333333333f + z2 * (0.2f + z2 * (0.142857143f + z2 * 0.111111111f))));
}

static float f_atan2_deg(float y, float x)     /* four-quadrant arctangent in degrees */
{
    float ax = (x < 0.0f) ? -x : x;
    float ay = (y < 0.0f) ? -y : y;
    float a, t, t2;
    if (ax == 0.0f && ay == 0.0f) return 0.0f;
    if (ay <= ax) t = ay / ax; else t = ax / ay;
    t2 = t * t;
    a = t * (0.9998660f + t2 * (-0.3302995f + t2 * (0.1801410f + t2 * (-0.0851330f + t2 * 0.0208351f))));
    if (ay > ax) a = 1.5707963f - a;
    if (x < 0.0f) a = 3.1415927f - a;
    if (y < 0.0f) a = -a;
    return a * 57.2957795f;
}

/* ---- Mean Kinetic Temperature:  MKT = (dH/R) / -ln( mean( exp(-dH/R / T) ) ),  T in kelvin ----
   Written relative to 25 degC so the running sum stays near 1 (float is then accurate for days of data).
   Kahan summation keeps rounding error from piling up over thousands of samples.                       */
typedef struct { float sum; float comp; uint32_t n; } Mkt_t;

static void Mkt_Reset(Mkt_t *m) { m->sum = 0.0f; m->comp = 0.0f; m->n = 0; }

static void Mkt_Add(Mkt_t *m, int32_t t100)    /* t100 = temperature in 0.01 degC */
{
    float tk   = (float)t100 * 0.01f + 273.15f;
    float term = f_exp(MKT_K * (1.0f / MKT_TREF - 1.0f / tk));
    float y    = term - m->comp;
    float t    = m->sum + y;
    m->comp = (t - m->sum) - y;
    m->sum  = t;
    m->n++;
}

static int32_t Mkt_Get100(const Mkt_t *m)      /* MKT in 0.01 degC (0 if there is no data yet) */
{
    float mean, den, c;
    if (m->n == 0) return 0;
    mean = m->sum / (float)m->n;
    den  = MKT_K / MKT_TREF - f_ln(mean);
    if (den < 1.0f) den = 1.0f;
    c = (MKT_K / den - 273.15f) * 100.0f;
    return (int32_t)(c >= 0.0f ? c + 0.5f : c - 0.5f);
}

/* ---- Least-squares trend over the last REG_N points (true time stamps, so uneven spacing is fine) ---- */
typedef struct { uint32_t ms[REG_N]; int16_t y[REG_N]; uint8_t head; uint8_t n; } Reg_t;

static void Reg_Reset(Reg_t *r) { r->head = 0; r->n = 0; }

static void Reg_Push(Reg_t *r, uint32_t ms, int16_t y100)
{
    r->ms[r->head] = ms;
    r->y[r->head]  = y100;
    r->head = (uint8_t)((r->head + 1) % REG_N);
    if (r->n < REG_N) r->n++;
}

/* slope = sum((x-xm)(y-ym)) / sum((x-xm)^2).  slope in 0.01 degC per second; now100 = fitted value at newest point */
static uint8_t Reg_Fit(const Reg_t *r, float *slope_per_s, float *now100)
{
    uint8_t i, idx, start, newest, n = r->n;
    uint32_t t0;
    float xm = 0.0f, ym = 0.0f, sxx = 0.0f, sxy = 0.0f, dx, dy, slope, xn;

    if (n < REG_MIN_POINTS) return 0;
    start = (uint8_t)((r->head + REG_N - n) % REG_N);
    t0 = r->ms[start];

    for (i = 0; i < n; i++)
    {
        idx = (uint8_t)((start + i) % REG_N);
        xm += (float)(r->ms[idx] - t0) * 0.001f;
        ym += (float)r->y[idx];
    }
    xm /= (float)n;
    ym /= (float)n;

    for (i = 0; i < n; i++)
    {
        idx = (uint8_t)((start + i) % REG_N);
        dx = (float)(r->ms[idx] - t0) * 0.001f - xm;
        dy = (float)r->y[idx] - ym;
        sxx += dx * dx;
        sxy += dx * dy;
    }
    if (sxx < 1.0f) return 0;

    slope  = sxy / sxx;
    newest = (uint8_t)((r->head + REG_N - 1) % REG_N);
    xn     = (float)(r->ms[newest] - t0) * 0.001f;
    *slope_per_s = slope;
    *now100      = ym + slope * (xn - xm);
    return 1;
}

/* Seconds until the fitted line reaches the limit it is heading for (0 = no useful prediction). *up = 1 when warming. */
static uint16_t Eta_Seconds(float slope_per_s, float now100, int32_t lo100, int32_t hi100, uint8_t *up)
{
    float per_min = slope_per_s * 60.0f;
    float d;
    if (per_min >= (float)RATE_MIN_100 && now100 < (float)hi100)
    {
        d = ((float)hi100 - now100) / slope_per_s;
        *up = 1;
    }
    else if (per_min <= -(float)RATE_MIN_100 && now100 > (float)lo100)
    {
        d = (now100 - (float)lo100) / (-slope_per_s);
        *up = 0;
    }
    else return 0;
    if (d > 5999.0f) d = 5999.0f;
    if (d < 1.0f)    d = 1.0f;
    return (uint16_t)d;
}

/* Equivalent drop height from free-fall time:  h = 1/2 g t^2,  t = n x 10 ms,  g = 9.81 m/s^2.  Result in mm. */
static uint16_t Drop_HeightMm(uint8_t n_samples)
{
    return (uint16_t)(((uint32_t)n_samples * n_samples * 4905UL) / 10000UL);
}

/* Angle between the current gravity vector a and the sealed reference r:  atan2(|a x r|, a . r), degrees */
static float Tilt_Deg(const float a[3], const float r[3])
{
    float cx = a[1] * r[2] - a[2] * r[1];
    float cy = a[2] * r[0] - a[0] * r[2];
    float cz = a[0] * r[1] - a[1] * r[0];
    float dot = a[0] * r[0] + a[1] * r[1] + a[2] * r[2];
    return f_atan2_deg(f_sqrt(cx * cx + cy * cy + cz * cz), dot);
}

/* Software model of the STM32 CRC unit (CRC-32, polynomial 0x04C11DB7, one 32-bit word at a time) */
static uint32_t crc_sw_word(uint32_t crc, uint32_t data)
{
    uint8_t i;
    crc ^= data;
    for (i = 0; i < 32; i++)
        crc = (crc & 0x80000000UL) ? ((crc << 1) ^ 0x04C11DB7UL) : (crc << 1);
    return crc;
}
/* COMPUTE-END */

/* ---- cycle counter (DWT) for timing the algorithms ---- */
enum { B_MKT, B_TREND, B_STEP, B_SAMPLE, B_TILT, B_CRC, B_COUNT };
static const char *const BENCH_NAME[B_COUNT] = { "MKT", "TREND", "STEP", "SAMPLE", "TILT", "CRC" };
static uint32_t bench_sum[B_COUNT], bench_max[B_COUNT], bench_cnt[B_COUNT];

static void Bench_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Bench_End(uint8_t idx, uint32_t start)
{
    uint32_t c = DWT->CYCCNT - start;
    if (c > bench_max[idx]) bench_max[idx] = c;
    bench_sum[idx] += c;
    bench_cnt[idx]++;
}

/* ---- hardware CRC unit, chained over the log lines ---- */
static uint8_t  g_crc_hw = 0;
static uint32_t g_crc_sw = 0xFFFFFFFFUL;

static void Crc_Reset(void)
{
    if (g_crc_hw) CRC->CR = CRC_CR_RESET;
    g_crc_sw = 0xFFFFFFFFUL;
}

static void Crc_Word(uint32_t w)
{
    if (g_crc_hw) CRC->DR = w;
    else          g_crc_sw = crc_sw_word(g_crc_sw, w);
}

static uint32_t Crc_Value(void)
{
    return g_crc_hw ? CRC->DR : g_crc_sw;
}

static void Crc_Bytes(const char *p, uint16_t n)       /* little-endian packing, last word zero-padded */
{
    uint32_t w = 0;
    uint8_t k = 0;
    while (n--)
    {
        w |= (uint32_t)(uint8_t)(*p++) << (8U * k);
        if (++k == 4) { Crc_Word(w); w = 0; k = 0; }
    }
    if (k) Crc_Word(w);
}

/* Returns 1 when the hardware CRC matches the software model (else the software model is used) */
static uint8_t Crc_Init(void)
{
    uint32_t hw, sw;
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    g_crc_hw = CRC_USE_HW;
    if (!g_crc_hw) return 0;
    Crc_Reset();
    Crc_Word(0x12345678UL);
    Crc_Word(0x0BADF00DUL);
    hw = Crc_Value();
    sw = crc_sw_word(crc_sw_word(0xFFFFFFFFUL, 0x12345678UL), 0x0BADF00DUL);
    if (hw != sw) { g_crc_hw = 0; return 0; }
    return 1;
}

/* ========================================================================
   6. SENSORS
   ======================================================================== */

/* ---- SHT31 temperature / humidity (hundredths of a degree / tenths of %RH) ---- */
static uint8_t SHT31_CRC(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF, i, j;
    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t SHT31_Read(int16_t *t100, int16_t *h10)
{
    uint8_t cmd[2] = {0x24, 0x00};       /* single shot, high repeatability */
    uint8_t d[6];
    uint16_t rt, rh;

    if (!I2C_WriteBytes(SHT31_ADDR, cmd, 2)) return 0;
    delay_ms(20);
    if (!I2C_ReadRaw(SHT31_ADDR, d, 6)) return 0;
    if (SHT31_CRC(&d[0], 2) != d[2] || SHT31_CRC(&d[3], 2) != d[5]) return 0;

    rt = (uint16_t)((d[0] << 8) | d[1]);
    rh = (uint16_t)((d[3] << 8) | d[4]);
    *t100 = (int16_t)((int32_t)((17500UL * rt) / 65535UL) - 4500);
    *h10 = (int16_t)((1000UL * rh) / 65535UL);
    return 1;
}

/* ---- ADXL345 accelerometer: 100 Hz, +/-16 g, FIFO in stream mode ----
   The FIFO holds the newest 32 samples (320 ms). We drain it every ACC_POLL_MS (also while waiting in delay_ms
   and while sending Bluetooth text), so no sample is lost and every sample is exactly 10 ms apart.            */
#define ACC_POLL_MS     80
#define ACC_IMPACT_WIN  10    /* samples (100 ms) watched after a fall ends, to catch the impact peak */

static uint8_t ADXL345_Init(void)
{
    uint8_t id = 0;
    uint8_t fmt[2]  = {0x31, 0x0B};      /* full resolution, +/-16 g (impacts are far above 2 g) */
    uint8_t rate[2] = {0x2C, 0x0A};      /* 100 Hz output data rate                              */
    uint8_t fifo[2] = {0x38, 0x9F};      /* stream mode                                          */
    uint8_t pwr[2]  = {0x2D, 0x08};      /* measurement mode                                     */

    if (!I2C_ReadRegs(ADXL_ADDR, 0x00, &id, 1) || id != 0xE5) return 0;
    if (!I2C_WriteBytes(ADXL_ADDR, fmt, 2))  return 0;
    if (!I2C_WriteBytes(ADXL_ADDR, rate, 2)) return 0;
    if (!I2C_WriteBytes(ADXL_ADDR, fifo, 2)) return 0;
    delay_ms(10);
    if (!I2C_WriteBytes(ADXL_ADDR, pwr, 2))  return 0;
    delay_ms(10);
    return 1;
}

static void ADXL345_FlushFifo(void)      /* bypass, then back to stream: throws away old samples */
{
    uint8_t bypass[2] = {0x38, 0x00};
    uint8_t stream[2] = {0x38, 0x9F};
    (void)I2C_WriteBytes(ADXL_ADDR, bypass, 2);
    (void)I2C_WriteBytes(ADXL_ADDR, stream, 2);
}

static uint8_t ADXL345_FifoCount(void)   /* number of samples waiting, 0xFF = bus error */
{
    uint8_t st = 0;
    if (!I2C_ReadRegs(ADXL_ADDR, 0x39, &st, 1)) return 0xFF;
    return (uint8_t)(st & 0x3F);
}

static uint8_t ADXL345_ReadSample(int16_t *x, int16_t *y, int16_t *z)   /* pops one sample from the FIFO */
{
    uint8_t d[6];
    if (!I2C_ReadRegs(ADXL_ADDR, 0x32, d, 6)) return 0;
    *x = (int16_t)((d[1] << 8) | d[0]);
    *y = (int16_t)((d[3] << 8) | d[2]);
    *z = (int16_t)((d[5] << 8) | d[4]);
    return 1;
}

/* ACCEL-BEGIN */
typedef struct
{
    uint8_t  run;                  /* 1 while the FIFO is being drained                         */
    uint8_t  cal_ok;               /* sealing calibration succeeded                             */
    uint8_t  fail;                 /* consecutive bus errors                                    */
    uint32_t last_poll;
    uint16_t cal;                  /* counts per 1 g, measured at sealing                       */
    float    ref[3];               /* resting gravity vector at sealing (counts)                */
    uint16_t limit_mg;             /* shock limit of the active profile                         */
    uint16_t tilt_lim;             /* tilt limit in degrees, 0 = off                            */

    uint16_t last_mg, peak_mg, peak_all_mg;
    float    vib_sum;              /* sum of (|a| - 1 g)^2 in mg^2                              */
    uint16_t vib_n;

    uint8_t  hit;                  /* a sample went above the shock limit (not yet judged)      */
    uint32_t hit_ms;
    uint16_t hit_peak;

    uint8_t  ff_n;                 /* consecutive free-fall samples                             */
    uint8_t  drop_active, imp_left, drop_n;
    uint16_t imp_peak;
    uint8_t  drop_ready, drop_len;
    uint16_t drop_peak;

    int32_t  tsum[3];              /* quasi-static samples collected for the tilt calculation   */
    uint16_t tn;
    uint8_t  tilt_cnt, tilt_flag;
    uint16_t tilt_deg;
} Acc_t;

static Acc_t acc;

static void Accel_ResetEvents(void)
{
    acc.hit = 0; acc.ff_n = 0; acc.drop_active = 0; acc.drop_ready = 0;
    acc.peak_mg = 0; acc.peak_all_mg = 0; acc.vib_sum = 0.0f; acc.vib_n = 0;
    acc.tsum[0] = 0; acc.tsum[1] = 0; acc.tsum[2] = 0; acc.tn = 0;
    acc.tilt_cnt = 0; acc.tilt_flag = 0; acc.tilt_deg = 0;
    acc.fail = 0;
}

/* One 10 ms sample: magnitude, peak, vibration, shock hit, free-fall/drop tracking, tilt collection */
static void Accel_Process(int16_t x, int16_t y, int16_t z, uint32_t ts)
{
    int32_t  xx = x, yy = y, zz = z, dev;
    uint32_t mg;

    mg = (isqrt32((uint32_t)(xx * xx + yy * yy + zz * zz)) * 1000UL) / acc.cal;
    if (mg > 60000UL) mg = 60000UL;
    acc.last_mg = (uint16_t)mg;
    if (mg > acc.peak_mg)     acc.peak_mg     = (uint16_t)mg;
    if (mg > acc.peak_all_mg) acc.peak_all_mg = (uint16_t)mg;

    dev = (int32_t)mg - 1000;                         /* vibration = RMS of the deviation from 1 g */
    acc.vib_sum += (float)(dev * dev);
    if (acc.vib_n < 65535U) acc.vib_n++;

    if (acc.limit_mg != 0 && mg >= acc.limit_mg)      /* shock: one sample above the profile limit */
    {
        if (!acc.hit) { acc.hit = 1; acc.hit_ms = ts; acc.hit_peak = (uint16_t)mg; }
        else if (mg > acc.hit_peak) acc.hit_peak = (uint16_t)mg;
    }

    if (acc.drop_active)                              /* watch the impact that follows a fall */
    {
        if (mg > acc.imp_peak) acc.imp_peak = (uint16_t)mg;
        if (--acc.imp_left == 0)
        {
            acc.drop_active = 0;
            acc.drop_len  = acc.drop_n;
            acc.drop_peak = acc.imp_peak;
            acc.drop_ready = 1;
        }
    }

    if (mg < FF_THRESH_MG)                            /* free fall: |a| close to 0 g */
    {
        if (acc.ff_n < 255) acc.ff_n++;
    }
    else
    {
        if (acc.ff_n >= FF_MIN_SAMPLES && acc.ff_n <= FF_MAX_SAMPLES && !acc.drop_active)
        {
            acc.drop_active = 1;
            acc.drop_n = acc.ff_n;
            acc.imp_left = ACC_IMPACT_WIN;
            acc.imp_peak = (uint16_t)mg;
        }
        acc.ff_n = 0;
    }

    if (mg >= 800UL && mg <= 1200UL)                  /* only a (nearly) still box shows true gravity */
    {
        acc.tsum[0] += xx; acc.tsum[1] += yy; acc.tsum[2] += zz;
        if (acc.tn < 65535U) acc.tn++;
    }
}

static void Accel_TiltUpdate(void)
{
    float a[3];
    float deg;
    uint32_t c0;

    if (acc.tn == 0) return;
    c0 = DWT->CYCCNT;
    a[0] = (float)acc.tsum[0] / (float)acc.tn;
    a[1] = (float)acc.tsum[1] / (float)acc.tn;
    a[2] = (float)acc.tsum[2] / (float)acc.tn;
    deg = Tilt_Deg(a, acc.ref);
    if (deg < 0.0f) deg = 0.0f;
    acc.tilt_deg = (uint16_t)(deg + 0.5f);
    if (acc.tilt_lim != 0 && acc.tilt_deg > acc.tilt_lim) { if (acc.tilt_cnt < 255) acc.tilt_cnt++; }
    else acc.tilt_cnt = 0;
    acc.tilt_flag = (uint8_t)(acc.tilt_cnt >= 2);     /* two checks in a row (about 0.2 s) */
    acc.tsum[0] = 0; acc.tsum[1] = 0; acc.tsum[2] = 0; acc.tn = 0;
    Bench_End(B_TILT, c0);
}
/* ACCEL-END */

static void Accel_Poll(void)
{
    uint32_t now, c0;
    uint8_t cnt, i;
    int16_t x, y, z;

    if (!acc.run) return;
    now = millis();
    if ((now - acc.last_poll) < ACC_POLL_MS) return;
    acc.last_poll = now;

    acc.run = 0;                                      /* no re-entry while we use the I2C bus */
    cnt = ADXL345_FifoCount();
    if (cnt == 0xFF)
    {
        if (acc.fail < 255) acc.fail++;
    }
    else
    {
        if (cnt > 32) cnt = 32;
        for (i = 0; i < cnt; i++)
        {
            if (!ADXL345_ReadSample(&x, &y, &z)) { if (acc.fail < 255) acc.fail++; break; }
            acc.fail = 0;
            c0 = DWT->CYCCNT;
            Accel_Process(x, y, z, now - (uint32_t)(cnt - 1U - i) * 10UL);
            Bench_End(B_SAMPLE, c0);
        }
        Accel_TiltUpdate();
    }
    acc.run = 1;
}

static void Background_Task(void)
{
    GPS_Task();
    Accel_Poll();
}

/* At sealing: measure the resting gravity vector. Its length is "1 g" for this particular sensor, so a module
   with a different scale still reads 1.00 g at rest; its direction is the reference for the tilt check.      */
static void Accel_Calibrate(void)
{
    uint8_t attempt, got = 0, cnt, have = 0;
    int16_t x, y, z;
    int32_t sumx, sumy, sumz, bx = 0, by = 0, bz = 0;
    uint32_t mmin, mmax, m, t0, spread, best = 0xFFFFFFFFUL;
    float mag;

    acc.run = 0;
    acc.cal = 256; acc.cal_ok = 0;
    acc.ref[0] = 0.0f; acc.ref[1] = 0.0f; acc.ref[2] = 256.0f;

    ADXL345_FlushFifo();
    delay_ms(30);

    for (attempt = 0; attempt < 8; attempt++)         /* keep the calmest 16-sample set */
    {
        sumx = 0; sumy = 0; sumz = 0; mmin = 0xFFFFUL; mmax = 0; got = 0;
        t0 = millis();
        while (got < 16 && (millis() - t0) < 500)
        {
            cnt = ADXL345_FifoCount();
            if (cnt == 0xFF) break;
            while (cnt > 0 && got < 16)
            {
                if (!ADXL345_ReadSample(&x, &y, &z)) { cnt = 0; break; }
                sumx += x; sumy += y; sumz += z;
                m = isqrt32((uint32_t)((int32_t)x * x + (int32_t)y * y + (int32_t)z * z));
                if (m < mmin) mmin = m;
                if (m > mmax) mmax = m;
                got++; cnt--;
            }
            if (got < 16) delay_ms(10);
        }
        if (got < 16) break;                          /* sensor not answering */
        spread = mmax - mmin;
        if (spread < best) { best = spread; bx = sumx; by = sumy; bz = sumz; have = 1; }
        if (spread <= 40) break;                      /* calm enough (0.15 g) */
    }

    if (have)
    {
        acc.ref[0] = (float)bx / 16.0f;
        acc.ref[1] = (float)by / 16.0f;
        acc.ref[2] = (float)bz / 16.0f;
        mag = f_sqrt(acc.ref[0] * acc.ref[0] + acc.ref[1] * acc.ref[1] + acc.ref[2] * acc.ref[2]);
        if (mag >= 64.0f && mag <= 512.0f)
        {
            acc.cal = (uint16_t)(mag + 0.5f);
            acc.cal_ok = 1;
        }
    }
    if (!acc.cal_ok) { acc.ref[0] = 0.0f; acc.ref[1] = 0.0f; acc.ref[2] = 256.0f; }
    ADXL345_FlushFifo();
    Accel_ResetEvents();
}

/* ---- LDR on ADC1 channel 0 (PA0) ---- */
static void ADC_Init_LDR(void)
{
    uint32_t i;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    GPIOA->MODER |= (3U << 0);           /* PA0 analog */

    ADC1->SQR1  = 0;                     /* 1 conversion in the sequence */
    ADC1->SQR3  = 0;                     /* channel 0                    */
    ADC1->SMPR2 = 7U;                    /* longest sample time          */
    ADC1->CR2   = ADC_CR2_ADON;
    for (i = 0; i < 1000; i++) __NOP();  /* ADC wake-up */
}

static uint16_t ADC_ReadLDR(void)
{
    uint32_t t = 100000;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC) && --t);
    return (uint16_t)ADC1->DR;
}

/* Median of 5 back-to-back conversions: a single noisy or spiky reading cannot trigger a false tamper alarm */
static uint16_t ADC_ReadLDR_Median5(void)
{
    uint16_t v[5], tmp;
    uint8_t i, j;
    for (i = 0; i < 5; i++) v[i] = ADC_ReadLDR();
    for (i = 1; i < 5; i++)                           /* insertion sort */
    {
        tmp = v[i];
        for (j = i; j > 0 && v[j - 1] > tmp; j--) v[j] = v[j - 1];
        v[j] = tmp;
    }
    return v[2];
}

/* ---- DS1307 / DS3231 real-time clock ---- */
typedef struct { uint8_t h, m, s, d, mo, y; } Rtc_t;

static uint8_t bcd2dec(uint8_t b) { return (uint8_t)(((b >> 4) * 10) + (b & 0x0F)); }
#if RTC_SET_TIME
static uint8_t dec2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
#endif

static uint8_t RTC_Read(Rtc_t *t)
{
    uint8_t r[7];
    if (!I2C_ReadRegs(RTC_ADDR, 0x00, r, 7)) return 0;
    t->s  = bcd2dec(r[0] & 0x7F);
    t->m  = bcd2dec(r[1] & 0x7F);
    t->h  = bcd2dec(r[2] & 0x3F);
    t->d  = bcd2dec(r[4] & 0x3F);
    t->mo = bcd2dec(r[5] & 0x1F);
    t->y  = bcd2dec(r[6]);
    return 1;
}

static uint8_t RTC_Init(void)
{
    uint8_t r0 = 0;
    uint8_t w[2];

    if (!I2C_ReadRegs(RTC_ADDR, 0x00, &r0, 1)) return 0;

#if RTC_SET_TIME
    {
        uint8_t i;
        uint8_t v[7] = { RTC_INIT_SEC, RTC_INIT_MIN, RTC_INIT_HOUR, 1,
                         RTC_INIT_DATE, RTC_INIT_MONTH, RTC_INIT_YEAR };
        for (i = 0; i < 7; i++)
        {
            w[0] = i;
            w[1] = dec2bcd(v[i]);
            I2C_WriteBytes(RTC_ADDR, w, 2);
        }
    }
#else
    if (r0 & 0x80)                       /* DS1307 clock-halt bit set: start the oscillator */
    {
        w[0] = 0x00; w[1] = 0x00;
        I2C_WriteBytes(RTC_ADDR, w, 2);
    }
#endif
    return 1;
}

/* ========================================================================
   7. OUTPUTS: buzzer, relay, RGB LED
   ======================================================================== */
static void Outputs_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* write the safe level first, then switch the pins to output */
    GPIOB->ODR &= ~(1U << 8);                       /* buzzer off */
#if RELAY_ACTIVE_HIGH
    GPIOB->ODR &= ~(1U << 9);
#else
    GPIOB->ODR |=  (1U << 9);
#endif
    GPIOB->MODER &= ~((3U << 16) | (3U << 18));
    GPIOB->MODER |=  ((1U << 16) | (1U << 18));
}

static void Buzzer_Set(uint8_t on)
{
    if (on) GPIOB->ODR |= (1U << 8); else GPIOB->ODR &= ~(1U << 8);
}

static void Relay_Set(uint8_t on)
{
#if RELAY_ACTIVE_HIGH
    if (on) GPIOB->ODR |= (1U << 9); else GPIOB->ODR &= ~(1U << 9);
#else
    if (on) GPIOB->ODR &= ~(1U << 9); else GPIOB->ODR |= (1U << 9);
#endif
}

static void RGB_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    /* PB3 = TIM2_CH2 (AF1), PB4 = TIM3_CH1 (AF2), PB5 = TIM3_CH2 (AF2) */
    GPIOB->MODER  &= ~((3U << 6) | (3U << 8) | (3U << 10));
    GPIOB->MODER  |=  ((2U << 6) | (2U << 8) | (2U << 10));
    GPIOB->AFR[0] &= ~((0xFU << 12) | (0xFU << 16) | (0xFU << 20));
    GPIOB->AFR[0] |=  ((1U << 12)   | (2U << 16)   | (2U << 20));

    TIM3->PSC   = 15;                    /* 1 MHz timer clock */
    TIM3->ARR   = 999;                   /* 1 kHz PWM         */
    TIM3->CCMR1 = (6U << 4) | TIM_CCMR1_OC1PE | (6U << 12) | TIM_CCMR1_OC2PE;
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CCR1  = 0;
    TIM3->CCR2  = 0;
    TIM3->CR1   = TIM_CR1_ARPE;
    TIM3->EGR   = TIM_EGR_UG;
    TIM3->CR1  |= TIM_CR1_CEN;

    TIM2->PSC   = 15;
    TIM2->ARR   = 999;
    TIM2->CCMR1 = (6U << 12) | TIM_CCMR1_OC2PE;
    TIM2->CCER  = TIM_CCER_CC2E;
    TIM2->CCR2  = 0;
    TIM2->CR1   = TIM_CR1_ARPE;
    TIM2->EGR   = TIM_EGR_UG;
    TIM2->CR1  |= TIM_CR1_CEN;
}

static void RGB_Set(uint8_t r, uint8_t g, uint8_t b)      /* 0..255 each */
{
    uint32_t dr = ((uint32_t)r * 999UL) / 255UL;
    uint32_t dg = ((uint32_t)g * 999UL) / 255UL;
    uint32_t db = ((uint32_t)b * 999UL) / 255UL;
#if RGB_COMMON_ANODE
    dr = 999UL - dr; dg = 999UL - dg; db = 999UL - db;
#endif
    TIM3->CCR1 = dr;
    TIM3->CCR2 = dg;
    TIM2->CCR2 = db;
}

/* ========================================================================
   8. INPUTS: reed switch + SW-420 (EXTI), buttons
   ======================================================================== */
static volatile uint8_t  g_shock_flag    = 0;     /* SW-420 vibration switch fired                */
static volatile uint32_t g_shock_time    = 0;     /* time of the first edge of this burst (ms)    */
static volatile uint8_t  g_lid_open_flag = 0;

static uint8_t Lid_IsClosed(void)
{
    return ((GPIOA->IDR >> 1) & 1U) == 0;          /* magnet near = LOW = closed */
}

static void Events_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PA1 reed: input + pull-up */
    GPIOA->MODER &= ~(3U << 2);
    GPIOA->PUPDR &= ~(3U << 2);
    GPIOA->PUPDR |=  (1U << 2);

    /* PB0 SW-420: input, module drives the level */
    GPIOB->MODER &= ~(3U << 0);
    GPIOB->PUPDR &= ~(3U << 0);

    /* EXTI0 <- port B, EXTI1 <- port A */
    SYSCFG->EXTICR[0] &= ~((0xFU << 0) | (0xFU << 4));
    SYSCFG->EXTICR[0] |=  (1U << 0);

    /* both edges: works whatever polarity the SW-420 module uses */
    EXTI->IMR  |= (1U << 0) | (1U << 1);
    EXTI->RTSR |= (1U << 0) | (1U << 1);
    EXTI->FTSR |= (1U << 0) | (1U << 1);
    EXTI->PR    = (1U << 0) | (1U << 1);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI0_IRQHandler(void)
{
    EXTI->PR = (1U << 0);
    if (!g_shock_flag) g_shock_time = g_ms;
    g_shock_flag = 1;
}

void EXTI1_IRQHandler(void)
{
    EXTI->PR = (1U << 1);
    if (!Lid_IsClosed()) g_lid_open_flag = 1;      /* only the "opened" direction counts */
}

typedef struct { uint8_t pin; uint8_t stable; uint8_t last_raw; uint32_t t; } Btn;

static void Buttons_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3U << 24) | (3U << 26));
    GPIOB->PUPDR &= ~((3U << 24) | (3U << 26));
    GPIOB->PUPDR |=  ((1U << 24) | (1U << 26));
}

static uint8_t Btn_Pressed(Btn *b)                 /* debounced, returns 1 once per press */
{
    uint8_t raw = (uint8_t)((GPIOB->IDR >> b->pin) & 1U);
    if (raw != b->last_raw) { b->last_raw = raw; b->t = millis(); }
    if ((millis() - b->t) > 30 && raw != b->stable)
    {
        b->stable = raw;
        if (raw == 0) return 1;
    }
    return 0;
}

/* ========================================================================
   9. SMALL STRING BUILDER (no printf, no libc dependencies)
   ======================================================================== */
typedef struct { char *p; char *end; } Str;

static void Str_Begin(Str *s, char *buf, uint16_t size) { s->p = buf; s->end = buf + size - 1; }
static void Str_End(Str *s)                              { *s->p = 0; }
static void s_char(Str *s, char c)                       { if (s->p < s->end) *s->p++ = c; }
static void s_str(Str *s, const char *t)                 { while (*t) s_char(s, *t++); }

static void s_uint(Str *s, uint32_t v)
{
    char tmp[11];
    uint8_t n = 0;
    if (v == 0) tmp[n++] = '0';
    while (v) { tmp[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n) s_char(s, tmp[--n]);
}

static void s_2d(Str *s, uint8_t v) { s_char(s, (char)('0' + (v / 10) % 10)); s_char(s, (char)('0' + v % 10)); }

static void s_hex8(Str *s, uint32_t v)
{
    int i;
    for (i = 28; i >= 0; i -= 4) s_char(s, "0123456789ABCDEF"[(v >> i) & 0xFU]);
}

static void s_fixed(Str *s, int32_t v, uint8_t decimals)   /* v scaled by 10^decimals */
{
    uint32_t div = 1, i, u, ip, fp;
    for (i = 0; i < decimals; i++) div *= 10;
    if (v < 0) { s_char(s, '-'); u = (uint32_t)(-v); } else u = (uint32_t)v;
    ip = u / div;
    fp = u % div;
    s_uint(s, ip);
    if (decimals)
    {
        s_char(s, '.');
        div /= 10;
        while (div) { s_char(s, (char)('0' + (fp / div) % 10)); div /= 10; }
    }
}

/* ========================================================================
   10. PROFILES, SYSTEM STATE, SCORING
   ======================================================================== */
typedef struct
{
    const char *name;
    int16_t  tmin10, tmax10;       /* tenths of a degree C              */
    int16_t  hmax10;               /* tenths of %RH                     */
    uint16_t g100;                 /* shock limit, hundredths of a g    */
    uint8_t  light_check;
    int16_t  demo_below10;         /* demo window: ambient - this       */
    int16_t  demo_above10;         /* demo window: ambient + this (0 = no demo override) */
    int16_t  demo_h_above10;       /* demo humidity limit: ambient RH + this (0 = none)  */
    uint16_t tilt_deg;             /* tilt limit from the sealed orientation, degrees (0 = off) */
} Profile_t;

static const Profile_t PROFILES[3] =
{
    /* name           tmin  tmax  hmax  g100 light  dBelow dAbove dHum tilt */
    { "Pharma-Cold",    20,   80,  600,  150,  1,     50,    15,  150,  45 },
    { "Electronics",  -100,  450,  850,  130,  1,      0,     0,    0,  45 },
    { "Produce",         0,  250,  950,  250,  0,    100,    80,    0,   0 },
};

#define BR_TEMP   0x01
#define BR_HUM    0x02
#define BR_SHOCK  0x04
#define BR_LIGHT  0x08
#define BR_OPEN   0x10
#define BR_TILT   0x20

#define ERR_SHT   0x01
#define ERR_ADXL  0x02
#define ERR_RTC   0x04

typedef enum { ST_MENU, ST_WAIT_SEAL, ST_IN_TRANSIT, ST_DELIVERED } State_t;

typedef struct
{
    State_t   state;
    uint8_t   profile;
    Profile_t act;               /* active limits (copy, may be demo-adjusted) */
    uint8_t   breach;            /* latched flags, never cleared in transit    */
    uint8_t   score;
    uint8_t   worst;
    uint8_t   shocks;
    uint8_t   err;               /* sensor read errors, informational          */
    uint32_t  start_ms;
    uint32_t  arm_at;
    uint16_t  light_base;
    int16_t   t10, h10;            /* t10 = filtered temperature (tenths of a degree)              */
    uint16_t  g100, ldr;           /* g100 = latest |a| in hundredths of a g (1.00 g at rest)      */

    /* ---- computed values ---- */
    int16_t   t100;                /* raw temperature, 0.01 degC                                   */
    int32_t   tf_q4;               /* filtered temperature (moving average), 0.01 degC x 16        */
    uint8_t   tf_init;
    uint32_t  t_prev;              /* time of the previous computation step                        */
    Mkt_t     mkt;
    int32_t   mkt100;              /* mean kinetic temperature, 0.01 degC                          */
    Reg_t     reg;
    uint8_t   reg_div;
    float     slope_s;             /* trend, 0.01 degC per second                                  */
    uint16_t  eta_s;               /* seconds until the predicted breach, 0 = none                 */
    uint8_t   eta_up;              /* 1 = predicted breach is on the hot side                      */
    uint32_t  exc_ms;              /* time spent outside the window                                */
    uint32_t  area_cs;             /* area outside the window, 0.01 degC * s                       */
    uint8_t   in_exc;
    uint16_t  peak100;             /* peak |a| during the last interval, hundredths of a g         */
    uint16_t  peak_max100;         /* peak |a| since sealing                                       */
    uint16_t  vib_mg;              /* vibration RMS, mg                                            */
    uint16_t  tilt_deg, tilt_max;
    uint8_t   tilt_active;
    uint8_t   drops, suspects;
    uint32_t  t_suspect;
} System_t;

static uint8_t Score_Compute(const System_t *s)
{
    int32_t sc = 100, pen;

    if (s->t10 > s->act.tmax10)       { pen = s->t10 - s->act.tmax10; sc -= (pen > 40) ? 40 : pen; }
    else if (s->t10 < s->act.tmin10)  { pen = s->act.tmin10 - s->t10; sc -= (pen > 40) ? 40 : pen; }

    if (s->h10 > s->act.hmax10) sc -= 10;
    if (s->peak100 > s->act.g100) sc -= 25;
    if (s->tilt_active)         sc -= 15;
    if (s->act.light_check && s->ldr > (uint16_t)(s->light_base + LIGHT_DELTA)) sc -= 30;

    return (uint8_t)(sc < 0 ? 0 : sc);
}

/* ========================================================================
   11. LOGGING OVER HC-05
   ======================================================================== */
static void BT_Print(const char *s)
{
    while (*s)
    {
        USART2_Write(*s++);
        Background_Task();                /* keep draining GPS and the accelerometer FIFO while we transmit slowly */
    }
}

static void RTC_Stamp(char *out)          /* "HH:MM:SS", needs 9 bytes */
{
    Rtc_t t;
    Str s;
    Str_Begin(&s, out, 9);
    if (RTC_Read(&t)) { s_2d(&s, t.h); s_char(&s, ':'); s_2d(&s, t.m); s_char(&s, ':'); s_2d(&s, t.s); }
    else s_str(&s, "--:--:--");
    Str_End(&s);
}

static void RTC_DateStamp(char *out)      /* "DD/MM/YY", needs 9 bytes */
{
    Rtc_t t;
    Str s;
    Str_Begin(&s, out, 9);
    if (RTC_Read(&t)) { s_2d(&s, t.d); s_char(&s, '/'); s_2d(&s, t.mo); s_char(&s, '/'); s_2d(&s, t.y); }
    else s_str(&s, "--/--/--");
    Str_End(&s);
}

/* ---- tamper-evident log -------------------------------------------------------------------------
   Every line sent during a shipment ends with  ,SEQ=n,CHK=XXXXXXXX
   CHK is the CRC-32 of this line's text AND of every earlier line (the CRC unit is never reset in
   between), started from a secret key word at sealing. Editing, deleting or inserting any line breaks
   the chain from that point on. SEQ tells a lost Bluetooth packet apart from an edited line.            */
static uint8_t  g_chain_on = 0;
static uint32_t g_seq = 0;

static void Chain_Begin(void)
{
    Crc_Reset();
    Crc_Word(LOG_KEY);
    g_seq = 0;
    g_chain_on = 1;
}

static void Log_End(Str *s, char *buf)    /* s points at the end of the line text; buf is the start */
{
    if (g_chain_on)
    {
        uint32_t c0, crc;
        s_str(s, ",SEQ=");
        s_uint(s, g_seq++);
        c0 = DWT->CYCCNT;
        Crc_Bytes(buf, (uint16_t)(s->p - buf));
        crc = Crc_Value();
        Bench_End(B_CRC, c0);
        s_str(s, ",CHK=");
        s_hex8(s, crc);
    }
    s_str(s, "\r\n");
    Str_End(s);
    BT_Print(buf);
}

typedef struct { const char *k; const char *sv; int32_t v; uint8_t dec; } Kv;   /* extra key=value on an event */

static void Log_EventKv(const char *name, const Kv *kv, uint8_t n)
{
    char ts[9], line[200];
    Str s;
    uint8_t i;
    RTC_Stamp(ts);
    Str_Begin(&s, line, sizeof(line));
    s_str(&s, ts); s_str(&s, ",EVENT="); s_str(&s, name);
    for (i = 0; i < n; i++)
    {
        s_char(&s, ','); s_str(&s, kv[i].k); s_char(&s, '=');
        if (kv[i].sv) s_str(&s, kv[i].sv); else s_fixed(&s, kv[i].v, kv[i].dec);
    }
    Log_End(&s, line);
}

static void Log_Event(const char *name)
{
    Log_EventKv(name, 0, 0);
}

static void Log_Sealed(const System_t *sy)
{
    char ts[9], dt[9], line[240];
    Str s;
    RTC_DateStamp(dt);
    RTC_Stamp(ts);
    Str_Begin(&s, line, sizeof(line));
    s_str(&s, dt); s_char(&s, ' '); s_str(&s, ts);
    s_str(&s, ",EVENT=SEALED,PROFILE="); s_str(&s, sy->act.name);
    s_str(&s, ",TMIN="); s_fixed(&s, sy->act.tmin10, 1);
    s_str(&s, ",TMAX="); s_fixed(&s, sy->act.tmax10, 1);
    s_str(&s, ",HMAX="); s_fixed(&s, sy->act.hmax10, 1);
    s_str(&s, ",GMAX="); s_fixed(&s, sy->act.g100, 2);
    s_str(&s, ",TILTLIM="); s_uint(&s, sy->act.tilt_deg);
    s_str(&s, ",ACAL="); s_uint(&s, acc.cal);
    s_str(&s, ",EXC_S="); s_uint(&s, EXCURSION_BUDGET ? EXC_TIME_BUDGET_S : 0);
    s_str(&s, ",EXC_A="); s_fixed(&s, EXCURSION_BUDGET ? EXC_AREA_BUDGET : 0, 2);
    Log_End(&s, line);
}

static void Log_Sample(const System_t *sy)
{
    char ts[9], line[320];
    Str s;
    int32_t rate = (int32_t)(sy->slope_s * 60.0f + (sy->slope_s >= 0.0f ? 0.5f : -0.5f));   /* 0.01 degC per minute */

    RTC_Stamp(ts);
    Str_Begin(&s, line, sizeof(line));
    s_str(&s, ts);
    s_str(&s, ",T=");   s_fixed(&s, sy->t10, 1);
    s_str(&s, ",H=");   s_fixed(&s, sy->h10, 1);
    s_str(&s, ",G=");   s_fixed(&s, sy->g100, 2);
    s_str(&s, ",LDR="); s_uint(&s, sy->ldr);
    s_str(&s, ",FIX="); s_uint(&s, g_gps.fix);
    s_str(&s, ",LAT="); s_fixed(&s, g_gps.lat5, 5);
    s_str(&s, ",LON="); s_fixed(&s, g_gps.lon5, 5);
    s_str(&s, ",SCORE="); s_uint(&s, sy->score);
    s_str(&s, ",BREACH="); s_uint(&s, sy->breach);
    s_str(&s, ",ERR=");  s_uint(&s, sy->err);
    s_str(&s, ",MKT=");  s_fixed(&s, sy->mkt100, 2);
    s_str(&s, ",RATE="); s_fixed(&s, rate, 2);
    s_str(&s, ",ETA=");  s_uint(&s, sy->eta_s);
    s_str(&s, ",EXC=");  s_uint(&s, sy->exc_ms / 1000UL);
    s_str(&s, ",DEG=");  s_fixed(&s, (int32_t)(sy->area_cs / 60UL), 2);
    s_str(&s, ",VIB=");  s_uint(&s, sy->vib_mg);
    s_str(&s, ",PEAK="); s_fixed(&s, sy->peak100, 2);
    s_str(&s, ",TILT="); s_uint(&s, sy->tilt_deg);
    Log_End(&s, line);
}

static void Log_Summary(const System_t *sy)
{
    char ts[9], line[360];
    Str s;
    uint8_t mkt_ok = (uint8_t)(sy->mkt.n == 0 || sy->mkt100 <= (int32_t)sy->act.tmax10 * 10);
    uint8_t pass   = (uint8_t)(sy->breach == 0 && mkt_ok);

    RTC_Stamp(ts);
    Str_Begin(&s, line, sizeof(line));
    s_str(&s, ts);
    s_str(&s, ",SUMMARY,PROFILE="); s_str(&s, sy->act.name);
    s_str(&s, ",DURATION_S=");      s_uint(&s, (millis() - sy->start_ms) / 1000UL);
    s_str(&s, ",WORST=");           s_uint(&s, sy->worst);
    s_str(&s, ",BREACH=");          s_uint(&s, sy->breach);
    s_str(&s, ",SHOCKS=");          s_uint(&s, sy->shocks);
    s_str(&s, ",LAT=");             s_fixed(&s, g_gps.lat5, 5);
    s_str(&s, ",LON=");             s_fixed(&s, g_gps.lon5, 5);
    s_str(&s, ",MKT=");             s_fixed(&s, sy->mkt100, 2);
    s_str(&s, ",MKT_LIM=");         s_fixed(&s, sy->act.tmax10, 1);
    s_str(&s, ",MKT_OK=");          s_uint(&s, mkt_ok);
    s_str(&s, ",EXC=");             s_uint(&s, sy->exc_ms / 1000UL);
    s_str(&s, ",DEG=");             s_fixed(&s, (int32_t)(sy->area_cs / 60UL), 2);
    s_str(&s, ",PEAK=");            s_fixed(&s, sy->peak_max100, 2);
    s_str(&s, ",DROPS=");           s_uint(&s, sy->drops);
    s_str(&s, ",TILTMAX=");         s_uint(&s, sy->tilt_max);
    s_str(&s, ",SUSPECT=");         s_uint(&s, sy->suspects);
    s_str(&s, ",VERDICT=");         s_str(&s, pass ? "PASS" : "FAIL");
    Log_End(&s, line);
}

/* Cycle counts of each algorithm (16 MHz clock: 16 cycles = 1 us). MAX includes interrupts that hit mid-run. */
static void Log_Bench(void)
{
    char ts[9], line[320];
    Str s;
    uint8_t i;
    RTC_Stamp(ts);
    Str_Begin(&s, line, sizeof(line));
    s_str(&s, ts); s_str(&s, ",BENCH,CLK_MHZ=16");
    for (i = 0; i < B_COUNT; i++)
    {
        s_str(&s, ",AVG_"); s_str(&s, BENCH_NAME[i]); s_char(&s, '=');
        s_uint(&s, bench_cnt[i] ? bench_sum[i] / bench_cnt[i] : 0);
        s_str(&s, ",MAX_"); s_str(&s, BENCH_NAME[i]); s_char(&s, '=');
        s_uint(&s, bench_max[i]);
    }
    Log_End(&s, line);
}

/* ========================================================================
   12. APPLICATION HELPERS
   ======================================================================== */
static void Show_Menu(const System_t *sy)
{
    LCD_Print(0, "Select profile");
    LCD_Print(1, PROFILES[sy->profile].name);
}

static int16_t Round10(int32_t v100)               /* 0.01 -> 0.1, rounded */
{
    return (int16_t)((v100 >= 0) ? (v100 + 5) / 10 : (v100 - 5) / 10);
}

static void Sample_Sensors(System_t *sy)
{
    int16_t t100, h;

    if (SHT31_Read(&t100, &h))
    {
        int32_t q = (int32_t)t100 * 16;
        if (!sy->tf_init) { sy->tf_q4 = q; sy->tf_init = 1; }
        else sy->tf_q4 += (q - sy->tf_q4) / 4;              /* moving average, weight 1/4 */
        sy->t100 = t100;
        sy->h10  = h;
        sy->t10  = Round10((sy->tf_q4 + 8) >> 4);
        sy->err &= (uint8_t)~ERR_SHT;
    }
    else sy->err |= ERR_SHT;

    /* accelerometer values come from the 100 Hz FIFO stream (see Accel_Poll) */
    sy->g100    = (uint16_t)(acc.last_mg / 10U);
    sy->peak100 = (uint16_t)(acc.peak_mg / 10U);
    if (sy->peak100 > sy->peak_max100) sy->peak_max100 = sy->peak100;
    acc.peak_mg = 0;
    if (acc.vib_n != 0) sy->vib_mg = (uint16_t)f_sqrt(acc.vib_sum / (float)acc.vib_n);
    acc.vib_sum = 0.0f;
    acc.vib_n = 0;
    sy->tilt_deg    = acc.tilt_deg;
    sy->tilt_active = acc.tilt_flag;
    if (sy->tilt_deg > sy->tilt_max) sy->tilt_max = sy->tilt_deg;
    if (acc.fail >= 3 || !acc.cal_ok) sy->err |= ERR_ADXL; else sy->err &= (uint8_t)~ERR_ADXL;

    sy->ldr = ADC_ReadLDR_Median5();
}

/* Runs once per sample: mean kinetic temperature, temperature trend + predicted time to breach, excursion budget.
   Returns a bit mask: 1 = excursion just started, 2 = time budget used up, 4 = area budget used up, 8 = hard limit. */
static uint8_t Compute_Step(System_t *sy, uint32_t now, uint8_t armed)
{
    uint32_t c_all, c0, dt;
    int32_t  tf100, hi100, lo100, over = 0;
    uint8_t  ret = 0;

    if (sy->err & ERR_SHT) { sy->t_prev = now; return 0; }   /* no valid reading: do not integrate old data */
    c_all = DWT->CYCCNT;

    tf100 = (sy->tf_q4 + 8) >> 4;
    hi100 = (int32_t)sy->act.tmax10 * 10;
    lo100 = (int32_t)sy->act.tmin10 * 10;
    dt = now - sy->t_prev;
    sy->t_prev = now;
    if (dt > 5000UL) dt = 5000UL;

    /* 1. Mean kinetic temperature (Arrhenius-weighted average of everything so far) */
    c0 = DWT->CYCCNT;
    Mkt_Add(&sy->mkt, tf100);
    sy->mkt100 = Mkt_Get100(&sy->mkt);
    Bench_End(B_MKT, c0);

    /* 2. Least-squares trend over the last REG_N points -> warming rate and predicted time to the limit */
    if (++sy->reg_div >= REG_EVERY)
    {
        float slope, fit;
        sy->reg_div = 0;
        c0 = DWT->CYCCNT;
        Reg_Push(&sy->reg, now, sy->t100);
        if (Reg_Fit(&sy->reg, &slope, &fit))
        {
            sy->slope_s = slope;
            sy->eta_s   = Eta_Seconds(slope, fit, lo100, hi100, &sy->eta_up);
        }
        else { sy->slope_s = 0.0f; sy->eta_s = 0; }
        Bench_End(B_TREND, c0);
    }

    /* 3. Excursion budget: time and area (degree-seconds) outside the allowed window */
    if (tf100 > hi100)      over = tf100 - hi100;
    else if (tf100 < lo100) over = lo100 - tf100;

    if (armed)
    {
        if (over > 0)
        {
            if (!sy->in_exc) { sy->in_exc = 1; ret |= 1; }
            sy->exc_ms  += dt;
            sy->area_cs += (uint32_t)over * dt / 1000UL;
        }
        else if (sy->in_exc && tf100 < hi100 - 5 && tf100 > lo100 + 5) sy->in_exc = 0;   /* small hysteresis */

        if (over >= EXC_HARD_MARGIN)                    ret |= 8;
        if (sy->exc_ms  >= EXC_TIME_BUDGET_S * 1000UL)  ret |= 2;
        if (sy->area_cs >= EXC_AREA_BUDGET * 60UL)      ret |= 4;
    }
    Bench_End(B_STEP, c_all);
    return ret;
}

static uint8_t Latch(System_t *sy, uint8_t bit, uint32_t now, uint32_t *buzzer_until)
{
    if (sy->breach & bit) return 0;
    sy->breach |= bit;
    *buzzer_until = now + 2000;
    return 1;
}

static void Shock_Report(System_t *sy, const char *name, uint16_t peak_mg, uint32_t now,
                         uint32_t *buzzer_until, uint32_t *t_lock)
{
    Kv kv;
    if ((now - *t_lock) <= 300UL) return;               /* one shock event per 300 ms */
    *t_lock = now;
    if (sy->shocks < 255) sy->shocks++;
    (void)Latch(sy, BR_SHOCK, now, buzzer_until);
    kv.k = "PEAK"; kv.sv = 0; kv.v = (int32_t)(peak_mg / 10U); kv.dec = 2;
    Log_EventKv(name, &kv, (uint8_t)(peak_mg != 0 ? 1 : 0));
}

/* Sensor fusion (ADXL345 + SW-420), drop analysis and tilt check. Called only while the box is armed.
     ADXL345 over the limit AND SW-420 within FUSION_MS  -> SHOCK_CONFIRMED
     ADXL345 over the limit, SW-420 quiet                -> SHOCK_ACCEL
     SW-420 alone                                        -> VIBRATION_SUSPECT (logged, no breach)           */
static void Motion_Step(System_t *sy, uint32_t now, uint32_t *buzzer_until, uint32_t *t_lock)
{
    uint8_t  sw, hit, drop_len, tilt;
    uint32_t sw_t, hit_t;
    uint16_t hit_pk, drop_pk, deg;
    int32_t  d;

    __disable_irq();
    sw = g_shock_flag;
    sw_t = g_shock_time;
    __enable_irq();
    hit = acc.hit; hit_t = acc.hit_ms; hit_pk = acc.hit_peak;

    if (hit)
    {
        d = (int32_t)(sw_t - hit_t);
        if (d < 0) d = -d;
        if (sw && d <= FUSION_MS)                          /* both sensors agree */
        {
            acc.hit = 0;
            __disable_irq(); g_shock_flag = 0; __enable_irq();
            Shock_Report(sy, "SHOCK_CONFIRMED", hit_pk, now, buzzer_until, t_lock);
        }
        else if ((now - hit_t) > FUSION_MS)                /* SW-420 stayed quiet: accelerometer alone */
        {
            acc.hit = 0;
            Shock_Report(sy, "SHOCK_ACCEL", hit_pk, now, buzzer_until, t_lock);
        }
    }
    else if (sw && (now - sw_t) > FUSION_MS)               /* SW-420 alone: could be plain vibration */
    {
        __disable_irq(); g_shock_flag = 0; __enable_irq();
        if (FUSION_SW420_ALONE_LATCHES || (sy->err & ERR_ADXL))
            Shock_Report(sy, "SHOCK_SWITCH", 0, now, buzzer_until, t_lock);
        else if ((now - sy->t_suspect) > 5000UL)
        {
            sy->t_suspect = now;
            if (sy->suspects < 255) sy->suspects++;
            Log_Event("VIBRATION_SUSPECT");
        }
    }

    if (acc.drop_ready)                                    /* a free fall ended and the impact was measured */
    {
        drop_len = acc.drop_len;
        drop_pk  = acc.drop_peak;
        acc.drop_ready = 0;
        if (Drop_HeightMm(drop_len) >= DROP_MIN_CM * 10)
        {
            Kv kv[3];
            kv[0].k = "HEIGHT_CM"; kv[0].sv = 0; kv[0].v = Drop_HeightMm(drop_len) / 10; kv[0].dec = 0;
            kv[1].k = "FALL_MS";   kv[1].sv = 0; kv[1].v = (int32_t)drop_len * 10;       kv[1].dec = 0;
            kv[2].k = "PEAK";      kv[2].sv = 0; kv[2].v = (int32_t)(drop_pk / 10U);     kv[2].dec = 2;
            if (sy->drops < 255) sy->drops++;
            (void)Latch(sy, BR_SHOCK, now, buzzer_until);
            Log_EventKv("DROP", kv, 3);
        }
    }

    tilt = acc.tilt_flag;
    deg  = acc.tilt_deg;
    if (tilt && sy->act.tilt_deg != 0 && Latch(sy, BR_TILT, now, buzzer_until))
    {
        Kv kv;
        kv.k = "DEG"; kv.sv = 0; kv.v = deg; kv.dec = 0;
        Log_EventKv(deg > 135 ? "INVERTED" : "TILT_EXCEEDED", &kv, 1);
    }
}

static void Do_Seal(System_t *sy)
{
    uint32_t sum = 0;
    uint8_t i;

    sy->act = PROFILES[sy->profile];

    Accel_Calibrate();                                      /* "1 g" for this sensor + resting orientation */
    acc.limit_mg  = (uint16_t)(sy->act.g100 * 10U);
    acc.tilt_lim  = acc.cal_ok ? sy->act.tilt_deg : 0;
    acc.last_poll = millis();
    acc.run = 1;

    /* every running computation starts from zero */
    sy->err = 0;
    sy->tf_init = 0;
    Mkt_Reset(&sy->mkt);  sy->mkt100 = 0;
    Reg_Reset(&sy->reg);  sy->reg_div = 0;  sy->slope_s = 0.0f;  sy->eta_s = 0;  sy->eta_up = 0;
    sy->exc_ms = 0;  sy->area_cs = 0;  sy->in_exc = 0;
    sy->drops = 0;  sy->suspects = 0;  sy->t_suspect = 0;
    sy->t_prev = millis();

    Sample_Sensors(sy);

    for (i = 0; i < 8; i++) { sum += ADC_ReadLDR_Median5(); delay_ms(5); }
    sy->light_base = (uint16_t)(sum / 8);

#if DEMO_MODE
    if (sy->act.demo_above10 != 0)
    {
        sy->act.tmin10 = (int16_t)(sy->t10 - sy->act.demo_below10);
        sy->act.tmax10 = (int16_t)(sy->t10 + sy->act.demo_above10);
        if (sy->act.demo_h_above10 != 0)
        {
            int32_t hl = (int32_t)sy->h10 + sy->act.demo_h_above10;
            sy->act.hmax10 = (int16_t)(hl > 1000 ? 1000 : hl);
        }
    }
#endif

    sy->breach = 0;
    sy->score  = 100;
    sy->worst  = 100;
    sy->shocks = 0;
    sy->peak100 = 0;  sy->peak_max100 = 0;  sy->vib_mg = 0;
    sy->tilt_max = 0;  sy->tilt_active = 0;
    sy->start_ms = millis();
    sy->arm_at   = millis() + ARM_DELAY_MS;

    __disable_irq(); g_shock_flag = 0; g_lid_open_flag = 0; __enable_irq();
    Chain_Begin();                                          /* from here on every log line is hash-chained */
    Log_Sealed(sy);
}

static void Show_Status(const System_t *sy, uint32_t now)
{
    char l0[17], l1[17];
    Str s;
    uint8_t page = (uint8_t)((now / 3000UL) % 4UL);        /* four screens, 3 s each */
    int32_t rate = (int32_t)(sy->slope_s * 60.0f + (sy->slope_s >= 0.0f ? 0.5f : -0.5f));

    Str_Begin(&s, l0, sizeof(l0));
    switch (page)
    {
    case 0:  s_str(&s, "T:"); s_fixed(&s, sy->t10, 1); s_str(&s, "C S:"); s_uint(&s, sy->score); break;
    case 1:  s_str(&s, "MKT:"); s_fixed(&s, sy->mkt100, 2); s_char(&s, 'C'); break;
    case 2:  s_str(&s, "Out:"); s_uint(&s, sy->exc_ms / 1000UL); s_str(&s, "s A:"); s_fixed(&s, (int32_t)(sy->area_cs / 60UL), 2); break;
    default: s_str(&s, "Vib:"); s_uint(&s, sy->vib_mg); s_str(&s, "mg Drp:"); s_uint(&s, sy->drops); break;
    }
    Str_End(&s);

    Str_Begin(&s, l1, sizeof(l1));
    switch (page)
    {
    case 0:
        if (sy->breach)
        {
            s_str(&s, "BREACH:");
            if (sy->breach & BR_TEMP)  s_char(&s, 'T');
            if (sy->breach & BR_HUM)   s_char(&s, 'H');
            if (sy->breach & BR_SHOCK) s_char(&s, 'S');
            if (sy->breach & BR_LIGHT) s_char(&s, 'L');
            if (sy->breach & BR_OPEN)  s_char(&s, 'O');
            if (sy->breach & BR_TILT)  s_char(&s, 'I');
        }
        else if (sy->in_exc)
        {
            s_str(&s, "WARN out:"); s_uint(&s, sy->exc_ms / 1000UL); s_char(&s, 's');
        }
        else if (sy->eta_s != 0)
        {
            s_str(&s, "Limit in ");
            if (sy->eta_s >= 60) { s_uint(&s, sy->eta_s / 60U); s_char(&s, 'm'); }
            s_uint(&s, sy->eta_s % 60U); s_char(&s, 's');
        }
        else s_str(&s, "Status: OK");
        break;
    case 1:
        s_str(&s, "dT:"); if (rate >= 0) s_char(&s, '+'); s_fixed(&s, rate, 2); s_str(&s, "C/min");
        break;
    case 2:
        s_str(&s, "Pk:"); s_fixed(&s, sy->peak100, 2); s_str(&s, "g Tl:"); s_uint(&s, sy->tilt_deg);
        break;
    default:
        s_str(&s, "Log#"); s_uint(&s, g_seq); s_str(&s, " Err:"); s_uint(&s, sy->err);
        break;
    }
    Str_End(&s);

    LCD_Print(0, l0);
    LCD_Print(1, l1);
}

/* ========================================================================
   13. MAIN
   ======================================================================== */
static void __attribute__((noinline)) app_main(void)
{
    static System_t S;
    Btn bA = {12, 1, 1, 0};
    Btn bB = {13, 1, 1, 0};

    uint32_t t_sample = 0, t_shock_lock = 0, buzzer_until = 0;
    uint32_t t_both = 0, lid_t = 0, t_lcd_retry = 0;
    uint8_t  both_active = 0, lid_seen = 0;

    Time_Init();
    Bench_Init();
    I2C1_Init();
    LCD_Init();
    Outputs_Init();
    RGB_Init();
    ADC_Init_LDR();
    Buttons_Init();
    Events_Init();
    USART2_Init();
    USART1_Init();

    /* ---------------- self-test screen ---------------- */
    {
        uint8_t ok_sht, ok_adx, ok_rtc, ok_crc;
        int16_t t, h;
        char l1[17];
        Str s;

        LCD_Print(0, "Shipment Monitor");
        LCD_Print(1, "Self-test...");

        ok_crc = Crc_Init();
        ok_rtc = RTC_Init();
        ok_adx = ADXL345_Init();
        ok_sht = SHT31_Read(&t, &h);

        Str_Begin(&s, l1, sizeof(l1));
        s_str(&s, "S:"); s_char(&s, ok_sht ? 'Y' : 'N');
        s_str(&s, " A:"); s_char(&s, ok_adx ? 'Y' : 'N');
        s_str(&s, " R:"); s_char(&s, ok_rtc ? 'Y' : 'N');
        Str_End(&s);
        LCD_Print(1, l1);

        BT_Print("\r\nSHIPMENT MONITOR READY\r\n");
        BT_Print(ok_sht ? "SHT31 OK\r\n" : "SHT31 FAIL\r\n");
        BT_Print(ok_adx ? "ADXL345 OK\r\n" : "ADXL345 FAIL\r\n");
        BT_Print(ok_rtc ? "RTC OK\r\n" : "RTC FAIL\r\n");
        BT_Print(ok_crc ? "CRC HW OK\r\n" : "CRC SOFTWARE\r\n");

        RGB_Set(255, 0, 0);   delay_ms(300);
        RGB_Set(0, 255, 0);   delay_ms(300);
        RGB_Set(0, 0, 255);   delay_ms(300);
        RGB_Set(0, 0, 0);
        Buzzer_Set(1); delay_ms(100); Buzzer_Set(0);
        delay_ms(1500);
    }

    S.state = ST_MENU;
    S.profile = 0;
    LCD_Clear();
    Show_Menu(&S);

    for (;;)
    {
        uint32_t now = millis();
        uint8_t pressA, pressB, lid_open;

        Background_Task();
        pressA = Btn_Pressed(&bA);
        pressB = Btn_Pressed(&bB);

        __disable_irq();
        lid_open = g_lid_open_flag; g_lid_open_flag = 0;
        __enable_irq();

        if ((now - t_lcd_retry) > 10000) { t_lcd_retry = now; g_lcd_fail = 0; }

        switch (S.state)
        {
        /* ------------------------------------------------ choose profile */
        case ST_MENU:
            if (pressA)
            {
                S.profile = (uint8_t)((S.profile + 1) % 3);
                Show_Menu(&S);
            }
            if (pressB)
            {
                LCD_Print(0, PROFILES[S.profile].name);
                LCD_Print(1, "Close the lid...");
                lid_seen = 0;
                S.state = ST_WAIT_SEAL;
            }
            break;

        /* ------------------------------------------------ wait for lid */
        case ST_WAIT_SEAL:
            if (pressA)
            {
                Show_Menu(&S);
                S.state = ST_MENU;
                break;
            }
            if (Lid_IsClosed())
            {
                if (!lid_seen) { lid_seen = 1; lid_t = now; }
                else if ((now - lid_t) > 500)
                {
                    LCD_Print(1, "Sealing...");
                    Do_Seal(&S);
                    t_sample = millis() - SAMPLE_MS;       /* sample straight away */
                    buzzer_until = 0;
                    both_active = 0;
                    S.state = ST_IN_TRANSIT;
                }
            }
            else lid_seen = 0;
            break;

        /* ------------------------------------------------ monitoring */
        case ST_IN_TRANSIT:
        {
            uint8_t armed = (now >= S.arm_at);

            if (armed)
            {
                if (lid_open && Latch(&S, BR_OPEN, now, &buzzer_until))
                    Log_Event("UNAUTHORIZED_OPEN");

                Motion_Step(&S, now, &buzzer_until, &t_shock_lock);   /* fusion, drop, tilt */
            }
            else
            {
                acc.hit = 0;                                  /* ignore everything during the arming delay */
                acc.drop_ready = 0;
                __disable_irq(); g_shock_flag = 0; __enable_irq();
            }

            if ((now - t_sample) >= SAMPLE_MS)
            {
                uint8_t cs;
                t_sample = now;
                Sample_Sensors(&S);
                cs = Compute_Step(&S, now, armed);

                if (armed)
                {
#if EXCURSION_BUDGET
                    if (cs & 1) Log_Event("TEMP_WARNING");
                    if ((cs & 0x0E) && Latch(&S, BR_TEMP, now, &buzzer_until))
                    {
                        Kv kv;
                        kv.k = "WHY"; kv.v = 0; kv.dec = 0;
                        kv.sv = (cs & 8) ? "HARD" : ((cs & 2) ? "TIME" : "AREA");
                        Log_EventKv("TEMP_EXCURSION", &kv, 1);
                    }
#else
                    (void)cs;
                    if ((S.t10 > S.act.tmax10 || S.t10 < S.act.tmin10) && !(S.err & ERR_SHT))
                        if (Latch(&S, BR_TEMP, now, &buzzer_until)) Log_Event("TEMP_EXCURSION");
#endif

                    if (S.h10 > S.act.hmax10 && !(S.err & ERR_SHT))
                        if (Latch(&S, BR_HUM, now, &buzzer_until)) Log_Event("HUMIDITY_EXCURSION");

                    if (S.act.light_check && S.ldr > (uint16_t)(S.light_base + LIGHT_DELTA))
                        if (Latch(&S, BR_LIGHT, now, &buzzer_until)) Log_Event("LIGHT_TAMPER");
                }

                S.score = Score_Compute(&S);
                if (armed && S.score < S.worst) S.worst = S.score;

                /* corrective action: cooling fan near the upper limit (with hysteresis) */
                if (!(S.err & ERR_SHT))
                {
                    /* on near the upper limit, or earlier when the trend says the limit is about to be reached */
                    uint8_t early = (uint8_t)(S.eta_s != 0 && S.eta_up && S.eta_s < PRED_FAN_S);
                    if (S.t10 > S.act.tmax10 - 10 || early)  Relay_Set(1);
                    else if (S.t10 < S.act.tmax10 - 25)      Relay_Set(0);
                }

                if (S.breach)           RGB_Set(255, 0, 0);
                else if (S.score < 80 || S.in_exc || (S.eta_s != 0 && S.eta_s < PRED_FAN_S)) RGB_Set(255, 100, 0);
                else                    RGB_Set(0, 255, 0);

                Show_Status(&S, now);
                Log_Sample(&S);
            }

            Buzzer_Set(now < buzzer_until);

            /* delivery: hold BOTH buttons for 2 s */
            if (((GPIOB->IDR >> 12) & 3U) == 0)
            {
                if (!both_active) { both_active = 1; t_both = now; }
                else if ((now - t_both) > 2000)
                {
                    Log_Event("DELIVERED");
                    Log_Summary(&S);
                    Log_Bench();
                    g_chain_on = 0;
                    acc.run = 0;
                    Relay_Set(0);
                    Buzzer_Set(0);
                    RGB_Set(0, 0, 255);
                    LCD_Print(0, "DELIVERED");
                    LCD_Print(1, S.breach ? "Breach recorded" : "No breach");
                    S.state = ST_DELIVERED;
                }
            }
            else both_active = 0;
            break;
        }

        /* ------------------------------------------------ finished */
        case ST_DELIVERED:
            if (pressB)                          /* new shipment */
            {
                RGB_Set(0, 0, 0);
                S.profile = 0;
                S.breach = 0;
                LCD_Clear();
                Show_Menu(&S);
                S.state = ST_MENU;
            }
            break;
        }
    }
}

int main(void)
{
    SCB->CPACR |= (0xFU << 20);          /* switch the FPU on BEFORE any float instruction can run */
    __DSB();
    __ISB();
    app_main();                          /* never returns */
    return 0;
}

# Timer Setup Reference

All timer configs assume **STM32CubeMX**. Values shown for a **72 MHz** APB timer clock — adjust prescaler for your clock.\
**Note:** For newer versions of CubeIDE, CubeMX is separate and needs to be imported into CubeIDE\
**Note:** Remember to always set your prescalar less by 1!

---


## Motor PWM Timer

| Parameter | Value | Notes |
|-----------|-------|-------|
| Mode | PWM Generation CHx | One channel per motor |
| Prescaler | `(72_000_000 / 20_000) - 1 = 3599` | → 20 kHz switching freq |
| Counter Period (ARR) | `999` | Set `ROBOT_MOTOR_PWM_MAX 1000` |
| Pulse | 0 | CubeMX default, library sets it |

**Tip:** Use the same timer for multiple motors (different channels) — they share the same frequency.\
**For beginners:** The counter period is the period is the cycles to enable one interrupt.

---

## Encoder Timer

| Parameter | Value | Notes |
|-----------|-------|-------|
| Combined Channels | Encoder Mode | NOT PWM |
| Prescaler | 0 | Count every edge |
| Counter Period | 0xFFFF | 16-bit wrap |
| Input Filter | 4–8 | Reduces noise |

For 32-bit timers (TIM2, TIM5 on F4): set ARR to `0xFFFFFFFF`.
**For beginners:** The encode has prescalar 0 that when used turns into 1, so that the timers frequency is directly used to maximize accuracy.

---

## Servo Timer (50 Hz)

| Timer Clock | Prescaler | ARR | Resolution |
|-------------|-----------|-----|------------|
| 72 MHz | 71 | 19999 | 1 µs |
| 84 MHz | 83 | 19999 | 1 µs |
| 168 MHz | 167 | 19999 | 1 µs |
| 180 MHz | 179 | 19999 | 1 µs |

Formula: `PSC = (TimerClock / 1_000_000) - 1`, ARR = 19999 (gives 20 ms period = 50 Hz).

Pass `timer_freq_hz = 1_000_000` to `Servo_Init()` for all cases above.

---

## Control Loop Timer (e.g. TIM6)

Basic 10 ms ( 1 ms for critical applications ) tick for the PID loop:

| Parameter | Value | Notes |
|-----------|-------|-------|
| Prescaler | 71 | → 1 MHz |
| Counter Period | 9999 | → 10 ms interrupt |
| NVIC | TIM6 global interrupt | Enable in CubeMX |

For beginners: 
For 1 ms interrupt, set counter period to 999 in CubeMX

Wire to `HAL_TIM_PeriodElapsedCallback`, check `htim->Instance == TIM6`.
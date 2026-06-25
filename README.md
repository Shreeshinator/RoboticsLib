# STM32-robotics

A clean, zero-dependency robotics library for **STM32 HAL** projects.  
Drop the `Inc/` and `Src/` folders into any STM32CubeIDE / Makefile project — no framework, no RTOS required.
**Note:** By be sure to inspect how many and which timer to use for each peripheral.\
Also, this library assumes a motor driver with separate PWM, IN1, IN2 channels for each motor


---

## Modules

| Module | File | What it does |
|--------|------|--------------|
| **Motor** | `motor.h/.c` | PWM DC motor with direction GPIO (L298N, DRV8833, …) |
| **Encoder** | `encoder.h/.c` | Quadrature encoder via TIM encoder mode |
| **PID** | `pid.h/.c` | Discrete PID with anti-windup & derivative filter |
| **Servo** | `servo.h/.c` | RC servo (50 Hz PWM, angle or µs control) |
| **Kinematics** | `kinematics.h/.c` | Differential drive & mecanum inverse kinematics + odometry |

---

## Quick Start

### 1. Copy library files

```
your_project/
├── Core/
│   ├── Inc/
│   │   └── (your existing headers)
│   └── Src/
│       └── (your existing sources)
├── Robotics/          ← add this
│   ├── Inc/
│   │   ├── robotics.h
│   │   ├── robot_config.h
│   │   ├── stm32_hal_include.h
│   │   ├── motor.h
│   │   ├── encoder.h
│   │   ├── pid.h
│   │   ├── servo.h
│   │   └── kinematics.h
│   └── Src/
│       ├── motor.c
│       ├── encoder.c
│       ├── pid.c
│       ├── servo.c
│       └── kinematics.c
```

### 2. Add include path

**STM32CubeIDE** → Project → Properties → C/C++ Build → Settings → Tool Settings → MCU GCC Compiler → Include paths → add `../Robotics/Inc`

**Makefile** projects — add to `C_INCLUDES`:
```makefile
C_INCLUDES += -IRobotics/Inc
C_SOURCES  += $(wildcard Robotics/Src/*.c)
```

### 3. Configure `robot_config.h`

Open `Robotics/Inc/robot_config.h` and set `ROBOT_HAL_INCLUDE` to match your MCU:

```c
// STM32F4
#define ROBOT_HAL_INCLUDE "stm32f4xx_hal.h"

// STM32G0
#define ROBOT_HAL_INCLUDE "stm32g0xx_hal.h"

// STM32H7
#define ROBOT_HAL_INCLUDE "stm32h7xx_hal.h"

// CubeMX default (works for all families)
#define ROBOT_HAL_INCLUDE "main.h"
```

### 4. Include in your code

```c
#include "robotics.h"   // pulls everything in
```

---

## Module Usage

### Motor

```c
Motor_t left, right;

Motor_Init(&left,  &htim2, TIM_CHANNEL_1,
           GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1);
Motor_Init(&right, &htim2, TIM_CHANNEL_2,
           GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_3);

Motor_SetInverted(&right, true);   // fix reversed wiring
Motor_SetSpeed(&left,  75.0f);     // 75 % forward
Motor_SetSpeed(&right, 75.0f);
Motor_Brake(&left);                // active brake
Motor_Coast(&left);                // freewheel
```

**CubeMX Timer setup:**
- Mode → PWM Generation CHx
- Prescaler + ARR → set for your desired PWM frequency (typically 10–20 kHz)
- `ROBOT_MOTOR_PWM_MAX` in `robot_config.h` must equal your ARR value

---

### Encoder

```c
Encoder_t enc;
// 1440 counts/rev, updated every 10 ms
Encoder_Init(&enc, &htim3, 1440, 0.01f);

// Call every 10 ms (e.g. in TIM period elapsed callback)
Encoder_Update(&enc);

float rpm   = Encoder_GetRPM(&enc);
float v_mps = Encoder_GetLinearVelocity(&enc, 0.033f); // 33 mm wheel
int32_t total = Encoder_GetTotalTicks(&enc);
```

**CubeMX Timer setup:**
- Combined Channels → Encoder Mode
- Prescaler → 0
- Counter Period → 0xFFFF (16-bit) or 0xFFFFFFFF (32-bit timer)

---

### PID

```c
PID_t pid;
PID_Init(&pid, 1.5f, 0.2f, 0.05f, 0.01f);  // Kp, Ki, Kd, dt
PID_SetOutputLimits(&pid, -100.0f, 100.0f);
PID_SetFilterCoeff(&pid, 50.0f);            // derivative LP filter

// Every dt seconds:
float output = PID_Compute(&pid, setpoint, measurement);
```

---

### Servo

```c
Servo_t servo;
// Timer at 1 MHz (PSC = 71 for 72 MHz clock), ARR = 19999
Servo_Init(&servo, &htim1, TIM_CHANNEL_1, 1000000u);

Servo_SetAngle(&servo, 90.0f);       // centre
Servo_SetAngle(&servo, 0.0f);        // min
Servo_SetPulse(&servo, 1500.0f);     // 1500 µs directly
```

**CubeMX Timer setup (50 Hz, 1 µs resolution):**

| Parameter | Value |
|-----------|-------|
| Prescaler | `(TimerClock_Hz / 1_000_000) - 1` |
| Counter Period (ARR) | `19999` |
| Channel | PWM Generation CHx |

---

### Kinematics — Differential Drive

```c
DiffDrive_t robot;
DiffDrive_Init(&robot, 0.033f, 0.160f);   // wheel radius, wheel base
DiffDrive_SetMaxSpeed(&robot, 0.5f);       // 0.5 m/s = 100%

float l_pct, r_pct;
DiffDrive_VelToWheels(&robot, 0.3f, 0.5f, &l_pct, &r_pct);
Motor_SetSpeed(&left,  l_pct);
Motor_SetSpeed(&right, r_pct);

// Odometry (call every dt):
DiffDrive_UpdateOdometry(&robot, v_left_mps, v_right_mps, 0.01f);
float x = robot.x, y = robot.y, heading = robot.theta;
```

### Kinematics — Mecanum Drive

```c
Mecanum_t mc;
Mecanum_Init(&mc, 0.050f, 0.15f, 0.12f);  // r, lx, ly

MecanumSpeeds_t spd;
Mecanum_VelToWheels(&mc, 0.3f, 0.2f, 0.4f, &spd);  // vx, vy, omega
Motor_SetSpeed(&fl_motor, spd.fl);
Motor_SetSpeed(&fr_motor, spd.fr);
Motor_SetSpeed(&rl_motor, spd.rl);
Motor_SetSpeed(&rr_motor, spd.rr);
```

---

## Examples

| Example | Path | Description |
|---------|------|-------------|
| Differential drive | `examples/differential_drive/` | Closed-loop velocity control with odometry |
| Servo arm | `examples/servo_arm/` | 3-DOF servo arm with homing |

---

## Compatibility

Tested on:

| Family | Examples |
|--------|---------|
| STM32F1 | F103C8 (Blue Pill) |
| STM32F4 | F401, F411, F446 |
| STM32G0 | G071, G0B1 |
| STM32H7 | H743 |
| STM32L4 | L432, L476 |

Any STM32 family with HAL and a timer peripheral should work.

---

## Repository Layout

```
stm32-robotics/
├── Inc/                  # All header files
│   ├── robotics.h        # Master include
│   ├── robot_config.h    # User configuration
│   ├── stm32_hal_include.h
│   ├── motor.h
│   ├── encoder.h
│   ├── pid.h
│   ├── servo.h
│   └── kinematics.h
├── Src/                  # Implementations
│   ├── motor.c
│   ├── encoder.c
│   ├── pid.c
│   ├── servo.c
│   └── kinematics.c
├── examples/
│   ├── differential_drive/
│   └── servo_arm/
├── docs/
│   └── timer_setup.md
├── .gitignore
├── LICENSE
└── README.md
```

---

## License

MIT — see [LICENSE](LICENSE).

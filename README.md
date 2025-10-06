# ⚙️ STM32F4 Bare-Metal Drivers

Low-level peripheral drivers for **STM32F407** written entirely in **C** — no HAL, no CubeMX, just pure register-level control.  
Developed to demonstrate understanding of **bare-metal embedded systems programming** and peripheral-level architecture.

---

## 🧩 Implemented Drivers
- **RCC** – Clock control and peripheral enable
- **GPIO** – Pin configuration and alternate functions
- **SysTick** – Millisecond delay and timing
- **EXTI** – External interrupt handling
- **I2C** – Blocking I2C master communication
- **SPI** – Full-duplex master communication

---

## 🧠 Example Projects
Each project under `/Examples` demonstrates a peripheral in bare-metal operation:

| Example | Description |
|----------|--------------|
| `GPIO_LED_Toggle` | Blinks LEDs using SysTick-based delay. |
| `EXTI_Button_LED` | Toggles an LED when the user button is pressed via EXTI interrupt. |
| `I2C_DeviceID_Read` | Reads device ID (e.g., MPU6050 WHO_AM_I) over I2C. |
| `SPI_DeviceID_Read` | Reads device version register over SPI. |

---

## 🗂 Directory Overview

**Project Structure**

```
stm32f4-baremetal-drivers/
│
├── Core/                 # Startup and system initialization
│
├── Drivers/              # RCC, GPIO, EXTI, I2C, SPI drivers
│
├── Examples/             # Example projects using drivers
│   ├── GPIO_LED_Toggle/
│   ├── EXTI_Button_LED/
│   ├── I2C_DeviceID_Read/
│   └── SPI_DeviceID_Read/
│
└── README.md
```

## 🔧 Build Info
- **MCU:** STM32F407VG  
- **Clock:** 16 MHz (HSI)  
- **Toolchain:** STM32CubeIDE / Keil / arm-none-eabi-gcc  
- **Language Standard:** C99  
- **Core:** ARM Cortex-M4  

---

## 👨‍💻 Author
**Hasan Erol**  
*Embedded Software Engineer Intern*  
Bare-metal firmware • STM32 • C • Peripheral Drivers  
🔗 [github.com/hasanerol](https://github.com/hasanerol)

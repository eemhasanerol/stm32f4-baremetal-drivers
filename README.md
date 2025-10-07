# ⚙️ STM32F4 Bare-Metal Drivers

Low-level peripheral drivers for STM32F407, written in pure C — no HAL, no CubeMX, full register-level control.

---

## 🧩 Implemented Drivers
- **RCC** – Clock configuration and peripheral enabling  
- **GPIO** – General-purpose input/output and alternate functions  
- **SysTick** – Millisecond tick timer and delay management  
- **EXTI** – External interrupt configuration and handling  
- **I2C** – I2C master communication interface  
- **SPI** – SPI master communication interface  

---

## 🧠 Example Projects
Each project under `/Examples` demonstrates a peripheral in bare-metal operation:

| Example | Description |
|----------|--------------|
| `GPIO_LED_Toggle` | Blinks LEDs using SysTick-based delay. |
| `EXTI_Button_LED` | Toggles an LED when the user button is pressed via EXTI interrupt. |
| `I2C_DeviceID_Read` | Reads device ID (e.g., MPU6050 WHO_AM_I). |
| `SPI_DeviceID_Read` | Reads device version register. |

---

## 🗂 Directory Overview

**Project Structure**

```
stm32f4-baremetal-drivers/
│
├── Core/                 # Startup code and SysTick timer
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
- **Toolchain:** STM32CubeIDE  
- **Core:** ARM Cortex-M4  

---

## 👤 Author  
**Hasan Erol**  
*Embedded Software Engineer Intern*  
Bare-metal firmware • STM32 • C • Peripheral Drivers  
📧 **eem.hasanerol@gmail.com**  
🔗 [github.com/eemhasanerol](https://github.com/eemhasanerol)

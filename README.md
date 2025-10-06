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
Each project under `/Examples` demonstrates a specific peripheral:

| Example | Description |
|----------|--------------|
| `GPIO_LED_Toggle` | Toggles LEDs on PD13/PD15 using SysTick delay |
| `EXTI_Button_LED` | Toggles LED via external interrupt on user button |
| `I2C_DeviceID_Read` | Reads WHO_AM_I register (e.g., MPU6050) over I2C1 |
| `SPI_DeviceID_Read` | Reads VersionReg (0x37) from MFRC522 over SPI1; LED PD13 lights if success, remains off if fail |

---

## 🗂 Directory Overview

**Project Structure**

---

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

# Bare-Metal STM32F4 Peripheral Drivers

### 🧠 Overview
This repository provides a collection of **bare-metal peripheral drivers** for the **STM32F407** microcontroller, written in pure **C (no HAL, no CMSIS)**.  
Each driver is implemented with a clean modular structure and minimal dependencies, designed for **low-level embedded system development** and educational purposes.

---

### ⚙️ Supported Drivers
| Peripheral | Description |
|-------------|-------------|
| **GPIO**    | General-Purpose I/O configuration, digital input/output, and EXTI interrupt control |
| **RCC**     | Reset and Clock Control — provides system, AHB/APB clock configuration and frequency calculation |
| **SPI**     | Serial Peripheral Interface master/slave driver with blocking and interrupt-based APIs |
| **I2C**     | Inter-Integrated Circuit communication (master mode) with register-level control |
| **USART**   | Universal Synchronous/Asynchronous Receiver/Transmitter for serial communication |
| **SysTick** | System timer for time base, delays, and periodic tick generation |

Each module is fully independent and includes its own header (`.h`) and source (`.c`) files.

---

### 📁 Directory Structure

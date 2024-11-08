# AEROFOX-H7 Flight Controller

The AEROFOX-H7 is a flight controller produced by AEROFOX(http://aerofox.cn)

<img src="AEROFOX-H7_IMG.png" alt="" width="400">

## Features
    Processor
        STM32H743

    Sensors
        ADIS16470 (appears in the advanced version)
        ICM45686 (appears in the advanced version)
        ICM42688
        QMC5883L
        SPL06-001

    Power
        2S-12S (MAX60V) Lipo input voltage
        5V BEC for system power supply( 5V peripheral current limit 1.2A)
        5V/12V BEC for VTX( Current limit 2.5A, need strong heat dissipation)
        Dual power automatic switching and condition monitoring
        
    Interfaces
        16x PWM output
        7x UARTs for RC, TELEM, GPS and other peripherals
        2x I2C ports for external compass, airspeed, baro
        2x CAN port
        4x Relay output
        4x ADC input

    FPC connector
        The conneimgctor includes an SPI, an I2C, an PWM IMU heating control pin. Firmware
        already includes drivers for ADIS16470, ICM45686, IMU heating, RM3100. It is
        automatically identify after installation。

## Pinout
<img src="AEROFOX-H7_pinout.png" alt="" width="800">

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> UART7
- SERIAL2 -> UART4
- SERIAL3 -> UART5
- SERIAL4 -> USART2
- SERIAL5 -> USART1 GPS
- SERIAL6 -> UART8 TELEM
- SERIAL7 -> USART3 RCIN

## RC Input

No special interface for RCIN is designed because any SERIAL interface can be used as
an RC input.

## PWM Output

The AEROFOXH7 support up to 16PWM outputs. All pins support Dshot. Outputs 1-8
supports STM32 native encoder mode and can input 4 groups of AB encoder signals simultaneously.

- PWM1 TIM2_CH1 encoder mode
- PWM2 TIM2_CH2 encoder mode
- PWM3 TIM1_CH1 encoder mode
- PWM4 TIM1_CH2 encoder mode
- PWM5 TIM4_CH1 encoder mode
- PWM6 TIM4_CH2 encoder mode
- PWM7 TIM3_CH1 encoder mode
- PWM8 TIM3_CH2 encoder mode
- PWM9 TIM15_CH1
- PWM10 TIM15_CH2
- PWM11 TIM14_CH1
- PWM12 TIM13_CH1
- PWM13 TIM8_CH4
- PWM14 TIM8_CH3
- PWM15 TIM12_CH2
- PWM16 TIM12_CH1

## Battery Monitoring

The board has a builting voltage and current sensor. The voltage sensor can handle up
to 12S LiPo batteries.

### The power A is onboard voltage sensor
- BATT_MONITOR 4
- BATT_VOLT_PIN 19
- BATT_CURR_PIN 9
- BATT_VOLT_MULT 21
- BATT_AMP_PERVL 60

### The power B is external PMU input
- BATT_MONITOR 4
- BATT_VOLT_PIN 10
- BATT_CURR_PIN 11
- BATT_VOLT_MULT 34
- BATT_AMP_PERVLT 60

## Compass

A 5883L compass is installed inside the H7 flight control. When high current devices such
as ESC and BEC are installed under the flight control board, the on-board compass
needs to be disabled.

## Loading Firmware
The board comes pre-installed with an ArduPilot compatible bootloader, allowing the
loading of *.apj firmware files with any ArduPilot compatible ground station. The
hardware also supports the PX4 Betaflight INAV firmware, which needs to be changed with STlink.


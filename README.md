![WE Logo](assets/WE_Logo_small_t.png)

# WE Sensors SDK for STM32

[Würth Elektronik](https://www.we-online.com/wco) offers a range of [sensors](https://www.we-online.de/katalog/en/wco/sensors) including temperature, pressure, humidity and acceleration sensor. In order to enable quick prototyping and evaluation of these sensors, Würth Elektronik offers libraries for the [STM32 platform](https://www.st.com/).

The example projects contained in this repository have been created using [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html), but other toolchains can be used if desired.

# Repository structure

| Directory | Description |
| --- | --- |
| examples | Contains the example code for basic and advanced usage of each sensor. The example code is placed in subdirectories named WSEN_XXXX... where XXXX corresponds to the sensor type. Examples for advanced functionalities have a suffix indicating what functionality is being demonstrated in the example. |
| platform | Contains platform-specific (here: STM32) implementations of functions called by the Sensors SDK. These are basically functions performing low-level communication with the sensor (i.e. read/write registers via I2C or SPI). |
| SensorsSDK | Platform-independent application programming interface providing access to sensor functionality. The directory contains general definitions in "WeSensorsSDK.h" and drivers for each sensor type in subdirectories prefixed with the sensor name (i.e. WSEN_XXXX where XXXX corresponds to the sensor type). |
| STM32G031xx | Example projects for the STM32G031xx MCU. |
| STM32L432xx | Example projects for the STM32L432xx MCU. |

# Example structure

Each example in the "examples" directory consists of a header and a source file. The example code is separated from the STM32CubeIDE projects and contains only an initialization function that needs to be called after HAL initialization and a "loop" function that needs to be called in the application's main loop (infinite). In the case of "WSEN_HIDS" for example, the functions are called WE_hidsExampleInit() and WE_hidsExampleLoop().

There are example projects for the STM32G031xx and STM32L432xx MCUs. The examples have been created using STM32CubeIDE and contain the hardware configuration file (.ioc) required for the example as well as the generated code. In the generated main.c, the header of the example is included and the corresponding initialization and loop functions are called.

# List of examples

| Example name | Description |
| --- | --- |
| WSEN_HIDS | Basic usage of the HIDS humidity sensor connected via I2C. |
| WSEN_HIDS_SPI | Basic usage of the HIDS humidity sensor connected via SPI. |
| WSEN_ISDS | Basic usage of the ISDS 6-axis accelerometer/gyroscope connected via I2C. |
| WSEN_ISDS_ACTIVITY_INACTIVITY | Activity/inactivity detection example for the ISDS 6-axis accelerometer/gyroscope demonstrating usage of the inactivity state interrupt. |
| WSEN_ISDS_FIFO | Example for the ISDS 6-axis accelerometer/gyroscope demonstrating usage of the FIFO buffer (FIFO mode, continuous-to-FIFO mode, bypass-to-continuous mode and continuous mode) |
| WSEN_ISDS_FIFO_ADVANCED | Example for the ISDS 6-axis accelerometer/gyroscope demonstrating advanced usage of the FIFO buffer (timestamps, temperature, high data only, patterns, DEN triggers) |
| WSEN_ISDS_FREE_FALL | Example for the ISDS 6-axis accelerometer/gyroscope demonstrating usage of the sensor's free-fall detection functionality. |
| WSEN_ISDS_ORIENTATION | Example for the ISDS 6-axis accelerometer/gyroscope demonstrating the sensor's 6D and 4D (e.g. portrait/landscape) orientation detection functionality. |
| WSEN_ISDS_SELF_TEST | Example for the ISDS 6-axis accelerometer/gyroscope showing how to execute the sensor's self test procedure. |
| WSEN_ISDS_SPI | Basic usage of the ISDS 6-axis accelerometer/gyroscope connected via SPI. |
| WSEN_ISDS_TAP | Example for the ISDS 6-axis accelerometer/gyroscope demonstrating usage of the sensor's tap functionality (single and double tap). |
| WSEN_ISDS_TILT | Example for the ISDS 6-axis accelerometer/gyroscope demonstrating the sensor's relative tilt detection functionality. |
| WSEN_ISDS_WAKE_UP | Example for the ISDS 6-axis accelerometer/gyroscope demonstrating usage of the wake-up interrupt. |
| WSEN_ITDS | Basic usage of the ITDS accelerometer connected via I2C. |
| WSEN_ITDS_ACTIVITY_INACTIVITY | Activity/inactivity detection example for the ITDS accelerometer demonstrating usage of the sleep change and sleep status interrupt as well as stationary (motion) detection. |
| WSEN_ITDS_FIFO | Example for the ITDS accelerometer demonstrating usage of the FIFO buffer (FIFO mode, continuous-to-FIFO mode, bypass-to-continuous mode and continuous mode) |
| WSEN_ITDS_FREE_FALL | Example for the ITDS accelerometer demonstrating usage of the sensor's free-fall detection functionality. |
| WSEN_ITDS_ORIENTATION | Example for the ITDS accelerometer demonstrating the sensor's 6D and 4D (e.g. portrait/landscape) orientation detection functionality. |
| WSEN_ITDS_SELF_TEST | Example for the ITDS accelerometer showing how to execute the sensor's self test procedure. |
| WSEN_ITDS_SINGLE_DATA_CONVERSION | Example for the ITDS accelerometer showing how to trigger single measurements using digital IOs or by writing to a register. |
| WSEN_ITDS_SPI | Basic usage of the ITDS accelerometer connected via SPI. |
| WSEN_ITDS_TAP | Example for the ITDS accelerometer demonstrating usage of the sensor's tap functionality (single and double tap). |
| WSEN_ITDS_WAKE_UP | Example for the ITDS accelerometer demonstrating usage of the wake-up interrupt. |
| WSEN_MULTI_SENSOR | Example for using multiple sensors connected via I2C simultaneously (HIDS, ITDS and TIDS) |
| WSEN_PADS | Basic usage of the PADS absolute pressure sensor connected via I2C. |
| WSEN_PADS_ADVANCED | Advanced usage of the PADS absolute pressure sensor connected via I2C (data-ready interrupt, AUTOZERO mode, usage of the FIFO buffer). |
| WSEN_PDUS | Basic usage of the PDUS differential pressure sensor connected via I2C. |
| WSEN_TIDS | Basic usage of the TIDS temperature sensor connected via I2C. |


# First steps

First steps using STM32CubeIDE:

- Clone this repository into your workspace directory
- Start STM32CubeIDE and select your workspace directory
- Close the startup screen ("Information Center") if necessary
- Run action "Import projects" in "Project Explorer" on left hand side
- Select "Existing Projects into Workspace" under "General"
- Pick the repository directory as "Root directory"
- Select the projects to be imported (take care to import the projects for the correct MCU) and confirm
- Open main file of the project to be run (Core/Src/main.c)
- Connect sensor to STM32 (consult sensor documentation and STM32CubeIDE's device configuration tool for details on the pin connection)
- Connect sensor to PC via USB
- Connect terminal program to STM32 (Baud rate 115200)
- Compile, flash and run either via "Debug (F5)" or "Run (Ctrl+F5)"

# Erroneous-Vector-Assessment
EVA pairs a phone camera with IMU to provide real time, high fidelity feedback to athletes performing potentially injury inducing repetitive actions. At the moment, our team is targeting weightlifting, specifically the incline bench press motion.

Currently:
With Python 3.8 installed, run python3.8 pose_detection in terminal. Consider doing pip3 install on any packages which are not installed.

***

# DEMO
 Note, have the android code (EVA/main folder) installed and ready to execute on an android emulator or android phone. Laslty, change any noted hardcoded addresses within wifi_server.py. To run, execute wifi_server.py and press the respective start and stop connection buttons on the app interface to start or end streaming.

# Analyzer

The Analyzer is a Python class (within utils.py) which contains main functions for analysis, namely update, error_check, and plot_segmentation.

## update(self, image, imu_readings)

The update function is responsible for all activity segmentation processes, intaking a new tuple of image and IMU data which have been time stamp synchronized. This function maintains the global state for parsing input data, updating which part of the exercise (or if moving) the user is currently conducting. This function also book keeps the skeletonized data for future reference.

## error_check(self, label, window)

The error_check function is responsible for analyzing a former window of data for incorrect motions. It takes the labeled frame window result from update to gather which indeces from the historical data to analyze. Error checking returns one of 6 codes, defined below.

1. self.FLEXION_BOTTOM_ERROR_CODE = 1 #'caution, at the bottom of your rep, bend your arms to 90 degrees'
2. self.FLEXION_TOP_ERROR_CODE = 2 #'caution, at the top of your rep, do not lock your elbows, allow for a slight bend'
3. self.TILT_DOWN_ERROR_CODE = 3 #'caution, bar tilted while going down'
4. self.TILT_UP_ERROR_CODE = 4 #'caution, bar tilted while going up'
5. self.INSTABILITY_ERROR_CODE = 5 #'caution, the motion is shaky'
6. self.ROTATOIN_ERROR_CODE = 6 #'caution, your wrist might be rotating'

The specifications for each error are elaborated in our final paper submission.

## plot_segmentation(self)

The plot_segmentation function is responsible for plotting the highlighted data over the standardized y values to acknowledge activity segmentation. This function is largely used as a sanity check to confirm proper threshold tuning as well as demonstration purposes for novel users. This final output also clarifies which repetitions a user struggled with or made an error. The highlights are as follows. 

    plt.plot(np.arange(len(self.all_ys)), self.all_ys)
    plt.axhline(y=self.top_threshold, color='r', linestyle='-')
    plt.axhline(y=self.bottom_threshold, color='b', linestyle='-')
    plt.ylim(-.5, 1)

    for down in self.down_windows:
        plt.axvspan(down[0], down[1], color='red')

    for up in self.up_windows:
        plt.axvspan(up[0], up[1], color='lime')

    for hold in self.hold_windows:
        plt.axvspan(hold[0], hold[1], color='purple')

The threshold for top and bottom are horizontal lines. The y axes is constrained. The red highlights are down motions, lime up motions, and purple hold motions. Error classifcations are printed in the terminal.


# Server

# Android Application

# Hardware

## WB55RG-examples-with-hal-lib
Microcontroller development for the exercise project, basically used for IMU functioning. Challenges lying in time synchronization for data gathering stage.

## Current stage
Currently we are on a testing stage, where we need to test the funtionalities of the board to give out/take in signals on GPIO pins, timer usages, UART/RS-232, I^2C, or SPI communications, also BLE/WiFi usages for data streaming from and onto laptop/cloud, as this micro-controller supports BLE/bluetooth 5/IEEE 802.15.4. All those should be right now to make sure our device is good for later utilization on sensor data collection and time synchronization of all data streaming.

## Usages

### layout of the MCU
Open the project in STM32 CubeIDE, look for something like "import already existing stm32 project", you could view the layout of the current board in the .ioc file and all pin arrangements could be modified in this file; when you finished editting the file, save it and code will be automatically generated in the .c file for you. (eg. if you want to use timer2 as PWM output, go to Pinout& Configuration, then select Timers and go to TIM2, select the output as PWM on channel 1 of TIM2, save it, and all the configuration for TIM2 will be done automatically) Ports& Pins on stm32 MCUs are named as Port X Pin X, so an example could be Port A Pin 0, where on the .ioc file it's marked as PA0. In function calls, it could be something like "GPIOA" and "GPIO_PIN_0".

### Run and load codes onto chip
Select the run button, and CubeIDE will start to compile and load code automatically by itself. If it doesn't detect a st-link supported device, it will report error and you need to check the connection is okay between laptop and the st-link micro-USB port of the board. Debug could be done easily, just click run with debug button "the bug button", you can go into the debug mode using GDB, and you can set breakpoints in the editor by double clicking the line that you want to put a bp, click "step into" to actually run the code line by line and see where it jumps to. 

### Language usages
Currently the project folder is all in .c style, however, CubeIDE supports language conversion and people could write .c/.cpp or even X86 assembly code on it to direcly manipulate registers.

### Contents included in this repo
This repo excluding the .md file, includes all necessary files to run on a MCU, including the HAL library functions also. If you want to get a brief idea of where to write the code or where the user codes actually begins, go to src/main.c, the IDE generates a lot of useful comments to indicate user code area. look for the main while loop just like in arduino, that's where everything starts.




## *VERSION 1.0*: PWM output achieved via HAL_TIM_PWM_PulseFinishedCallback
Go through timer, PWM output, and callback function usages in detail. Equipment required: a laptop with CubeIDE, a wb55 development board, some dupon cables, and a little LED to be plugged in for validating PWM output, and probably multi-meter for more accurate measurement. Complete everything needed for PWM sweping from 0% to 100% then back to 0% with only callback functions. There are several Timer setting needed for this feature, like give CH1 PWM output and set Trigger Event to be Compare Pulse(OC1) (to compar pulse emitting from ouptut channel1). Since we achieve changable PWM output via assigning values directly to CCR register, no need to assign value to pulse category. The most difficult part is to figure out the setting for callback
function usages, rather than writting the code. Achieve things on software level is easy, just to count and use private variables to compare, but hardware level callback could be more accurate on real-time reaction of the edege device. Timer usages are extremely important if wee wanna achieve specific or modifiable sensor reading gathering rate(sampling rate).  

1. Go to .ioc file, go to Pinout Config/Timers, Timer2, enable clock src to be internal clk.
2. In TIM2 parameter setting, check the PWM Generation choice in the channel configuration that you selected, PSC = (1 + val you entered), Period = (1 + val you entered), auto-reload-preload = enable, master/slave = disabled, IMPORTANT setting here:  _trigger-event=compare-pulse-oc1_
3. Keep all below parameters to be as default, go to NVIC setting in TIM2, check global interrupt, check GPIO output port for PWM are known and set accordingly.
4. Directly use the code in version 1.0, you would see PWM output sweeping from 0% to 100% and back to 0% on GPIO PIN A0.




## *VERSION 1.1*: I^2C to retrieve TEMP sensor(STTS751) reading, UART to laptop to view (not working example, just intuition)
Go through I^2C and UART hardware communication protocol and gather temperature readings for trials. Basic functionalities like SLK and SDA wiring from sensors to MCU, comm port setup for UART/USRT, settings for proper code generation. Equipment required: a laptop with CubeIDE, a nucleo-wb55 development board, a nucleo-IKS01A3. Basic knowledges about I^2C and USART will not be discussed here, consult https://deepbluembedded.com/stm32-i2c-tutorial-hal-examples-slave-dma/ . There are many implemented interfaces for those functionalities, but we prefer HAL libraries written by ST, as it's more standard and good for industrial cooperation in future stages. Functions mostly used are 

```c
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
uint16_t 	DevAddress,
uint8_t * 	pData,
uint16_t 	Size,
uint32_t 	Timeout 
)	

```
```c
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
uint16_t 	DevAddress,
uint16_t 	MemAddress,
uint16_t 	MemAddSize,
uint8_t * 	pData,
uint16_t 	Size,
uint32_t 	Timeout 
)	
```	

With MasterReceive and MemRead very similar to those two functions. Since we are communicating via 8-bit addr, 1 bit for R/W, there's only 7-bit for target addr specification, that's DevAddress. For example, I^2C addr given by ST for STTS751 temp sensor is 0x94(0b10010100), with left-shift 1 bit, we have the DevAddress as 0x4a(0b1001010). Also, data reading might be splitted into many registers stored on sensor, so we often need to indicate where in the sensor, that we want to write/read. Datasheet of STTS751 indicated that 0x00 is for (64-0), 0x02 is for (1/2-1/16), and 0x03 is for the configuration of the sensor output. So we first need to write to MemAddress 0x03 with DevAddress 0x4a, with MemAddSize 8 (for a single register), pData store a 8-digit mode that you want it to function in, Size is the amount of data to be sent, Timeout const is as you wish.



## *VERSION 1.1.1*: Working example of I^2C to retrieve temp sensor data on Adafruit Si7021, UART to mac to view

```c
//check the status of the device first
ret = HAL_I2C_IsDeviceReady(&hi2c1, I2C_SLAVE_ADDR, CODY_MAX_TRIAL, CODY_MAX_DELAY);

//1st MASTER START---->MASTER transmit SLAVE_ADDR + WRITE bit---->SLAVE ACK---->transmit command(0xF3)---->SLAVE ACK
uint8_t command = 0xE3;
ret = HAL_I2C_Master_Transmit(&hi2c1, I2C_SLAVE_ADDR, &command, 1, HAL_MAX_DELAY);

//2nd MASTER START---->MASTER transmit SLAVE_ADDR + READ bit---->SLAVE ACK---->SLAVE transmit 2 bytes temp reading
//---->MASTER save into register and ACK
ret = HAL_I2C_Master_Receive(&hi2c1, I2C_SLAVE_ADDR, temp_buffer, 2, CODY_MAX_DELAY); //read into MSByte of temp_returned
```
In this tiny experiment, what I want to test is the my former usages of HAL library implemented I^2C protocols are correct and the issue might be within the memory copy and control, or directly with the sensor board ST-IKS01A3(probably it disables direct communication between MCU and each of its sensing unit). I achieve the handshake between master and slave by viewing a HAL_OK const returned from the function call HAL_I2C_IsDeviceReady(). This function will take a slave address and try sending data streams to it for TRIAL times, and wait for DELAY time to test whether there are ACK signal sent from slave. After successfully establishing a connection, we transmit slave address again with the command 0xE3 according to the sensor datasheet with WRITE actvity, (calling HAL_I2C_Master_Transmit()), then we should also receive a ACK signal. Then the last step should be the reading, we use Si7021's hold-master temperature reading mode and since the I2C_Init_TypeDef is initialised with the category NoStretchMode=0 which means it defaultly enable timer-stretch on the master end for the slave to sense, collect, and finally transmit the data. I tested with mobile phone's flash light, temperature sensed could increase from room temperature 23'C to 25, 27, 31, etc. 

#### NOTE 
It's worth pointing out that, pointer manipulations, type casts and convresion, bit-wise manipulations are verey important in these lower-level implementation of data aquisition. Sometimes, communications are already established but due to improper usages of pointers and containers like uint8_t/uint16_t/char/short/float/double, results maybe weird and should be distinguished from errors on communication protocols, by using functions like IsDeivceReady(). Also, improper usages of the datasheet might also leading into these cases, should be differentiated.

## *VERSION 1.1.2*: Trials of ST LSM6DSOX IMU data gathering via I^2C protocol on wb55


<img
  src="https://github.com/codywangyaohui/WB55RG-examples-with-hal-lib/blob/main/graphics/chip_I2C_specification_datasheet.png" title="LSM6DSOX I^2C Specification on Datasheet" width="720" height="540"/>
### Figure 1: LSM6DSOX I^2C Specification on Datasheet

<img
  src="https://github.com/codywangyaohui/WB55RG-examples-with-hal-lib/blob/main/graphics/technical_details_of_Adafruit_product.png" title="LSM6DSOX I^2C Specification from Adafruit"  width="760" height="540"/>
### Figure 2: LSM6DSOX I^2C Specification from Adafruit Website  

As we can see from the first image that the built-in I2C address for the chip LSM6DSOX is 0xD5(Read) and 0xD4(Write) if the SA0 pin of the chip is connected to ground, while the address becomes 0xD7(Read) and 0xD6(Write) if the significant pin is connected to VCC. This is used for multiple connections of same type of sensor on a same I^2C bus. 

However, in this adafruit piece, they've configured the address to be 0x6A/0x6B for the LSM6DSOX and 0x1C/0x1D for LIS3MDL. They didn't specify which address is to be used in reading and which is for writing in its specification. Also, all those address specified above should be left shifted by 1 bit.


<img
  src="https://github.com/codywangyaohui/WB55RG-examples-with-hal-lib/blob/main/graphics/IMU_simple_functionality.png" title="LSM6DSOX IMU_simple_functionality"  width="521" height="637"/>
### Figure 3: LSM6DSOX IMU Simple Functionality in Version 1.1.2

By thursday, Oct 13th, I've achieved simple data acquisition system through sensor LSM6DSOX, with a wrapped sensor interface as a struct defined in /Core/Inc/sensor_driver.h, containing sensor type, raw data buffer, concatenated word buffer, final data reading buffer, and the I^2C address. This interface usage will make code more readable as everything is clearer, like for LISM6DSOX, which includes an accelerometer and a gyroscope, I defined two sensor driver objects to collect and process each sensor reading separately, and that makes more sense.

This version of code, will simply collect data from LSM6DSOX accelerometer and gyroscope, and stream data onto macOS via UART. I'll make magnetomerter reading available tomorrow, then move onto easy on-edge data fusion and sampling freqeuency arrangement, and finally to bluetooth streaming and data synchronization.

By friday, Oct 14th, I^2C driver wrapper functions are put into sensor_driver.c and use extern instead of static to avoid scope limitations. everything working properly and relatively readable.

#### NOTE
1. One special find here is that when all left side pins of the development board are set on the breadboard, the power could still be supplied if cable connected while st-link will not be working (on CubeIDE when burnning code into chips, it would show as st-link device not found)
2. Another thing is that even though I put the driver.c into the Core/Src folder, it seems cannot be found and all the functions defined inside will be with no reference if using in main.c. So I can only define all those wrapper functions in main.c upper part. I think it's something related to compilation process, like linker folders are not set properly, which makes the compiler not able to find driver.c source file, leading to compilation error. --FIXED, this problem is due the keyword usages.


## *VERESION 1.2.1*: Working example retreiving data from LSM6DSOX/LIS3MDL
This is an working example for gathering data from both the XL/GR sensor and the MG sensor. Really need to pay attention to configuration registers when comming to collecting data. All things pre-hand like the measuring scale, sensitivity, precision, any additional filters to be applied, more importantly, probably there are one to two features are down or one to two axes are in low-power mode and doesn't reach the ODR that you indicate in other config registers. As long as HAL I^2C driver is not reporting error, the problem should be within the sensor configurations. In this version of code, output data rate(ODR) are set to 833Hz for accelerometer and gyroscope while the magnetic field sensor is set to 560Hz, which should satisfy the basic requirement of the exercise monitoring. Also, these digital sensors do not function in request->measure in real-time->return style, instead, what we carry out is just to read its data output registers. If it has just done the measurement, new data will be read; otherwise, former round data will be read again. There will somewhat unalignment of the data flows yet, due to incampatible sampling frequencies, but no delays in the data reading process will occur.
Next step will be moving from inf loop style data collection to timer interrupt style data collection, and do some simple software-level calibrations.

## *VERESION 1.2.2*: Madgwick Fusion applied, row and pitch are working while yaw is abnormal, due to uncallibrated magnetometer
1. Data acquisition and processing modules move into main loop, to avoid too much load on callback, result in delay of systick.
2. Only UART/BLE transmission should be called inside timer callback functions.
3. Unsuccessful calibration tried via the ipynb file inside data_processing directory of this repo, probably due to hardware malfunctioning(but I
've tried 2 adafruit IMUs); also, the application "MotionCal" is also tried while no valid result reported.
4. Row and pitch are working properly and reporting accurate euler angles while yaw is always wrong due to magnetometer issues.
5. Relative position is wrong, currently don't know whether it's also caused by weird magnetometer readings.
6. To be continued.



## TODO:
1. BLE transmission should be implemented first.
2. Add timestampes to data for later synchronization stage.
3. Calibrate MAGNETOMETER to have more reliable data.
4. Look into the MADGWICK fusion algo, try to increase the reliability of data.
5. Time each step of the main while loop to see which one is taking most of the time-->implement free-RTOS and threading if needed afterwards.

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_DRIVER_H
#define __SENSOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

#include "Fusion.h"
#include "stm32wbxx_hal.h"


//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************
//*******General macro values***************************************************************
//*******Declarations*************************************************************************
//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************

typedef union {
    uint8_t array[6];

    struct {
        uint8_t x_l;
        uint8_t x_h;
        uint8_t y_l;
        uint8_t y_h;
        uint8_t z_l;
        uint8_t z_h;
    } data_register;

} Data_Register;

typedef union {
    int16_t array[3];

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } axis;

} Combined_Word;


typedef union {
    int16_t array[10];

    struct {
        int16_t reading_0;
        int16_t reading_1;
        int16_t reading_2;
        int16_t reading_3;
		int16_t reading_4;

    } axis;

} Word_Buffer;


typedef union {
	Word_Buffer array[3];

	struct {
		Word_Buffer x_axis;
		Word_Buffer y_axis;
		Word_Buffer z_axis;

	} all_word_buffer;

} All_Words;


#define SAMPLE_FREQ		(77)

#define MILISEC_TO_SEC  (0.001)

/**
 * sensor selection coefficient when choosing sensors to read
 * in IMU_Read_Data() function
 */
#define XL_SENSOR  0
#define GR_SENSOR 1
#define MG_SENSOR 2

/**
 * RAW: raw data from data buffer
 * UNI: calculated data from driver->read_float
 */
#define RAW 	0
#define UNI   	1

/**
 * macros to indicate which data type
 */
#define UINT8_T	0
#define INT16_T		1
#define FLOAT		2

/**
 * word buffer size to indicate zeroing of the indx for inserting the combined word
 */
#define WORD_BUFF_SIZE 5


//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************
//*******Place for LSM6DSOX IMU unit specification****************************************
//*******Declarations*************************************************************************
//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************

/**
 * the i2c address of lsm6dsox operating in slave mode
 */
#define LSM6DSOX_I2C_SLAVE_ADDR 0x6A << 1

/**
 * CTRL1_XL[7:4] accelerometer ODR mode selection: sampling freq from 1.6Hz to 6.66kHz
 * CTRL1_XL[3:2] accelerometer full-scale selection: range from 2g to 16g
 * CTRL1_XL[1]     accelerometer resolution selection: 0 for default; 1 for selecting another LP filter
 * CTRL1_XL[0]     default to 0
 */
#define LSM6DSOX_CTRL1_XL_REG      0x10

/**
 * CTRL2_GR[7:4] gyroscope data rate selection, default 0000
 * CTRL2_GR[3:2] gyroscope full scale range selection 250dps to 2000dps
 */
#define LSM6DSOX_CTRL2_GR_REG		0x11


/**
 * accelerometer readings reported in following registers
 */
#define LSM6DSOX_ACC_X_LOW_REG  0x28
#define LSM6DSOX_ACC_X_HIGH_REG 0x29
#define LSM6DSOX_ACC_Y_LOW_REG  0x2A
#define LSM6DSOX_ACC_Y_HIGH_REG 0x2B
#define LSM6DSOX_ACC_Z_LOW_REG  0x2C
#define LSM6DSOX_ACC_Z_HIGH_REG 0x2D

/**
 * gyroscope readings reported in following registers
 */
#define LSM6DSOX_GYRO_X_LOW_REG  0x22
#define LSM6DSOX_GYRO_X_HIGH_REG 0x23
#define LSM6DSOX_GYRO_Y_LOW_REG  0x24
#define LSM6DSOX_GYRO_Y_HIGH_REG 0x25
#define LSM6DSOX_GYRO_Z_LOW_REG  0x26
#define LSM6DSOX_GYRO_Z_HIGH_REG 0x27



/**
 * conversion coefficient of LSM6DSOX accelerometer
 * unit is G PER Least Significant Bit
 **********************************************************************************************
 * conversion method: using the 16-bit 2's complement integer, times 0.000061 to get
 * the reading in 2g-measuring scale for example
 */
#define LSM6DSOX_XL_2G_COEFF 	   0.000061
#define LSM6DSOX_XL_4G_COEFF		   0.000122
#define LSM6DSOX_XL_8G_COEFF 	   0.000244
#define LSM6DSOX_XL_16G_COEFF	   0.000488

/**
 * conversion coefficient of LSM6DSOX gyroscope
 * unit is dps PER Least Significant Bit
 ***********************************************************************************************
 * conversion method: using the 16-bit 2's complement integer, times 0.004375 to get
 * the reading in 125dps-measuring scale for example
 */
#define LSM6DSOX_GR_125_COEFF		0.004375
#define LSM6DSOX_GR_250_COEFF		0.008750
#define LSM6DSOX_GR_500_COEFF		0.017500
#define   LSM6DSOX_GR_1K_COEFF		0.035000
#define   LSM6DSOX_GR_2K_COEFF		0.070000


//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************
//*******Place for LIS3MDL magnetometer specification***********************************
//*******Declarations*************************************************************************
//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************

/**
 * slave address of the LIS3MDL sensor for I2C communication
 */
#define LIS3MDL_I2C_SLAVE_ADDR 		0x1C << 1

/**
 * config reg to set operative mode and output data rate
 */
#define LIS3MDL_CTRL_REG1				0x20
#define LIS3MDL_CTRL_REG2				0x21
#define LIS3MDL_CTRL_REG3				0x22
#define LIS3MDL_CTRL_REG4				0x23
#define LIS3MDL_CTRL_REG5				0x24


/**
 * all data register address in LIS3MDL
 */
#define LIS3MDL_MAGN_X_LOW_REG  0x28
#define LIS3MDL_MAGN_X_HIGH_REG 0x29
#define LIS3MDL_MAGN_Y_LOW_REG  0x2A
#define LIS3MDL_MAGN_Y_HIGH_REG 0x2B
#define LIS3MDL_MAGN_Z_LOW_REG  0x2C
#define LIS3MDL_MAGN_Z_HIGH_REG 0x2D

/**
 * LIS3MDL calculation coefficients to be divided
 */
#define LIS3MDL_MG_4_COEFF					0.00014615609
#define LIS3MDL_MG_8_COEFF					0.000292312189
#define LIS3MDL_MG_12_COEFF					0.000438404208
#define LIS3MDL_MG_16_COEFF					0.0005844535359




//#define I2C_MEMADD_SIZE_8BIT 			         1
//#define I2C_MEMADD_SIZE_16BIT			     2
#define CODY_MAX_TRIAL 						       50
#define CODY_MAX_DELAY 						10000


//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************
//**********Place for sensor zero-calibration offset*****************************************
//**********constant declarations************************************************************
//**********************************************************************************************
//**********************************************************************************************
//**********************************************************************************************

#define LSM6DSOX_XL_X_OFF					0.0312
#define LSM6DSOX_XL_Y_OFF					0.0442
#define LSM6DSOX_XL_Z_OFF				   -1.0174

#define LSM6DSOX_GR_X_OFF					1.17
#define LSM6DSOX_GR_Y_OFF					0.32
#define LSM6DSOX_GR_Z_OFF					0.11



#define RAW_DATA_BUFF_ZERO ((Data_Register) {.array = {0, 0, 0, 0, 0, 0}  })
//#define COMB_WORD_BUFF_ZERO ((Word_Buffer) {.array = {0, 0, 0, 0, 0} })
//#define ALL_WORD_BUFF_ZERO ((All_Words) {.array = {COMB_WORD_BUFF_ZERO, COMB_WORD_BUFF_ZERO, COMB_WORD_BUFF_ZERO}    })


#define COMBINED_WORD_BUFF_ZERO ((Combined_Word) {.array = {0, 0, 0}  })
#define REG_ADDR_LS_XL ((Data_Register){ .array = {LSM6DSOX_ACC_X_LOW_REG, LSM6DSOX_ACC_X_HIGH_REG,LSM6DSOX_ACC_Y_LOW_REG, LSM6DSOX_ACC_Y_HIGH_REG,LSM6DSOX_ACC_Z_LOW_REG, LSM6DSOX_ACC_Z_HIGH_REG}   })
#define REG_ADDR_LS_GR ((Data_Register){ .array = {LSM6DSOX_GYRO_X_LOW_REG, LSM6DSOX_GYRO_X_HIGH_REG,LSM6DSOX_GYRO_Y_LOW_REG, LSM6DSOX_GYRO_Y_HIGH_REG,LSM6DSOX_GYRO_Z_LOW_REG, LSM6DSOX_GYRO_Z_HIGH_REG}   })
#define REG_ADDR_LS_MG ((Data_Register){ .array =  {LIS3MDL_MAGN_X_LOW_REG, LIS3MDL_MAGN_X_HIGH_REG,LIS3MDL_MAGN_Y_LOW_REG, LIS3MDL_MAGN_Y_HIGH_REG,LIS3MDL_MAGN_Z_LOW_REG, LIS3MDL_MAGN_Z_HIGH_REG}  })


/**
 * TODO still need to find good sensor calibration methodology
 */


struct SENSOR_Driver
{
	/**
	 * macro to indicate which sensor is it
	 */
	uint8_t sensor_type;
	/**
	 * sensor data register listing
	 */
	Data_Register reg_addr_ls;
	/**
	 * data buffer when retrieving the data read from the sensor
	 * buffer[0:1] buffer retaining value from x LOW and HIGH register
	 * buffer[2:3] buffer retaining value from y_LOW and HIGH register
	 * buffer[4:5] buffer retaining value from z_LOW and HIGH register
	 */
	Data_Register raw_data_buffer;
	/**
	 * buffer for combining three 16-bit 2's complement readings
	 *
	 * combined_word[0]: int16_t[10] for all x_axis readings
	 * combined_word[1]: int16_t[10] for all y_axis readings
	 * combined_word[2]: int16_t[16] for all z_axis readings
	 */
	 Combined_Word combined_word;
	/**
		 * buffer for final 3 axis readings
		 *
		 * reading_float[0] for x
		 * reading_float[1] for y
		 * reading_float[2] for z
		 */
	FusionVector reading_float;
	/**
	 * slave address to call on when setting up I^2C communication
	 */
	uint8_t slave_addr;
};



//struct SENSOR_Driver slave_xl;
//struct SENSOR_Driver slave_gr;
//struct SENSOR_Driver slave_mg;
//

/**
 * function to initialize a sensor struct for data retaining
 */
extern void Sensor_Unit_Init(struct SENSOR_Driver *target_to_init, uint8_t which_sensor);
/**
 * function to pre-check the status of a sensor, whether listening on the bus
 *
 * SENSOR_Driver slave_to_check: sensor unit that we want to check
 * I2C_HandleTypeDef i2c_handle: i2c information to be used,
 */
extern HAL_StatusTypeDef Sensor_Check_Ready(struct SENSOR_Driver *slave_to_check, I2C_HandleTypeDef i2c_handle);


/**
 * function to convert 2 8-bit unsigned integer to 1 16-bit signed integer
 */
extern int16_t convert_to_int16(uint8_t low_register, uint8_t high_register);

/**
 * function to convert 1 16-bit signed integer to a scaled acceleration reading
 */
extern float process_xl_code(int16_t xl_code, double coeff_to_be_used);

///**
// * function for converting 16-bit word to temperature reading
// * according to Adafruit LS7021 temperature humid sensor
// */
//extern float process_temp_code(uint16_t temp_code)
//{
//  return (float)(((175.72 * temp_code) / 65536.0) - 46.85);
//}


/**
 * function for setting the configuration info into registers
 *
 * SENSOR_Driver slave_to_set: slave driver to use
 * I2C_HandleTypeDef i2c_handle: i2c bus to be used
 * uint8_t config_reg_addr: sensor internal register address for configuration
 * uint8_t val_to_set: values to be written into
 */
extern HAL_StatusTypeDef Sensor_Set_Config(struct SENSOR_Driver *slave_to_set, I2C_HandleTypeDef i2c_handle, uint8_t config_reg_addr, uint8_t val_to_set);


/**
 * function to read data from sensor
 *
 * SENSOR_Driver slave_to_read: slave driver to use
 * I2C_HandleTypeDef i2c_handle: i2c bus to be used
 */
extern HAL_StatusTypeDef Sensor_Read_Data(struct SENSOR_Driver *slave_to_read, I2C_HandleTypeDef i2c_handle);

/**
 * function to carry out all the calculations and save the
 * floating point representation of IMU reading into slave_to_read.xl_double array for X Y Z axes accordingly
 *
 * SENSOR_Driver slave_to_read: slave driver to use
 */
extern void Sensor_Data_Process(struct SENSOR_Driver *slave_to_read, float coeff);


/**
 * function for calibrating the sensor readings prior to output
 */
extern void Sensor_Data_Calibrate_Simple(struct SENSOR_Driver *slave_to_calibrate);


/**
 * function for printing the current IMU readings into a word buffer for later UART or streaming
 */
extern void Sensor_Print_Data(char *word_buffer, struct SENSOR_Driver *slave_to_print);


/**
 * function to print all information together
 */
extern void Sensor_Log_All(char *word_buffer, struct SENSOR_Driver *slave_xl,
						   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg);


extern void Sensor_Log_All_Label(char *word_buffer, struct SENSOR_Driver *slave_xl,
						   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg);

/**
 * function to print out fused result
 */
extern void Sensor_Fusion_Log(char *word_buffer, FusionEuler *orientation, FusionVector *location);

/**
 * function to print all information for calibration purposes
 */
extern void Sensor_Log_Calibrate(char *word_buffer, struct SENSOR_Driver *slave_xl,
						   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg, uint8_t mode);


/**
 * function for UARTing to macOS
 */
extern void Sensor_Uart(char *word_buffer, UART_HandleTypeDef uart_handle);


/**
 * function for doing average step when 10th word has been collected
 */
extern void Sensor_Data_Avg_Process(struct SENSOR_Driver *slave_to_avg, float coeff_to_be_used);

/**
 * function for concatenating all 2 8-bit int buffer into 1 16-bit signed word
 */
extern void Sensor_Data_Concat(struct SENSOR_Driver *slave_to_read, uint8_t indx);

extern void experiment_log(char *word_buffer, FusionEuler * orientation, FusionVector *location, struct SENSOR_Driver *slave_xl,
		   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg);


#endif /*__SENSOR_DRIVER_H */




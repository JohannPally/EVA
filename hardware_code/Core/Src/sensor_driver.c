#include "sensor_driver.h"



extern void Sensor_Unit_Init(struct SENSOR_Driver *target_to_init, uint8_t which_sensor) {
	switch(which_sensor) {
		case(XL_SENSOR):
			target_to_init->sensor_type 			= XL_SENSOR;
			target_to_init->reg_addr_ls 			= REG_ADDR_LS_XL;
			target_to_init->raw_data_buffer 	= RAW_DATA_BUFF_ZERO;
			target_to_init->combined_word 	= COMBINED_WORD_BUFF_ZERO;
			target_to_init->reading_float 			= FUSION_VECTOR_ZERO;
			target_to_init->slave_addr 				= LSM6DSOX_I2C_SLAVE_ADDR;
			break;

		case(GR_SENSOR):
			target_to_init->sensor_type 			= GR_SENSOR;
			target_to_init->reg_addr_ls 			= REG_ADDR_LS_GR;
			target_to_init->raw_data_buffer 	= RAW_DATA_BUFF_ZERO;
			target_to_init->combined_word 	= COMBINED_WORD_BUFF_ZERO;
			target_to_init->reading_float 			= FUSION_VECTOR_ZERO;
			target_to_init->slave_addr 				= LSM6DSOX_I2C_SLAVE_ADDR;
			break;

		case(MG_SENSOR):
			target_to_init->sensor_type 			= MG_SENSOR;
			target_to_init->reg_addr_ls 			= REG_ADDR_LS_MG;
			target_to_init->raw_data_buffer 	= RAW_DATA_BUFF_ZERO;
			target_to_init->combined_word 	= COMBINED_WORD_BUFF_ZERO;
			target_to_init->reading_float 			= FUSION_VECTOR_ZERO;
			target_to_init->slave_addr 				= LIS3MDL_I2C_SLAVE_ADDR;
			break;

		default:
			return;
	}
	return;
}


extern HAL_StatusTypeDef Sensor_Check_Ready(struct SENSOR_Driver *slave_to_check, I2C_HandleTypeDef i2c_handle)
{
	HAL_StatusTypeDef ret =    HAL_I2C_IsDeviceReady(&i2c_handle,
																		    slave_to_check->slave_addr,
																							  CODY_MAX_TRIAL,
																							 CODY_MAX_DELAY);

	  return ret;
}

extern HAL_StatusTypeDef Sensor_Set_Config(struct SENSOR_Driver *slave_to_set, I2C_HandleTypeDef i2c_handle, uint8_t config_reg_addr, uint8_t val_to_set)
{
	//set the config for the sensor {might be called multiple times on one sensor}
	HAL_StatusTypeDef ret_1 = HAL_I2C_Mem_Write(&i2c_handle,
																	   	    slave_to_set->slave_addr,
																	   	   	   	   	   	     config_reg_addr,
																		    I2C_MEMADD_SIZE_8BIT,
																		  	  	  	  	  	  	      &val_to_set,
																			I2C_MEMADD_SIZE_8BIT,
																				  	    CODY_MAX_DELAY);

	return (ret_1);
}

extern HAL_StatusTypeDef Sensor_Read_Data(struct SENSOR_Driver *slave_to_read, I2C_HandleTypeDef i2c_handle)
{

	//loading data from X_AXIS LOW register
	HAL_StatusTypeDef ret_0 = HAL_I2C_Mem_Read(&i2c_handle,
																		 slave_to_read->slave_addr,
									  slave_to_read->reg_addr_ls.data_register.x_l,
																  	  	    I2C_MEMADD_SIZE_8BIT,
		                &(slave_to_read->raw_data_buffer.data_register.x_l),
																			 I2C_MEMADD_SIZE_8BIT,
																			 	 	 	 CODY_MAX_DELAY);
	//loading data from X_AXIS HIGH register
	HAL_StatusTypeDef ret_1 = HAL_I2C_Mem_Read(&i2c_handle,
																		 slave_to_read->slave_addr,
									 slave_to_read->reg_addr_ls.data_register.x_h,
																  	  	    I2C_MEMADD_SIZE_8BIT,
                       &(slave_to_read->raw_data_buffer.data_register.x_h),
																			I2C_MEMADD_SIZE_8BIT,
																			 	 	 	CODY_MAX_DELAY);
	//loading data from Y_AXIS LOW register
	HAL_StatusTypeDef ret_2 = HAL_I2C_Mem_Read(&i2c_handle,
																		 slave_to_read->slave_addr,
									  slave_to_read->reg_addr_ls.data_register.y_l,
																  	  	    I2C_MEMADD_SIZE_8BIT,
						&(slave_to_read->raw_data_buffer.data_register.y_l),
																			 I2C_MEMADD_SIZE_8BIT,
																			 	 	 	 CODY_MAX_DELAY);
	//loading data from Y_AXIS HIGH register
	HAL_StatusTypeDef ret_3 = HAL_I2C_Mem_Read(&i2c_handle,
																		 slave_to_read->slave_addr,
									 slave_to_read->reg_addr_ls.data_register.y_h,
																  	  	    I2C_MEMADD_SIZE_8BIT,
						&(slave_to_read->raw_data_buffer.data_register.y_h),
																			 I2C_MEMADD_SIZE_8BIT,
																			 	 	 	 CODY_MAX_DELAY);
	//loading data from Z_AXIS LOW register
	HAL_StatusTypeDef ret_4 = HAL_I2C_Mem_Read(&i2c_handle,
																		 slave_to_read->slave_addr,
									   slave_to_read->reg_addr_ls.data_register.z_l,
																  	  	     I2C_MEMADD_SIZE_8BIT,
						 &(slave_to_read->raw_data_buffer.data_register.z_l),
																			 I2C_MEMADD_SIZE_8BIT,
																			 	 	 	 CODY_MAX_DELAY);
	//loading data from Z_AXIS HIGH register
	HAL_StatusTypeDef ret_5 = HAL_I2C_Mem_Read(&i2c_handle,
																		 slave_to_read->slave_addr,
									 slave_to_read->reg_addr_ls.data_register.z_h,
																  	  	    I2C_MEMADD_SIZE_8BIT,
						&(slave_to_read->raw_data_buffer.data_register.z_h),
																			I2C_MEMADD_SIZE_8BIT,
																			 	 	 	CODY_MAX_DELAY);

	return (ret_0 && ret_1 && ret_2 && ret_3 && ret_4 && ret_5);
}

extern int16_t convert_to_int16(uint8_t low_register, uint8_t high_register)
{
//  return (int16_t)((bytes[1]<<8) | bytes[0]);
	return (int16_t) (high_register << 8 | low_register);
}

extern float process_code(int16_t code, double coeff_to_be_used)
{
	return (float) ( (double)code * coeff_to_be_used);
}

/**
 * TODO need to adjust for sensor reading averaging
 */
extern void Sensor_Data_Process(struct SENSOR_Driver *slave_to_read, float coeff)
{
	//all raw readings will be combined in this chunk
	slave_to_read->combined_word.axis.x = convert_to_int16(slave_to_read->raw_data_buffer.data_register.x_l,
																													slave_to_read->raw_data_buffer.data_register.x_h);

	slave_to_read->combined_word.axis.y = convert_to_int16(slave_to_read->raw_data_buffer.data_register.y_l,
																													slave_to_read->raw_data_buffer.data_register.y_h);

	slave_to_read->combined_word.axis.z = convert_to_int16(slave_to_read->raw_data_buffer.data_register.z_l,
																													slave_to_read->raw_data_buffer.data_register.z_h);

	//calculations happen  here
	slave_to_read->reading_float.axis.x = process_code(slave_to_read->combined_word.axis.x, coeff);
	slave_to_read->reading_float.axis.y = process_code(slave_to_read->combined_word.axis.y, coeff);
	slave_to_read->reading_float.axis.z = process_code(slave_to_read->combined_word.axis.z, coeff);

	return;
}
//
//extern void Sensor_Data_Concat(struct SENSOR_Driver *slave_to_read, uint8_t indx)
//{
//#define x_axis slave_to_read->combined_word.all_word_buffer.x_axis.axis
//#define y_axis slave_to_read->combined_word.all_word_buffer.y_axis.axis
//#define z_axis slave_to_read->combined_word.all_word_buffer.z_axis.axis
//	//all raw readings will be combined in this chunk
//	switch(indx) {
//
//		case(0):
//			x_axis.reading_0 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.x_l,
//																				  slave_to_read->raw_data_buffer.data_register.x_h);
//			y_axis.reading_0 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.y_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.y_h);
//			z_axis.reading_0 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.z_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.z_h);
//			break;
//
//		case(1):
//			x_axis.reading_1 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.x_l,
//																				  slave_to_read->raw_data_buffer.data_register.x_h);
//			y_axis.reading_1 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.y_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.y_h);
//			z_axis.reading_1 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.z_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.z_h);
//			break;
//
//		case(2):
//			x_axis.reading_2 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.x_l,
//																				  slave_to_read->raw_data_buffer.data_register.x_h);
//			y_axis.reading_2 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.y_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.y_h);
//			z_axis.reading_2 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.z_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.z_h);
//			break;
//
//		case(3):
//			x_axis.reading_3 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.x_l,
//																				  slave_to_read->raw_data_buffer.data_register.x_h);
//			y_axis.reading_3 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.y_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.y_h);
//			z_axis.reading_3 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.z_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.z_h);
//			break;
//
//		case(4):
//			x_axis.reading_4 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.x_l,
//																				  slave_to_read->raw_data_buffer.data_register.x_h);
//			y_axis.reading_4 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.y_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.y_h);
//			z_axis.reading_4 = convert_to_int16(slave_to_read->raw_data_buffer.data_register.z_l,
//																			  	  slave_to_read->raw_data_buffer.data_register.z_h);
//			break;
//
//
//	}
//
//#undef x_axis
//#undef y_axis
//#undef z_axis
//	return;
//}

extern int32_t Avg_Helper(Word_Buffer *words_to_avg)
{
	int32_t to_return = 0;
	to_return  = (int32_t) (words_to_avg->axis.reading_0 + words_to_avg->axis.reading_1 + words_to_avg->axis.reading_2 +
										     words_to_avg->axis.reading_3 + words_to_avg->axis.reading_4);
	return (int32_t)(to_return/10);
}

//extern void Sensor_Data_Avg_Process(struct SENSOR_Driver *slave_to_avg, float coeff_to_be_used)
//{
//	//x_axis first
//	slave_to_avg->reading_float.axis.x =  (float) ( (double)Avg_Helper(&(slave_to_avg->combined_word.all_word_buffer.x_axis)) * coeff_to_be_used);
//	//y_axis
//	slave_to_avg->reading_float.axis.y =  (float) ( (double)Avg_Helper(&(slave_to_avg->combined_word.all_word_buffer.y_axis)) * coeff_to_be_used);
//	//z_axis
//	slave_to_avg->reading_float.axis.z =  (float) ( (double)Avg_Helper(&(slave_to_avg->combined_word.all_word_buffer.z_axis)) * coeff_to_be_used);
//
//}

extern void Sensor_Data_Calibrate_Simple(struct SENSOR_Driver *slave_to_calibrate)
{
	switch(slave_to_calibrate->sensor_type) {

		case(XL_SENSOR):
				slave_to_calibrate->reading_float.axis.x += LSM6DSOX_XL_X_OFF;
				slave_to_calibrate->reading_float.axis.y += LSM6DSOX_XL_Y_OFF;
				slave_to_calibrate->reading_float.axis.z += LSM6DSOX_XL_Z_OFF;
				break;

		case(GR_SENSOR):
				slave_to_calibrate->reading_float.axis.x += LSM6DSOX_GR_X_OFF;
				slave_to_calibrate->reading_float.axis.y += LSM6DSOX_GR_Y_OFF;
				slave_to_calibrate->reading_float.axis.z += LSM6DSOX_GR_Z_OFF;
				break;
		case(MG_SENSOR):
				slave_to_calibrate->reading_float.axis.x *= 100;
				slave_to_calibrate->reading_float.axis.y *= 100;
				slave_to_calibrate->reading_float.axis.z *= 100;
	}

	return;
}

extern void Sensor_Print_Data(char *word_buffer, struct SENSOR_Driver *slave_to_print)
{
	switch(slave_to_print->sensor_type)
	{
		case (XL_SENSOR):
				sprintf(word_buffer,
				  "xl_x-axis: %f'\r\nxl_y-axis: %f'\r\nxl_z-axis: %f'\r\n\r\n",
				   slave_to_print->reading_float.axis.x,
				   slave_to_print->reading_float.axis.y,
				   slave_to_print->reading_float.axis.z);
			break;
		case (GR_SENSOR):
				sprintf(word_buffer,
				  "gr_x-axis: %f'\r\ngr_y-axis: %f'\r\ngr_z-axis: %f'\r\n\r\n",
				   slave_to_print->reading_float.axis.x,
				   slave_to_print->reading_float.axis.y,
				   slave_to_print->reading_float.axis.z);
			break;
		case (MG_SENSOR):
				sprintf(word_buffer,
				  "mg_x-axis: %f'\r\nmg_y-axis: %f'\r\nmg_z-axis: %f'\r\n\r\n",
				   slave_to_print->reading_float.axis.x,
				   slave_to_print->reading_float.axis.y,
				   slave_to_print->reading_float.axis.z);
			break;
	}

	return;
}

extern void Sensor_Log_All(char *word_buffer, struct SENSOR_Driver *slave_xl,
						   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg)
{

	sprintf(word_buffer, "%f' %f' %f' %f' %f' %f' %f' %f' %f\r\n",
			slave_xl->reading_float.axis.x, slave_xl->reading_float.axis.y, slave_xl->reading_float.axis.z,
			slave_gr->reading_float.axis.x, slave_gr->reading_float.axis.y, slave_gr->reading_float.axis.z,
			slave_mg->reading_float.axis.x, slave_mg->reading_float.axis.y, slave_mg->reading_float.axis.z);

	return;
}

extern void Sensor_Log_All_Label(char *word_buffer, struct SENSOR_Driver *slave_xl,
						   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg)
{

	sprintf(word_buffer, "x_xl: %f' y_xl: %f' z_xl: %f' x_gr: %f' y_gr: %f' z_gr: %f' x_mg: %f' y_mg: %f' z_mg: %f\r\n",
			slave_xl->reading_float.axis.x, slave_xl->reading_float.axis.y, slave_xl->reading_float.axis.z,
			slave_gr->reading_float.axis.x, slave_gr->reading_float.axis.y, slave_gr->reading_float.axis.z,
			slave_mg->reading_float.axis.x, slave_mg->reading_float.axis.y, slave_mg->reading_float.axis.z);

	return;
}

extern void Sensor_Fusion_Log(char *word_buffer, FusionEuler *orientation, FusionVector *location) {

	sprintf(word_buffer, "Roll: %f' Pitch: %f' Yaw: %f' X: %f' Y: %f' Z: %f\r\n",
			orientation->angle.roll, orientation->angle.pitch, orientation->angle.yaw,
			location->axis.x, location->axis.y, location->axis.z);

	return;
}


extern void experiment_log(char *word_buffer, FusionEuler * orientation, FusionVector *location, struct SENSOR_Driver *slave_xl,
		   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg) {

	sprintf(word_buffer, "%f' %f' %f' %f' %f' %f' %f' %f' %f' %f' %f' %f' %f' %f' %f\r\n",
				slave_xl->reading_float.axis.x, slave_xl->reading_float.axis.y, slave_xl->reading_float.axis.z,
				slave_gr->reading_float.axis.x, slave_gr->reading_float.axis.y, slave_gr->reading_float.axis.z,
				slave_mg->reading_float.axis.x, slave_mg->reading_float.axis.y, slave_mg->reading_float.axis.z,
				orientation->angle.roll, orientation->angle.pitch, orientation->angle.yaw,
				location->axis.x, location->axis.y, location->axis.z);

		return;
}
//
//extern void Sensor_Log_Calibrate(char *word_buffer, struct SENSOR_Driver *slave_xl,
//						   struct SENSOR_Driver *slave_gr, struct SENSOR_Driver *slave_mg, uint8_t mode)
//{
//	switch(mode) {
//	case(RAW):
//		sprintf(word_buffer, "Raw : %d, %d', %d, %d, %d, %d, %d, %d, %d\r\n",
//					slave_xl->combined_word.axis.x * 8192/9.8, slave_xl->combined_word.axis.y * 8192/9.8, slave_xl->combined_word.axis.z * 8192/9.8,
//					slave_gr->combined_word.axis.x * 16, slave_gr->combined_word.axis.y * 16, slave_gr->combined_word.axis.z * 16,
//					slave_mg->combined_word.axis.x * 10, slave_mg->combined_word.axis.y * 10, slave_mg->combined_word.axis.z * 10);
//		break;
//
//	case(UNI):
//		sprintf(word_buffer, "Raw: %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
//					(int)(slave_xl->reading_float.axis.x * 8192/9.8), (int)(slave_xl->reading_float.axis.y * 8192/9.8), (int)(slave_xl->reading_float.axis.z * 8192/9.8),
//					(int)(slave_gr->reading_float.axis.x * 16), (int)(slave_gr->reading_float.axis.y * 16), (int)(slave_gr->reading_float.axis.z * 16),
//					(int)(slave_mg->reading_float.axis.x * 10), (int)(slave_mg->reading_float.axis.y * 10), (int)(slave_mg->reading_float.axis.z * 10));
//		break;
//
//	}
//	return;
//}

extern void Sensor_Uart(char *word_buffer, UART_HandleTypeDef uart_handle)
{
	HAL_UART_Transmit(&uart_handle,
							  (uint8_t *)word_buffer,
				   strlen((char * )word_buffer),
				   	   	   	   	   CODY_MAX_DELAY);
	return;
}
//
//extern void Sensor_Data_Calibrate_Advance(struct SENSOR_Driver *sensor_xl, struct SENSOR_Driver *sensor_gr, struct SENSOR_Driver *sensor_mg)
//{
//	sensor_xl->reading_float = FusionCalibrationInertial(sensor_xl->reading_float, misalignment, sensitivity, offset);
//}

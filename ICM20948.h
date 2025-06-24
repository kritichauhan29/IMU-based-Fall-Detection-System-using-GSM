/*
 * ICM20948.h
 *
 *  Created on: Mar 7, 2023
 *      Author: Anubhav
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_



#endif /* INC_ICM20948_H_ */
#define READ							0x80
#define WRITE							0x00
#define ICM20948_SPI					(&hspi1)

#define ICM20948_SPI_CS_PIN_PORT		GPIOA
#define ICM20948_SPI_CS_PIN_NUMBER		GPIO_PIN_0

//static float gyro_scale_factor;
//static float accel_scale_factor;

/* Typedefs */

typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

typedef enum
{
	_250dps,
	_500dps,
	_1000dps,
	_2000dps
} GYRO_RANGE_VALUE;

typedef enum
{
	_2g,
	_4g,
	_8g,
	_16g
} ACCEL_RANGE_VALUE;

typedef struct
{
	float x_accel;
	float y_accel;
	float z_accel;
	float x_gyro;
	float y_gyro;
	float z_gyro;

} icm_20948_data;

typedef enum
{
	power_down_mode = 0,
	single_measurement_mode = 1,
	continuous_measurement_10hz = 2,
	continuous_measurement_20hz = 4,
	continuous_measurement_50hz = 6,
	continuous_measurement_100hz = 8
} operation_mode;

/* ICM-20948 Registers */
#define ICM20948_ID						0xEA
#define REG_BANK_SEL					0x7F

// USER BANK 0
    // Register name          Register address in Hex
#define B0_WHO_AM_I						0x00     		//used to identify the device in use
#define B0_USER_CTRL					0x03 			//control DMP features;I2C mode;SPI only mode;SRAM,DMP&I2C_MST_RST
#define B0_LP_CONFIG					0x05			//control Duty cycle mode of aceel,gyro and I2C
#define B0_PWR_MGMT_1					0x06			//sleep wake function,low power mode function, disable temp sensor,CLK select
#define B0_PWR_MGMT_2					0x07			//enable disable accel,gryo
#define B0_INT_PIN_CFG					0x0F
#define B0_INT_ENABLE					0x10
#define B0_INT_ENABLE_1					0x11
#define B0_INT_ENABLE_2					0x12
#define B0_INT_ENABLE_3					0x13
#define B0_I2C_MST_STATUS				0x17
#define B0_INT_STATUS					0x19
#define B0_INT_STATUS_1					0x1A
#define B0_INT_STATUS_2					0x1B
#define B0_INT_STATUS_3					0x1C
#define B0_DELAY_TIMEH					0x28
#define B0_DELAY_TIMEL					0x29
#define B0_ACCEL_XOUT_H					0x2D			//High Byte of Accelerometer x-axis data.
#define B0_ACCEL_XOUT_L					0x2E			//Low Byte of Accelerometer x-axis data.
#define B0_ACCEL_YOUT_H					0x2F			//High Byte of Accelerometer y-axis data.
#define B0_ACCEL_YOUT_L					0x30			//Low Byte of Accelerometer y-axis data.
#define B0_ACCEL_ZOUT_H					0x31			//High Byte of Accelerometer Z-axis data.
#define B0_ACCEL_ZOUT_L					0x32			//Low Byte of Accelerometer Z-axis data.
#define B0_GYRO_XOUT_H					0x33			//High Byte of Gyroscope x-axis data.
#define B0_GYRO_XOUT_L					0x34			//Low Byte of Gyroscope x-axis data.
#define B0_GYRO_YOUT_H					0x35			//High Byte of Gyroscope y-axis data.
#define B0_GYRO_YOUT_L					0x36			//Low Byte of Gyroscope y-axis data.
#define B0_GYRO_ZOUT_H					0x37			//High Byte of Gyroscope Z-axis data.
#define B0_GYRO_ZOUT_L					0x38			//Low Byte of Gyroscope Z-axis data.
#define B0_TEMP_OUT_H					0x39			//High Byte of Temp sensor data.
#define B0_TEMP_OUT_L					0x3A			//Low Byte of Temp sensor data.
#define B0_EXT_SLV_SENS_DATA_00			0x3B
#define B0_EXT_SLV_SENS_DATA_01			0x3C
#define B0_EXT_SLV_SENS_DATA_02			0x3D
#define B0_EXT_SLV_SENS_DATA_03			0x3E
#define B0_EXT_SLV_SENS_DATA_04			0x3F
#define B0_EXT_SLV_SENS_DATA_05			0x40
#define B0_EXT_SLV_SENS_DATA_06			0x41
#define B0_EXT_SLV_SENS_DATA_07			0x42
#define B0_EXT_SLV_SENS_DATA_08			0x43
#define B0_EXT_SLV_SENS_DATA_09			0x44
#define B0_EXT_SLV_SENS_DATA_10			0x45
#define B0_EXT_SLV_SENS_DATA_11			0x46
#define B0_EXT_SLV_SENS_DATA_12			0x47
#define B0_EXT_SLV_SENS_DATA_13			0x48
#define B0_EXT_SLV_SENS_DATA_14			0x49
#define B0_EXT_SLV_SENS_DATA_15			0x4A
#define B0_EXT_SLV_SENS_DATA_16			0x4B
#define B0_EXT_SLV_SENS_DATA_17			0x4C
#define B0_EXT_SLV_SENS_DATA_18			0x4D
#define B0_EXT_SLV_SENS_DATA_19			0x4E
#define B0_EXT_SLV_SENS_DATA_20			0x4F
#define B0_EXT_SLV_SENS_DATA_21			0x50
#define B0_EXT_SLV_SENS_DATA_22			0x51
#define B0_EXT_SLV_SENS_DATA_23			0x52
#define B0_FIFO_EN_1					0x66
#define B0_FIFO_EN_2					0x67
#define B0_FIFO_RST						0x68
#define B0_FIFO_MODE					0x69
#define B0_FIFO_COUNTH					0X70
#define B0_FIFO_COUNTL					0X71
#define B0_FIFO_R_W						0x72
#define B0_DATA_RDY_STATUS				0x74
#define B0_FIFO_CFG						0x76

// USER BANK 1
#define B1_SELF_TEST_X_GYRO				0x02
#define B1_SELF_TEST_Y_GYRO				0x03
#define B1_SELF_TEST_Z_GYRO				0x04
#define B1_SELF_TEST_X_ACCEL			0x0E
#define B1_SELF_TEST_Y_ACCEL			0x0F
#define B1_SELF_TEST_Z_ACCEL			0x10
#define B1_XA_OFFS_H					0x14
#define B1_XA_OFFS_L					0x15
#define B1_YA_OFFS_H					0x17
#define B1_YA_OFFS_L					0x18
#define B1_ZA_OFFS_H					0x1A
#define B1_ZA_OFFS_L					0x1B
#define B1_TIMEBASE_CORRECTION_PLL		0x28

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV				0x00
#define B2_GYRO_CONFIG_1				0x01
#define B2_GYRO_CONFIG_2				0x02
#define B2_XG_OFFS_USRH					0x03
#define B2_XG_OFFS_USRL 				0x04
#define B2_YG_OFFS_USRH					0x05
#define B2_YG_OFFS_USRL					0x06
#define B2_ZG_OFFS_USRH					0x07
#define B2_ZG_OFFS_USRL					0x08
#define B2_ODR_ALIGN_EN					0x09
#define B2_ACCEL_SMPLRT_DIV_1			0x10
#define B2_ACCEL_SMPLRT_DIV_2			0x11
#define B2_ACCEL_INTEL_CTRL				0x12
#define B2_ACCEL_WOM_THR				0x13
#define B2_ACCEL_CONFIG_1				0x14
#define B2_ACCEL_CONFIG_2				0x15
#define B2_FSYNC_CONFIG					0x52
#define B2_TEMP_CONFIG					0x53
#define B2_MOD_CTRL_USR					0X54

// USER BANK 3
#define B3_I2C_MST_ODR_CONFIG			0x00
#define B3_I2C_MST_CTRL					0x01
#define B3_I2C_MST_DELAY_CTRL			0x02
#define B3_I2C_SLV0_ADDR				0x03
#define B3_I2C_SLV0_REG					0x04
#define B3_I2C_SLV0_CTRL				0x05
#define B3_I2C_SLV0_DO					0x06
#define B3_I2C_SLV1_ADDR				0x07
#define B3_I2C_SLV1_REG					0x08
#define B3_I2C_SLV1_CTRL				0x09
#define B3_I2C_SLV1_DO					0x0A
#define B3_I2C_SLV2_ADDR				0x0B
#define B3_I2C_SLV2_REG					0x0C
#define B3_I2C_SLV2_CTRL				0x0D
#define B3_I2C_SLV2_DO					0x0E
#define B3_I2C_SLV3_ADDR				0x0F
#define B3_I2C_SLV3_REG					0x10
#define B3_I2C_SLV3_CTRL				0x11
#define B3_I2C_SLV3_DO					0x12
#define B3_I2C_SLV4_ADDR				0x13
#define B3_I2C_SLV4_REG					0x14
#define B3_I2C_SLV4_CTRL				0x15
#define B3_I2C_SLV4_DO					0x16
#define B3_I2C_SLV4_DI					0x17

#define SAMPLE_TIME_MS_USB  20
#define SAMPLE_TIME_MS_BAR  125
#define SAMPLE_TIME_MS_LED  250
#define SAMPLE_TIME_MS_ATT  10

#define COMP_FILT_ALPHA     0.9500000000  //alpha of complimentary filter
#define RAD_TO_DEG			57.2957795131 //radian to degrees conversion (180/Pi)
#define G				9.8100000000 //accelaration due to gravity

/*
 Copyright (C) 2014  Cagdas Caglak cagdascaglak@gmail.com http://expcodes.blogspot.com.tr/

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MPU6050_H
#define    MPU6050_H

#include "I2cPort.h"
#include <math.h>

#define SELF_TEST_X				0x0D
#define SELF_TEST_Y				0x0E
#define SELF_TEST_Z				0x0F
#define SELF_TEST_A				0x10
#define SMPLRT_DIV				0x19
#define CONFIG					0x1A
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define MOT_THR					0x1F
#define FIFO_EN					0x23
#define I2C_MST_CTRL			0x24
#define I2C_SLV0_ADDR			0x25
#define I2C_SLV0_REG			0x26
#define I2C_SLV0_CTRL			0x27
#define I2C_SLV1_ADDR			0x28
#define I2C_SLV1_REG			0x29
#define I2C_SLV1_CTRL			0x2A
#define I2C_SLV2_ADDR			0x2B
#define I2C_SLV2_REG			0x2C
#define I2C_SLV2_CTRL			0x2D
#define I2C_SLV3_ADDR			0x2E
#define I2C_SLV3_REG			0x2F
#define I2C_SLV3_CTRL			0x30
#define I2C_SLV4_ADDR			0x31
#define I2C_SLV4_REG			0x32
#define I2C_SLV4_DO				0x33
#define I2C_SLV4_CTRL			0x34
#define I2C_SLV4_DI				0x35
#define I2C_MST_STATUS			0x36
#define INT_PIN_CFG				0x37
#define INT_ENABLE				0x38
#define INT_STATUS				0x3A
#define ACCEL_XOUT_H			0x3B
#define ACCEL_XOUT_L			0x3C
#define ACCEL_YOUT_H			0x3D
#define ACCEL_YOUT_L			0x3E
#define ACCEL_ZOUT_H			0x3F
#define ACCEL_ZOUT_L			0x40
#define TEMP_OUT_H				0x41
#define TEMP_OUT_L				0x42
#define GYRO_XOUT_H				0x43
#define GYRO_XOUT_L				0x44
#define GYRO_YOUT_H				0x45
#define GYRO_YOUT_L				0x46
#define GYRO_ZOUT_H				0x47
#define GYRO_ZOUT_L				0x48
#define I2C_SLV0_DO				0x63
#define I2C_SLV1_DO				0x64
#define I2C_SLV2_DO				0x65
#define I2C_SLV3_DO				0x66
#define I2C_MST_DELAY_CTRL		0x67
//#define SIGNAL_PATH_RESET		0x68
#define MOT_DETECT_CTRL			0x69
#define USER_CTRL				0x6A
#define PWR_MGMT_1				0x6B
#define PWR_MGMT_2				0x6C
#define FIFO_COUNTH				0x72
#define FIFO_COUNTL				0x73
#define FIFO_R_W				0x74
#define WHO_AM_I				0x75
#define MPU6050_DEV_ADD			0x68
#define AFS_SEL					0x1C
#define FS_SEL					0x1B
#define TEMP_FIFO_EN_BIT		7
#define XG_FIFO_EN_BIT			6
#define YG_FIFO_EN_BIT			5
#define ZG_FIFO_EN_BIT			4
#define ACCEL_FIFO_EN_BIT		3
#define SLV2_FIFO_EN_BIT		2
#define SLV1_FIFO_EN_BIT		1
#define SLV0_FIFO_EN_BIT		0
#define DEV_RESET_BIT			7
#define FIFO_EN_BIT				6
#define FIFO_RESET_BIT			2

//DEFINED BY DEAN HOVINGHOFF
#define RA_BANK_SEL     				0x6D
#define RA_MEM_START_ADDR				0x6E
#define RA_MEM_R_W      				0x6F
#define RA_XG_OFFS_TC					0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define TC_OFFSET_LENGTH				6
#define TC_OFFSET_BIT					6
#define RA_XG_OFFS_TC   				0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_YG_OFFS_TC   				0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_ZG_OFFS_TC   				0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_USER_CTRL					0x6A
#define USERCTRL_I2C_MST_EN_BIT 		5
#define RA_I2C_SLV0_ADDR				0x25
#define USERCTRL_I2C_MST_RESET_BIT      1
#define DMP_MEMORY_CHUNK_SIZE			16
#define RA_INT_ENABLE					0x38
#define PWR1_CLKSEL_LENGTH      		3
#define PWR1_CLKSEL_BIT 				2
#define CLOCK_PLL_XGYRO					0x01
#define CLOCK_PLL_YGYRO					0x02
#define CLOCK_PLL_ZGYRO					0x03
#define RA_CONFIG 						0x1A
#define CFG_EXT_SYNC_SET_LENGTH 		3
#define CFG_EXT_SYNC_SET_BIT			5
#define CFG_DLPF_CFG_BIT				2
#define CFG_DLPF_CFG_LENGTH				3
#define RA_GYRO_CONFIG      			0x1B
#define GCONFIG_FS_SEL_BIT      		4
#define GCONFIG_FS_SEL_LENGTH   		2
#define GYRO_FS_2000        			0x03
#define EXT_SYNC_TEMP_OUT_L    	 		0x1
#define RA_DMP_CFG_1       				0x70
#define RA_DMP_CFG_2       				0x71
#define RA_XG_OFFS_TC       			0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define TC_OTP_BNK_VLD_BIT  			0
#define USERCTRL_FIFO_RESET_BIT         2
#define RA_FIFO_COUNTH      			0x72
#define RA_FIFO_R_W         			0x74
#define RA_MOT_THR          			0x1F
#define RA_ZRMOT_THR        			0x21
#define RA_ZRMOT_DUR        			0x22
#define USERCTRL_FIFO_EN_BIT            6
#define USERCTRL_DMP_EN_BIT             7
#define USERCTRL_DMP_RESET_BIT          3
#define RA_INT_STATUS       			0x3A
#define RA_ZA_OFFS_H        			0x0A
#define DLPF_BW_42          			0x03
#define MPU6050_RA_MOT_DUR         	 	0x20
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16
#define MPU6050_RA_MEM_R_W          	0x6F


using namespace bigowl_i2cport;

namespace bigowl_mpu6050 {

    class MPU6050 {
    public:
        MPU6050(I2cPort *i2c);

        virtual ~MPU6050();

        void initialize();

        void reset();

        void setDeviceAddress(uint8_t DEV_ADD);

        uint8_t getDeviceAddress() const;

        void setSleepMode(bool mode);

        bool getSleepMode();

        void setRangeAcceleration(uint8_t range);

        uint8_t getRangeAcceleration();

        void setRangeGyroscope(uint8_t range);

        uint8_t getRangeGyroscope();

        void getAccelerations(int16_t *accels);

        int16_t getAccelerationX();

        int16_t getAccelerationY();

        int16_t getAccelerationZ();

        void getAngularVelocities(int16_t *gyros);

        int16_t getAngularVelocityX();

        int16_t getAngularVelocityY();

        int16_t getAngularVelocityZ();

        void getMotions6(int16_t *motion6);

        int16_t getTemperature();

        void setDLPFMode(uint8_t mode);

        uint8_t getDLPFMode();

        void setSampleRate(uint8_t rate);

        uint8_t getSampleRate();

        void setMotionDetectionThresold(uint8_t value);

        uint8_t getMotionDetectionThresold();

        void setTEMP_FIFO_EN(uint8_t value);

        uint8_t getTEMP_FIFO_EN();

        void setXG_FIFO_EN(uint8_t value);

        uint8_t getXG_FIFO_EN();

        void setYG_FIFO_EN(uint8_t value);

        uint8_t getYG_FIFO_EN();

        void setZG_FIFO_EN(uint8_t value);

        uint8_t getZG_FIFO_EN();

        void setACCEL_FIFO_EN(uint8_t value);

        uint8_t getACCEL_FIFO_EN();

        void setSLV2_FIFO_EN(uint8_t value);

        uint8_t getSLV2_FIFO_EN();

        void setSLV1_FIFO_EN(uint8_t value);

        uint8_t getSLV1_FIFO_EN();

        void setSLV0_FIFO_EN(uint8_t value);

        uint8_t getSLV0_FIFO_EN();

        uint16_t getFIFO_Count();

        void setFIFO_Enable(uint8_t value);

        uint8_t getFIFO_Enable();

        void getFIFO_Data(uint8_t *data, uint8_t length);

        void setFIFO_Reset(uint8_t value);

        uint8_t getFIFO_Reset();
		//ADDED BY DEAN HOVINGHOFF
		
		void setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank);
		void setMemoryStartAddress(uint8_t address);
		uint8_t readMemoryByte();
		uint8_t getOTPBankValid();
		int8_t getXGyroOffsetTC();
		void setXGyroOffsetTC(int8_t offset);
		int8_t getYGyroOffsetTC();
		void setYGyroOffsetTC(int8_t offset);
		int8_t getZGyroOffsetTC();
		void setZGyroOffsetTC(int8_t offset);
		void setSlaveAddress(uint8_t num, uint8_t address);
		void setI2CMasterModeEnabled(bool enabled);
		void resetI2CMaster();
		bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);
		bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem);
		bool writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);
		bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem);
		void setClockSource(uint8_t source);
		void setIntEnabled(uint8_t enabled);
		void setRate(uint8_t rate);
		void setExternalFrameSync(uint8_t sync);
		void setFullScaleGyroRange(uint8_t range);
		void setDMPConfig1(uint8_t config);
		void setDMPConfig2(uint8_t config);
		void setOTPBankValid(bool enabled);
		void resetFIFO();
		uint16_t getFIFOCount();
		void getFIFOBytes(uint8_t *data, uint8_t length);
		void setMotionDetectionThreshold(uint8_t threshold);
		void setZeroMotionDetectionThreshold(uint8_t threshold);
		uint8_t getZeroMotionDetectionDuration();
		void setZeroMotionDetectionDuration(uint8_t duration);
		void setFIFOEnabled(bool enabled);
		void setDMPEnabled(bool enabled);
		void resetDMP();
		uint8_t getIntStatus();
		int16_t getZAccelOffsetTC();
		void setZAccelOffsetTC(int16_t offset);
		void setMotionDetectionDuration(uint8_t duration);
		void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
		
		// special methods for MotionApps 2.0 implementation
        #ifdef INCLUDE_DMP_MOTIONAPPS20
            uint8_t *dmpPacketBuffer;
            uint16_t dmpPacketSize;

            uint8_t dmpInitialize();
            bool dmpPacketAvailable();

            uint8_t dmpSetFIFORate(uint8_t fifoRate);
            uint8_t dmpGetFIFORate();
            uint8_t dmpGetSampleStepSizeMS();
            uint8_t dmpGetSampleFrequency();
            int32_t dmpDecodeTemperature(int8_t tempReg);
            
            // Register callbacks after a packet of FIFO data is processed
            //uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
            //uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
            uint8_t dmpRunFIFORateProcesses();
            
            // Setup FIFO for various output
            uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
            uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
            uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
            uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

            // Get Fixed Point data from FIFO
            uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
            uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
            uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
            uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
            uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
            uint8_t dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
            uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
            uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet=0);
            
            uint8_t dmpGetEuler(float *data, Quaternion *q);
            uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

            // Get Floating Point data from FIFO
            uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet=0);
            uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

            uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
            uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

            uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

            uint8_t dmpInitFIFOParam();
            uint8_t dmpCloseFIFO();
            uint8_t dmpSetGyroDataSource(uint8_t source);
            uint8_t dmpDecodeQuantizedAccel();
            uint32_t dmpGetGyroSumOfSquare();
            uint32_t dmpGetAccelSumOfSquare();
            void dmpOverrideQuaternion(long *q);
            uint16_t dmpGetFIFOPacketSize();
        #endif

    private:
        I2cPort *i2c;
        uint8_t device_address;

    };
}  // namespace bigowl_mpu6050
#endif	/* MPU6050_H */


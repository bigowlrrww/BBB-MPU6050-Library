#ifndef _6AXIS_MOTIONAPPS20_H_
#define _6AXIS_MOTIONAPPS20_H_

#include <stdio.h>
#include <string.h>
#include "I2cPort.h"
#include "helper_3dmath.h"
#include <csignal>
#include <iostream>
#include <bitset>
#include "Debug.h"
#include "MPU6050.h"
// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define INCLUDE_DMP_MOTIONAPPS20


// Tom Carpenter's conditional PROGMEM code
// http://forum.arduino.cc/index.php?topic=129407.0
// Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
#ifndef __PGMSPACE_H_
	#define __PGMSPACE_H_ 1
	#include <inttypes.h>

	#define PROGMEM
	#define PGM_P  const char *
	#define PSTR(str) (str)
	#define F(x) x

	typedef void prog_void;
	typedef char prog_char;
	typedef unsigned char prog_uchar;
	typedef int8_t prog_int8_t;
	typedef uint8_t prog_uint8_t;
	typedef int16_t prog_int16_t;
	typedef uint16_t prog_uint16_t;
	typedef int32_t prog_int32_t;
	typedef uint32_t prog_uint32_t;
	
	#define strcpy_P(dest, src) strcpy((dest), (src))
	#define strcat_P(dest, src) strcat((dest), (src))
	#define strcmp_P(a, b) strcmp((a), (b))
	
	#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
	#define pgm_read_word(addr) (*(const unsigned short *)(addr))
	#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
	#define pgm_read_float(addr) (*(const float *)(addr))
	
	#define pgm_read_byte_near(addr) pgm_read_byte(addr)
	#define pgm_read_word_near(addr) pgm_read_word(addr)
	#define pgm_read_dword_near(addr) pgm_read_dword(addr)
	#define pgm_read_float_near(addr) pgm_read_float(addr)
	#define pgm_read_byte_far(addr) pgm_read_byte(addr)
	#define pgm_read_word_far(addr) pgm_read_word(addr)
	#define pgm_read_dword_far(addr) pgm_read_dword(addr)
	#define pgm_read_float_far(addr) pgm_read_float(addr)
#endif

/* Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).

#define DMP_CODE_SIZE       1929   // dmpMemory[] //small 1929 // large program is 3062
#define DMP_CONFIG_SIZE     192     // dmpConfig[]
#define DMP_UPDATES_SIZE    47      // dmpUpdates[]
namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };
}
namespace bigowl_mpu6050
{
	class MPU6050DMP 
	{
    public:
		MPU6050DMP(I2cPort *i2c);
		
		uint8_t dmpInitialize(MPU6050 *mpu6050);

		bool dmpPacketAvailable();

		// uint8_t dmpSetFIFORate(uint8_t fifoRate);
		// uint8_t dmpGetFIFORate();
		// uint8_t dmpGetSampleStepSizeMS();
		// uint8_t dmpGetSampleFrequency();
		// int32_t dmpDecodeTemperature(int8_t tempReg);

		//uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
		//uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
		//uint8_t dmpRunFIFORateProcesses();

		// uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
		// uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
		// uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
		// uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

		uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet);
		uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet);
		uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet);
		uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet);
		uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet);
		uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
		// uint8_t dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
		// uint8_t dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
		uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet);
		uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet);
		uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet);
		// uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
		// uint8_t dmpGetLinearAccel(long *data, const uint8_t* packet);
		uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
		// uint8_t dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
		uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
		// uint8_t dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
		// uint8_t dmpGetGyroSensor(long *data, const uint8_t* packet);
		// uint8_t dmpGetControlData(long *data, const uint8_t* packet);
		// uint8_t dmpGetTemperature(long *data, const uint8_t* packet);
		// uint8_t dmpGetGravity(long *data, const uint8_t* packet);
		uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
		// uint8_t dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
		// uint8_t dmpGetQuantizedAccel(long *data, const uint8_t* packet);
		// uint8_t dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
		// uint8_t dmpGetEIS(long *data, const uint8_t* packet);

		uint8_t dmpGetEuler(float *data, Quaternion *q);
		uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

		// uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet);
		// uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet);

		uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
		uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);

		// uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

		// uint8_t dmpInitFIFOParam();
		// uint8_t dmpCloseFIFO();
		// uint8_t dmpSetGyroDataSource(uint_fast8_t source);
		// uint8_t dmpDecodeQuantizedAccel();
		// uint32_t dmpGetGyroSumOfSquare();
		// uint32_t dmpGetAccelSumOfSquare();
		// void dmpOverrideQuaternion(long *q);
		uint16_t dmpGetFIFOPacketSize();
	private:
		I2cPort *i2c;
        uint8_t device_address;
	};
}
#endif /* _6AXIS_MOTIONAPPS20_H_ */
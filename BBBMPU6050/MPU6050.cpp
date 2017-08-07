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

#include "MPU6050.h"

namespace bigowl_mpu6050 {


    MPU6050::MPU6050(I2cPort *i2c) {
        this->i2c = i2c;
    }

    MPU6050::~MPU6050() {
        delete i2c;
    }

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
    void MPU6050::initialize() {
        setRangeAcceleration(0);
        setRangeGyroscope(0);
        setSleepMode(false);
    }

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see PWR_MGMT_1
 * @see DEV_RESET_BIT
 */
    void MPU6050::reset() {
        i2c->writeBit(PWR_MGMT_1, 1, DEV_RESET_BIT);
    }

/** Set device address.
 * @param dev_addr device address
 * @see DEF_DEV_ADD
 */
    void MPU6050::setDeviceAddress(uint8_t DEV_ADD) {
        this->device_address = DEV_ADD;
    }

/** Get device address.
 * @return device address
 */
    uint8_t MPU6050::getDeviceAddress() const {
        return this->device_address;
    }

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see PWR_MGMT_1
 */
    void MPU6050::setSleepMode(bool mode) {
        int8_t byte = i2c->readByte(PWR_MGMT_1);
        if (mode) {
            byte |= 64;
        } else {
            byte &= ~64;
        }
        i2c->writeByte(PWR_MGMT_1, byte);
    }

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see PWR_MGMT_1
 */
    bool MPU6050::getSleepMode() {
        int8_t byte = i2c->readByte(PWR_MGMT_1);
        byte >>= 6;
        byte %= 2;
        if (byte == 1) {
            return true;
        } else {
            return false;
        }
    }

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getRangeAcceleration()
 */
    void MPU6050::setRangeAcceleration(uint8_t range) {
        uint8_t afs;
        afs = i2c->readByte(ACCEL_CONFIG);
        if (range == 0) {
            afs &= ~24;
        } else if (range == 1) {
            afs |= 8;
            afs &= ~16;
        } else if (range == 2) {
            afs &= ~8;
            afs |= 16;
        } else {
            afs |= 24;
        }
        i2c->writeByte(ACCEL_CONFIG, afs);
    }

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see ACCEL_CONFIG
 */
    uint8_t MPU6050::getRangeAcceleration() {
        uint8_t afs;
        afs = i2c->readByte(ACCEL_CONFIG);
        afs >>= 3;
        afs %= 4;
        return afs;
    }

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getRangeGyroscope()
 * @see GYRO_CONFIG
 */
    void MPU6050::setRangeGyroscope(uint8_t range) {
        uint8_t fs;
        fs = i2c->readByte(GYRO_CONFIG);
        if (range == 0) {
            fs &= ~24;
        } else if (range == 1) {
            fs |= 8;
            fs &= ~16;
        } else if (range == 2) {
            fs &= ~8;
            fs |= 16;
        } else {
            fs |= 24;
        }
        i2c->writeByte(GYRO_CONFIG, fs);
    }

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see GYRO_CONFIG
 */
    uint8_t MPU6050::getRangeGyroscope() {
        uint8_t fs;
        fs = i2c->readByte(GYRO_CONFIG);
        fs >>= 3;
        fs %= 4;
        return fs;
    }

    void MPU6050::getAccelerations(int16_t *accels) {
        accels[0] = i2c->readWord(ACCEL_XOUT_H, ACCEL_XOUT_L);
        accels[1] = i2c->readWord(ACCEL_YOUT_H, ACCEL_YOUT_L);
        accels[2] = i2c->readWord(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
    }

    int16_t MPU6050::getAccelerationX() {
        return i2c->readWord(ACCEL_XOUT_H, ACCEL_XOUT_L);
    }

    int16_t MPU6050::getAccelerationY() {
        return i2c->readWord(ACCEL_YOUT_H, ACCEL_YOUT_L);
    }

    int16_t MPU6050::getAccelerationZ() {
        return i2c->readWord(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
    }

    void MPU6050::getAngularVelocities(int16_t *gyros) {
        gyros[0] = i2c->readWord(GYRO_XOUT_H, GYRO_XOUT_L);
        gyros[1] = i2c->readWord(GYRO_YOUT_H, GYRO_YOUT_L);
        gyros[2] = i2c->readWord(GYRO_ZOUT_H, GYRO_ZOUT_L);
    }

    int16_t MPU6050::getAngularVelocityX() {
        return i2c->readWord(GYRO_XOUT_H, GYRO_XOUT_L);
    }

    int16_t MPU6050::getAngularVelocityY() {
        return i2c->readWord(GYRO_YOUT_H, GYRO_YOUT_L);
    }

    int16_t MPU6050::getAngularVelocityZ() {
        return i2c->readWord(GYRO_ZOUT_H, GYRO_ZOUT_L);
    }

    int16_t MPU6050::getTemperature() {
        return i2c->readWord(TEMP_OUT_H, TEMP_OUT_L);
    }

    void MPU6050::getMotions6(int16_t *motion6) {
        motion6[0] = i2c->readWord(ACCEL_XOUT_H, ACCEL_XOUT_L);
        motion6[1] = i2c->readWord(ACCEL_YOUT_H, ACCEL_YOUT_L);
        motion6[2] = i2c->readWord(ACCEL_ZOUT_H, ACCEL_ZOUT_L);

        motion6[3] = i2c->readWord(GYRO_XOUT_H, GYRO_XOUT_L);
        motion6[4] = i2c->readWord(GYRO_YOUT_H, GYRO_YOUT_L);
        motion6[5] = i2c->readWord(GYRO_ZOUT_H, GYRO_ZOUT_L);
    }

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFMode()
 * @see CONFIG
 */
    void MPU6050::setDLPFMode(uint8_t mode) {
        uint8_t config = i2c->readByte(CONFIG);
        if (mode == 0) {
            config = config & ~7;
        } else if (mode == 1) {
            config |= 1;
            config &= ~6;
        } else if (mode == 2) {
            config |= 2;
            config &= ~5;
        } else if (mode == 3) {
            config |= 3;
            config &= ~4;
        } else if (mode == 4) {
            config |= 4;
            config &= ~3;
        } else if (mode == 5) {
            config |= 5;
            config &= ~2;
        } else if (mode == 6) {
            config |= 6;
            config &= ~1;
        } else if (mode == 7) {
            config |= 7;
        }

        i2c->writeByte(CONFIG, config);
    }

/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see CONFIG
 */
    uint8_t MPU6050::getDLPFMode() {
        uint8_t config = i2c->readByte(CONFIG);
        return config % 8;
    }

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getSampleRate()
 * @see SMPLRT_DIV
 */
    void MPU6050::setSampleRate(uint8_t rate) {
        i2c->writeByte(SMPLRT_DIV, rate);
    }

/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current sample rate
 * @see SMPLRT_DIV
 */
    uint8_t MPU6050::getSampleRate() {
        return i2c->readByte(SMPLRT_DIV);
    }

    void MPU6050::setMotionDetectionThresold(uint8_t value) {
        i2c->writeByte(MOT_THR, value);
    }

    uint8_t MPU6050::getMotionDetectionThresold() {
        return i2c->readByte(MOT_THR);
    }

    void MPU6050::setTEMP_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, TEMP_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getTEMP_FIFO_EN() {
        return i2c->readBit(FIFO_EN, TEMP_FIFO_EN_BIT);
    }

    void MPU6050::setXG_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, XG_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getXG_FIFO_EN() {
        return i2c->readBit(FIFO_EN, XG_FIFO_EN_BIT);
    }

    void MPU6050::setYG_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, YG_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getYG_FIFO_EN() {
        return i2c->readBit(FIFO_EN, YG_FIFO_EN_BIT);
    }

    void MPU6050::setZG_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, ZG_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getZG_FIFO_EN() {
        return i2c->readBit(FIFO_EN, ZG_FIFO_EN_BIT);
    }

    void MPU6050::setACCEL_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, ACCEL_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getACCEL_FIFO_EN() {
        return i2c->readBit(FIFO_EN, ACCEL_FIFO_EN_BIT);
    }

    void MPU6050::setSLV2_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, SLV2_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getSLV2_FIFO_EN() {
        return i2c->readBit(FIFO_EN, SLV2_FIFO_EN_BIT);
    }

    void MPU6050::setSLV1_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, SLV1_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getSLV1_FIFO_EN() {
        return i2c->readBit(FIFO_EN, SLV1_FIFO_EN_BIT);
    }

    void MPU6050::setSLV0_FIFO_EN(uint8_t value) {
        i2c->writeBit(FIFO_EN, value, SLV0_FIFO_EN_BIT);
    }

    uint8_t MPU6050::getSLV0_FIFO_EN() {
        return i2c->readBit(FIFO_EN, SLV0_FIFO_EN_BIT);
    }

    uint16_t MPU6050::getFIFO_Count() {
        return i2c->readWord(FIFO_COUNTH, FIFO_COUNTL);
    }

    void MPU6050::setFIFO_Enable(uint8_t value) {
        i2c->writeBit(USER_CTRL, value, FIFO_EN_BIT);
    }

    uint8_t MPU6050::getFIFO_Enable() {
        return i2c->readBit(USER_CTRL, FIFO_EN_BIT);
    }

    void MPU6050::getFIFO_Data(uint8_t *data, uint8_t length) {
        i2c->readByteBuffer(FIFO_R_W, data, length);
    }

    void MPU6050::setFIFO_Reset(uint8_t value) {
        i2c->writeBit(USER_CTRL, value, FIFO_RESET_BIT);
    }

    uint8_t MPU6050::getFIFO_Reset() {
        return i2c->readBit(USER_CTRL, FIFO_RESET_BIT);
    }
	//ADDED BY DEAN HOVINGHOFF
	void MPU6050::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
		bank &= 0x1F; //bank == bank AND 0x1F Bitwise Operand
		if (userBank) bank |= 0x20; //bank == bank OR 0x20 Bitwise Operand
		if (prefetchEnabled) bank |= 0x40; //bank == bank OR 0x40 Bitwise Operand 
		i2c->writeByte(RA_BANK_SEL, bank);
	}
	
	void MPU6050::setMemoryStartAddress(uint8_t address) {
		i2c->writeByte(RA_MEM_START_ADDR, address);
	}
	
	uint8_t MPU6050::readMemoryByte() {
		int8_t buffer;
		buffer = i2c->readByte(RA_MEM_R_W);
		return buffer;
	}
	
	uint8_t MPU6050::getOTPBankValid() {
		int8_t buffer;
		buffer = i2c->readBit(RA_XG_OFFS_TC, TC_OTP_BNK_VLD_BIT);
		return buffer;
	}
	
	//XGyro VAL
	int8_t MPU6050::getXGyroOffsetTC() {
		int8_t buffer;
		buffer = i2c->readMoreBits(RA_XG_OFFS_TC, TC_OFFSET_LENGTH, TC_OFFSET_BIT);
		return buffer;
	}
	
	void MPU6050::setXGyroOffsetTC(int8_t offset) {
		i2c->writeMoreBits(RA_XG_OFFS_TC, offset, TC_OFFSET_LENGTH, TC_OFFSET_BIT);
	}
	
	//YGyro VAL
	int8_t MPU6050::getYGyroOffsetTC() {
		int8_t buffer;
		buffer = i2c->readMoreBits(RA_YG_OFFS_TC, TC_OFFSET_LENGTH, TC_OFFSET_BIT);
		return buffer;
	}
	
	void MPU6050::setYGyroOffsetTC(int8_t offset) {
		i2c->writeMoreBits(RA_YG_OFFS_TC, offset, TC_OFFSET_LENGTH, TC_OFFSET_BIT);
	}
	
	//ZGyro VAL
	int8_t MPU6050::getZGyroOffsetTC() {
		int8_t buffer;
		buffer = i2c->readMoreBits(RA_ZG_OFFS_TC, TC_OFFSET_LENGTH, TC_OFFSET_BIT);
		return buffer;
	}
	
	void MPU6050::setZGyroOffsetTC(int8_t offset) {
		i2c->writeMoreBits(RA_ZG_OFFS_TC, offset, TC_OFFSET_LENGTH, TC_OFFSET_BIT);
	}
	
	//Slave Handleing
	void MPU6050::setSlaveAddress(uint8_t num, uint8_t address) {
		if (num > 3) return;
		i2c->writeByte(RA_I2C_SLV0_ADDR + num*3, address);
	}
	
	void MPU6050::setI2CMasterModeEnabled(bool enabled) {
		i2c->writeBit(RA_USER_CTRL, enabled, USERCTRL_I2C_MST_EN_BIT);
	}
	
	void MPU6050::resetI2CMaster() {
		i2c->writeBit(USER_CTRL, 1, USERCTRL_I2C_MST_RESET_BIT);
	}
	
	bool MPU6050::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
		return writeMemoryBlock(data, dataSize, bank, address, verify, false);
	}
	
	bool MPU6050::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem) {
		
		setMemoryBank(bank, false, false);
		setMemoryStartAddress(address);
		uint8_t chunkSize;
		uint8_t *verifyBuffer;
		uint8_t *progBuffer;
		uint8_t *progBufferT[16]; //temparary buffer array to load into progBuffer
		uint16_t i;
		uint8_t j;
		if (verify) verifyBuffer = (uint8_t *)malloc(DMP_MEMORY_CHUNK_SIZE);
		for (i = 0; i < dataSize;) {
			// determine correct chunk size according to bank position and data size
			chunkSize = DMP_MEMORY_CHUNK_SIZE;

			// make sure we don't go past the data size
			if (i + chunkSize > dataSize) chunkSize = dataSize - i; //if i+16 > all data chunk = all data-i

			// make sure this chunk doesn't go past the bank boundary (256 bytes)
			if (chunkSize > 256 - address) chunkSize = 256 - address;
			
			// write the chunk of data as specified
			for (j = 0; j < chunkSize; j++) 
			{
				progBufferT[j] = (uint8_t *)data + i + j;
			}
			progBuffer = (uint8_t *) *progBufferT; 

			i2c->writeByteBuffer(RA_MEM_R_W, progBuffer, chunkSize);

			// verify data if needed
			if (verify && verifyBuffer) {
				DEBUG_PRINTLN("Verifying MEMORY BANK");
				setMemoryBank(bank, false, false);
				setMemoryStartAddress(address);
				i2c->readByteBuffer(RA_MEM_R_W, verifyBuffer, chunkSize);
				if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
					std::cout << "Block write verification error, bank 0x";
					std::cout << std::hex <<static_cast<int>(bank); // needs to read as dec
					std::cout << ", address 0x";
					std::cout << std::hex <<static_cast<int>(address); // needs to read as dec
					std::cout << "!\nExpected:";
					for (j = 0; j < chunkSize; j++) {
						std::cout << " 0x";
						if (progBuffer[j] < 16) std::cout << "0";
						std::cout << std::hex << static_cast<int>(progBuffer[j]);
					}
					std::cout << "\nReceived:";
					for (uint8_t j = 0; j < chunkSize; j++) {
						std::cout << " 0x";
						if (verifyBuffer[i + j] < 16) std::cout << "0";
						std::cout << std::hex << static_cast<int>(verifyBuffer[i + j]);
					}
					std::cout << "" << std:: endl;
					free(verifyBuffer);
					if (useProgMem) free(progBuffer);
					return false; // uh oh.
				}
				DEBUG_PRINTLN("Writing MEMORY BANK");
			}

			// increase byte index by [chunkSize]
			i += chunkSize;

			// uint8_t automatically wraps to 0 at 256
			address += chunkSize;

			// if we aren't done, update bank (if necessary) and address
			if (i < dataSize) {
				if (address == 0) bank++;
				setMemoryBank(bank, false, false);
				setMemoryStartAddress(address);
				usleep(20000);
			}
		}
		if (verify) free(verifyBuffer);
		if (useProgMem) free(progBuffer);
		return true;
	}
	
	bool MPU6050::writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
		return writeDMPConfigurationSet(data, dataSize, false);
	}
	
	bool MPU6050::writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem) {
		uint8_t *progBuffer, success, special;
		uint16_t i, j;
		if (useProgMem) {
			progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
		}

		// config set data is a long string of blocks with the following structure:
		// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
		uint8_t bank, offset, length;
		for (i = 0; i < dataSize;) {
			if (useProgMem) {
				//bank = pgm_read_byte(data + i++);
				//offset = pgm_read_byte(data + i++);
				//length = pgm_read_byte(data + i++);
			} else {
				bank = data[i++];
				offset = data[i++];
				length = data[i++];
			}

			// write data or perform special action
			if (length > 0) {
				// regular block of data to write
				/*Serial.print("Writing config block to bank ");
				Serial.print(bank);
				Serial.print(", offset ");
				Serial.print(offset);
				Serial.print(", length=");
				Serial.println(length);*/
				if (useProgMem) {
					//if (sizeof(progBuffer) < length) progBuffer = (uint8_t *)realloc(progBuffer, length);
					//for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
				} else {
					progBuffer = (uint8_t *)data + i;
				}
				success = writeMemoryBlock(progBuffer, length, bank, offset, true, false);
				i += length;
			} else {
				// special instruction
				// NOTE: this kind of behavior (what and when to do certain things)
				// is totally undocumented. This code is in here based on observed
				// behavior only, and exactly why (or even whether) it has to be here
				// is anybody's guess for now.
				if (useProgMem) {
					//special = pgm_read_byte(data + i++);
				} else {
					special = data[i++];
				}
				/*Serial.print("Special command code ");
				Serial.print(special, HEX);
				Serial.println(" found...");*/
				if (special == 0x01) {
					// enable DMP-related interrupts
					
					//setIntZeroMotionEnabled(true);
					//setIntFIFOBufferOverflowEnabled(true);
					//setIntDMPEnabled(true);
					i2c->writeByte(0x32, RA_INT_ENABLE);  // single operation

					success = true;
				} else {
					// unknown special command
					success = false;
				}
			}
			
			if (!success) {
				if (useProgMem) free(progBuffer);
				return false; // uh oh
			}
		}
		if (useProgMem) free(progBuffer);
		return true;
	}

	void MPU6050::setClockSource(uint8_t source) {
		i2c->writeMoreBits(PWR_MGMT_1, source, PWR1_CLKSEL_LENGTH, PWR1_CLKSEL_BIT);
	}
	
	void MPU6050::setIntEnabled(uint8_t enabled) {
		i2c->writeByte(RA_INT_ENABLE, enabled);
	}
	
	void MPU6050::setRate(uint8_t rate) {
		i2c->writeByte(SMPLRT_DIV, rate);
	}
	
	void MPU6050::setExternalFrameSync(uint8_t sync) {
		i2c->writeMoreBits(RA_CONFIG, sync, CFG_EXT_SYNC_SET_LENGTH, CFG_EXT_SYNC_SET_BIT);
	}
	
	void MPU6050::setFullScaleGyroRange(uint8_t range) {
		i2c->writeMoreBits(RA_GYRO_CONFIG, range, GCONFIG_FS_SEL_LENGTH, GCONFIG_FS_SEL_BIT);
	}
	
	void MPU6050::setDMPConfig1(uint8_t config) {
		i2c->writeByte( RA_DMP_CFG_1, config);
	}

	void MPU6050::setDMPConfig2(uint8_t config) {
		i2c->writeByte(RA_DMP_CFG_2, config);
	}
	
	void MPU6050::setOTPBankValid(bool enabled) {
		i2c->writeBit(RA_XG_OFFS_TC, enabled, TC_OTP_BNK_VLD_BIT);
	}
	
	void MPU6050::resetFIFO() {
		//setFIFOEnabled(false);
		i2c->writeBit(USER_CTRL, true, USERCTRL_FIFO_RESET_BIT);
		//setFIFOEnabled(true);//
	}
	
	uint16_t MPU6050::getFIFOCount() {
		uint8_t buffer[2];
		i2c->readByteBuffer(RA_FIFO_COUNTH, buffer, 2);
		return (((uint16_t)buffer[0]) << 8) | buffer[1];
	}
	
	void MPU6050::getFIFOBytes(uint8_t *data, uint8_t length) {
		i2c->readByteBuffer(RA_FIFO_R_W, data, length);
	}
	
	void MPU6050::setMotionDetectionThreshold(uint8_t threshold) {
		i2c->writeByte(RA_MOT_THR, threshold);
	}
	
	void MPU6050::setMotionDetectionDuration(uint8_t duration) {
		i2c->writeByte(MPU6050_RA_MOT_DUR, duration);
	}
	
	void MPU6050::setZeroMotionDetectionThreshold(uint8_t threshold) {
		i2c->writeByte(RA_ZRMOT_THR, threshold);
	}

	uint8_t MPU6050::getZeroMotionDetectionDuration() {
		int8_t buffer;
		buffer = i2c->readByte(RA_ZRMOT_DUR);
		return buffer;
	}
	
	void MPU6050::setZeroMotionDetectionDuration(uint8_t duration) {
		i2c->writeByte(RA_ZRMOT_DUR, duration);
	}
	
	void MPU6050::setFIFOEnabled(bool enabled) {
		i2c->writeBit(USER_CTRL, enabled, USERCTRL_FIFO_EN_BIT);
	}

	void MPU6050::setDMPEnabled(bool enabled) {
		i2c->writeBit(USER_CTRL, enabled, USERCTRL_DMP_EN_BIT);
	}

	void MPU6050::resetDMP() {
		i2c->writeBit(USER_CTRL, true, USERCTRL_DMP_RESET_BIT);
	}
	
	uint8_t MPU6050::getIntStatus() {
		int8_t buffer;
		buffer = i2c->readByte(RA_INT_STATUS);
		return buffer;
	}
	void MPU6050::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
		setMemoryBank(bank, false, false);
		setMemoryStartAddress(address);
		uint8_t chunkSize;
		for (uint16_t i = 0; i < dataSize;) {
			// determine correct chunk size according to bank position and data size
			chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

			// make sure we don't go past the data size
			if (i + chunkSize > dataSize) chunkSize = dataSize - i;

			// make sure this chunk doesn't go past the bank boundary (256 bytes)
			if (chunkSize > 256 - address) chunkSize = 256 - address;

			// read the chunk of data as specified
			i2c->readByteBuffer(MPU6050_RA_MEM_R_W, data + i, chunkSize);
			
			// increase byte index by [chunkSize]
			i += chunkSize;

			// uint8_t automatically wraps to 0 at 256
			address += chunkSize;

			// if we aren't done, update bank (if necessary) and address
			if (i < dataSize) {
				if (address == 0) bank++;
				setMemoryBank(bank, false, false);
				setMemoryStartAddress(address);
			}
		}
	}
	//major test zone.
	
}  // namespace bigowl_mpu6050

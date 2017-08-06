#include "AllDevices.h"
#include "DMP-processing.h"
#include "Debug.h"

using namespace bigowl_i2cport;
using namespace bigowl_mpu6050;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
int ctrl;
int main() {

	ctrl = 1;
	signal(SIGINT, signal_handler);

	I2cPort *i2c = new I2cPort(0x68, 1);
	i2c->openConnection();

	MPU6050 *mpu6050 = new MPU6050(i2c);
	mpu6050->initialize();
	MPU6050DMP *mpu6050dmp = new MPU6050DMP(i2c);

    /*float k = 16000;
    //int16_t *accels = (int16_t *) calloc(3, sizeof(int16_t));
    //int16_t *gyros = (int16_t *) calloc(3, sizeof(int16_t));
    //while (ctrl) {
    //    std::cout << "MPU6050" << std::endl;

    //    mpu6050->getAccelerations(accels);
    //    std::cout << "Accel X: " << (float) accels[0] / k << std::endl;
    //    std::cout << "Accel Y: " << (float) accels[1] / k << std::endl;
    //    std::cout << "Accel Z: " << (float) accels[2] / k << std::endl;

    //   mpu6050->getAngularVelocities(gyros);
    //   std::cout << "Gyro X: " << (float) gyros[0] / k << std::endl;
    //   std::cout << "Gyro Y: " << (float) gyros[1] / k << std::endl;
    //   std::cout << "Gyro Z: " << (float) gyros[2] / k << std::endl;

    //   std::cout << "----------------------" << std::endl;
    //   usleep(200000);
    //}*/
	// load and configure the DMP
	std::cout << "Initializing DMP..." << std::endl;
	devStatus = mpu6050dmp->dmpInitialize(mpu6050);

	// supply your own gyro offsets here, scaled for min sensitivity
	
	mpu6050->setXGyroOffsetTC(200); 
	mpu6050->setYGyroOffsetTC(76);  
	mpu6050->setZGyroOffsetTC(-85); 
	
	// not needed? mpu6050->setZAccelOffsetTC(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		std::cout << "Enabling DMP..." << std::endl;
		mpu6050->setDMPEnabled(true);

		// TODO enable Beaglebone interrupt detection
		/*std::cout << "Enabling interrupt detection (Arduino external interrupt 0)..." << std::endl;
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);*/
		//mpuIntStatus = mpu6050.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		std::cout << "DMP ready! Waiting for first interrupt..." << std::endl;
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu6050dmp->dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		std::cout << "DMP Initialization failed (code ";
		std::cout << devStatus;
		std::cout << ")" << std::endl;
	}
		
	// if programming failed, don't try to do anything
	if (!dmpReady) return 1;
	while (ctrl) {
		// wait for MPU interrupt or extra packet(s) available
		while (fifoCount < packetSize) {
			// other program behavior stuff here
			// .
			// .
			// .
			// if you are really paranoid you can frequently test in between other
			// stuff to "break;" from the
			// while() loop to immediately process the MPU data
			// .
			// .
			// .
			fifoCount = mpu6050->getFIFO_Count();
		}
		
		// reset interrupt flag and get INT_STATUS byte
		mpuIntStatus = mpu6050->getIntStatus();

		// get current FIFO count
		fifoCount = mpu6050->getFIFO_Count();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu6050->resetFIFO();
			DEBUG_PRINT("FIFOCURRENT: 0x");
			DEBUG_PRINTLN(fifoCount);
			
			std::cout << "FIFO overflow!" << std::endl;

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize)
			{
				fifoCount = mpu6050->getFIFO_Count();
			}
			// read a packet from FIFO
			mpu6050->getFIFOBytes(fifoBuffer, packetSize);
			
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
				// display quaternion values in easy matrix form: w x y z
			mpu6050dmp.dmpGetQuaternion(&q, fifoBuffer);
			std::cout << "quat\t";
			std::cout << q.w;
			std::cout << "\t";
			std::cout << q.x;
			std::cout << "\t";
			std::cout << q.y;
			std::cout << "\t";
			std::cout << q.z << std::endl;
#endif

#ifdef OUTPUT_READABLE_EULER
				// display Euler angles in degrees
			mpu6050dmp.dmpGetQuaternion(&q, fifoBuffer);
			mpu6050dmp.dmpGetEuler(euler, &q);
			std::cout << "euler\t";
			std::cout << euler[0] * 180 / M_PI;
			std::cout << "\t";
			std::cout << euler[1] * 180 / M_PI;
			std::cout << "\t";
			std::cout << euler[2] * 180 / M_PI << std::endl;
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
				// display Euler angles in degrees
			mpu6050dmp->dmpGetQuaternion(&q, fifoBuffer);
			mpu6050dmp->dmpGetGravity(&gravity, &q);
			mpu6050dmp->dmpGetYawPitchRoll(ypr, &q, &gravity);
			std::cout << "ypr\t";
			std::cout << ypr[0] * 180 / M_PI;
			std::cout << "\t";
			std::cout << ypr[1] * 180 / M_PI;
			std::cout << "\t";
			std::cout << ypr[2] * 180 / M_PI << std::endl;
#endif

#ifdef OUTPUT_READABLE_REALACCEL
				// display real acceleration, adjusted to remove gravity
			mpu6050dmp.dmpGetQuaternion(&q, fifoBuffer);
			mpu6050dmp.dmpGetAccel(&aa, fifoBuffer);
			mpu6050dmp.dmpGetGravity(&gravity, &q);
			mpu6050dmp.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			std::cout << "areal\t";
			std::cout << aaReal.x;
			std::cout << "\t";
			std::cout << aaReal.y;
			std::cout << "\t";
			std::cout << aaReal.z << std::endl;
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
				// display initial world-frame acceleration, adjusted to remove gravity
				// and rotated based on known orientation from quaternion
			mpu6050dmp.dmpGetQuaternion(&q, fifoBuffer);
			mpu6050dmp.dmpGetAccel(&aa, fifoBuffer);
			mpu6050dmp.dmpGetGravity(&gravity, &q);
			mpu6050dmp.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu6050dmp.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			std::cout << "aworld\t";
			std::cout << aaWorld.x;
			std::cout << "\t";
			std::cout << aaWorld.y;
			std::cout << "\t";
			std::cout << aaWorld.z << std::endl;
#endif
		
#ifdef OUTPUT_TEAPOT
		// display quaternion values in InvenSense Teapot demo format:
			teapotPacket[2] = fifoBuffer[0];
			teapotPacket[3] = fifoBuffer[1];
			teapotPacket[4] = fifoBuffer[4];
			teapotPacket[5] = fifoBuffer[5];
			teapotPacket[6] = fifoBuffer[8];
			teapotPacket[7] = fifoBuffer[9];
			teapotPacket[8] = fifoBuffer[12];
			teapotPacket[9] = fifoBuffer[13];
			std::cout << (string)teapotPacket << std::endl;
			teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
		}
	}
	i2c->closeConnection();
	delete i2c, mpu6050, mpu6050dmp;
	return 0;
}
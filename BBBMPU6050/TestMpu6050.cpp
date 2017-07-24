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
#include "AllDevices.h"
#include "MPU6050.h"
#include <time.h>


using namespace bigowl_i2cport;
using namespace bigowl_mpu6050;

int ctrl;
float count = 0;
float vecX = 0;
float vecY = 0;
float vecZ = 0;
float vecXprev = 0;
float vecYprev = 0;
float vecZprev = 0;
float posX = 0;
float posY = 0;
float posZ = 0;
float posXprev = 0;
float posYprev = 0;
float posZprev = 0;
float timeC = 0.000002;//this is in fractions of a second.

float k = 16000;
int16_t *accels = (int16_t *) calloc(3, sizeof(int16_t));
int16_t *gyros = (int16_t *) calloc(3, sizeof(int16_t));

int main() {
	
    ctrl = 1;
    signal(SIGINT, signal_handler);

    I2cPort *i2c = new I2cPort(0x68, 1);
    i2c->openConnection();

    MPU6050 *mpu6050 = new MPU6050(i2c);
    mpu6050->initialize();

    
	std::cout << "Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z,Vel X,Vel Y,Vel Z,Pos X,Pos Y,Pos Z,f(X),f(X^2)" << std::endl;
    
    while (ctrl) {
		count += 1;
        mpu6050->getAccelerations(accels);
		std::cout << (float) accels[0] / k << ",";
        std::cout << (float) accels[1] / k << ",";
        std::cout << (float) accels[2] / k << ",";

        mpu6050->getAngularVelocities(gyros);
        std::cout << (float) gyros[0] / k << ",";
        std::cout << (float) gyros[1] / k << ",";
        std::cout << (float) gyros[2] / k << ",";
		
		//veleocity calculations
		vecX = (float) ((accels[0]/ k)*(timeC)+vecXprev);
		vecY = (float) ((accels[1]/ k)*(timeC)+vecYprev);
		vecZ = (float) ((accels[2]/ k)*(timeC)+vecZprev);
		std::cout << vecX << ",";
		std::cout << vecY << ",";
		std::cout << vecZ << ",";
		
		//Position calculations
		posX = (float) (vecX*(timeC)+posXprev);
		posY = (float) (vecY*(timeC)+posYprev);
		posZ = (float) (vecZ*(timeC)+posZprev);
		std::cout << posX << ",";
		std::cout << posY << ",";
		std::cout << posZ << ",";
		std::cout << (float) (count*timeC) << ",";
		std::cout << (float) ((count*timeC)*(count*timeC)) << std::endl;
		
		vecXprev = vecX;
		vecYprev = vecY;
		vecZprev = vecZ;
		
		posXprev = posX;
		posYprev = posY;
		posZprev = posZ;
        usleep(2);
    }

    free(accels);
    free(gyros);
    i2c->closeConnection();
    delete i2c, mpu6050;

    return 0;
}




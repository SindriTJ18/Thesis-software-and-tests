#include <pigpio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iomanip>
#include <chrono>

#include <iostream>
#include <fstream>
#include <thread>

#include "../header/leg_control.hpp"
#include "../header/receiver_control.hpp"
#include "../header/rotary_encoder.hpp"
#include "../header/mpuXX50.hpp"

leg leg1(5, 13, 6, 21, 20);
leg leg2(24, 18, 23, 9, 11);
leg leg3(7, 25, 8, 22, 10);

re_decoder enc1(19, 26);
re_decoder enc2(15, 14);
re_decoder enc3(16, 12);

receiver rec(17);



int homeflag = 0;
int input = 0;
int switchPos = -1;
int lastPos1 = 2;
int lastPos2 = 2;
int lastPos3 = 2;

// Variable definitions
int sampleRate = 8000; // Microseconds (~250 Hz)
int time2Delay;
int fd;
float dt;

// I2C read and write functions
int i2c_read(int addr, unsigned char *data, char len){
	if (read(addr, data, len) != len){
		return -1;
	} else {
		return 0;
	}
};

int i2c_write(int addr, unsigned char *data, char len){
	if (write(addr, data, len) != len){
		return -1;
	} else {
    return 0;
  }
};

// Time stabilization function
float timeSync(auto t1){
	// Find duration to sleep thread
	auto t2 = std::chrono::high_resolution_clock::now();
	time2Delay = std::chrono::duration<float, std::micro>(t2-t1).count();

	// Sleep thread
	std::this_thread::sleep_for(std::chrono::microseconds(sampleRate-time2Delay));

	// Calculate dt
	auto t3 = std::chrono::high_resolution_clock::now();
	dt = (std::chrono::duration<float, std::micro>(t3-t1).count()) * 1E-6;
	//std::cout << "\t" << 1/dt << std::endl;

	// Return dt and begin main loop again
	return dt;
}
// Open the I2C Bus
char *filename = (char*)"/dev/i2c-1";

// Setup the MPU class

struct i2c_device_t i2c_dev;
MPUXX50 *mpuXX50;

void airmode1(){
  leg1.retract();
  while(enc1.pos < 3230){
    time_sleep(0.01);
  }
  leg1.lock();
  time_sleep(0.1);
  leg1.stop();
}
void airmode2(){
  leg2.retract();
  while(enc2.pos < 3230){
    time_sleep(0.01);
  }
  leg2.lock();
  time_sleep(0.1);
  leg2.stop();
}
void airmode3(){
  leg3.retract();
  while(enc3.pos < 3230){
    time_sleep(0.01);
  }
  leg3.lock();
  time_sleep(0.1);
  leg3.stop();
}
void Leg_thread1() {
  while (1) {
    if (switchPos != lastPos1 && switchPos == 2) {
      leg1.home();
      time_sleep(0.5);
      enc1.pos = 0;
      homeflag = 1;
    } else if (switchPos != lastPos1 && homeflag == 1 && switchPos == 1) {
      airmode1();
    } else if (switchPos != lastPos1 && switchPos == 0){
      leg1.land();
    }
    else{
      leg1.stop();
    }
    lastPos1 = switchPos;
    time_sleep(0.1);
  }
}

void Leg_thread2() {
  while (1) {
    
    if (switchPos != lastPos2 && switchPos == 2) {
      leg2.home();
      enc2.pos = 0;
      input = 0;
    } else if (switchPos != lastPos2 && homeflag == 1 && switchPos == 1) {
      airmode2();
    } else if (switchPos != lastPos2 && switchPos == 0){
      leg2.land();
    }
    else{
      leg2.stop();
    }
    lastPos2 = switchPos;
    time_sleep(0.1);
  }
  
}

void Leg_thread3() {
  while (1) {
    
    if (switchPos != lastPos3 && switchPos == 2) {
      leg3.home();
      enc3.pos = 0;
      input = 0;
    } else if (switchPos != lastPos3 && homeflag == 1 && switchPos == 1) {
      airmode3();
    } else if (switchPos != lastPos3 && switchPos == 0){
      leg3.land();
    }
    else{
      leg3.stop();
    }
    lastPos3 = switchPos;
    time_sleep(0.1);
  }
}

void Logging_thread() {
  std::cout << "ENC1" << " , " << "ENC2" << " , " << "ENC3" << " , " << "SWITCH" << " , " << "PITCH" << " , " << "ROLL"  << "TICK" << std::endl;
  while (1) {
    auto loopTimer = std::chrono::high_resolution_clock::now();
    switchPos = rec.getPos();
    std::cout << enc1.pos << " , " << enc2.pos << " , " << enc3.pos << " , " <<  rec.getPos() << " , ";

    mpuXX50->compFilter(dt, 0.98);
    std::cout << std::setprecision(3) << mpuXX50->attitude.roll  << ", ";
    std::cout << std::setprecision(3) << mpuXX50->attitude.pitch << ", ";
    std::cout << gpioTick() << std::endl;
		// Stabilize the data rate
		dt = timeSync(loopTimer);
  }
}

int main(int argc, char *argv[]) {
  	// Start I2C
	if ((fd = open(filename, O_RDWR)) < 0){
    std::cout << "Failed to open the i2c bus" << std::endl;
    return -1;
	}

  // Connect on 0x68
	if (ioctl(fd, I2C_SLAVE, 0x68) < 0){
    std::cout << "Failed to acquire bus access and/or talk to slave." << std::endl;
		return -1;
	}

  // Prepare I2C functions for read and write
  i2c_dev.i2c_write = (i2c_read_write_t) &i2c_write;
  i2c_dev.i2c_read = (i2c_read_write_t) &i2c_read;

  // Connect to sensor using file identifier
  mpuXX50 = new MPUXX50(fd, i2c_dev);

  // Initialize the IMU and set the senstivity values
  std::cout << "IMU initialize. Pass/Fail: ";
  std::cout << mpuXX50->initIMU(MPU9250) << std::endl;
  mpuXX50->getAres(AFS_4G);
  mpuXX50->getGres(GFS_500DPS);
  sleep(1);

  // Calibrate the gyroscope
  gyro_cal_t gyro_cal;
  std::cout << "Calibrating gyroscope, hold IMU stationary." << std::endl;
  sleep(2);
  mpuXX50->gyroCalibration(1000);

  // Load saved gyroscope calibration values
  // gyro_cal.x = 0;
  // gyro_cal.y = 0;
  // gyro_cal.z = 0;
  // mpuXX50->setGyroCalibration(gyro_cal);

  // Display calibration values to user
  gyro_cal = mpuXX50->getGyroCalibration();
  std::cout << "---------------------------------------" << std::endl;
  std::cout << "Gyroscope bias values:" << std::endl;
  std::cout << "\t X: " << mpuXX50->gyro_cal.x << std::endl;
  std::cout << "\t Y: " << mpuXX50->gyro_cal.y << std::endl;
  std::cout << "\t Z: " << mpuXX50->gyro_cal.z << std::endl;
  std::cout << "---------------------------------------" << std::endl;
  sleep(2);

  if (gpioInitialise() < 0) return 1;
  std::thread th1(Leg_thread1);
  std::thread th2(Leg_thread2);
  std::thread th3(Leg_thread3);
  std::thread th4(Logging_thread);
  while (1) {
    std::cin >> input;
    if (input == 8) {
      gpioTerminate();
      return 0;
    }
  }

  th1.join();
  th2.join();
  th3.join();
  th4.join();
  gpioTerminate();
  return 0;
}

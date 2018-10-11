#include "EduBotics_MPU6050.h"

static void EduBoticsMPU6050::dmpDataReady(void) {
    mpuInterrupt_ = true;
}

EduBoticsMPU6050::EduBoticsMPU6050() {

}

int EduBoticsMPU6050::initialise(void) {
	 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    mpu_.initialize();

    // verify connection
    if (!mpu_.testConnection()) {
    	return false; 
    }

    // load and configure the DMP
    uint8_t devStatus = mpu_.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu_.setXGyroOffset(187);
    mpu_.setYGyroOffset(-35);
    mpu_.setZGyroOffset(5);
    mpu_.setXAccelOffset(-2003);
    mpu_.setYAccelOffset(1176);
    mpu_.setZAccelOffset(1246); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu_.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, this->dmpDataReady, RISING);
        mpuIntStatus_ = mpu_.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize_ = mpu_.dmpGetFIFOPacketSize();
    } 
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        return -devStatus; 
    }
    return true; 
}

int EduBoticsMPU6050::update(void) {
	// wait for MPU interrupt or extra packet(s) available
    if (mpuInterrupt_ || fifoCount_ > packetSize_) {
    	// reset interrupt flag and get INT_STATUS byte
	    mpuInterrupt_ = false;
	    mpuIntStatus_ = mpu_.getIntStatus();

	    // get current FIFO count
	    fifoCount_ = mpu_.getFIFOCount();

	    // check for overflow (this should never happen unless our code is too inefficient)
	    if ((mpuIntStatus_ & 0x10) || fifoCount_ == 1024) {
	        // reset so we can continue cleanly
	        mpu_.resetFIFO();
	        return false; 
	    // otherwise, check for DMP data ready interrupt (this should happen frequently)
	    } 
	    else if (mpuIntStatus_ & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount_ < packetSize_) {
	            fifoCount_ = mpu_.getFIFOCount();
	        }

	        // read a packet from FIFO
	        mpu_.getFIFOBytes(fifoBuffer_, packetSize_);
	        
	        // track FIFO count here in case there is > 1 packet available
	        // (this lets us immediately read more without waiting for an interrupt)
	        fifoCount_ -= packetSize_;
    	}
	}	    

	return true; 
}

int EduBoticsMPU6050::getYawPitchRoll(double *yaw, double *pitch, double *roll) {

	if (update()) {
	    mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
	    mpu_.dmpGetGravity(&gravity_, &q_);
	    mpu_.dmpGetYawPitchRoll(ypr_, &q_, &gravity_);
	    *yaw = double(ypr_[0]*180/M_PI); 
	    *pitch = double(ypr_[1]*180/M_PI); 
	    *roll = double(ypr_[2]*180/M_PI); 
	    return true;
	}
	else {
		return false; 
	}
}

int EduBoticsMPU6050::getQuaternion(float &w, float &x, float &y, float &z) {
	mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
	w = q_.w;
	x = q_.x; 
	y = q_.y; 
	z = q_.z; 
}

int EduBoticsMPU6050::getAcceleration(float &x, float &y, float &z) {
	//display real acceleration, adjusted to remove gravity
	mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
	mpu_.dmpGetAccel(&aa_, fifoBuffer_);
	mpu_.dmpGetGravity(&gravity_, &q_);
	mpu_.dmpGetLinearAccel(&aaReal_, &aa_, &gravity_);

	x = aaReal_.x; 
	y = aaReal_.y; 
	z = aaReal_.z; 
}

int EduBoticsMPU6050::getWorldAcceleration(float &x, float &y, float &z) {
	 // display initial world-frame acceleration, adjusted to remove gravity
	// and rotated based on known orientation from quaternion
	mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
	mpu_.dmpGetAccel(&aa_, fifoBuffer_);
	mpu_.dmpGetGravity(&gravity_, &q_);
	mpu_.dmpGetLinearAccel(&aaReal_, &aa_, &gravity_);
	mpu_.dmpGetLinearAccelInWorld(&aaWorld_, &aaReal_, &q_);
	
	x = aaWorld_.x; 
	y = aaWorld_.y; 
	z = aaWorld_.z; 
}

int EduBoticsMPU6050::getEuler(float &x, float &y, float &z) {
	// display Euler angles in degrees
	mpu_.dmpGetQuaternion(&q_, fifoBuffer_);
	mpu_.dmpGetEuler(euler_, &q_);
	
	x = euler_[0] * 180/M_PI;
	y = euler_[1] * 180/M_PI;
	z = euler_[2] * 180/M_PI;
}
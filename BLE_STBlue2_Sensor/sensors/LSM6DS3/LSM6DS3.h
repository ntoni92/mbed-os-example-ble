#ifndef __LSM6DS3IMU_H__
#define __LSM6DS3IMU_H__


#include "mbed.h"
#include "stdint.h"
#include "LSM6DS3_Types.h"
#include "LSM6DS3_Registers.h"

#include "math.h"

#define I2C_MODE 0
#define SPI_MODE 1

// Return values 
typedef enum
{
    IMU_SUCCESS,
    IMU_HW_ERROR,
    IMU_NOT_SUPPORTED,
    IMU_GENERIC_ERROR,
    IMU_OUT_OF_BOUNDS,
    IMU_ALL_ONES_WARNING,
    //...
} status_t;

//This is the core operational class of the driver.
//  LSM6DS3Core contains only read and write operations towards the IMU.
//  To use the higher level functions, use the class LSM6DS3 which inherits
//  this class.
//========================self define=================

//====================================================

class LSM6DS3Core
{
public:
    LSM6DS3Core( uint8_t );
    LSM6DS3Core( uint8_t, uint8_t );
 //   ~LSM6DS3Core() = default;
    
    status_t beginCore( void );
    
    //The following utilities read and write to the IMU

    //ReadRegisterRegion takes a uint8 array address as input and reads
    //  a chunk of memory into that array.
    status_t readRegisterRegion(uint8_t*, uint8_t, uint8_t );
    
    //readRegister reads one 8-bit register
    status_t readRegister(uint8_t*, uint8_t);
    
    //Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
    //  Acts as a 16-bit read operation
    status_t readRegisterInt16(int16_t*, uint8_t offset );
    
    //Writes an 8-bit byte;
    status_t writeRegister(uint8_t, uint8_t);
    
    //Change to embedded page
    status_t embeddedPage( void );
    
    //Change to base page
    status_t basePage( void );
    SPI spi_;
    //I2C i2c_;
    DigitalOut cs_;

    
private:
    
    //Communication stuff
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;

};

//This struct holds the settings the driver uses to do calculations
struct SensorSettings {
public:
    //Gyro settings
    uint8_t gyroEnabled;
    uint16_t gyroRange;
    uint16_t gyroSampleRate;
    uint16_t gyroBandWidth;

    uint8_t gyroFifoEnabled;
    uint8_t gyroFifoDecimation;

    //Accelerometer settings
    uint8_t accelEnabled;
    uint8_t accelODROff;
    uint16_t accelRange;
    uint16_t accelSampleRate;
    uint16_t accelBandWidth;
    
    uint8_t accelFifoEnabled;
    uint8_t accelFifoDecimation;
    
    //Temperature settings
    uint8_t tempEnabled;
    
    //Non-basic mode settings
    uint8_t commMode;
    
    //FIFO control data
    uint16_t fifoThreshold;
    int16_t fifoSampleRate;
    uint8_t fifoModeWord;
    
};


//This is the highest level class of the driver.
//
//  class LSM6DS3 inherits the core and makes use of the beginCore()
//method through it's own begin() method.  It also contains the
//settings struct to hold user settings.

class LSM6DS3 : public LSM6DS3Core
{
public:
    //IMU settings
    SensorSettings settings;
    
    //Error checking
    uint16_t allOnesCounter;
    uint16_t nonSuccessCounter;

    //Constructor generates default SensorSettings.
    //(over-ride after construction if desired)
    LSM6DS3( uint8_t busType = I2C_MODE, uint8_t inputArg = 0x6B );
//    ~LSM6DS3() = default;
    
    //Call to apply SensorSettings
    status_t begin(void);

    //Returns the raw bits from the sensor cast as 16-bit signed integers
    int16_t readRawAccelX( void );
    int16_t readRawAccelY( void );
    int16_t readRawAccelZ( void );
    int16_t readRawGyroX( void );
    int16_t readRawGyroY( void );
    int16_t readRawGyroZ( void );
    
    int16_t accelOffset[3];
    int16_t gyroOffset[3];
    void setOffset(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t);
    


    //Returns the values as floats.  Inside, this calls readRaw___();
    float readFloatAccelX( void );
    float readFloatAccelY( void );
    float readFloatAccelZ( void );
    float readFloatGyroX( void );
    float readFloatGyroY( void );
    float readFloatGyroZ( void );

    //Temperature related methods
    int16_t readRawTemp( void );
    float readTempC( void );
    float readTempF( void );

    //FIFO stuff
    void fifoBegin( void );
    void fifoClear( void );
    int16_t fifoRead( void );
    uint16_t fifoGetStatus( void );
    void fifoEnd( void );
    
    float calcGyro( int32_t );
    float calcAccel( int32_t );
    
private:

};

#endif

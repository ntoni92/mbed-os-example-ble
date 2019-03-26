#ifndef MBED_BLE_BLUENRG2_SENSOR_SERVICE_H__
#define MBED_BLE_BLUENRG2_SENSOR_SERVICE_H__

#include "ble/BLE.h"

#if BLE_FEATURE_GATT_SERVER

static const char uuid_char1[] = "00140000-0001-11e1-ac36-0002a5d5c51b";
static const char uuid_char2[] = "00e00000-0001-11e1-ac36-0002a5d5c51b";
static const char uuid_ser[]   = "00000000-0001-11e1-9ab4-0002a5d5c51b";
static const UUID _uuid1(uuid_char1);
static const UUID _uuid2(uuid_char2);
static const UUID _uuid_ser(uuid_ser);

class BluenrgSensorService {

public:

    BluenrgSensorService(BLE &_ble, int16_t temp, int16_t* accelValAxis, int16_t* gyroValAxis) :
        ble(_ble),
        sensValueBytes(temp, accelValAxis, gyroValAxis),
        _char_env(
            _uuid1,
            sensValueBytes.getEnvPointer(),
            sensValueBytes.getEnvNumValueBytes(),
			SensorValueBytes::MAX_VALUE_BYTES_ENV,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
        ),
        _char_imu(
            _uuid2,
            sensValueBytes.getImuPointer(),
            sensValueBytes.getImuNumValueBytes(),
            SensorValueBytes::MAX_VALUE_BYTES_IMU,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
        )
    {
        setupService();
    }

    void updateTemperature(uint16_t temp) {
        sensValueBytes.updateTemp(temp);
        ble.gattServer().write(
            _char_env.getValueHandle(),
            sensValueBytes.getEnvPointer(),
            sensValueBytes.getEnvNumValueBytes()
        );
    }

    void updateAccel(int16_t* accelValAxis) {
        sensValueBytes.updateAccel(accelValAxis);
        ble.gattServer().write(
            _char_imu.getValueHandle(),
            sensValueBytes.getImuPointer(),
            sensValueBytes.getImuNumValueBytes()
        );
    }

    void updateGyro(int16_t* gyroValAxis) {
        sensValueBytes.updateGyro(gyroValAxis);
        ble.gattServer().write(
            _char_imu.getValueHandle(),
            sensValueBytes.getImuPointer(),
            sensValueBytes.getImuNumValueBytes()
        );
    }

protected:

    void setupService(void) {
        GattCharacteristic *charTable[] = {
            &_char_env,
            &_char_imu
        };
        GattService SensorService(
            _uuid_ser,
            charTable,
            sizeof(charTable) / sizeof(GattCharacteristic*)
        );


        ble.gattServer().addService(SensorService);
    }

protected:

    struct SensorValueBytes {
        /* 1 byte for the Flags, and up to two bytes for heart rate value. */
        static const unsigned MAX_VALUE_BYTES_IMU = 14;
        static const unsigned MAX_VALUE_BYTES_ENV = 8;
        static const unsigned FLAGS_BYTE_INDEX = 0;

        SensorValueBytes(int16_t temp, int16_t* accelValAxis, int16_t* gyroValAxis) : envValueBytes()
        {
            updateTemp(temp);
            updateAccel(accelValAxis);
            updateGyro(gyroValAxis);
//            updatePress(press);
        }

        void updateTemp(int16_t temp)
        {

        	envValueBytes[6] = (uint8_t)((temp+22));
        	envValueBytes[7] = (uint8_t)((temp+22) >> 8);
        }

//        void updatePress(int32_t press)
//        {
//
//        	envValueBytes[2] |= (uint8_t)(press & 0xFF);
//        	envValueBytes[3] |= (uint8_t)(press >> 8);
//        	envValueBytes[4] |= (uint8_t)(press >> 16);
//        	envValueBytes[5] |= (uint8_t)(press >> 24);
//        }

        void updateAccel(int16_t* accelValAxis) //valAxis[] = {-valY, valX, -valZ}
        {
        	imuValueBytes[2] = (uint8_t)(-accelValAxis[0]>>2);
        	imuValueBytes[3] = (uint8_t)((-accelValAxis[0]>>2) >> 8);
        	imuValueBytes[4] = (uint8_t)(accelValAxis[1]>>2);
        	imuValueBytes[5] = (uint8_t)((accelValAxis[1]>>2) >> 8);
        	imuValueBytes[6] = (uint8_t)(-accelValAxis[2]>>2);
        	imuValueBytes[7] = (uint8_t)((-accelValAxis[2]>>2) >> 8);
        }

        void updateGyro(int16_t* gyroValAxis)  //valAxis[] = {valY, valX, valZ}
        {
        	imuValueBytes[8] = (uint8_t)(gyroValAxis[0]<<4);
        	imuValueBytes[9] = (uint8_t)((gyroValAxis[0]<<4) >> 8);
        	imuValueBytes[10] = (uint8_t)(gyroValAxis[1]<<4);
        	imuValueBytes[11] = (uint8_t)((gyroValAxis[1]<<4) >> 8);
        	imuValueBytes[12] = (uint8_t)(gyroValAxis[2]<<4);
        	imuValueBytes[13] = (uint8_t)((gyroValAxis[2]<<4) >> 8);
        }

        uint8_t *getEnvPointer(void)
        {
            return envValueBytes;
        }

        const uint8_t *getEnvPointer(void) const
        {
            return envValueBytes;
        }

        unsigned getEnvNumValueBytes(void) const
        {
        	return this->MAX_VALUE_BYTES_ENV;
        }

        uint8_t *getImuPointer(void)
        {
            return imuValueBytes;
        }

        const uint8_t *getImuPointer(void) const
        {
            return imuValueBytes;
        }

        unsigned getImuNumValueBytes(void) const
        {
        	return this->MAX_VALUE_BYTES_IMU;
        }

    private:
        uint8_t envValueBytes[MAX_VALUE_BYTES_ENV];
        uint8_t imuValueBytes[MAX_VALUE_BYTES_IMU];
    };

protected:
    BLE &ble;
    SensorValueBytes sensValueBytes;
    GattCharacteristic _char_env;
    GattCharacteristic _char_imu;
};

#endif // BLE_FEATURE_GATT_SERVER

#endif /* #ifndef MBED_BLE_BLUENRG2_SENSOR_SERVICE_H__*/

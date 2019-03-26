/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "BluenrgSensorService.h"
#include "pretty_printer.h"
#include "LSM6DS3.h"

#ifdef BLUENRG2_DEVICE
#include "bluenrg1_stack.h"
#endif //BLUENRG2_DEVICE

const static char DEVICE_NAME[] = "MBED_SENSOR";
const uint8_t data[] = {0x01,0x02,0x00,0xD4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const unsigned char passkey[] = "123456";

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

class SensorDemo : ble::Gap::EventHandler {
public:
    SensorDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
		_imu_sensor(SPI_MODE, DIO1),
        _event_queue(event_queue),
        _led1(LED1, 1),
        _connected(false),
        _temp(0x0000),
        _b_service(ble, _temp, _accel, _gyro),
        _adv_data_builder(_adv_buffer)
		{
    		_imu_sensor.begin();
    	}

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &SensorDemo::on_init_complete);

        _event_queue.call_every(500, this, &SensorDemo::blink);
        _event_queue.call_every(10000, this, &SensorDemo::update_env_sensor_value);
        _event_queue.call_every(50, this, &SensorDemo::update_imu_sensor_value);

#ifdef BLUENRG2_DEVICE
        _event_queue.call_every(10, &BTLE_StackTick);
#endif //BLUENRG2_DEVICE

        _event_queue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        print_mac_address();

        start_advertising();
    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        _ble.securityManager().init(true, true, SecurityManager::IO_CAPS_NONE, passkey);

        _adv_data_builder.setTxPowerAdvertised(0);
        _adv_data_builder.setName(DEVICE_NAME);
        _adv_data_builder.addData(ble::adv_data_type_t::MANUFACTURER_SPECIFIC_DATA, mbed::make_Span(data, sizeof(data)));
        _adv_data_builder.setFlags();

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            printf("_ble.gap().setAdvertisingParameters() failed\r\n");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            printf("_ble.gap().setAdvertisingPayload() failed\r\n");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            printf("_ble.gap().startAdvertising() failed\r\n");
            return;
        }
    }

    void update_env_sensor_value() {
        if (_connected) {
        	_temp = (int16_t)(_imu_sensor.readTempC() * 10);
        	_b_service.updateTemperature(_temp);
        }
    }

    void update_imu_sensor_value() {
        if (_connected) {
        	_accel[0] = (_imu_sensor.readRawAccelY());
        	_accel[1] = (_imu_sensor.readRawAccelX());
        	_accel[2] = (_imu_sensor.readRawAccelZ());
        	_b_service.updateAccel(_accel);
        	_gyro[0] = _imu_sensor.readRawGyroY();
        	_gyro[1] = _imu_sensor.readRawGyroX();
        	_gyro[2] = _imu_sensor.readRawGyroZ();
        	_b_service.updateGyro(_gyro);
        }
    }

    void blink(void) {
        _led1 = !_led1;
    }

private:
    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _connected = false;
    }

    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            _connected = true;
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _led1;
    bool _connected;

    int16_t _temp;
    int16_t _accel[3]; //X Y Z
    int16_t _gyro[3];  //X Y Z
    BluenrgSensorService _b_service;
    LSM6DS3 _imu_sensor;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    SensorDemo demo(ble, event_queue);
    demo.start();

    return 0;
}

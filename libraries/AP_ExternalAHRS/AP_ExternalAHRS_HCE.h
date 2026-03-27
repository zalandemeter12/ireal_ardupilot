#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_HCE_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CANManager/AP_CANSensor.h>

class AP_ExternalAHRS_HCE;

/*
  Helper class to handle CAN frames for a specific bus.
*/
class HCE_CAN_Handler : public CANSensor {
public:
    // We now pass the protocol to the constructor so the handler can register itself
    HCE_CAN_Handler(AP_ExternalAHRS_HCE* parent, const char* name, AP_CAN::Protocol protocol) 
        : CANSensor(name), _parent(parent) {
        register_driver(protocol); 
    }

    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    AP_ExternalAHRS_HCE* _parent;
};

class AP_ExternalAHRS_HCE : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_HCE(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    const char* get_name() const override { return "HCE eAHRS"; }
    uint8_t num_gps_sensors(void) const override { return 1; }
    int8_t get_port(void) const override { return 100; }

    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    void update() override;
    void handle_can_frame(AP_HAL::CANFrame &frame);

private:
    HCE_CAN_Handler *can_handler0 = nullptr;
    HCE_CAN_Handler *can_handler1 = nullptr;

    uint32_t boot_time_ms;
    uint32_t last_imu_update_ms;

    void update_thread();
    void send_ins();    
    void send_baro();   
    void send_mag();    
    void send_gps();    
    void update_state(); 

    float noise() const;
    float noise_after(uint32_t delay_ms, float scale) const;
};

#endif
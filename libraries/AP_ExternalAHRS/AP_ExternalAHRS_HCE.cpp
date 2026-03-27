#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_HCE.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>
#include <AP_RTC/AP_RTC.h>
#include <stdlib.h>
#include <cstdio>

extern const AP_HAL::HAL &hal;

// ─────────────────────────────────────────────────────────────────────────────
// CAN Handler Implementation
// ─────────────────────────────────────────────────────────────────────────────

void HCE_CAN_Handler::handle_frame(AP_HAL::CANFrame &frame)
{
    printf("DEBUG: HCE received frame ID 0x%X\n", (unsigned int)frame.id);
    if (_parent != nullptr) {
        _parent->handle_can_frame(frame);
    }
    else {
        gcs().send_text(MAV_SEVERITY_ERROR, "HCE_CAN_Handler received frame but parent is null!");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

AP_ExternalAHRS_HCE::AP_ExternalAHRS_HCE(AP_ExternalAHRS *_frontend,
                                           AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend(_frontend, _state)
    , boot_time_ms(AP_HAL::millis())
    , last_imu_update_ms(0)
{
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_HCE::update_thread, void),
        "HCE_AHRS", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// CAN Frame Processing
// ─────────────────────────────────────────────────────────────────────────────

void AP_ExternalAHRS_HCE::handle_can_frame(AP_HAL::CANFrame &frame)
{
    // CANSensor calls this from its own thread. 
    // Frame handles both Standard and FD automatically.
    if (frame.isExtended()) {
        [[maybe_unused]] const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;
        
        // Example logic:
        // if (id == 0x100) { parse_imu(frame); }

        gcs().send_text(MAV_SEVERITY_INFO, "Received CAN frame with Extended ID: 0x%X", id);
    }
    else {
        [[maybe_unused]] const uint32_t id = frame.id & AP_HAL::CANFrame::MaskStdID;

        // Example logic:
        // if (id == 0x50) { parse_gps(frame); }

        gcs().send_text(MAV_SEVERITY_INFO, "Received CAN frame with Standard ID: 0x%X", id);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Threading and Data Updates (Existing logic preserved)
// ─────────────────────────────────────────────────────────────────────────────

void AP_ExternalAHRS_HCE::update_thread()
{
    uint32_t last_imu_ms  = 0;
    uint32_t last_slow_ms = 0;

    while (true) {
        const uint32_t now = AP_HAL::millis();

        if (now - last_imu_ms >= 4) {
            last_imu_ms = now;
            last_imu_update_ms = now;
            send_ins();
        }

        if (now - last_slow_ms >= 20) {
            last_slow_ms = now;
            send_baro();
            send_mag();
            send_gps();
        }
        hal.scheduler->delay(1);
    }
}

void AP_ExternalAHRS_HCE::update()
{
    if (can_handler0 == nullptr) {
        can_handler0 = new HCE_CAN_Handler(this, "can0", AP_CAN::Protocol::HCE_CAN);
    }

    if (can_handler1 == nullptr) {
        can_handler1 = new HCE_CAN_Handler(this, "can1", AP_CAN::Protocol::HCE_CAN);
    }

    update_state();
}

// ... Rest of your helper functions (send_ins, send_baro, etc.) remain the same ...

void AP_ExternalAHRS_HCE::send_ins() {
    const float n = noise_after(5000, 0.005f);
    AP_ExternalAHRS::ins_data_message_t ins;
    ins.accel = Vector3f(n, n, -GRAVITY_MSS + n); 
    ins.gyro = Vector3f(n, n, n);                 
    ins.temperature = 25.0f + (n * 10.0f);              
    AP::ins().handle_external(ins);
}

void AP_ExternalAHRS_HCE::send_baro() {
    const float n = noise_after(10000, 1.5f);
    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance = 0;
    baro.pressure_pa = 101325.0f + n;
    baro.temperature = 25.0f;
    AP::baro().handle_external(baro);
}

void AP_ExternalAHRS_HCE::send_mag() {
    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f(200.0f, 50.0f, 400.0f); 
    AP::compass().handle_external(mag);
}

void AP_ExternalAHRS_HCE::send_gps() {
    const uint32_t now_ms = AP_HAL::millis();
    AP_ExternalAHRS::gps_data_message_t gps;
    gps.gps_week = 2285;
    gps.ms_tow = now_ms % 604800000UL;
    gps.fix_type = AP_GPS_FixType::FIX_3D;
    gps.satellites_in_view = 12;
    gps.horizontal_pos_accuracy = 0.15f;
    gps.vertical_pos_accuracy = 0.25f;
    gps.horizontal_vel_accuracy = 0.10f;
    gps.hdop = 70.0f;
    gps.vdop = 90.0f;
    gps.latitude = int32_t(-35.363 * 1.0e7);
    gps.longitude = int32_t(149.165 * 1.0e7);
    gps.msl_altitude = 58400;
    gps.ned_vel_north = 0.0f;
    gps.ned_vel_east = 0.0f;
    gps.ned_vel_down = 0.0f;
    AP::gps().handle_external(gps, 0);
}

void AP_ExternalAHRS_HCE::update_state() {
    const uint32_t now_us = AP_HAL::micros();
    WITH_SEMAPHORE(state.sem);
    state.quat = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    state.have_quaternion = true;
    state.velocity = Vector3f(0.0f, 0.0f, 0.0f);
    state.have_velocity = true;
    if (!state.have_origin) {
        state.origin = Location(int32_t(-35.363 * 1.0e7), int32_t(149.165 * 1.0e7), 58400, Location::AltFrame::ABSOLUTE);
        state.have_origin = true;
    }
    state.location = state.origin;
    state.have_location = true;
    state.last_location_update_us = now_us;
}

bool AP_ExternalAHRS_HCE::healthy(void) const { return (AP_HAL::millis() - last_imu_update_ms) < 100; }
bool AP_ExternalAHRS_HCE::initialised(void) const { return true; }
bool AP_ExternalAHRS_HCE::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const { return true; }
void AP_ExternalAHRS_HCE::get_filter_status(nav_filter_status &status) const {
    memset(&status, 0, sizeof(status));
    status.flags.initalized = true;
    status.flags.attitude = true;
    status.flags.vert_vel = true;
    status.flags.vert_pos = true;
    status.flags.horiz_vel = true;
    status.flags.horiz_pos_abs = true;
    status.flags.using_gps = true;
}
bool AP_ExternalAHRS_HCE::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
    velVar = posVar = hgtVar = tasVar = 0.05f;
    magVar.zero();
    return true;
}

float AP_ExternalAHRS_HCE::noise() const { return ((static_cast<float>((static_cast<unsigned>(rand())) % 2000000U) - 1.0e6f)) / 1.0e6f; }
float AP_ExternalAHRS_HCE::noise_after(uint32_t delay_ms, float scale) const {
    if ((AP_HAL::millis() - boot_time_ms) < delay_ms) return 0.0f;
    return noise() * scale;
}
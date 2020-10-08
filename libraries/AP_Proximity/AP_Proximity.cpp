/*
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

#include "AP_Proximity.h"
#include "AP_Proximity_LightWareSF40C.h"
#include "AP_Proximity_TeraRangerTower.h"
#include "AP_Proximity_RangeFinder.h"
#include "AP_Proximity_MAV.h"
#include "AP_Proximity_SITL.h"
#include "AP_Proximity_Analog.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Proximity::var_info[] = {
    // 0 is reserved for possible addition of an ENABLED parameter

    // @Param: _TYPE
    // @DisplayName: Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C,2:MAVLink,3:TeraRangerTower,4:RangeFinder
    // @User: Standard
    AP_GROUPINFO("_TYPE",   1, AP_Proximity, _type[0], 0),

    // @Param: _ORIENT
    // @DisplayName: Proximity sensor orientation
    // @Description: Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("_ORIENT", 2, AP_Proximity, _orientation[0], 0),

    // @Param: _YAW_CORR
    // @DisplayName: Proximity sensor yaw correction
    // @Description: Proximity sensor yaw correction
    // @Units: degrees
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("_YAW_CORR", 3, AP_Proximity, _yaw_correction[0], PROXIMITY_YAW_CORRECTION_DEFAULT),

    // @Param: _IGN_ANG1
    // @DisplayName: Proximity sensor ignore angle 1
    // @Description: Proximity sensor ignore angle 1
    // @Units: degrees
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG1", 4, AP_Proximity, _ignore_angle_deg[0], 0),

    // @Param: _IGN_WID1
    // @DisplayName: Proximity sensor ignore width 1
    // @Description: Proximity sensor ignore width 1
    // @Units: degrees
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("_IGN_WID1", 5, AP_Proximity, _ignore_width_deg[0], 0),

    // @Param: _IGN_ANG2
    // @DisplayName: Proximity sensor ignore angle 2
    // @Description: Proximity sensor ignore angle 2
    // @Units: degrees
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG2", 6, AP_Proximity, _ignore_angle_deg[1], 0),

    // @Param: _IGN_WID2
    // @DisplayName: Proximity sensor ignore width 2
    // @Description: Proximity sensor ignore width 2
    // @Units: degrees
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("_IGN_WID2", 7, AP_Proximity, _ignore_width_deg[1], 0),

    // @Param: _IGN_ANG3
    // @DisplayName: Proximity sensor ignore angle 3
    // @Description: Proximity sensor ignore angle 3
    // @Units: degrees
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG3", 8, AP_Proximity, _ignore_angle_deg[2], 0),

    // @Param: _IGN_WID3
    // @DisplayName: Proximity sensor ignore width 3
    // @Description: Proximity sensor ignore width 3
    // @Units: degrees
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("_IGN_WID3", 9, AP_Proximity, _ignore_width_deg[2], 0),

    // @Param: _IGN_ANG4
    // @DisplayName: Proximity sensor ignore angle 4
    // @Description: Proximity sensor ignore angle 4
    // @Units: degrees
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG4", 10, AP_Proximity, _ignore_angle_deg[3], 0),

    // @Param: _IGN_WID4
    // @DisplayName: Proximity sensor ignore width 4
    // @Description: Proximity sensor ignore width 4
    // @Units: degrees
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("_IGN_WID4", 11, AP_Proximity, _ignore_width_deg[3], 0),

    // @Param: _IGN_ANG5
    // @DisplayName: Proximity sensor ignore angle 5
    // @Description: Proximity sensor ignore angle 5
    // @Units: degrees
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG5", 12, AP_Proximity, _ignore_angle_deg[4], 0),

    // @Param: _IGN_WID5
    // @DisplayName: Proximity sensor ignore width 5
    // @Description: Proximity sensor ignore width 5
    // @Units: degrees
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("_IGN_WID5", 13, AP_Proximity, _ignore_width_deg[4], 0),

    // @Param: _IGN_ANG6
    // @DisplayName: Proximity sensor ignore angle 6
    // @Description: Proximity sensor ignore angle 6
    // @Units: degrees
    // @Range: 0 360
    // @User: Standard
    AP_GROUPINFO("_IGN_ANG6", 14, AP_Proximity, _ignore_angle_deg[5], 0),

    // @Param: _IGN_WID6
    // @DisplayName: Proximity sensor ignore width 6
    // @Description: Proximity sensor ignore width 6
    // @Units: degrees
    // @Range: 0 45
    // @User: Standard
    AP_GROUPINFO("_IGN_WID6", 15, AP_Proximity, _ignore_width_deg[5], 0),

#if PROXIMITY_MAX_ANALOG > 0
    // @Param: _ANLG_FUNC
    // @DisplayName: Analog sensor function
    // @Description: Function that is used to convert voltage to distance for analog sensors
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic,3:Power
    // @User: Standard
    AP_GROUPINFO("_ANLG_FUNC", 19, AP_Proximity, state[0].function, 0),

    // @Param: _ANLG_SCALING
    // @DisplayName: Analog sensor scaling
    // @Description: Scaling for analog sensors
    // @User: Standard
    AP_GROUPINFO("_ANLG_SCALING", 20, AP_Proximity, state[0].scaling, 3.0f),

    // @Param: _ANLG_OFFSET
    // @DisplayName: Analog sensor offset
    // @Description: Offset for analog sensors
    // @User: Standard
    AP_GROUPINFO("_ANLG_OFFSET",  21, AP_Proximity, state[0].offset, 0.0f),

    // @Param: _ANLG_MIN_M
    // @DisplayName: Analog sensor minimum distance
    // @Description: Minimum distance in meters for analog sensors
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_ANLG_MIN_M", 22, AP_Proximity, state[0].distance_min, 0.2f),

    // @Param: _ANLG_MAX_M
    // @DisplayName: Analog sensor maximum distance
    // @Description: Maximum distance in meters for analog sensors
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_ANLG_MAX_M", 23, AP_Proximity, state[0].distance_max, 7.0f),

    // @Param: _ANLG_CUTOFF
    // @DisplayName: Analog sensor cutoff frequency
    // @Description: Cutoff frequency in hertz for filtering distance readings
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_ANLG_CUTOFF", 24, AP_Proximity, state[0].cutoff_frequency, 0.0f),

    // @Param: _ANLG1_PIN
    // @DisplayName: Analog sensor 1 pin
    // @Description: Pin number for first analog sensor. Set to -1 to disable
    // @User: Standard
    AP_GROUPINFO("_ANLG1_PIN", 25, AP_Proximity, state[0].pin[0], -1),

    // @Param: _ANLG1_ORIENT
    // @DisplayName: Analog sensor 1 orientation
    // @Description: Orientation of first analog sensor
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up
    AP_GROUPINFO("_ANLG1_ORIENT", 26, AP_Proximity, state[0].orientation[0], 0),
#endif

#if PROXIMITY_MAX_ANALOG > 1
    // @Param: _ANLG2_PIN
    // @DisplayName: Analog sensor 2 pin
    // @Description: Pin number for second analog sensor. Set to -1 to disable
    // @User: Standard
    AP_GROUPINFO("_ANLG2_PIN", 27, AP_Proximity, state[0].pin[1], -1),

    // @Param: _ANLG2_ORIENT
    // @DisplayName: Analog sensor 2 orientation
    // @Description: Orientation of second analog sensor
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up
    // @User: Standard
    AP_GROUPINFO("_ANLG2_ORIENT", 28, AP_Proximity, state[0].orientation[1], 2),
#endif

#if PROXIMITY_MAX_ANALOG > 2
    // @Param: _ANLG3_PIN
    // @DisplayName: Analog sensor 3 pin
    // @Description: Pin number for third analog sensor. Set to -1 to disable
    // @User: Standard
    AP_GROUPINFO("_ANLG3_PIN", 29, AP_Proximity, state[0].pin[2], -1),

    // @Param: _ANLG3_ORIENT
    // @DisplayName: Analog sensor 3 orientation
    // @Description: Orientation of third analog sensor
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up
    // @User: Standard
    AP_GROUPINFO("_ANLG3_ORIENT", 30, AP_Proximity, state[0].orientation[2], 4),
#endif

#if PROXIMITY_MAX_ANALOG > 3
    // @Param: _ANLG4_PIN
    // @DisplayName: Analog sensor 4 pin
    // @Description: Pin number for fourth analog sensor. Set to -1 to disable
    // @User: Standard
    AP_GROUPINFO("_ANLG4_PIN", 31, AP_Proximity, state[0].pin[3], -1),

    // @Param: _ANLG4_ORIENT
    // @DisplayName: Analog sensor 4 orientation
    // @Description: Orientation of fourth analog sensor
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up
    // @User: Standard
    AP_GROUPINFO("_ANLG4_ORIENT", 32, AP_Proximity, state[0].orientation[3], 6),
#endif

#if PROXIMITY_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C,2:MAVLink,3:TeraRangerTower,4:RangeFinder
    // @User: Advanced
    AP_GROUPINFO("2_TYPE", 16, AP_Proximity, _type[1], 0),

    // @Param: 2_ORIENT
    // @DisplayName: Second Proximity sensor orientation
    // @Description: Second Proximity sensor orientation
    // @Values: 0:Default,1:Upside Down
    // @User: Standard
    AP_GROUPINFO("2_ORIENT", 17, AP_Proximity, _orientation[1], 0),

    // @Param: 2_YAW_CORR
    // @DisplayName: Second Proximity sensor yaw correction
    // @Description: Second Proximity sensor yaw correction
    // @Units: degrees
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("2_YAW_CORR", 18, AP_Proximity, _yaw_correction[1], PROXIMITY_YAW_CORRECTION_DEFAULT),
#endif

    AP_GROUPEND
};

AP_Proximity::AP_Proximity(AP_SerialManager &_serial_manager) :
    primary_instance(0),
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the Proximity class. We do detection of attached sensors here
// we don't allow for hot-plugging of sensors (i.e. reboot required)
void AP_Proximity::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<PROXIMITY_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        // initialise status
        state[i].status = Proximity_NotConnected;
    }
}

// update Proximity state for all instances. This should be called at a high rate by the main loop
void AP_Proximity::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (_type[i] == Proximity_Type_None) {
                // allow user to disable a proximity sensor at runtime
                state[i].status = Proximity_NotConnected;
                continue;
            }
            drivers[i]->update();
        }
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != nullptr && (state[i].status == Proximity_Good)) {
            primary_instance = i;
        }
    }
}

// return sensor orientation
uint8_t AP_Proximity::get_orientation(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 0;
    }

    return _orientation[instance].get();
}

// return sensor yaw correction
int16_t AP_Proximity::get_yaw_correction(uint8_t instance) const
{
    if (instance >= PROXIMITY_MAX_INSTANCES) {
        return 0;
    }

    return _yaw_correction[instance].get();
}

// return sensor health
AP_Proximity::Proximity_Status AP_Proximity::get_status(uint8_t instance) const
{
    // sanity check instance number
    if (instance >= num_instances) {
        return Proximity_NotConnected;
    }

    return state[instance].status;
}

AP_Proximity::Proximity_Status AP_Proximity::get_status() const
{
    return get_status(primary_instance);
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Proximity::handle_msg(mavlink_message_t *msg)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (_type[i] != Proximity_Type_None)) {
            drivers[i]->handle_msg(msg);
        }
    }
}

//  detect if an instance of a proximity sensor is connected.
void AP_Proximity::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];
    if (type == Proximity_Type_SF40C) {
        if (AP_Proximity_LightWareSF40C::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF40C(*this, state[instance], serial_manager);
            return;
        }
    }
    if (type == Proximity_Type_MAV) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_MAV(*this, state[instance]);
        return;
    }
    if (type == Proximity_Type_TRTOWER) {
        if (AP_Proximity_TeraRangerTower::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_TeraRangerTower(*this, state[instance], serial_manager);
            return;
        }
    }
    if (type == Proximity_Type_RangeFinder) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_RangeFinder(*this, state[instance]);
        return;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (type == Proximity_Type_SITL) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_SITL(*this, state[instance]);
        return;
    }
#endif
#if PROXIMITY_MAX_ANALOG > 0
    if (instance == 0 && type == Proximity_Type_Analog) {
        state[instance].instance = instance;
        drivers[instance] = new AP_Proximity_Analog(*this, state[instance]);
        return;
    }
#endif
}

// get distance in meters in a particular direction in degrees (0 is forward, clockwise)
// returns true on successful read and places distance in distance
bool AP_Proximity::get_horizontal_distance(uint8_t instance, float angle_deg, float &distance) const
{
    if ((drivers[instance] == nullptr) || (_type[instance] == Proximity_Type_None)) {
        return false;
    }
    // get distance from backend
    return drivers[instance]->get_horizontal_distance(angle_deg, distance);
}

// get distance in meters in a particular direction in degrees (0 is forward, clockwise)
// returns true on successful read and places distance in distance
bool AP_Proximity::get_horizontal_distance(float angle_deg, float &distance) const
{
    return get_horizontal_distance(primary_instance, angle_deg, distance);
}

// get distances in 8 directions. used for sending distances to ground station
bool AP_Proximity::get_horizontal_distances(Proximity_Distance_Array &prx_dist_array) const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return false;
    }
    // get distances from backend
    return drivers[primary_instance]->get_horizontal_distances(prx_dist_array);
}

// get boundary points around vehicle for use by avoidance
//   returns nullptr and sets num_points to zero if no boundary can be returned
const Vector2f* AP_Proximity::get_boundary_points(uint8_t instance, uint16_t& num_points) const
{
    if ((drivers[instance] == nullptr) || (_type[instance] == Proximity_Type_None)) {
        num_points = 0;
        return nullptr;
    }
    // get boundary from backend
    return drivers[instance]->get_boundary_points(num_points);
}

const Vector2f* AP_Proximity::get_boundary_points(uint16_t& num_points) const
{
    return get_boundary_points(primary_instance, num_points);
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity::get_closest_object(float& angle_deg, float &distance) const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return false;
    }
    // get closest object from backend
    return drivers[primary_instance]->get_closest_object(angle_deg, distance);
}

// get number of objects, used for non-GPS avoidance
uint8_t AP_Proximity::get_object_count() const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return 0;
    }
    // get count from backend
    return drivers[primary_instance]->get_object_count();
}

// get an object's angle and distance, used for non-GPS avoidance
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity::get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return false;
    }
    // get angle and distance from backend
    return drivers[primary_instance]->get_object_angle_and_distance(object_number, angle_deg, distance);
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity::distance_max() const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return 0.0f;
    }
    // get maximum distance from backend
    return drivers[primary_instance]->distance_max();
}
float AP_Proximity::distance_min() const
{
    if ((drivers[primary_instance] == nullptr) || (_type[primary_instance] == Proximity_Type_None)) {
        return 0.0f;
    }
    // get minimum distance from backend
    return drivers[primary_instance]->distance_min();
}

// get distance in meters upwards, returns true on success
bool AP_Proximity::get_upward_distance(uint8_t instance, float &distance) const
{
    if ((drivers[instance] == nullptr) || (_type[instance] == Proximity_Type_None)) {
        return false;
    }
    // get upward distance from backend
    return drivers[instance]->get_upward_distance(distance);
}

bool AP_Proximity::get_upward_distance(float &distance) const
{
    return get_upward_distance(primary_instance, distance);
}

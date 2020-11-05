#include <iostream>
#include "Copter.h"
#include <math.h>

using namespace std;

/*
 * Init and run calls for autonomous flight mode (largely based off of the AltHold flight mode)
 */

// autonomous_init - initialise autonomous controller
bool Copter::autonomous_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    // reset integrators for roll and pitch controllers
    g.pid_roll.reset_I();
    g.pid_pitch.reset_I();

    return true;
}

// autonomous_run - runs the autonomous controller
// should be called at 100hz or more
void Copter::autonomous_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // desired roll, pitch, and yaw_rate
    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif
    target_climb_rate = 0.0f;

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

        case AltHold_MotorStopped:

            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME
            // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
#else
            pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
            pos_control->update_z_controller();
            break;

        case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME
            if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
            // set motors to full range
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

            // initiate take-off
            if (!takeoff_state.running) {
                takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
                // indicate we are taking off
                set_land_complete(false);
                // clear i terms
                set_throttle_takeoff();
            }

            // get take-off adjusted pilot and takeoff climb rates
            takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

            // call position controller
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
            pos_control->update_z_controller();
            break;

        case AltHold_Landed:
            // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
            if (target_climb_rate < 0.0f) {
                motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
            } else {
                motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
            }
            //okok

#if FRAME_CONFIG == HELI_FRAME
            if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
#endif
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
            pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
            pos_control->update_z_controller();
            break;

        case AltHold_Flying:
            // compute the target climb rate, roll, pitch and yaw rate
            // land if autonomous_controller returns false
            if (!autonomous_controller(target_climb_rate, target_roll, target_pitch, target_yaw_rate)) {
                // switch to land mode
                set_mode(LAND, MODE_REASON_MISSION_END);
                break;
            }

            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
            // apply avoidance
            avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

            // adjust climb rate using rangefinder
            if (rangefinder_alt_ok()) {
                // if rangefinder is ok, use surface tracking
                target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
            }

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // call position controller
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->update_z_controller();
            break;
    }
}

// autonomous_controller - computes target climb rate, roll, pitch, and yaw rate for autonomous flight mode
// returns true to continue flying, and returns false to land
bool Copter::autonomous_controller(float &target_climb_rate, float &target_roll, float &target_pitch, float &target_yaw_rate)
{
    //static std::vector pts;
    // get downward facing sensor reading in meters
    static int mstate=0;
    //static int switches=0;

    // get horizontal sensor readings in meters
    float dist_forward, dist_right, dist_backward, dist_left;
    g2.proximity.get_horizontal_distance(0, dist_forward);
    g2.proximity.get_horizontal_distance(90, dist_right);
    g2.proximity.get_horizontal_distance(180, dist_backward);
    g2.proximity.get_horizontal_distance(270, dist_left);

    float dists [4]={dist_forward,dist_right,dist_backward,dist_left};
    /*for (int i = 0; i < 4; i++) {
        float orient=
    }*/


    // set desired climb rate in centimeters per second
    target_climb_rate = 0.0f;
    target_pitch=0;
    target_roll=0;
    // set desired roll and pitch in centi-degrees
    float crashlim=10*(0.3f);
    for (int i = 0; i < 4; i++) {
        if (dists[i]<crashlim){
            float mult=(i==3||i==0)?100.0f:-100.0f;
            if (i==0||i==2){
                g.pid_pitch.set_input_filter_all(crashlim-dists[i]);
                target_pitch=mult*g.pid_pitch.get_pid();
            }else{
                g.pid_roll.set_input_filter_all(crashlim-dists[i]);
                target_roll=mult*g.pid_roll.get_pid();
            }
        }
    }
    /*
    g.pid_pitch.set_input_filter_all(10*(0.5f)-dist_forward);
    target_pitch=100.0f*g.pid_pitch.get_pid();
    //TODO REDO THIS SO IT ONLY CONTROLS WHEN VERY CLOSE
    g.pid_roll.set_input_filter_all(10*(0.5f)-dist_right);
    target_roll=100.0f*g.pid_roll.get_pid();
     */
    bool runningmaze= true;
    float limit=10*(0.4f);
    if (runningmaze){
        if (dist_forward>limit){
            mstate=0;
        }else if (mstate==0){
            if (dist_left>limit){
                mstate=3;
            }else if (dist_right>limit){
                mstate=1;
            }
        }else if (mstate==1){//Searching right
            if (dist_right<limit){
                mstate=3;
            }
        }else if (mstate==3){//Searching left
            if (dist_left<limit){
                mstate=1;
            }
        }
        float mult=(mstate==0||mstate==3)?100.0f:-100.0f;
        if (mstate==0||mstate==2){
            g.pid_pitch.set_input_filter_all(10*(0.4f)-dists[mstate]);
            target_pitch=mult*g.pid_pitch.get_pid();
            if (target_pitch>400){target_pitch=400;}
            if (target_pitch<-400){target_pitch=-400;}
        }else{
            g.pid_roll.set_input_filter_all(10*(0.4f)-dists[mstate]);
            target_roll=mult*g.pid_roll.get_pid();
            if (target_roll>400){target_roll=400;}
            if (target_roll<-400){target_roll=-400;}
        }
    }
    float slimit=10*(0.5f);
    int numin=0;
    for (int i = 0; i < 4; i++) {
        if(i==1){
            if (dists[i]>slimit){
                numin++;
            }
        }else{
            if (dists[i]<slimit){
                numin++;
            }
        }
    }
    bool running=numin!=4;
    if (!running){
        const string stmsg3=(" BREAKING ");
        gcs_send_text(MAV_SEVERITY_INFO,stmsg3.c_str());
    }


    //current_loc.lng

    // set desired yaw rate in centi-degrees per second (set to zero to hold constant heading)
    target_yaw_rate = 0.0f;
    //if(maze_clear()){return false;}
    static int counter=0;
    if(counter++>400){
        /*for (int i = 0; i < 4; i++) {
            float orient=
        }*/
        //string test="orient "+std::to_string(channel_yaw->get_control_in())+" pos : "+std::to_string(current_loc.lng)+", "+std::to_string(current_loc.lat);
        //char *word;
        const string stmsg=("target roll : "+std::to_string(target_roll)+", pitch : "+std::to_string(target_pitch));
        const string stmsg1=(" state : "+std::to_string(mstate)+", running = "+std::string(running));
        const string stmsg2=(" dists : "+std::to_string(dists[0])+", "+std::to_string(dists[1])+", "+std::to_string(dists[2])+", "+std::to_string(dists[3])+", ");

        gcs_send_text(MAV_SEVERITY_INFO,stmsg.c_str());
        gcs_send_text(MAV_SEVERITY_INFO,stmsg1.c_str());
        gcs_send_text(MAV_SEVERITY_INFO,stmsg2.c_str());
        counter=0;
    }

    return running;
}

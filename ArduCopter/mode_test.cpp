#include "Copter.h"

#if MODE_TEST_ENABLED == ENABLED

bool ModeTest::init(bool ignore_checks)
{
    // copter.set_auto_armed(true);
    //init_flag = false;
    set_submode(SubMode::TAKEOFF);
    //takeoff_start(500);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "_yaw_angle_cd:%.2f fixed_yaw_angle_cd:%.2f",
    //                 auto_yaw.get_yaw_angle_cd(),
    //                 auto_yaw.get_fixed_yaw_offset_cd());

    //pos_control->init_xy_controller();
    //pos_control->init_z_controller();
    //wp_nav->wp_and_spline_init();
    
    return true;
}

void ModeTest::run()
{
    switch (_mode){
   
    case SubMode::TAKEOFF:
        if (!init_flag){           
            takeoff_init(500);
            init_flag = true;
            //wp_nav->wp_and_spline_init();            
        }
        else{
            takeoff_run();
        }
        break;

    case SubMode::PROCESS:
        if (!init_flag){           
            process_init();
            init_flag = true; 
        }
        else{
            process_run();
        } 
        break;

    case SubMode::LAND:
        if (!init_flag){
            land_init();
            init_flag = true;
        }
        else{
            land_run();
        }                
        break;

    }
}

void ModeTest::takeoff_run()
{
    //这行代码如果放在这里，解锁后就能自动起飞
    //如果放在init()中，就不行
    copter.set_auto_armed(true);

    auto_takeoff_run();
    if (auto_takeoff_complete)
    {
        static uint32_t takeoff_finish_time = millis();
        if (millis() - takeoff_finish_time > 5000)
        {
            set_submode(SubMode::PROCESS);
            gcs().send_text(MAV_SEVERITY_INFO, "start test");
        }
        
    }
    
}

void ModeTest::process_run()
{
    static uint32_t sim_time = millis();

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed())
    {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    if (!auto_yaw.reached_fixed_yaw_target())
    {
        if (millis() - sim_time > 500) // call as 2Hz
        {
            sim_time = millis();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "_yaw_angle_cd:%.2f fixed_yaw_angle_cd:%.2f",
                            auto_yaw.get_yaw_angle_cd(),
                            auto_yaw.get_fixed_yaw_offset_cd());
        }
    }
    else
    {
        set_submode(SubMode::LAND);
        gcs().send_text(MAV_SEVERITY_INFO, "start land");
        //copter.set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
    }
}

void ModeTest::land_run()
{
        // disarm when the landing detector says we've landed(降落后上锁-disarmed)
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
        set_submode(SubMode::TAKEOFF);
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // 在开始下降之前暂停一下(等待5s后下降)
        if (land_pause && millis()-land_start_time >= 5000) {
            land_pause = false;
            gcs().send_text(MAV_SEVERITY_INFO, "start landing");

        }

        // 进行正常的下降（10m之上1.5m/s；10米以内0.5m/s）
        land_run_normal_or_precland(land_pause);
        //gcs().send_text(MAV_SEVERITY_INFO, "nihao");

    }
}

void ModeTest::set_submode(SubMode new_submode)
{
        // return immediately if the submode has not been changed
    if (new_submode == _mode) {
        return;
    }

    // backup old mode
    //SubMode old_submode = _mode;

    // set mode
    _mode = new_submode;

    init_flag = false;//切换到一个新的子模式时，还未进行初始化
}

void ModeTest::takeoff_init(int32_t alt_target_cm)
{
    bool alt_target_terrain = false;
    float current_alt_cm = inertial_nav.get_position_z_up_cm();

    int32_t alt_target_min_cm = current_alt_cm + (copter.ap.land_complete ? 100 : 0);
    alt_target_cm = MAX(alt_target_cm, alt_target_min_cm);

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_z_controller();    

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff_start(alt_target_cm, alt_target_terrain);

    // set submode
    set_submode(SubMode::TAKEOFF);

}

void ModeTest::process_init()
{
    set_yaw();
    wp_nav->wp_and_spline_init();
}

void ModeTest::land_init()
{
    /*

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the horizontal position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }
    
    */

    land_start_time = millis();
    gcs().send_text(MAV_SEVERITY_INFO, "land init finish");
    land_pause = true;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

void ModeTest::set_yaw()
{
    float yaw_cd = 90.0f;       // 偏航角度
    float turn_rate_ds = 45.0f; // 旋转角速度
    int8_t direction = 1;       // 旋转方向（正->顺时针）
    bool relative_angle = true; // 角度类型(true->相对角)

    auto_yaw.set_fixed_yaw(yaw_cd, turn_rate_ds, direction, relative_angle);
}

#endif

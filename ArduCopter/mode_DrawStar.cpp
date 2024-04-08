#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * 五角星航线模式初始化
 */



bool ModeDrawStar::init(bool ignore_checks)
{
    path_num = 0;   //航点号清0，从别的模式切到此模式时，可以飞出一个新的五角星
    generate_path();    //生成五角星航线
    pos_control_start();    //开始位置控制

    return true;
}

//生成五角星航线
void ModeDrawStar::generate_path()
{
    float radius_cm = g2.star_radius_cm;

    wp_nav->get_wp_stopping_point(path[0]);

    //xy轴绘制五角星
    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];   

}


//开始位置控制
void ModeDrawStar::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

//此模式的周期调用
void ModeDrawStar::run()
{
    switch (_mode)
    {
    case SubMode::TAKEOFF:
        //takeoff_run();
        break;

    case SubMode::DRAW5STAR:
        draw5star_run();
        break;

    case SubMode::LAND:
        land_run();
        break;
    }


}

void ModeDrawStar::draw5star_run()
{
    if (path_num < 6) {  // 五角星航线尚未走完
        if (wp_nav->reached_wp_destination()) 
        {  // 到达某个端点
            path_num++;
            wp_nav->set_wp_destination(path[path_num], false);  // 将下一个航点位置设置为导航控制模块的目标位置
        }
    } else if ((path_num == 6) && wp_nav->reached_wp_destination()) 
    {  // 五角星航线运行完成，自动进入Loiter模式
        gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into land submode");
        //copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
        set_submode(SubMode::LAND);
        land_start_time = millis();
    }

    pos_control_run();  // 位置控制器
}

void ModeDrawStar::pos_control_run()  // 注意，此函数直接从mode_guided.cpp中复制过来，不需要改其中的内容
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
    
}

void ModeDrawStar::set_submode(SubMode new_submode)
{
    // return immediately if the submode has not been changed
    if (new_submode == _mode) {
        return;
    }

    // backup old mode
    //SubMode old_submode = _mode;

    // set mode
    _mode = new_submode;
    
}

void ModeDrawStar::land_run()
{
    
    // disarm when the landing detector says we've landed(降落后上锁-disarmed)
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // // if not armed set throttle to zero and exit immediately
    // if (is_disarmed_or_landed()) {
    //     make_safe_ground_handling();
    //     return;
    // }

    // // set motors to full range
    // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // // run normal landing or precision landing (if enabled)
    // land_run_normal_or_precland();
        // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent(等待10s后下降)
        if (land_pause && millis()-land_start_time >= 10000) {
            land_pause = false;
            gcs().send_text(MAV_SEVERITY_INFO, "real land");

        }

        // run normal landing or precision landing (if enabled)
        land_run_normal_or_precland(land_pause);
    }
}

// void ModeDrawStar::takeoff_start()
// {
//     // initialise yaw
//     auto_yaw.set_mode(AutoYaw::Mode::HOLD);

//     // clear i term when we're taking off
//     pos_control->init_z_controller();
    
//     set_submode(SubMode::TAKEOFF);
// }

#endif

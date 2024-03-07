#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * 画爱心
 */



bool ModeDrawLove::init(bool ignore_checks)
{
    path_num = 0;   //航点号清0，从别的模式切到此模式时，可以飞出一个新的五角星
    generate_point();    //生成爱心的四个航电
    pos_control_start();    //开始位置控制

    return true;
}

//生成圆心和航点
void ModeDrawLove::generate_point()
{
    //float radius_cm = g2.star_radius_cm;
    float radius_cm = 1000.0f;
    wp_nav->get_wp_stopping_point(path[0]);

    //生成圆心
    circle_center[0] = path[0] - Vector3f(0, 1.0f ,0) * radius_cm;
    circle_center[1] = path[0] + Vector3f(0, 1.0f ,0) * radius_cm;
    circle_center[2] = circle_center[0];
    circle_center[3] = circle_center[1];    

    //生成航点
    path[1] = path[0] - Vector3f(0, 2.0f ,0) * radius_cm;
    path[2] = path[0] - Vector3f(20.0f * safe_sqrt(2.0f), 0, 0) * radius_cm;
    path[3] = path[0] + Vector3f(0, 2.0f ,0) * radius_cm; 
    path[4] = path[1];

}


//开始位置控制
void ModeDrawLove::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
    // 这两个目的地的初始化也不知道 有啥区别，看着效果差不多
    wp_nav->set_wp_destination(path[0], false);
    // wp_nav->set_spline_destination(path[1], false, path[2], false, false);

    // initialise circle controller
    copter.circle_nav->init(circle_center[0], false, copter.circle_nav->get_rate());
    
    // initialise yaw
    //auto_yaw.set_mode_to_default(false);
    auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);
}

//此模式的周期调用
void ModeDrawLove::run()
{
    if (path_num < 5) {  // 五角星航线尚未走完
        if (wp_nav->reached_wp_destination()) {  // 到达某个端点
            path_num++;
            wp_nav->set_spline_destination(path[path_num], false, path[path_num+1], false, true);  // 将下一个航点位置设置为导航控制模块的目标位置
        }
    }else if ((path_num == 5) && wp_nav->reached_wp_destination()){
        path_num = 6;
        wp_nav->set_spline_destination(path[6], false, path[2], false, true);
    }
    else if ((path_num == 6) && wp_nav->reached_wp_destination()) {  // 五角星航线运行完成，自动进入Loiter模式
        gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into loiter mode");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
    }

    pos_control_run();  // 位置控制器

}

void ModeDrawLove::pos_control_run()  // 注意，此函数直接从mode_guided.cpp中复制过来，不需要改其中的内容
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
    //copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // run circlr controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
    
}

void ModeDrawLove::circle_start()
{
    // initialise circle controller
    copter.circle_nav->init(circle_center[0], false, copter.circle_nav->get_rate());

    if (auto_yaw.mode() != AutoYaw::Mode::ROI) {
        auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);
    }

}

#endif

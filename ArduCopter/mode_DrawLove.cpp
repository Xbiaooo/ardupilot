#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * 画爱心
 */
#define DO_NOT_USE_DOUBLE_MATHS


bool ModeDrawLove::init(bool ignore_checks)
{
    path_num = 0;   //航点号清0
    center_num = 0;
    generate_point();    //生成爱心的四个航点
    pos_control_start();    //开始位置控制

    return true;
}

//生成圆心和航点
void ModeDrawLove::generate_point()
{
    //float radius_cm = g2.star_radius_cm;
    
    wp_nav->get_wp_stopping_point(path[0]);

    //生成圆心
    center[0] = path[0] + Vector3f(1.0f, 0 ,0) * radius_cm;
    center[1] = path[0] - Vector3f(1.0f, 0 ,0) * radius_cm;
    center[2] = center[0];
    center[3] = center[1];    

    //生成半径
    radius[0] = radius_cm;
    radius[1] = 3*radius_cm;
    radius[2] = radius[1];
    radius[3] = radius[0];

    //生成航点
    path[1] = path[0] + Vector3f(2.0f, 0 ,0) * radius_cm;
    path[2] = path[0] - Vector3f(0, 2.0f * safe_sqrt(2.0f), 0) * radius_cm;
    path[3] = path[0] - Vector3f(2.0f, 0 ,0) * radius_cm; 
    path[4] = path[0];

    

}


//开始位置控制
void ModeDrawLove::pos_control_start()
{
    // initialise waypoint and spline controller
    //wp_nav->wp_and_spline_init();

    // set speed and acceleration limits
    // pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    // pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    // pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // no need to check return status because terrain data is not used
    // 这两个目的地的初始化也不知道 有啥区别，看着效果差不多
    //wp_nav->set_wp_destination(path[0], false);
    // wp_nav->set_spline_destination(path[1], false, path[2], false, false);

    // 初始化画圆控制器
    copter.circle_nav->set_radius_cm(radius[0]);
    copter.circle_nav->init(center[0], false, -10.0f);
    //初始化圆心、画圆角速度和半径
    /// copter.circle_nav->set_center(center[0], false);
    /// copter.circle_nav->set_rate(-20.0f);
    

    //得到该圆上距离无人机当前位置最近的点
    /// Vector3f circle_edge_neu;
    /// copter.circle_nav->get_closest_point_on_circle(circle_edge_neu);

    /// wp_nav->set_wp_destination(circle_edge_neu,false);

    //copter.circle_nav->get_closest_point_on_circle(path[1]);
    //copter.circle_nav->init();//这种初始化能使直接画圆

    
    // initialise yaw
    auto_yaw.set_mode_to_default(false);
    //auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);
}

//此模式的周期调用
void ModeDrawLove::run()
{
    // if (path_num < 5) {  // 五角星航线尚未走完
    //     if (wp_nav->reached_wp_destination()) {  // 到达某个端点
    //         path_num++;
    //         wp_nav->set_spline_destination(path[path_num], false, path[path_num+1], false, true);  // 将下一个航点位置设置为导航控制模块的目标位置
    //     }
    // }else if ((path_num == 5) && wp_nav->reached_wp_destination()){
    //     path_num = 6;
    //     wp_nav->set_spline_destination(path[6], false, path[2], false, true);
    // }
    // else if ((path_num == 6) && wp_nav->reached_wp_destination()) {  // 五角星航线运行完成，自动进入Loiter模式
    //     gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into loiter mode");
    //     copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
    // }
    // gcs().send_text(MAV_SEVERITY_CRITICAL, 
    //             "目前航点: %d",path_num);
               
    pos_xy = inertial_nav.get_position_xy_cm();
    float distance; 
    distance = sqrtf(powf(pos_xy.x-path[path_num+1].x, 2.0f) + powf(pos_xy.y-path[path_num+1].y, 2.0f));

    gcs().send_text(MAV_SEVERITY_CRITICAL, 
                "当前距离: %.1f",distance); 


    if (path_num < 3){
        //distance = (pow(pos_xy.x-path[path_num+1].x, 2) + pow(pos_xy.y-path[path_num+1].y, 2));
        if ((distance/100.0f) < 1.0f){//到达了要去的下个航点
            path_num++;
            center_num++;
            copter.circle_nav->init(center[center_num], false, copter.circle_nav->get_rate());
            copter.circle_nav->set_radius_cm(radius[center_num]);
        }
    }else if((path_num == 3) && ((distance/100.0f) < 1.0f)){//到达了最终航点(起始点)
        gcs().send_text(MAV_SEVERITY_INFO, "Draw love finished, now go into loiter mode");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
    }

    pos_control_run();  // 位置控制器

}

void ModeDrawLove::pos_control_run()
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
    //copter.failsafe_terrain_set_status(wp_nav->update_wpnav());//运行航点控制器

    // run circlr controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
    
}


#endif

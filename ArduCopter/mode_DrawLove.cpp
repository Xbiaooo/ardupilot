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
    //path[0] = inertial_nav.get_position_neu_cm();

    //生成圆心
    center[0] = path[0] + Vector3f(0, 1.0f ,0) * radius_cm;
    center[1] = path[0] - Vector3f(0, 1.0f ,0) * radius_cm;
    center[2] = center[0];
    center[3] = center[1];    

    //生成半径
    radius[0] = radius_cm;
    radius[1] = 3*radius_cm;
    radius[2] = radius[1];
    radius[3] = radius[0];

    //生成航点
    path[1] = path[0] + Vector3f(0, 2.0f ,0) * radius_cm;
    path[2] = path[0] - Vector3f(2.0f * safe_sqrt(2.0f), 0, 0) * radius_cm;
    path[3] = path[0] - Vector3f(0, 2.0f ,0) * radius_cm; 
    path[4] = path[0];

    

}


//开始位置控制
void ModeDrawLove::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // set speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // no need to check return status because terrain data is not used
    // 这两个目的地的初始化也不知道 有啥区别，看着效果差不多
    //wp_nav->set_wp_destination(path[0], false);
    // wp_nav->set_spline_destination(path[1], false, path[2], false, false);

    // 初始化画圆控制器
    
    //copter.circle_nav->init(center[0], false, -10.0f);
    //初始化圆心、画圆角速度和半径
    copter.circle_nav->set_center(center[0], false);
    copter.circle_nav->set_rate(15.0f);
    copter.circle_nav->set_radius_cm(radius[0]);


    
    copter.circle_nav->init(copter.circle_nav->get_center(), copter.circle_nav->center_is_terrain_alt(), copter.circle_nav->get_rate());

    //得到该圆上距离无人机当前位置最近的点
    // Vector3f circle_edge_neu;
    // copter.circle_nav->get_closest_point_on_circle(circle_edge_neu);
    // wp_nav->set_wp_destination(circle_edge_neu,false);

    //copter.circle_nav->get_closest_point_on_circle(path[1]);
    //copter.circle_nav->init();//这种初始化能使直接画圆

    
    // initialise yaw
    //auto_yaw.set_mode_to_default(false);
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

//此模式的周期调用
void ModeDrawLove::run()
{      
    pos_xy = inertial_nav.get_position_xy_cm();
    float distance; 
    distance = sqrtf(powf(pos_xy.x-path[path_num+1].x, 2.0f) + powf(pos_xy.y-path[path_num+1].y, 2.0f));

    // gcs().send_text(MAV_SEVERITY_CRITICAL, 
    //             "当前距离: %.1f",distance); 
    gcs().send_text(MAV_SEVERITY_CRITICAL, 
                "当前距离: %.1f",distance);


    if (path_num < 3){
        //distance = (pow(pos_xy.x-path[path_num+1].x, 2) + pow(pos_xy.y-path[path_num+1].y, 2));
        if ((distance/100.0f) < 0.30f){//到达了要去的下个航点
            if (flag == 0)
            {
                wp_nav->wp_and_spline_init();
                flag = 1;
                //auto_yaw.set_mode_to_default(false);
                wp_nav->set_wp_destination(path[path_num+1],false);
            }
            else
            {
                if(wp_nav->reached_wp_destination()){
                    flag = 0;
                    path_num++;
                    center_num++;
                    //auto_yaw.set_mode(AutoYaw::Mode::HOLD);
                    //auto_yaw.set_mode_to_default(false);
                    if (path_num == 1 || path_num == 2)
                    {
                        copter.circle_nav->set_rate(10.0f);
                    }
                    else
                    {
                       copter.circle_nav->set_rate(15.0f); 
                    }
                    copter.circle_nav->init(center[center_num], false, copter.circle_nav->get_rate());
                    copter.circle_nav->set_radius_cm(radius[center_num]);
                } 
            }
        }
    }else if(path_num == 3){//到达了最终航点(起始点)
        if ((distance/100.0f) < 0.30f){
            if (flag == 0)
            {
                flag = 1;
                wp_nav->wp_and_spline_init();
                //auto_yaw.set_mode_to_default(false);
                wp_nav->set_wp_destination(path[path_num+1],false);
            }
            else
            {
                if(wp_nav->reached_wp_destination()){
                    flag = 0;
                    gcs().send_text(MAV_SEVERITY_INFO, "Draw love finished, now go into loiter mode");
                    copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
                }
            }
            
            
        }
    }

    if (flag == 0)
    {
        pos_control_run1();  // 位置控制器    
    }
    else
    {
        pos_control_run2();
    }
    


}

void ModeDrawLove::pos_control_run1()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    // motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
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
void ModeDrawLove::pos_control_run2()
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

#endif

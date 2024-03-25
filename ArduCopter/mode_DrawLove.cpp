#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * 画爱心
 */
#define DO_NOT_USE_DOUBLE_MATHS


bool ModeDrawLove::init(bool ignore_checks)
{
    line_num = 0;   //航点号清0
    center_num = 0;
    generate_point();    //生成爱心的四个航点
    pos_control_start();    //开始位置控制
    

    return true;
}

//生成圆心和航点
void ModeDrawLove::generate_point()
{
    //float radius_cm = g2.star_radius_cm;
    
    wp_nav->get_wp_stopping_point(start_point);
    //path[0] = inertial_nav.get_position_neu_cm();

    //生成圆心
    center[0] = start_point - Vector3f(0, 1.0f ,0) * radius_cm;
    center[1] = start_point + Vector3f(0, 1.0f ,0) * radius_cm;
    center[2] = center[0];
    center[3] = center[1];    

    //生成半径
    radius[0] = radius_cm;
    radius[1] = 3*radius_cm;
    radius[2] = radius[1];
    radius[3] = radius[0];

    //生成四段飞行弧线的角度
    angle_set[0] = 180;
    //angle_set[1] = degrees(atanf(2*sqrtf(2)));
    angle_set[1] = 90;
    angle_set[2] = angle_set[1];
    angle_set[3] = angle_set[0];
    

}


//开始位置控制
void ModeDrawLove::pos_control_start()
{
    // initialise waypoint and spline controller
    //wp_nav->wp_and_spline_init();

    // set speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);


    // 初始化画圆控制器
    
    //copter.circle_nav->init(center[0], false, -10.0f);
    //初始化圆心、画圆角速度和半径
    copter.circle_nav->set_center(center[0], false);
    copter.circle_nav->set_rate(-20.0f);
    copter.circle_nav->set_radius_cm(radius[0]);


    
    copter.circle_nav->init(copter.circle_nav->get_center(), copter.circle_nav->center_is_terrain_alt(), copter.circle_nav->get_rate());

    //copter.circle_nav->init();//这种初始化能使直接画圆

    
    // initialise yaw
    //auto_yaw.set_mode_to_default(false);
    //auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

//此模式的周期调用
void ModeDrawLove::run()
{

    _angle_tatol = degrees(copter.circle_nav->get_angle_total());
    //pos_xy = inertial_nav.get_position_xy_cm();
    //float distance; 
    //distance = sqrtf(powf(pos_xy.x-path[line_num+1].x, 2.0f) + powf(pos_xy.y-path[line_num+1].y, 2.0f));

    // gcs().send_text(MAV_SEVERITY_CRITICAL, 
    //             "当前距离: %.1f",distance); 
    gcs().send_text(MAV_SEVERITY_CRITICAL, 
                "当前角度: %.1f",_angle_tatol);


    if (line_num < 3){
        angle_tatol[line_num] = degrees(copter.circle_nav->get_angle_total());
        if ((abs(abs(angle_tatol[line_num]) - angle_set[line_num])) <= epsilon){//到达了要去的下个航点
            //angle_tatol = 0;
            //circle_nav _angle_total
            line_num++;
            center_num++;
            copter.circle_nav->set_radius_cm(radius[center_num]);
            copter.circle_nav->init(center[center_num], false, copter.circle_nav->get_rate());
            int i;
            for(i=0; i<=800; i++){

            }
        }
    }else if(line_num == 3){//到达了最终航点(起始点)
        if ((abs(abs(angle_tatol[line_num]) - angle_set[line_num])) <= epsilon){
            //wp_nav->set_wp_destination(path[line_num+1],false);
            //if(wp_nav->reached_wp_destination()){
                gcs().send_text(MAV_SEVERITY_INFO, "Draw love finished, now go into loiter mode");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
        }
        
        
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

#include "Copter.h"

#if MODE_DRAWSTAR_ENABLED == ENABLED

/*
 * 五角星航线模式初始化
 */


//模式初始化
bool ModeDrawStar::init(bool ignore_checks)
{
    _mode = SubMode::TAKEOFF;

    wp_nav->wp_and_spline_init();
    
    takeoff_flag = 0;
    draw5star_flag = 0;
    land_pause = true;

    takeoff_finish = true;
     
    //_mode = SubMode::DRAW5STAR;

    return true;
}

//此模式的周期调用
void ModeDrawStar::run()
{
    //gcs().send_text(MAV_SEVERITY_INFO,"%d",int(_mode));
    switch (_mode)
    {
    case SubMode::TAKEOFF:
        takeoff_run();
        break;

    case SubMode::DRAW5STAR:
        draw5star_run();
        break;

    case SubMode::LAND:
        land_run();
        break;
    }


}

//————切换子模式
void ModeDrawStar::set_submode(SubMode new_submode)
{
    // return immediately if the submode has not been changed
    if (new_submode == _mode) {
        return;
    }

    // set mode
    _mode = new_submode;
    
}

//————起飞初始化
// void ModeDrawStar::takeoff_start()
// {
//     float take_off_start_alt = inertial_nav.get_position_z_up_cm();//获取UAV当前海拔高度作为开始起飞时的高度
//     float take_off_complete_alt  = take_off_start_alt + target_alt_cm;  //计算UAV起飞完成时高度

//     // initialise yaw
//     auto_yaw.set_mode(AutoYaw::Mode::HOLD);

//     // clear i term when we're taking off
//     pos_control->init_z_controller();

//     auto_takeoff_start(take_off_complete_alt, false);
//     gcs().send_text(MAV_SEVERITY_INFO, "fuck!!!!");

// }

//————开始起飞
void ModeDrawStar::takeoff_run()
{
    //gcs().send_text(MAV_SEVERITY_INFO, "NOW: take off");
    // if (!takeoff_flag)
    // {
    //     takeoff_flag = 1;
        
    //     // initialise yaw
    //     auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    //     // clear i term when we're taking off
    //     pos_control->init_z_controller();

    //     // initialise alt for WP_NAVALT_MIN and set completion alt
    //     auto_takeoff_start(target_alt_cm, false);

        
    // }
    // else
    // {
    //     //gcs().send_text(MAV_SEVERITY_INFO, "real takeoff");
    //     copter.set_auto_armed(true);
    //     auto_takeoff_run();
    //     if(auto_takeoff_complete)
    //     {
    //         draw5star_flag = 0;
    //         set_submode(SubMode::DRAW5STAR);
    //         gcs().send_text(MAV_SEVERITY_INFO, "takeoff finished, now go into draw5star submode");
    //     }
    // }
    
    
    auto_takeoff_start(1000, false);
    copter.set_auto_armed(true);
    auto_takeoff_run(); 
    if(auto_takeoff_complete && takeoff_finish)
    {
        takeoff_finish = false;
        takeoff_finish_time = millis();
        gcs().send_text(MAV_SEVERITY_INFO, "takeoff finished,wait 5 seconds");
    }
    if(auto_takeoff_complete && millis()-takeoff_finish_time >= 5000 )
    {
        //draw5star_flag = 0;
        set_submode(SubMode::DRAW5STAR);
        gcs().send_text(MAV_SEVERITY_INFO, "now go into draw5star submode");
    }
    
    

}

// void ModeDrawStar::takeoff_run()
// {
//     if (!takeoff_flag)
//     {   takeoff_start();
//         takeoff_flag = 1;
//         //copter.set_auto_armed();
//     }
//     else
//     {
//         auto_takeoff_run();   
//         gcs().send_text(MAV_SEVERITY_INFO, "bad idea");
//         if(auto_takeoff_complete)
//         {
//             draw5star_flag = 0;
//             set_submode(SubMode::DRAW5STAR);
//             gcs().send_text(MAV_SEVERITY_INFO, "takeoff finished, now go into draw5star submode");
//         }
//     }
// }

//————画五角星初始化
void ModeDrawStar::draw5star_start()
{
    path_num = 0;   //航点号清0，从别的模式切到此模式时，可以飞出一个新的五角星
    generate_path();    //生成五角星航线

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);   
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

//————开始画五角星
void ModeDrawStar::draw5star_run()
{
    if (!draw5star_flag)
    {
        draw5star_start();
        draw5star_flag = 1;
    }
    else
    {
        if (path_num < 6) 
        {  // 五角星航线尚未走完
            if (wp_nav->reached_wp_destination()) 
            {  // 到达某个端点
                path_num++;
                wp_nav->set_wp_destination(path[path_num], false);  // 将下一个航点位置设置为导航控制模块的目标位置
            }
        } else if ((path_num == 6) && wp_nav->reached_wp_destination()) 
        {  // 五角星航线运行完成，自动进入Loiter模式
            gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, wait 5 seconds");
            //copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
            draw5star_flag = 0;
            set_submode(SubMode::LAND);
            land_start_time = millis();
        }

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
    
    
}

//————开始降落
void ModeDrawStar::land_run()
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
            gcs().send_text(MAV_SEVERITY_INFO, "now go into land submode");

        }

        // 进行正常的下降（10m之上1.5m/s；10米以内0.5m/s）
        land_run_normal_or_precland(land_pause);
    }
}



#endif

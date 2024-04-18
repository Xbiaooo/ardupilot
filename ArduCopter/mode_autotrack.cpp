#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * 五角星航线模式初始化
 */


//模式初始化
bool ModeAutoTrack::init(bool ignore_checks)
{
    _mode = SubMode::TAKEOFF;

    wp_nav->wp_and_spline_init();
    
    //takeoff_flag = 0;
    cruise_flag = 0;
    land_pause = true;

    takeoff_finish = true;
     
    //_mode = SubMode::DRAW5STAR;

    return true;
}

//此模式的周期调用
void ModeAutoTrack::run()
{
    switch (_mode)
    {
    case SubMode::TAKEOFF:
        takeoff_run();
        break;

    case SubMode::AB_cruise:
        cruise_run();
        break;

    case SubMode::LAND:
        land_run();
        break;
    }


}

//————切换子模式
void ModeAutoTrack::set_submode(SubMode new_submode)
{
    // return immediately if the submode has not been changed
    if (new_submode == _mode) {
        return;
    }

    // set mode
    _mode = new_submode;
    
}

//————开始起飞
void ModeAutoTrack::takeoff_run()
{   
    copter.set_auto_armed(true);
    auto_takeoff_start(target_alt_cm, false);
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
        set_submode(SubMode::AB_cruise);
        gcs().send_text(MAV_SEVERITY_INFO, "now go into draw5star submode");
    }
    
    

}

//————巡航初始化
void ModeAutoTrack::cruise_start()
{
    generate_point();    //生成A、B两点坐标
    cruise_count = 1;

    point_pause = true;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
    //wp_nav->set_wp_destination(point_B, false);
    //target_point = Point::B;
    set_cruise_point(Point::B,0,false);

    // initialise yaw
    //auto_yaw.set_mode_to_default(false);   
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

//生成巡航中的A、B两点坐标
void ModeAutoTrack::generate_point()
{
    float radius_cm = g2.AB_distance;

    wp_nav->get_wp_stopping_point(point_A);

    point_B = point_A + Vector3f(1.0f, 0, 0) * radius_cm;
}

//设置下一个航点
// void ModeAutoTrack::set_cruise_point(Point next_point, uint16_t time_ms = 2000, bool ready_set_next_cruise_point = false)
void ModeAutoTrack::set_cruise_point(Point next_point, uint16_t time_ms, bool need_delay)
{
    if (!need_delay) //如果不需要延时(起始时从A点出发不需要延时)
    {
        if (next_point == Point::A)
        {
            wp_nav->set_wp_destination(point_A, false);
            target_point = Point::A;
        }
        else if (next_point == Point::B)
        {
            wp_nav->set_wp_destination(point_B, false);
            target_point = Point::B;
            //cruise_count++;
        }
    }
    else if (need_delay && millis() - point_reach_time >= time_ms)
    {
        if (next_point == Point::A)
        {
            wp_nav->set_wp_destination(point_A, false);
            target_point = Point::A;
        }
        else if (next_point == Point::B)
        {
            wp_nav->set_wp_destination(point_B, false);
            target_point = Point::B;
            cruise_count++;
        }
        point_pause = true;
    }   
    
}    

// }

//————开始巡航
void ModeAutoTrack::cruise_run()
{
    if (!cruise_flag)
    {
        cruise_start();
        cruise_flag = 1;
    }
    else
    {
        	gcs().send_text(MAV_SEVERITY_INFO, 
                "count: %d",
                 cruise_count);

        if (cruise_count < cruise_sum)
        {
            if (target_point == Point::B)
            {
                if (wp_nav->reached_wp_destination())
                {
                    //wp_nav->set_wp_destination(point_A, false);
                    if(point_pause)
                    {
                        //gcs().send_text(MAV_SEVERITY_INFO,  "wait 5s");
                        point_reach_time = millis();
                        point_pause = false;
                    }
                    set_cruise_point(Point::A);
                }                
            }
            else if (target_point == Point::A)
            {
                if (wp_nav->reached_wp_destination())
                {
                    //wp_nav->set_wp_destination(point_B, false);
                    if(point_pause)
                    {
                        point_reach_time = millis();
                        point_pause = false;
                    }
                    set_cruise_point(Point::B);
                    //cruise_count++;
                } 
            }                       
        }
        else if (cruise_count == cruise_sum)
        {
            if (target_point == Point::B)
            {
                if (wp_nav->reached_wp_destination())
                {
                    //wp_nav->set_wp_destination(point_A, false);
                    if(point_pause)
                    {
                        point_reach_time = millis();
                        point_pause = false;
                    }
                    set_cruise_point(Point::A);
                }                
            }
            else if (target_point == Point::A)
            {
                if (wp_nav->reached_wp_destination())
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "Cruise finished, wait 5 seconds, then land start");
                    set_submode(SubMode::LAND);
                    cruise_flag = 0;
                    cruise_finish_time = millis();
                }                 
            }
                
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
void ModeAutoTrack::land_run()
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
        if (land_pause && millis()-cruise_finish_time >= 5000) {
            land_pause = false;
            gcs().send_text(MAV_SEVERITY_INFO, "start landing");

        }

        // 进行正常的下降（10m之上1.5m/s；10米以内0.5m/s）
        land_run_normal_or_precland(land_pause);
    }
}



#endif

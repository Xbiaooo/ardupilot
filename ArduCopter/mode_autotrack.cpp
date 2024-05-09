#include "Copter.h"

#if MODE_AUTOTRACK_ENABLED == ENABLED


//模式初始化
bool ModeAutoTrack::init(bool ignore_checks)
{
    _mode = SubMode::TAKEOFF;

    wp_nav->wp_and_spline_init();
    
    cruise_flag = 0;
    //track_flag = 0;

    land_pause = true;

    takeoff_finish = true;

    return true;
}

//此模式的周期调用
void ModeAutoTrack::run()
{
    static bool track_flag = 0;
    switch (_mode)
    {
    case SubMode::TAKEOFF:
        takeoff_run();
        break;

    case SubMode::AB_CRUISE:
        static int sim_time = millis();
        if (millis() - sim_time >= 10000)
        {
            track_flag = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "error");
        }
        if (!track_flag)
        {
            cruise_run();
        }
        else
        {
            //cruise_run();
            track_run();
        }  
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

////////////////////////////////////////////////////////////
// takeoff 函数
// ————开始起飞
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
        set_submode(SubMode::AB_CRUISE);
        gcs().send_text(MAV_SEVERITY_INFO, "now go into draw5star submode");
    }
    
    

}

////////////////////////////////////////////////////////////
// cruise 函数
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

//设置下一个巡航航点
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
        	// gcs().send_text(MAV_SEVERITY_INFO, 
            //     "count: %d",
            //      cruise_count);

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

        pos_control_run();
        
    }
    
    
}

//位置控制器，由cruise和track共用
void ModeAutoTrack::pos_control_run()
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

////////////////////////////////////////////////////////////
// track 函数
//————开始跟踪
void ModeAutoTrack::track_run()
{
    gcs().send_text(MAV_SEVERITY_INFO, "why????");

    //simulation
    static bool sim_openmv_new_data = false;
    static uint32_t last_sim_new_data_time_ms = millis();
    if (millis() - last_sim_new_data_time_ms < 5000)
    {
        openmv.cx = 1;
        openmv.cy = 1;
        sim_openmv_new_data = true;
    }
    else if (millis() - last_sim_new_data_time_ms < 10000)
    {
        openmv.cx = 120;
        openmv.cy = 80;
        sim_openmv_new_data = true;
    }
    else if (millis() - last_sim_new_data_time_ms < 15000)
    {
        openmv.cx = 80;
        openmv.cy = 60;
        sim_openmv_new_data = true;
    }
    else if (millis() - last_sim_new_data_time_ms < 20000)
    {
        openmv.cx = 60;
        openmv.cy = 90;
        sim_openmv_new_data = true;
    }
    else
    {
        sim_openmv_new_data = false;
    }
    
    
    
    

    static uint32_t last_set_pos_target_time_ms = 0;

    // if(openmv.update() || sim_openmv_new_data)
    if(sim_openmv_new_data)
    {       
        //将像素坐标系转换为图像坐标系
        //像素坐标系-->图像坐标系
        int16_t image_frame_x = (int16_t)openmv.cx - 80;
        int16_t image_frame_y = (int16_t)openmv.cy - 60;

        //图像坐标系(2D)-->相机坐标系(3D)
        //相机坐标系：以相机光心为原点，前为z轴、右为x轴、下为y轴
        //由于openmv固定在无人机正下方，且跟踪时无人机高度不变，即机体坐标系中z轴不变，
            //所以只需将图像坐标系转化为相机坐标系中的x轴和y轴即可
        //利用相似三角形计算目标中心点在相机坐标系中相对原点在x轴和y轴的偏移
        float altitude =  copter.flightmode->get_alt_above_ground_cm() * 10.0f; //altitude实际上即为目标在相机坐标系中的z轴坐标
        float camera_frame_x = image_frame_x * openmv.px_length * altitude / openmv.focal_length;
        float camera_frame_y = image_frame_y * openmv.px_length * altitude / openmv.focal_length;

        Vector3f v = Vector3f(camera_frame_x, camera_frame_y, altitude);

        //相机坐标系-->机体坐标系
        const Matrix3f rotMat1 = Matrix3f(Vector3f(0, -1, 0), //旋转矩阵
                                          Vector3f(1, 0, 0),
                                          Vector3f(0, 0, 0)); //同时将z轴坐标变为0
        v = rotMat1 * v;         

        //机体坐标系-->NED坐标系  
        const Matrix3f &rotMat2 = copter.ahrs.get_rotation_body_to_ned();        
        v = rotMat2 * v;

        //NED坐标系-->NEU坐标系
        v.z = -v.z; //但由于z轴坐标为0，所以此处并无变化
        v = v*100.0f;

        //获取机体当前坐标(相对于EKF原点)
        Vector3f current_pos = inertial_nav.get_position_neu_cm();

        //目标位置坐标(相对于EKF原点)
        target = current_pos + v;
        //首次更新目标位置
        if (last_set_pos_target_time_ms == 0)
        {
            wp_nav->set_wp_destination(target, false);
            last_set_pos_target_time_ms= millis();
        }   

        if(millis() - last_set_pos_target_time_ms > 500) // call in 2Hz
        {  
            wp_nav->set_wp_destination(target, false);
            last_set_pos_target_time_ms= millis();
        }
    }
    else
    {
         wp_nav->set_wp_destination(target, false);
         //视野中失去目标5s后，原地降落
         if (millis() - last_set_pos_target_time_ms > 5000)
         {
            set_submode(SubMode::LAND);
            cruise_finish_time = millis();
         }
         
    }
    pos_control_run();

//————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

    //     if (!cruise_flag)
    // {
    //     cruise_start();
    //     cruise_flag = 1;
    // }
    // else
    // {
    //     	// gcs().send_text(MAV_SEVERITY_INFO, 
    //         //     "count: %d",
    //         //      cruise_count);

    //     if (cruise_count < cruise_sum)
    //     {
    //         if (target_point == Point::B)
    //         {
    //             if (wp_nav->reached_wp_destination())
    //             {
    //                 //wp_nav->set_wp_destination(point_A, false);
    //                 if(point_pause)
    //                 {
    //                     //gcs().send_text(MAV_SEVERITY_INFO,  "wait 5s");
    //                     point_reach_time = millis();
    //                     point_pause = false;
    //                 }
    //                 set_cruise_point(Point::A);
    //             }                
    //         }
    //         else if (target_point == Point::A)
    //         {
    //             if (wp_nav->reached_wp_destination())
    //             {
    //                 //wp_nav->set_wp_destination(point_B, false);
    //                 if(point_pause)
    //                 {
    //                     point_reach_time = millis();
    //                     point_pause = false;
    //                 }
    //                 set_cruise_point(Point::B);
    //                 //cruise_count++;
    //             } 
    //         }                       
    //     }
    //     else if (cruise_count == cruise_sum)
    //     {
    //         if (target_point == Point::B)
    //         {
    //             if (wp_nav->reached_wp_destination())
    //             {
    //                 //wp_nav->set_wp_destination(point_A, false);
    //                 if(point_pause)
    //                 {
    //                     point_reach_time = millis();
    //                     point_pause = false;
    //                 }
    //                 set_cruise_point(Point::A);
    //             }                
    //         }
    //         else if (target_point == Point::A)
    //         {
    //             if (wp_nav->reached_wp_destination())
    //             {
    //                 gcs().send_text(MAV_SEVERITY_INFO, "Cruise finished, wait 5 seconds, then land start");
    //                 set_submode(SubMode::LAND);
    //                 cruise_flag = 0;
    //                 cruise_finish_time = millis();
    //             }                 
    //         }
                
    //     }

    //     pos_control_run();
        
    // }
    
}

////////////////////////////////////////////////////////////
// land 函数
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

#include <bits/stdc++.h>
#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <time.h>

krpc::services::SpaceCenter::Vessel rocket;
time_t ignition_time; // 点火瞬间的时间戳

signed int main()
{
    // connect to local server and KSC
    auto conn = krpc::connect("", "127.0.0.1", 50002, 50003);
    krpc::services::KRPC krpc(&conn);
    krpc::services::SpaceCenter space_center(&conn);
    std::cout << "[INFO] Spacecenter connected" << std::endl;

    // connect to the vehicle
    rocket = space_center.active_vessel();
    std::cout << "[INFO] Rocket system online" << std::endl;

    // Telemetry data stream setup
    auto altitude  = rocket.flight().mean_altitude_stream();
    auto apoapsis  = rocket.orbit().apoapsis_altitude_stream();  // 遥测信号数据流：远地点高度
    auto periapsis = rocket.orbit().periapsis_altitude_stream(); // 遥测信号数据流：近地点高度
    std::cout << "[INFO] Telemetry system initialized." << std::endl;

    // Fuel amount setup
    auto booster_resources  = rocket.resources_in_decouple_stage(2, false);
    auto booster_UDMH   = booster_resources.amount_stream("UDMH");   // 助推器偏二甲肼余量遥测数据流
    
    auto stage1_resources   = rocket.resources_in_decouple_stage(3, false); 
    auto stage1_UDMH    = stage1_resources.amount_stream("UDMH");    // 芯一级偏二甲肼余量遥测数据流

    // 长征三号乙的第 4 分离时序是一级级间段，不含燃料
    
    auto stage2_resources   = rocket.resources_in_decouple_stage(5, false);
    auto stage2_UDMH    = stage2_resources.amount_stream("UDMH");    // 芯二级偏二甲肼余量遥测数据流

    // 长征三号乙的第 6 分离时序是整流罩，不含燃料
    
    // 长征三号乙的芯三级，使用氢氧发动机，燃料种类为液氢液氧
    auto stage3_resources   = rocket.resources_in_decouple_stage(7, false);
    auto stage3_LqdOxygen   = stage3_resources.amount_stream("LqdOxygen");   // 芯三级液氧余量遥测数据流
    auto stage3_LqdHydrogen = stage3_resources.amount_stream("LqdHydrogen"); // 芯三级液氢余量遥测数据流

    std::cout << "[INFO] Fuel tank telemetry signal online." << std::endl;

    // Seperate order initialize
    bool booster_attached = true;
    bool stage1_attached  = true;
    bool stage2_attached  = true;
    bool fairing_attached = true;
    bool stage3_attached  = true;
    
    std::cout << "[ROCKET] Seperate order confirmed." << std::endl;

    // flight system initialize, include:
    // rcs init
    rocket.control().set_rcs(false);
    std::cout << "[INFO] RCS system initialized." << std::endl;
    // sas init
    rocket.control().set_sas(false);
    std::cout << "[INFO] SAS system initialized." << std::endl;
    
    std::cout << "[INFO] Rocket system initialized." << std::endl;
    
    // countdown!
    std::cout << "[EVENT] Countdown for launch" << std::endl;
    std::cout << "[COUNTDOWN] Countdown: 5" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "[COUNTDOWN] Countdown: 4" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "[COUNTDOWN] Countdown: 3" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "[COUNTDOWN] Countdown: 2" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "[COUNTDOWN] Countdown: 1" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "[COUNTDOWN] Ignition sequence start!" << std::endl;
    
    // Ignition procedure
    std::cout << "[ROCKET] Ignition!" << std::endl;
    double org_altitude = rocket.flight().mean_altitude();
    rocket.control().set_throttle(1.0);
    rocket.control().activate_next_stage();
    rocket.auto_pilot().engage();
    std::cout << "[ROCKET] Autopilot Engage!" << std::endl;
    
    // Flight time calculate
    ignition_time = time(0);
    tm *ltm = localtime(&ignition_time);
    printf("T0: %d-%d-%d %d:%d:%d\n", 1900+ltm->tm_year, ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

    std::this_thread::sleep_for(std::chrono::seconds(4));
    rocket.control().activate_next_stage();

    /* Flight procedure */

    while (true)
    {
        if (altitude() - org_altitude > 1.0)
        {
            std::cout << "[LIFTOFF] We have a liftoff!" << std::endl;
            goto label1;
        }
    }
    label1:;
    
    while(true)
    {
        if (altitude() - org_altitude > org_altitude * 3)
        {
            std::cout << "[LIFTOFF] Tower clear" << std::endl;
            goto label2;
        }
    }
    label2:;

    // Rocket rotation
    rocket.auto_pilot().target_pitch_and_heading(90, 90);

    // Setting gravity turn
    double turn_start_altitude = 500.00;
    double turn_end_altitude   = 70000.00; // yy's result

    // Main loop before into orbit
    double turn_deg = 0;
    while (true)
    {
        // Gravity turn
        if (altitude() > turn_start_altitude && altitude() < turn_end_altitude)
        {
            double fact = (altitude() - turn_start_altitude) / (turn_end_altitude - turn_start_altitude);
            double new_deg = fact * 90.0;

            if (std::abs(new_deg - turn_deg) >= 0.5)
            {
                turn_deg = new_deg;
                rocket.auto_pilot().target_pitch_and_heading(90 - turn_deg, 90);
            }
        }

        // Booster fuel amount check
        if (booster_attached && booster_UDMH() < 0.1)
        {
            booster_attached = false;
            rocket.control().activate_next_stage();
            std::cout << "[ROCKET] Booster Seperation!" << std::endl;
        }

        // Stage 1 fuel amount check
        if (stage1_attached && stage1_UDMH() < 0.1)
        {
            stage1_attached = false;
            rocket.control().activate_next_stage(); // 一级分离
            std::cout << "[ROCKET] Stage Seperation" << std::endl;
            std::cout << "[ROCKET] SES-1" << std::endl;
        }
    }    
}
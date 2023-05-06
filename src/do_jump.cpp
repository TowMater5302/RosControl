#include "pololu_maestro_ros/set_servo.h"
#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#define MICROSECOND 1000000

// JUMP CONFIG
// const int RPM = 6630;
// const int STOP_RPM = 6000;

// const double FORWARD_TIME_SECONDS = 2.0;
// const double AIRTIME_SECONDS = 0.5;
// const double SLOW_TIME_SECONDS = 0.75;
// const int SLOW_TIME_SEGMENTS = 10;

int main(int argc, char** argv) 
{

    //Initialize node
    ros::init(argc, argv, "do_jump");

    //create node handle
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<pololu_maestro_ros::set_servo>("set_servo");

    // get the params
    double RPM; 
    double STOP_RPM;
    double FORWARD_TIME_SECONDS;
    // double AIRTIME_SECONDS;
    // double START_SPEED;
    // double RAMP_UP_TIME;
    // int START_SEGMENTS;

    n.getParam("jump_speed", RPM);
    n.getParam("stop_speed", STOP_RPM);
    n.getParam("forward_time", FORWARD_TIME_SECONDS);
    // n.getParam("air_time", AIRTIME_SECONDS);
    // n.getParam("start_speed", START_SPEED);
    // n.getParam("ramp_up_time", RAMP_UP_TIME);

    // ROS_INFO("Jumping with speed %f", RPM);
    // ROS_INFO("Stopping with speed %f", STOP_RPM);
    // ROS_INFO("Forward time %f", FORWARD_TIME_SECONDS);
    // ROS_INFO("Air time %f", AIRTIME_SECONDS);

    // Go forward call
    pololu_maestro_ros::set_servo srv;

    // start at START_SPEED
    // srv.request.channel = 1;
    // srv.request.target = (int) START_SPEED;
    // client.call(srv);

    // double seg_time = RAMP_UP_TIME / START_SEGMENTS;
    // for (int i = 0; i < START_SEGMENTS; i++)
    // {
    //     usleep(seg_time * MICROSECOND);
    //     srv.request.channel = 1;
    //     srv.request.target = (int) (START_SPEED + (RPM - START_SPEED) * ((double) i / START_SEGMENTS));
    //     client.call(srv);
    // }

    // full speed 
    srv.request.channel = 1;
    srv.request.target = (int) RPM;
    client.call(srv);

    // Wait to speed up seconds
    usleep(FORWARD_TIME_SECONDS * MICROSECOND);
    // Wait for airtime
    // usleep(AIRTIME_SECONDS * MICROSECOND);
    
    // Slow stop loop
    // double RPM_DECREASE = (RPM - STOP_RPM) / SLOW_TIME_SEGMENTS;
    // for (int i =1; i <= SLOW_TIME_SEGMENTS; i++){
    //     pololu_maestro_ros::set_servo srv;
    //     srv.request.channel = 1;
    //     srv.request.target = (int) (RPM - (RPM_DECREASE * i));
    //     client.call(srv);
    //     usleep((SLOW_TIME_SECONDS / SLOW_TIME_SEGMENTS) * MICROSECOND);
    // }

    // just stop power to motors and coast
    // Stopping call
    srv.request.channel = 1;
    srv.request.target = STOP_RPM;
    client.call(srv);

    return 0;
}
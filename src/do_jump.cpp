#include "pololu_maestro_ros/set_servo.h"
#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#define MICROSECOND 1000000

// JUMP CONFIG
const int RPM = 6400;
const int STOP_RPM = 6000;

const double FORWARD_TIME_SECONDS = 0.5;
const double AIRTIME_SECONDS = 0.2;
const double SLOW_TIME_SECONDS = 0.75;
const int SLOW_TIME_SEGMENTS = 10;

int main(int argc, char** argv) {

    //Initialize node
    ros::init(argc, argv, "do_jump");

    //create node handle
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<pololu_maestro_ros::set_servo>("set_servo");

    // // Go forward call
    // pololu_maestro_ros::set_servo srv;
    // srv.request.channel = 1;
    // srv.request.target = (int) RPM;
    // client.call(srv);

    // // Wait to speed up seconds
    // usleep(FORWARD_TIME_SECONDS * MICROSECOND);
    // // Wait for airtime
    // usleep(AIRTIME_SECONDS * MICROSECOND);

    // double RPM_DECREASE = (RPM - STOP_RPM) / SLOW_TIME_SEGMENTS;
    
    // for (int i =1; i <= SLOW_TIME_SEGMENTS; i++){
    //     pololu_maestro_ros::set_servo srv;
    //     srv.request.channel = 1;
    //     srv.request.target = (int) (RPM - (RPM_DECREASE * i));
    //     client.call(srv);
    //     usleep((SLOW_TIME_SECONDS / SLOW_TIME_SEGMENTS) * MICROSECOND);
    // }

    // // Stopping call
    // srv.request.channel = 1;
    // srv.request.target = STOP_RPM;
    // client.call(srv);

    return 0;
}
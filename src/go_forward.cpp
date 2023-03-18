#include "pololu_maestro_ros/set_servo.h"
#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#define MICROSECOND 1000000

int main(int argc, char** argv) {
    //Initialize node
    ros::init(argc, argv, "go_forward");

    //Check if channel and target were passed to node
    if (argc != 3)
    {
        ROS_INFO("usage: go_forward time target");
        return 1;
    }

    //create node handle
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<pololu_maestro_ros::set_servo>("set_servo");

    // Go forward call
    pololu_maestro_ros::set_servo srv;
    srv.request.channel = 1;
    srv.request.target = atoll(argv[2]);
    client.call(srv);

    usleep(atoll(argv[1]) * MICROSECOND);

    // Stopping call
    srv.request.channel = 1;
    srv.request.target = 6000;
    client.call(srv);

    return 0;
}
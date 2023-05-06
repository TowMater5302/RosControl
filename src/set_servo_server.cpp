#include "ros/ros.h"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "pololu_maestro_ros/set_servo.h"

int fd; // file description for the serial port

// bool get(polulu_maestro_ros::get_servo::Request  &req,
//          polulu_maestro_ros::get_servo::Response &res){
//     unsigned char command[] = {0x90, req.channel, req.target & 0x7F, req.target >> 7 & 0x7F};

// }

//callback for service
bool set(pololu_maestro_ros::set_servo::Request  &req,
         pololu_maestro_ros::set_servo::Response &res)
{  
    //command written to pololu maestro
    unsigned char command[] = {0x84, req.channel, req.target & 0x7F, req.target >> 7 & 0x7F};

    ROS_INFO("Trying to set channel %d to %d.", req.channel, req.target);
    
    //ZCheck if the command was written to the device
    if (write(fd, command, sizeof(command)) == -1)
    {
        ROS_INFO("Cannot set servo channel");
        return false;
    } else {
        ROS_INFO("Successfully set servo channel");
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_servo_server");
    ros::NodeHandle n;

    //Serial device (Pololu Maestro)
    // Need to run sudo chmod 0666 /dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_18-Channel_USB_Servo_Controller_00257934-if00 for some reason.
    const char * device = "/dev/ttyACM0"; 
    ROS_INFO(device);
    fd = open(device, O_RDWR | O_NOCTTY);

    //check if the device was opened
    if (fd == -1)
    {
    char buffer[ 256 ];
    char * errorMsg = strerror_r( errno, buffer, 256 );
    ROS_INFO(errorMsg);
    ROS_INFO("Cannot open serial connection to pololu maestro");
    return 1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tcsetattr(fd, TCSANOW, &options);

    //create service server
    ros::ServiceServer service = n.advertiseService("set_servo", set);
    ROS_INFO("Ready to set servo.");
    ros::spin();

    return 0;
}

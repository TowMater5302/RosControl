#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sharp_node");
    ros::NodeHandle nh;

    // Open a serial connection to the Maestro controller
    const char * device = "/dev/ttyACM0";
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        ROS_ERROR("Error opening serial port %s", device);
        return -1;
    }

    // Set the serial port parameters
    struct termios options;
    tcgetattr(fd, &options);
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 100;  // 10 second timeout

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    tcsetattr(fd, TCSANOW, &options);

    // Create a publisher for the input values
    ros::Publisher input_pub = nh.advertise<std_msgs::Int32>("/sharp/distance", 10);

    // Set the loop rate
    ros::Rate loop_rate(250);  // 250 Hz

    while (ros::ok()) {
        // Send the command to read an input value
        int channel = 11;
        unsigned char command[2] = {0x90, channel};
        if(write(fd, command, 2) == -1) {
            ROS_ERROR("Error writing to serial port");
        }

        // Read the response from the controller
        unsigned char response[2];
        int numBytes = read(fd, response, 2);
        if (numBytes != 2) {
            ROS_ERROR("Error reading from serial port");
            char buffer[ 256 ];
            char * errorMsg = strerror_r( errno, buffer, 256 );
            ROS_INFO(errorMsg);
            break;
        }
        int input_value = (response[0] + 256*response[1]);

        // Publish the input value
        std_msgs::Int32 msg;
        msg.data = input_value;
        input_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close the serial connection
    close(fd);

    return 0;
}

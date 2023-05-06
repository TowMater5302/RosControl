#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <signal.h>
#include <pololu_maestro_ros/set_servo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#define DEPTH_SCALE 0.001
#define FOV_H 87.0
#define FOV_V 58.0
#define FPS 30.0

void maestroControl(float left_avg, float right_avg); // Control the pololu maestro servo controller
void steer(int target); // Control the steer servo
void drive(int target); // Control the drive motor esc
void sigintHandler(int sig);
inline double getAbsoluteDiff2Angles(const double x, const double y)
{
    // c can be PI (for radians) or 180.0 (for degrees);
    return M_PI - fabs(fmod(fabs(x - y), 2*M_PI) - M_PI);
}

class WallFollow {
    private:
        // ROS parameters
        std::string turn_direction; // "left" or "right" - when the robot is approaching a turn
        float turn_threshold; // distance at which the robot should start turning
        float alpha; // field of view within which the robot will look for a wall
        int normal_speed; // speed at which the robot will drive when not turning
        int turn_speed; // speed at which the robot will drive when turning
        std::string control_type; // "pid" or "threshold"
        double Kp; // proportional gain
        double Ki; // integral gain
        double Kd; // derivative gain
        bool enable_stop_sign; // whether to look for stop signs
        cv::CascadeClassifier stop_classifier;

        
        float slow_threshold;
        double turn_exit_speed;
        double turn_yaw_threshold;
        double turn_timer;
        double slow_timer;
        double drive_speed;
        double yaw;
        std::string drive_mode;
        double yaw_before_turn;
        float inlet_threshold;

        int stay_in_pid_ct = 10;

        cv_bridge::CvImagePtr cv_ptr; // depth image
        cv::Mat depth_array; // depth image converted to a cv::Mat
        cv::Mat color_array; // color image converted to a cv::Mat
        ros::Subscriber sub1;
        ros::Subscriber sub2;
        ros::Subscriber sub3;
        ros::NodeHandle *nh;
        ros::Timer timer;

    public:
        WallFollow(ros::NodeHandle* nh);
        void depthCallback(const sensor_msgs::Image::ConstPtr& data);
        void colorCallback(const sensor_msgs::Image::ConstPtr& data);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& data);
        void run();
        void handle_pid();
        void handle_slow_down();
        void handle_turn();
        void pidControl(float left_avg, float right_avg);
        void thresholdControl(float left_avg, float right_avg);
        void timerCallback(const ros::TimerEvent& event);
        bool stopSignDetect();
};

WallFollow::WallFollow(ros::NodeHandle* nh) {
    // set private variables using rosparam
    this->nh = nh;
    this->drive_mode = "PID";

    ROS_INFO("Inside the constructor");
    nh->getParam("turn_direction", this->turn_direction);
    ROS_INFO("Set turn direction: %s", this->turn_direction.c_str());

    nh->getParam("control_type", this->control_type);
    ROS_INFO("Set control type: %s", this->control_type.c_str());

    nh->getParam("turn_threshold", this->turn_threshold);
    ROS_INFO("Set turn threshold: %f", this->turn_threshold);

    nh->getParam("slow_threshold", this->slow_threshold);
    ROS_INFO("Set slow threshold: %f", this->slow_threshold);

    nh->getParam("turn_yaw_threshold", this->turn_yaw_threshold);
    ROS_INFO("Set turn yaw threshold: %f", this->turn_yaw_threshold);

    nh->getParam("alpha", this->alpha);
    ROS_INFO("Set alpha: %f", this->alpha);

    nh->getParam("inlet_threshold", this->inlet_threshold);
    ROS_INFO("Set inlet threshold: %f", this->inlet_threshold);

    nh->getParam("normal_speed", this->normal_speed);
    ROS_INFO("Set normal speed: %d", this->normal_speed);

    nh->getParam("turn_speed", this->turn_speed);
    ROS_INFO("Set turn speed: %d", this->turn_speed);

    nh->getParam("turn_exit_speed", this->turn_exit_speed);
    ROS_INFO("Set turn exit speed: %f", this->turn_exit_speed);

    nh->getParam("turn_timer", this->turn_timer);
    ROS_INFO("Set turn timer: %f", this->turn_timer);

    nh->getParam("slow_timer", this->slow_timer);
    ROS_INFO("Set slow timer: %f", this->slow_timer);

    nh->getParam("pid/kp", this->Kp);
    ROS_INFO("Set Kp: %f", this->Kp);

    nh->getParam("pid/ki", this->Ki);
    ROS_INFO("Set Ki: %f", this->Ki);

    nh->getParam("pid/kd", this->Kd);
    ROS_INFO("Set Kd: %f", this->Kd);

    this->drive_speed = this->normal_speed;
    // this->timer = this->nh->createTimer(ros::Duration(this->slow_timer), &WallFollow::timerCallback, this);
    this->sub1 = nh->subscribe("/camera/depth/image_rect_raw", 1, &WallFollow::depthCallback, this);
    // subscribe to imu data
    this->sub2 = nh->subscribe("/imu/data", 1, &WallFollow::imuCallback, this);

    nh->getParam("enable_stop_sign", this->enable_stop_sign);
    ROS_INFO("Set enable stop sign: %d", this->enable_stop_sign);

    if (this->enable_stop_sign){
        this->sub3 = nh->subscribe("/camera/color/image_raw", 1, &WallFollow::colorCallback, this);

        if (!this->stop_classifier.load("/home/odroid/Desktop/towmater_ws/src/RosControl/data/cascade.xml"))
        {
            ROS_ERROR("Error loading cascade classifier XML file.");
        }
    }
    drive(this->drive_speed);
}

void WallFollow::colorCallback(const sensor_msgs::Image::ConstPtr& data){
    // Convert the image to a cv::Mat
    try {
        this->color_array = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void WallFollow::depthCallback(const sensor_msgs::Image::ConstPtr& data) {
    // Convert the depth image to a cv::Mat
    try {
        this->cv_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    this->depth_array = this->cv_ptr->image;
    this->depth_array.convertTo(this->depth_array, CV_32FC1);
    this->depth_array *= DEPTH_SCALE;

    // interpolate the depth data to fill in the missing values
    // TODO - this is a hack, need to find a better way to interpolate
    cv::medianBlur(this->depth_array, this->depth_array, 5);
    // call run() to process the depth image
    run();
}

bool WallFollow::stopSignDetect() {
    
    if (this->color_array.empty()){
        return false;
    }

    cv::Mat frame_gray;
    cvtColor(this->color_array, frame_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Rect> stop_signs;
    this->stop_classifier.detectMultiScale( frame_gray, stop_signs );

    // Define the lower and upper bounds of the red color range
    cv::Scalar lower_red(0, 0, 100); // BGR format
    cv::Scalar upper_red(50, 50, 255); // BGR format
    cv::Scalar red_color(0, 0, 255);
    cv::Scalar black_color(255, 255, 255);
    
    for (cv::Rect r : stop_signs) {
        cv::Mat section = this->color_array(r);

        // Apply the color mask to the ROI
        cv::Mat mask;
        cv::inRange(section, lower_red, upper_red, mask);
        cv::Mat detected_output;
        cv::bitwise_and(section, section, detected_output, mask);

        // Convert the masked image to grayscale
        cv::Mat mask_gray;
        cv::cvtColor(detected_output, mask_gray, cv::COLOR_BGR2GRAY);

        int total_pixels = mask_gray.rows * mask_gray.cols;
        int red_pixels = cv::countNonZero(mask_gray);
        double percentage_red = static_cast<double>(red_pixels) / total_pixels;

        if (percentage_red > .1){
            // Draw the rectangle on the image
            cv::rectangle(this->color_array, r, red_color, 2);
        } else {
            cv::rectangle(this->color_array, r, black_color, 2);
        }
    }

    cv::imshow( "RealSense with stop signs", this->color_array);

    return stop_signs.size() > 0;
}

void WallFollow::imuCallback(const sensor_msgs::Imu::ConstPtr& data) {
    this->yaw = tf::getYaw(data->orientation);
}

void WallFollow::run() {
    if (this->depth_array.empty()) {
        ROS_INFO("run: the depth array is empty");
        return;
    }

    if (this->enable_stop_sign && this->stopSignDetect()){
        ROS_INFO("stop sign found!");
        return;
    } else if(this->enable_stop_sign) {
        ROS_INFO("no stop sign");
        return;
    }

    if (this->drive_mode == "PID"){
        this->handle_pid();
    }
    else if (this->drive_mode == "SLOW_DOWN"){
        this->handle_slow_down();
    } else if (this->drive_mode == "TURN"){
        this->handle_turn();
    }
}

void WallFollow::handle_pid(){
    // get average depth at the center of image
    static int center_h = this->depth_array.cols / 2;
    static int center_v = this->depth_array.rows * (2.0/3.0);
    // static float d = center_h * std::tan(this->alpha * M_PI / 180.0) / std::tan(FOV_H * M_PI / 180.0);
    static int d = std::floor(center_h * std::tan(this->alpha * M_PI / 360.0) / std::tan(FOV_H * M_PI / 360.0));

    cv::Mat center_wall = this->depth_array(cv::Range(0, center_v), cv::Range(center_h - d, center_h + d));
    cv::Mat left_wall = this->depth_array(cv::Range(0, center_v), cv::Range(0, center_h - d));
    cv::Mat right_wall = this->depth_array(cv::Range(0, center_v), cv::Range(center_h + d, depth_array.cols));

    float center_avg = cv::mean(center_wall)[0];
    float left_avg = cv::mean(left_wall)[0];
    float right_avg = cv::mean(right_wall)[0];

    // To avoid inlets in walls, clip the right and left averages if they are too far away
    right_avg = std::min(this->inlet_threshold, right_avg);
    left_avg = std::min(this->inlet_threshold, left_avg);


    ROS_INFO("[PID] Center wall: %f", center_avg);
    

    // We have been in pid mode long enough and can now look to see if we should exit
    if (this->stay_in_pid_ct == 0){
        // Decide if we should leave "PID" mode and go to "SLOW_DOWN" mode
        if (center_avg < this->slow_threshold || center_avg < this->turn_threshold) {
            ROS_INFO("(Mode Change) PID -> SLOW_DOWN");
            drive(this->turn_speed);
            this->drive_mode = "SLOW_DOWN";
            // if(this->turn_direction == "left") {
            //     steer(4800);
            // } else {
            //     steer(7200);
            // }
        }
    }
    else{
        this->stay_in_pid_ct --;
    }

    // handle control to the car in order to wall follow
    if(this->control_type == "pid") {
        pidControl(left_avg, right_avg);
    } else {
        thresholdControl(left_avg, right_avg);
    }
}


// detect when to start to turn
void WallFollow::handle_slow_down(){
    // get average depth at the center of image
    static int center_h = this->depth_array.cols / 2;
    static int center_v = this->depth_array.rows * (2.0/3.0);
    static int d = std::floor(center_h * std::tan(this->alpha * M_PI / 360.0) / std::tan(FOV_H * M_PI / 360.0));

    cv::Mat center_wall = this->depth_array(cv::Range(0, center_v), cv::Range(center_h - d, center_h + d));
    float center_avg = cv::mean(center_wall)[0];

    ROS_INFO("[SLOW] Center wall: %f", center_avg);

    if (center_avg < this->turn_threshold) {
        this->drive_mode = "TURN";
        ROS_INFO("(Mode Change) SLOW_DOWN -> TURN");
        this->yaw_before_turn = this->yaw;
        // if turn direction is left
        if (this->turn_direction == "left") {
            steer(4000);
        }
        else {
            steer(8000);
        }
    }
    return;
}

void WallFollow::handle_turn(){
    double diff = getAbsoluteDiff2Angles(this->yaw, this->yaw_before_turn);
    ROS_INFO("[TURN] (yaw prev: %f, yaw cur: %f, diff: %f)", this->yaw_before_turn, this->yaw, diff);

    if (diff > this->turn_yaw_threshold) {
        ROS_INFO("(Mode Change) TURN -> PID");
        this->drive_mode = "PID";
        this->drive_speed = this->turn_exit_speed;
        this->stay_in_pid_ct = 10;
        drive(this->drive_speed);
    }
}


void WallFollow::timerCallback(const ros::TimerEvent& event) {
    ROS_INFO("Inside the timer callback");
    this->drive_speed = this->normal_speed;
}

void WallFollow::pidControl(float left_avg, float right_avg) {
    static double prev_error = 0;
    static double integral = 0;

    // error would be the difference between the left and right averages
    double error = left_avg - right_avg;
    double dt = 1.0 / FPS;

    // Proportional term
    double Pout = this->Kp * error;
    
    // Integral term
    integral += (error * dt);
    double Iout = this->Ki * error;

    // Derivative term
    double derivative = (error - prev_error) / dt;
    double Dout = this->Kd * derivative;

    double control = Pout + Iout + Dout + 1500;
    prev_error = error;

    ROS_INFO("[PID] WallFollow: turn value = %f", control);
    steer(4*static_cast<int>(control));
}

void WallFollow::thresholdControl(float left_avg, float right_avg) {
    // if left_avg is greater than right_avg, turn left
    double threshold = 0.5;
    if (left_avg > right_avg + threshold)
    {
        // ROS_INFO("turn left");
        steer(1250*4);
    }
    else if (right_avg > left_avg + threshold)
    {
        // ROS_INFO("turn right");
        steer(1750*4);
    }
    else
    {
        // ROS_INFO("go straight");
        steer(1500*4);
    }
}

void maestroControl(int channel, int target) {
    static ros::NodeHandle* nh = new ros::NodeHandle();
    ros::ServiceClient client = nh->serviceClient<pololu_maestro_ros::set_servo>("set_servo");
    pololu_maestro_ros::set_servo srv;
    srv.request.channel = channel;
    srv.request.target = target;
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service set_servo");
    }
}

void steer(int target) {
    // target is the value to which the servo should be set
    // 4000 is the minimum value, 8000 is the maximum value, 6000 is the center
    maestroControl(0, target);
}

void drive(int target) {
    // target is the value to which the esc should be set
    // 4000 is the minimum value (backward), 8000 is the maximum value (forward), 6000 is stop
    maestroControl(1, target);
}

void sigintHandler(int sig) {
    drive(6000);
    steer(6000);
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follow");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
    WallFollow wall_follow(&nh);
    ros::spin();
    return 0;
}

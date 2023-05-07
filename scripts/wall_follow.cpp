#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <signal.h>
#include <pololu_maestro_ros/set_servo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#define DEPTH_SCALE 0.001
#define FOV_H 87.0
#define FOV_V 58.0
#define FPS 30.0
#define MICROSECOND 1000000

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
        int turn_exit_speed;
        double Kp; // proportional gain
        double Ki; // integral gain
        double Kd; // derivative gain

        int turn_angle;
        const int TURN_ANGLE_STRAIGHT = 6000;

        float slow_threshold;
        double turn_yaw_threshold;
        double drive_speed;
        double yaw;
        std::string drive_mode;
        double yaw_before_turn;
        float inlet_threshold;
        float a;
        int ir_dist;
        int ir_dist_avg;

        int stay_in_pid_ct = 10;

        cv_bridge::CvImagePtr cv_ptr; // depth image
        cv::Mat depth_array; // depth image converted to a cv::Mat
        cv::Mat color_array; // color image converted to a cv::Mat
        ros::Subscriber sub1, sub2 ,sub3;
        ros::NodeHandle *nh;
        ros::Timer timer;

    public:
        WallFollow(ros::NodeHandle* nh);
        void depthCallback(const sensor_msgs::Image::ConstPtr& data);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& data);
        void irCallback(const std_msgs::Int32::ConstPtr& data);
        void run();
        void handle_pid();
        void handle_slow_down();
        void handle_turn();
        void pidControl(float left_avg, float right_avg);
};

WallFollow::WallFollow(ros::NodeHandle* nh) {
    // set private variables using rosparam
    this->nh = nh;
    this->drive_mode = "PID";

    ROS_INFO("Inside the constructor");
    nh->getParam("turn_direction", this->turn_direction);
    ROS_INFO("Set turn direction: %s", this->turn_direction.c_str());

    nh->getParam("turn_angle", this->turn_angle);
    ROS_INFO("Set turn angle: %i", this->turn_angle);

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

    nh->getParam("pid/kp", this->Kp);
    ROS_INFO("Set Kp: %f", this->Kp);

    nh->getParam("pid/ki", this->Ki);
    ROS_INFO("Set Ki: %f", this->Ki);

    nh->getParam("pid/kd", this->Kd);
    ROS_INFO("Set Kd: %f", this->Kd);

    nh->getParam("run_avg_alpha", this->a);
    ROS_INFO("Set a: %f", this->a);

    this->sub1 = nh->subscribe("/camera/depth/image_rect_raw", 1, &WallFollow::depthCallback, this);
    // subscribe to imu data
    this->sub2 = nh->subscribe("/imu/data", 1, &WallFollow::imuCallback, this);
    // subscribe to ir sensor data
    this->sub3 = nh->subscribe("/sharp/distance", 1, &WallFollow::irCallback, this);

    this->drive_speed = this->normal_speed;
    drive(this->drive_speed);
}

void WallFollow::irCallback(const std_msgs::Int32::ConstPtr& data){

    this->ir_dist = data->data;
    // get running average
    static int ir_dist_avg = ir_dist;
    ir_dist_avg = this->a*this->ir_dist + (1-this->a)*this->ir_dist_avg; 

    // if drive mode is PID and approaching wall then switch to slow down mode
    // NOTE: The ir sensor measurements increase as distance decreases
    if(this->drive_mode == "PID" && this->ir_dist_avg > this->slow_threshold) {
        ROS_INFO("[PID] The IR Callback dist %f", this->ir_dist_avg);
        ROS_INFO("(Mode Change) %s -> SLOW_DOWN", this->drive_mode.c_str());
        drive(this->turn_speed);
        this->drive_mode = "SLOW_DOWN"; 
        return;
    }
    // if drive mode is slow down 
    else if (this->drive_mode == "SLOW_DOWN") {
        ROS_INFO("[SLOW_DOWN] The IR Callback dist %f", this->ir_dist_avg);
        // approaching turn
        if (this->ir_dist_avg > turn_threshold) {
            this->drive_mode = "TURN";
            ROS_INFO("(Mode Change) SLOW_DOWN -> TURN");
            this->yaw_before_turn = this->yaw;
            // if turn direction is left
            if (this->turn_direction == "left") {
                //subtract angle for left turn
                steer(this->TURN_ANGLE_STRAIGHT - this->turn_angle);
            }
            else {
                // add angle for right turn
                steer(this->TURN_ANGLE_STRAIGHT + this->turn_angle);
            }
            // ********* //
            // delete this when ESC is in F/R mode, keep in F/B mode. Might drift in F/B mode.
            ros::Duration(0.4).sleep();
            this->drive_speed = this->turn_exit_speed;
            drive(this->drive_speed);
            // ********* //
        } 
        // if drive mode is slow down but was a false measurement then switch back to pid
        else if (this->ir_dist_avg < this->slow_threshold) {
            ROS_INFO("(Mode Change) SLOW_DOWN -> PID");
            this->drive_speed = this->normal_speed;
            drive(this->drive_speed);
            this->drive_mode = "PID";
        }
        return;
    }
    ROS_INFO("The IR Callback dist %f", this->ir_dist_avg);
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
    run();
}

void WallFollow::imuCallback(const sensor_msgs::Imu::ConstPtr& data) {
    this->yaw = tf::getYaw(data->orientation);
}

void WallFollow::run() {
    if (this->depth_array.empty()) {
        ROS_INFO("run: the depth array is empty");
        return;
    }

    if (this->drive_mode == "PID" || this->drive_mode == "SLOW_DOWN"){
        this->handle_pid();
    } else if (this->drive_mode == "TURN"){
        this->handle_turn();
    } 
}

void WallFollow::handle_pid(){
    // get average depth at the center of image
    static int center_h = this->depth_array.cols / 2;
    static int center_v = this->depth_array.rows * (2.0/3.0);
    static int d = std::floor(center_h * std::tan(this->alpha * M_PI / 360.0) / std::tan(FOV_H * M_PI / 360.0));

    cv::Mat left_wall = this->depth_array(cv::Range(0, center_v), cv::Range(0, center_h - d));
    cv::Mat right_wall = this->depth_array(cv::Range(0, center_v), cv::Range(center_h + d, depth_array.cols));

    float left_avg = cv::mean(left_wall)[0];
    float right_avg = cv::mean(right_wall)[0];

    // To avoid inlets in walls, clip the right and left averages if they are too far away
    right_avg = std::min(this->inlet_threshold, right_avg);
    left_avg = std::min(this->inlet_threshold, left_avg);

    // handle control to the car in order to wall follow
    pidControl(left_avg, right_avg);
}

void WallFollow::handle_turn(){
    double diff = getAbsoluteDiff2Angles(this->yaw, this->yaw_before_turn);
    ROS_INFO("[TURN] (yaw prev: %f, yaw cur: %f, diff: %f)", this->yaw_before_turn, this->yaw, diff);

    if (diff > this->turn_yaw_threshold) {
        ROS_INFO("(Mode Change) TURN -> PID");
        this->drive_mode = "PID";
        this->drive_speed = this->normal_speed;
        drive(this->drive_speed);
    }
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

    ROS_INFO("[%s] WallFollow: turn value = %f", this->drive_mode.c_str(), control);
    steer(4*static_cast<int>(control));
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
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    // ros::spin();
    return 0;
}
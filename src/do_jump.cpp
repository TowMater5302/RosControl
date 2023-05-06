#include "pololu_maestro_ros/set_servo.h"
#include "ros/ros.h"
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#define MICROSECOND 1000000

class Jump
{
    private:
        ros::NodeHandle *m_node_handle;
        ros::ServiceClient m_client;

        // PARAMETERS - from config file or command line
        double m_ramp_base;
        double m_ramp_height;
        std::string m_ramp_units;

        double m_jump_distance;     // meters
        double m_speed_up_distance; // meters

        int m_stop_speed;
        int m_slow_speed;  // speed & time to go after landing
        double m_slow_time;

        // CALCULATED
        double m_ramp_diagonal;  // meters
        double m_ramp_angle;     // radians

        // Constants
        const double GRAVITY = 9.81;   // meters / sec^2
        const double CAR_LENGTH = 0.2; // meters

        struct Result 
        {
            double rpm;
            double jump_time;
            double speed;
        };

        double projectile_speed(double distance)
        {
            double velocity = std::sqrt((distance * GRAVITY) / std::sin(2 * m_ramp_angle));
            return velocity;
        }

        double calculate_rpm(double v)
        {
            double r = 4.21 * 0.03048 / 2;
            double omega = v / r;
            double rpm = (omega * 60) / (2 * M_PI);
            rpm = rpm + 6000;
            return rpm;
        }

        double jump_time(double v)
        {
            double t_max = v * std::sin(m_ramp_angle) / GRAVITY; 
            double t_total = 2 * t_max; 
            return t_total;
        }

        Result calculate_speed()
        {
            // landing meter (m)
            double half_ramp = m_ramp_base / 2.0;
            double car = 0.2;
            
            double d = m_jump_distance + half_ramp + car;

            double v = projectile_speed(d);
            double t = jump_time(v);
            double rpm = calculate_rpm(v);
            
            return {rpm, t, v};
        }

    public:

        Jump(ros::NodeHandle *node_handle)
        {
            m_node_handle = node_handle;
            // connect to servo for motor control
            m_client = m_node_handle->serviceClient<pololu_maestro_ros::set_servo>("set_servo");

            std::string param_name; 

            if (searchParam("ramp_base", param_name))
            {
                getParam(param_name, m_ramp_base);
            }
            else
            {
                ROS_WARN("Parameter 'ramp_base' not defined");
            }

            if (searchParam("ramp_height", param_name))
            {
                getParam(param_name, m_ramp_height);
            }
            else
            {
                ROS_WARN("Parameter 'ramp_height' not defined");
            }

            if (searchParam("ramp_units", param_name))
            {
                getParam(param_name, m_ramp_units);
            }
            else
            {
                ROS_WARN("Parameter 'ramp_units' not defined");
            }

            if (searchParam("jump_distance", param_name))
            {
                getParam(param_name, m_jump_distance);
            }
            else
            {
                ROS_WARN("Parameter 'jump distance' not defined");
            }

            if (searchParam("speed_up_distance", param_name))
            {
                getParam(param_name, m_speed_up_distance);
            }
            else
            {
                ROS_WARN("Parameter 'speed_up_distance' not defined");
            }

            if (searchParam("stop_speed", param_name))
            {
                getParam(param_name, m_stop_speed);
            }
            else
            {
                ROS_WARN("Parameter 'stop_speed' not defined");
            }

            if (searchParam("slow_speed", param_name))
            {
                getParam(param_name, m_slow_speed);
            }
            else
            {
                ROS_WARN("Parameter 'stop_speed' not defined");
            }

            if (searchParam("slow_time", param_name))
            {
                getParam(param_name, m_slow_time);
            }
            else
            {
                ROS_WARN("Parameter 'slow_time' not defined");
            }

            // convert to meters
            if (m_ramp_units == "inch")
            {
                m_ramp_base = m_ramp_base * 0.0254;
                m_ramp_height = m_ramp_height * 0.0254;
                m_ramp_units = "meter";
            }

            // calculate some constants that we need
            m_ramp_diagonal = std::sqrt(std::pow(m_ramp_base, 2) + std::pow(m_ramp_height, 2));
            m_ramp_angle = std::asin(m_ramp_height / m_ramp_diagonal);
        }

    ~Jump(){}

    void doJump()
    {
        Result result = calculate_speed();

        double speed_up_time = m_speed_up_distance / result.speed;
        double air_time = result.jump_time;
        int rpm = (int) result.rpm;

        ROS_INFO("Calculated Speed Up Time: %f", speed_up_time);
        ROS_INFO("Calculated Air Time: %f", air_time);
        ROS_INFO("Calculate RPM: %i", rpm);
        
        pololu_maestro_ros::set_servo srv;

        // set speed to initial speed for jump
        srv.request.channel = 1;
        srv.request.target = rpm;
        m_client.call(srv);

        usleep(speed_up_time * MICROSECOND);
        usleep(air_time / 2.0 * MICROSECOND);

        // stop
        srv.request.channel = 1;
        srv.request.target = m_stop_speed;
        m_client.call(srv);

        usleep(air_time / 2.0 * MICROSECOND);

        // slow
        srv.request.channel = 1;
        srv.request.target = m_slow_speed;
        m_client.call(srv);

        usleep(m_slow_time * MICROSECOND);

        // stop
        srv.request.channel = 1;
        srv.request.target = m_stop_speed;
        m_client.call(srv);
    }
};

int main(int argc, char** argv) 
{

    //Initialize node
    ros::init(argc, argv, "do_jump");
    ros::NodeHandle nh;
     
    auto jump_node = Jump(&nh);
    jump_node.doJump();

    ros::spin();
    return 0;
}

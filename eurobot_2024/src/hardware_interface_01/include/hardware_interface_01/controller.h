#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int16.h" 
#include "std_msgs/Bool.h" 
#include "std_msgs/Float32.h" 
#include "geometry_msgs/Pose.h"

using namespace std;

class Controller {
// Description: control of the robot's motors 

    public:
        // Constructor
        Controller();
        void init(ros::NodeHandle& node);

        // Movement methods
        void move(double distance, float max_power);
        void turn(double degrees, int motor, double max_power_p, double final_power);
        void makeMovement();

    private:

        // Init methods
        void moveCallback(const geometry_msgs::Point::ConstPtr& movement);
        void turnCallback(const geometry_msgs::Point::ConstPtr& turn);
        void encoderLeftCallback(const std_msgs::Float32::ConstPtr& encoder_position);
        void encoderRightCallback(const std_msgs::Float32::ConstPtr& encoder_position);
        

        // Publisher attributes
        ros::Publisher _pub_drivetrain;
        ros::Publisher _pub_state;

        // Subscriber attributes
        ros::Subscriber _sub_move;
        ros::Subscriber _sub_turn;
        ros::Subscriber _sub_encoder_l;
        ros::Subscriber _sub_encoder_r;

        // Attributes
        int _encoder_position_l;
        int _encoder_position_r;
        pair<float, float> _move_msg;
        pair<float, float> _turn_msg;

        // Constants
        static const double INIT_POWER;
        static const double FINAL_POWER;
        static const double ACCEL_DIST;
        static const double DESACCEL_DIST;
};
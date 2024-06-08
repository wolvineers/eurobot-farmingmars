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

class Controller02 {
// Description: control of the robot's motors 

    public:
        // Constructor
        Controller02();

        // Movement methods
        void moveArm(int arm, int posArm, int posPlier);
        void moveRack(int pos);
        void init(ros::NodeHandle& node);

    private:
        

        // Publisher attributes
        ros::Publisher _pub_arm_l;
        ros::Publisher _pub_arm_r;
        ros::Publisher _pub_rack;

};
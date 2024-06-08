#include "ros/ros.h" 
#include "controller_02.h"


int arm01, arm02, plier;
bool switch01, switch02;

ros::Publisher _pub_state;

Controller02 controller;

// Description: callback function for processing arm messages
void armCallback(const geometry_msgs::Point::ConstPtr& armPos) {
// Pre: valid geometry_msgs::Point message containing arm position information
// Post: stores the received arm position value and logs it to the console

    std_msgs::Bool state;
    state.data = false;
    _pub_state.publish(state);

    controller.moveArm(armPos->x, armPos->y, armPos->z);

    state.data = true;
    _pub_state.publish(state);

    // Show the info to the console
    std::stringstream ss;
    ss << "/arm | movement message value: " << armPos->x;
    ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing rack messages
void rackCallback(const std_msgs::Int16::ConstPtr& rack) {
// Pre: valid std_msgs::Int16 message containing rack direction information
// Post: stores the received rack direction value and logs it to the console

    ros::Rate loop_rate(50); 

    std_msgs::Bool state_rack;
    state_rack.data = false;
    _pub_state.publish(state_rack);

    ROS_WARN_STREAM("PREVI");

    if (rack->data == 0 && !switch01) {
        ROS_WARN_STREAM("DINS IF 1");
        controller.moveRack(rack->data);

        ros::spinOnce();
        loop_rate.sleep();
    }
    else if (rack->data == 2 && !switch02) {
        ROS_WARN_STREAM("DINS IF 2");
        controller.moveRack(rack->data);

        ros::spinOnce();
        loop_rate.sleep();
    }

    state_rack.data = true;
    _pub_state.publish(state_rack);

    // Show the info to the console
    std::stringstream ss;
    ss << "/arm | movement message value: " << switch02;
    ROS_WARN_STREAM(ss.str());

}

// Description: callback function for processing rack messages
void rackSwitch01Callback(const std_msgs::Bool::ConstPtr& rack) {
// Pre: valid std_msgs::Int16 message containing rack direction information
// Post: stores the received rack direction value and logs it to the console

    switch01 = rack->data;

    // Show the info to the console
    std::stringstream ss;
    ss << "/rack switch 01 ";
    ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing rack messages
void rackSwitch02Callback(const std_msgs::Bool::ConstPtr& rack) {
// Pre: valid std_msgs::Int16 message containing rack direction information
// Post: stores the received rack direction value and logs it to the console

    switch02 = rack->data;

    // Show the info to the console
    std::stringstream ss;
    ss << "/rack switch 02";
    ROS_WARN_STREAM(ss.str());
}


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "hardware_interface_02");

    ros::NodeHandle controllerNode;

    _pub_state = controllerNode.advertise<std_msgs::Bool>("state", 1000);

    controller.init(controllerNode);

    ros::Subscriber _sub_arm_move = controllerNode.subscribe("arm", 1000, armCallback);
    ros::Subscriber _sub_rack_move = controllerNode.subscribe("rack_pos", 1000, rackCallback);
    ros::Subscriber _sub_rack_switch_01 = controllerNode.subscribe("rack_switch_01", 1000, rackSwitch01Callback);
    ros::Subscriber _sub_rack_switch_02 = controllerNode.subscribe("rack_switch_02", 1000, rackSwitch02Callback);

    ros::spin();

    return 0;
}
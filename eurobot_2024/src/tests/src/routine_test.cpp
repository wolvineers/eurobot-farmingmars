#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int16.h" 
#include "std_msgs/Bool.h" 
#include "std_msgs/Float32.h" 

/* --- Variables globals --- */
ros::Publisher _pub_arm;
ros::Publisher _pub_rack;
ros::Subscriber _sub_state;

bool _state = false;
bool _switch_startup = false;


/* --- Funcions --- */

void stateCallback(const std_msgs::Bool::ConstPtr& state) {
    // Save the new value of the movement
    _state = state->data;

    // Show the info to the console
    std::stringstream ss;
    ss << "Action state";
    ROS_WARN_STREAM(ss.str());
}

void startupCallback(const std_msgs::Bool::ConstPtr& state) {
    // Save the new value of the movement
    _switch_startup = state->data;

    // Show the info to the console
    std::stringstream ss;
    ss << "Action switch startup";
    ROS_WARN_STREAM(ss.str());
}

// Description: initialize all the publishers and subscribers
void init(ros::NodeHandle& n) {
// Pre: --
// Pro: initialize all the publishers and subscribers

    // Initialize publishers
    _pub_arm = n.advertise<geometry_msgs::Point>("arm", 1000);
    _pub_rack = n.advertise<std_msgs::Int16>("rack_pos", 1000);

    // Initialize subscribers
    _sub_state = n.subscribe("state", 1000, stateCallback);
}

// Description: take and leave a plant with the left arm
void routineLeftArm(ros::NodeHandle& n) {
// Pre: --
// Post: take and leave a plant with the left arm

    ros::Rate loop_rate(10);
    geometry_msgs::Point arm_msg;


    /* --- POSITION: TAKING --- */

    ROS_WARN_STREAM("Position: taking | start");
        
    arm_msg.x = 0; arm_msg.y = 0; arm_msg.z = 0;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);


    /* --- TAKE THE PLANT --- */

    ROS_WARN_STREAM("Take the plant | start");
        
    arm_msg.x = 0; arm_msg.y = 0; arm_msg.z = 2;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);


    /* --- POSITION: TURNING --- */

    ROS_WARN_STREAM("Position: turning | start");
        
    arm_msg.x = 0; arm_msg.y = 1; arm_msg.z = 2;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);


    /* --- POSITION: LEAVING --- */

    ROS_WARN_STREAM("Position: leaving | start");
        
    arm_msg.x = 0; arm_msg.y = 2; arm_msg.z = 2;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);


    /* --- LEAVE THE PLANT --- */

    ROS_WARN_STREAM("Leave the plant | start");
        
    arm_msg.x = 0; arm_msg.y = 2; arm_msg.z = 1;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Description: take and leave a plant with the right arm
void routineRightArm(ros::NodeHandle& n) {
// Pre: --
// Post: take and leave a plant with the right arm

    ros::Rate loop_rate(10);
    geometry_msgs::Point arm_msg;


    /* --- POSITION: TAKING --- */
        
    arm_msg.x = 1; arm_msg.y = 0; arm_msg.z = 0;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }


    /* --- TAKE THE PLANT --- */
        
    arm_msg.x = 1; arm_msg.y = 0; arm_msg.z = 2;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);


    /* --- POSITION: TURNING --- */
        
    arm_msg.x = 1; arm_msg.y = 1; arm_msg.z = 2;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);


    /* --- POSITION: LEAVING --- */
        
    arm_msg.x = 1; arm_msg.y = 2; arm_msg.z = 2;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(1);


    /* --- LEAVE THE PLANT --- */
        
    arm_msg.x = 1; arm_msg.y = 2; arm_msg.z = 1;
    _pub_arm.publish(arm_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Description: take and leave a plant with the right arm
void routineRack(ros::NodeHandle& n) {
// Pre: --
// Post: take and leave a plant with the right arm

    ros::Rate loop_rate(10);
    std_msgs::Int16 rack_msg;


    /* --- POSITION: TAKING --- */
        
    rack_msg.data = 2;
    _pub_rack.publish(rack_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    rack_msg.data = 1;
    _pub_rack.publish(rack_msg);

    ros::spinOnce();
    loop_rate.sleep();



}


int main(int argc, char **argv) {

    ros::init(argc, argv, "test_routine");

    ros::NodeHandle routineNode;

    ros::Rate loop_rate(50);

    init(routineNode);

    ros::Subscriber _sub_switch_startup = routineNode.subscribe("switch_startup", 1000, startupCallback);

    sleep(1);

    while(!_switch_startup) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    routineRack(routineNode);

    return 0;
}
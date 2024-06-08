#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int16.h" 
#include "std_msgs/Bool.h" 
#include "std_msgs/Float32.h" 

/* --- Variables globals --- */
ros::Publisher _pub_move;
ros::Publisher _pub_turn;
ros::Subscriber _sub_state;

bool _state = false;

/* --- Funcions --- */

void moveCallback(const std_msgs::Bool::ConstPtr& state) {
    // Save the new value of the movement
    _state = state->data;

    // Show the info to the console
    std::stringstream ss;
    ss << "Action state: " << state;
    ROS_WARN_STREAM(ss.str());
}

// Description: initialize all the publishers and subscribers
void init(ros::NodeHandle& n) {
// Pre: --
// Pro: initialize all the publishers and subscribers

    // Initialize publishers
    _pub_move = n.advertise<geometry_msgs::Point>("move", 1000);
    _pub_turn = n.advertise<geometry_msgs::Point>("turn", 1000);

    // Initialize subscribers
    _sub_state = n.subscribe("state", 1000, moveCallback);
}

void move(int direction, int power, int distance) {
// Pre: forward: direction=1, backward: direction=-1
// Post: moves de robot

    ros::Rate loop_rate(10);

    ROS_WARN_STREAM("Dins de routine01");

    geometry_msgs::Point move_msg;
        
    move_msg.x = distance;
    move_msg.y = power * direction;
    move_msg.z = 0;

    ROS_WARN_STREAM("Abans de publicar missatge");

    _pub_move.publish(move_msg);

    ROS_WARN_STREAM("missatge publicat");

    ros::spinOnce();
    loop_rate.sleep();

    ROS_WARN_STREAM("Waiting to finish action");

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void routine01(ros::NodeHandle& n) {

    ros::Rate loop_rate(10);

    ROS_WARN_STREAM("Dins de routine01");

    geometry_msgs::Point move_msg;
        
    move_msg.x = 1000;
    move_msg.y = 35;
    move_msg.z = 0;

    ROS_WARN_STREAM("Abans de publicar missatge");

    _pub_move.publish(move_msg);

    ROS_WARN_STREAM("missatge publicat");

    ros::spinOnce();
    loop_rate.sleep();

    ROS_WARN_STREAM("Waiting to finish action");

    while (!_state) {
        ros::spinOnce();
        loop_rate.sleep();
    }//ROS_WARN_STREAM("Waiting to finish action"); }

    ROS_WARN_STREAM("finish action");

    move_msg.x = 2000;
    move_msg.y = 35;

    _pub_move.publish(move_msg);

    ros::spinOnce();
    loop_rate.sleep();

    while (!_state) { ROS_WARN_STREAM("Waiting to finish action"); }

}

void routineYellow01() {

    move(1, 30, 750);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "routine");

    ros::NodeHandle routineNode;

    ros::Rate loop_rate(50);

    init(routineNode);

    sleep(1);

    routineYellow01();

    return 0;
}
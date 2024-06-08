#include "ros/ros.h" 
#include "controller.h"


pair<float, float> _move_msg;
pair<float, float> _turn_msg;

ros::Publisher _pub_state;
ros::Publisher _pub_switch_1;
ros::Publisher _pub_switch_2;

Controller controller;

// Description: callback function for processing movement messages
void moveCallback(const geometry_msgs::Point::ConstPtr& movement) {
// Pre: valid geometry_msgs::Point message containing movement information
// Post: stores the received movement value and logs it to the console

    // Save the new value of the movement
    _move_msg.first = movement->x;
    _move_msg.second = movement->y;

    if (movement->x != 0) controller.move(movement->x, movement->y);

    // Show the info to the console
    std::stringstream ss;
    ss << "/move | movement message value: " << _move_msg.first << ", " << _move_msg.second;
    ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing turn messages
void turnCallback(const geometry_msgs::Point::ConstPtr& turn) {
// Pre: valid std_msgs::Float32 message containing turn information
// Post: stores the received turn value and logs it to the console

    // Save the new value of the turn
    _turn_msg.first = turn->x;
    _turn_msg.second = turn->y;

    if (turn->x != 0) {
        std_msgs::Bool state;
        state.data = true;
        _pub_state.publish(state);

        controller.turn(turn->x, turn->y, 50, 30);

        state.data = false;
        _pub_state.publish(state);
    }

    // Show the info to the console
    std::stringstream ss;
    ss << "/turn | turn message value: " << _turn_msg.first << ", " << _turn_msg.second;
    ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing turn messages
void switchCallback01(const std_msgs::Bool::ConstPtr& switch_state) {
// Pre: valid std_msgs::Float32 message containing turn information
// Post: stores the received turn value and logs it to the console

    std_msgs::Bool state;
    state.data = true;
    _pub_state.publish(state);
    _pub_switch_1.publish(switch_state);

    // Show the info to the console
    std::stringstream ss;
    ss << "/switch 01 | dins callback";
    ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing turn messages
void switchCallback02(const std_msgs::Bool::ConstPtr& switch_state) {
// Pre: valid std_msgs::Float32 message containing turn information
// Post: stores the received turn value and logs it to the console

    std_msgs::Bool state;
    state.data = true;
    _pub_state.publish(state);
    _pub_switch_2.publish(switch_state);

    // Show the info to the console
    std::stringstream ss;
    ss << "/switch 02 | dins callback";
    ROS_WARN_STREAM(ss.str());

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "hardware_interface_01");

    ros::NodeHandle controllerNode;

    _pub_state = controllerNode.advertise<std_msgs::Bool>("state", 1000);
    _pub_switch_1 = controllerNode.advertise<std_msgs::Bool>("rack_switch_01", 1000);
    _pub_switch_2 = controllerNode.advertise<std_msgs::Bool>("rack_switch_02", 1000);

    controller.init(controllerNode);

    ros::Subscriber _sub_move = controllerNode.subscribe("move", 1000, moveCallback);
    ros::Subscriber _sub_turn = controllerNode.subscribe("turn", 1000, turnCallback);
    ros::Subscriber _sub_switch_1 = controllerNode.subscribe("fi_rack_esq", 1000, switchCallback01);
    ros::Subscriber _sub_switch_2 = controllerNode.subscribe("fi_rack_drt", 1000, switchCallback02);

    ROS_WARN_STREAM("NOSE");

    ros::spin();

    

    return 0;
}
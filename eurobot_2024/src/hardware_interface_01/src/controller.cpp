#include "controller.h"

/* --- CONSTANTS --- */

const double Controller::INIT_POWER = 20;
const double Controller::FINAL_POWER = 20;
const double Controller::ACCEL_DIST = 0;
const double Controller::DESACCEL_DIST = 0;


/* --- CONSTRUCTOR --- */

// Description: initialize all the class attributes
Controller::Controller() {
// Pre: --
// Post: initialize all the class attributes
    /*
    // Initialize publishers and subscribers
    init(node);

    // Initialize attributes
    _encoder_position_l = 0; _encoder_position_r = 0;
    _move_msg.first = 0; _move_msg.second = 0;
    _turn_msg.first = 0; _turn_msg.second = 0;*/
}


/* --- PRIVATE METHODS --- */

// Description: initialize all the publishers and subscribers
void Controller::init(ros::NodeHandle& n) {
// Pre: --
// Pro: initialize all the publishers and subscribers

    _encoder_position_l = 0; _encoder_position_r = 0;
    _move_msg.first = 0; _move_msg.second = 0;
    _turn_msg.first = 0; _turn_msg.second = 0;

    // Initialize publishers
    _pub_drivetrain = n.advertise<geometry_msgs::Point>("drivetrain", 1000);

    // Initialize subscribers
    _sub_move      = n.subscribe("move", 1000, &Controller::moveCallback, this);
    _sub_turn      = n.subscribe("turn", 1000, &Controller::turnCallback, this);
    _sub_encoder_l = n.subscribe("enc1", 1000, &Controller::encoderLeftCallback, this);
    _sub_encoder_r = n.subscribe("enc2", 1000, &Controller::encoderRightCallback, this);
}

// Description: callback function for processing movement messages
void Controller::moveCallback(const geometry_msgs::Point::ConstPtr& movement) {
// Pre: valid geometry_msgs::Point message containing movement information
// Post: stores the received movement value and logs it to the console

    // Save the new value of the movement
    _move_msg.first = movement->x;
    _move_msg.second = movement->y;

    // Show the info to the console
    std::stringstream ss;
    ss << "/move | movement message value: " << _move_msg.first << ", " << _move_msg.second;
    ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing turn messages
void Controller::turnCallback(const geometry_msgs::Point::ConstPtr& turn) {
// Pre: valid std_msgs::Float32 message containing turn information
// Post: stores the received turn value and logs it to the console

    // Save the new value of the turn
    _turn_msg.first = turn->x;
    _turn_msg.second = turn->y;

    // Show the info to the console
    std::stringstream ss;
    ss << "/turn | turn message value: " << _turn_msg.first << ", " << _turn_msg.second;
    ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing encoder left position messages
void Controller::encoderLeftCallback(const std_msgs::Float32::ConstPtr& encoder_position) {
// Pre: valid std_msgs::Int16 message containing encoder left position
// Post: stores the received encoder_position_l value and logs it to the console

    // Save the new value of the encoder
    _encoder_position_l = fabs(encoder_position->data);

    // Show the info to the console
    std::stringstream ss;
    ss << "enc1 | encoder position value: " << _encoder_position_l;
    //ROS_WARN_STREAM(ss.str());
}

// Description: callback function for processing encoder right position messages
void Controller::encoderRightCallback(const std_msgs::Float32::ConstPtr& encoder_position) {
// Pre: valid std_msgs::Int16 message containing encoder right position
// Post: stores the received encoder_position_r value and logs it to the console

    // Save the new value of the encoder
    _encoder_position_r = fabs(encoder_position->data);

    // Show the info to the console
    std::stringstream ss;
    ss << "enc2 | encoder position value: " << _encoder_position_r;
    //ROS_WARN_STREAM(ss.str());
}


/* --- PUBLIC METHODS --- */

// Description: move the robot straight with the trapeze form to the distance desired
void Controller::move(double distance, float max_power_p) {
// Pre: distance >= 0 and -1 < max_power < 1
// Post: move the robot the distance specified with the max_power

    ros::Rate loop_rate(50); 

    std::stringstream ss;
    ss << "encoder_l: " << _encoder_position_l << " | encoder_r: " << _encoder_position_r << " | max_power: " << max_power_p;
    ROS_WARN_STREAM(ss.str());

    // Set the robot direction
    float forward_d = -1;   // point.x = right motor
    float forward_e = 1;    // point.y = left motor
    float max_power = max_power_p;
    
    if (max_power < 0) {
        forward_e = -1;
        forward_d = 1;
        max_power = max_power_p * -1;

        std::stringstream ss;
        ss << "Robot direction | back";
        ROS_WARN_STREAM(ss.str());
    }


    // Acceleration

    /// Calculate the acceleration values
    float m_accel = (max_power - INIT_POWER) / ACCEL_DIST;
    float n_accel = INIT_POWER;

    /// Move the robot with the acceleration calculated before
    while (ros::ok() && _encoder_position_l < ACCEL_DIST) {

        // Create the drivetrain message
        geometry_msgs::Point drivetrain_msg;

        // Calculate correction factor
        float factorCorreccioLeft   = 1.0 - (_encoder_position_l - _encoder_position_r) / 1000.0;
        float factorCorreccioRight  = 1.0 - (_encoder_position_r - _encoder_position_l) / 1000.0;

        if (factorCorreccioLeft < 0.9) factorCorreccioLeft = 0.9;
        else if (factorCorreccioLeft > 1.1) factorCorreccioLeft = 1.1; 
        if (factorCorreccioRight < 0.9) factorCorreccioRight = 0.9;
        else if (factorCorreccioRight > 1.1) factorCorreccioRight = 1.1; 

        std::stringstream ss;
        ss << "A: Factor esquerre: " << factorCorreccioLeft << " | " << factorCorreccioRight;
        ROS_WARN_STREAM(ss.str());

        // Save the new power value
        float powerR = (m_accel * _encoder_position_l + n_accel) * factorCorreccioLeft;
        float powerL = (m_accel * _encoder_position_l + n_accel) * factorCorreccioRight;
        
        // Set the power to each motor
        drivetrain_msg.x = (powerR / 100) * forward_d;
        drivetrain_msg.y = (powerL / 100) * forward_e;

        // Publish the message
        _pub_drivetrain.publish(drivetrain_msg);
        
        // Process callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Straight

    double straight_distance = distance - ACCEL_DIST - DESACCEL_DIST;

    std::stringstream s1;
    s1 << "Encoder esquerre: " << _encoder_position_l << " | " << "Distancia: " << straight_distance;
    ROS_WARN_STREAM(s1.str());

    while (_encoder_position_l < straight_distance) {
        // Create the drivetrain message
        geometry_msgs::Point drivetrain_msg;
        
        // Calculate correction factor
        float factorCorreccioLeft   = 1.0 - (_encoder_position_l - _encoder_position_r) / 1000.0;
        float factorCorreccioRight  = 1.0 - (_encoder_position_r - _encoder_position_l) / 1000.0;

        if (factorCorreccioLeft < 0.9) factorCorreccioLeft = 0.9;
        else if (factorCorreccioLeft > 1.1) factorCorreccioLeft = 1.1; 
        if (factorCorreccioRight < 0.9) factorCorreccioRight = 0.9;
        else if (factorCorreccioRight > 1.1) factorCorreccioRight = 1.1; 

        std::stringstream ss;
        ss << "R: Factor esquerre: " << (_encoder_position_l - _encoder_position_r) << " | " << factorCorreccioLeft << " | " << factorCorreccioRight << " | ";
        //ROS_WARN_STREAM(ss.str());

        // Set the power to each motor
        drivetrain_msg.x = (max_power * factorCorreccioLeft / 100.0) * forward_d;
        drivetrain_msg.y = (max_power * factorCorreccioRight / 100.0) * forward_e;
        drivetrain_msg.z = 0;

        // Publish the message
        _pub_drivetrain.publish(drivetrain_msg);

        // Process callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Deceleration

    /// Calculate the deceleration values
    float m_desaccel = (FINAL_POWER - max_power) / (distance - _encoder_position_l);
    float n_desaccel = FINAL_POWER - m_desaccel * distance;

    /// Move the robot with the deceleration calculated before
    while (_encoder_position_l < distance) {
        // Create the drivetrain message
        geometry_msgs::Point drivetrain_msg;
        
        // Calculate correction factor
        float factorCorreccioLeft   = 1.0 - (_encoder_position_l - _encoder_position_r) / 1000.0;
        float factorCorreccioRight  = 1.0 - (_encoder_position_r - _encoder_position_l) / 1000.0;

        if (factorCorreccioLeft < 0.9) factorCorreccioLeft = 0.9;
        else if (factorCorreccioLeft > 1.1) factorCorreccioLeft = 1.1; 
        if (factorCorreccioRight < 0.9) factorCorreccioRight = 0.9;
        else if (factorCorreccioRight > 1.1) factorCorreccioRight = 1.1; 

        std::stringstream ss;
        ss << "D: Factor esquerre: " << factorCorreccioLeft << " | " << factorCorreccioRight;
        ROS_WARN_STREAM(ss.str());

        // Save the new power value
        float powerR = (m_accel * _encoder_position_l + n_desaccel) * factorCorreccioLeft;
        float powerL = (m_accel * _encoder_position_l + n_desaccel) * factorCorreccioRight;
        
        // Set the power to each motor
        drivetrain_msg.x = (powerR / 100) * forward_d;
        drivetrain_msg.y = (powerL / 100) * forward_e;

        // Publish the message
        _pub_drivetrain.publish(drivetrain_msg);

        // Process callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }


    std::stringstream ss1;
    ss1 << "encoder_left: " << _encoder_position_l << " | encoder_right: " << _encoder_position_r;
    ROS_WARN_STREAM(ss1.str());

    // Stop motors
    geometry_msgs::Point drivetrain_msg;
        
    drivetrain_msg.x = 0;
    drivetrain_msg.y = 0;
    drivetrain_msg.z = 1;

    _pub_drivetrain.publish(drivetrain_msg);

    ros::spinOnce();
    loop_rate.sleep();  

    ROS_WARN_STREAM("FIIN")  ;
}

void Controller::turn(double degrees, int motor, double max_power_p, double final_power) {
// Pre: -100 < max_power < 100
// Post: turn the robot the degrees specified with the max_power and the motor (0: left, 1: right)

    ros::Rate loop_rate(10); 

    // Set the robot direction
    float left_motor = 0;
    float right_motor = 0;
    float max_power = max_power_p;

    if (motor == 0) {   // Configure the left motor (point.y)
        if (max_power < 0) {
            left_motor = 1;
            max_power = max_power_p * -1;
        } else { left_motor = -1; }
    } else {    // Configure the right motor (point.x)
        if (max_power < 0) {
            right_motor = -1;
            max_power = max_power_p * -1;
        } else { right_motor = 1; }
    }


    // Turn

    double turn_distance = degrees - DESACCEL_DIST;

    int position;

    if (left_motor != 0) {
        position = _encoder_position_l;
    } else {
        position = _encoder_position_r;
    }

    while (position < turn_distance) {
        // Create the drivetrain message
        geometry_msgs::Point drivetrain_msg;
        
        // Set the power to each motor
        drivetrain_msg.x = max_power * left_motor;
        drivetrain_msg.y = max_power * right_motor;
        drivetrain_msg.z = 0;

        // Publish the message
        _pub_drivetrain.publish(drivetrain_msg);

        // Calculate the position
        if (left_motor != 0) {
            position = _encoder_position_l;
        } else {
            position = _encoder_position_r;
        }

        // Process callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Deceleration

    /// Calculate the deceleration values
    float m_desaccel = (final_power - max_power) / (degrees - position);
    float n_desaccel = final_power - m_desaccel * degrees;

    /// Move the robot with the deceleration calculated before
    while (position < degrees) {
        // Create the drivetrain message
        geometry_msgs::Point drivetrain_msg;

        // Save the new power value
        float power = m_desaccel * position + n_desaccel;
        
        // Set the power to each motor
        drivetrain_msg.x = (power / 100) * left_motor;
        drivetrain_msg.y = (power / 100) * right_motor;

        // Publish the message
        _pub_drivetrain.publish(drivetrain_msg);

        // Calculate the position
        if (left_motor != 0) {
            position = _encoder_position_l;
        } else {
            position = _encoder_position_r;
        }

        // Process callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }


    // Stop motors
    geometry_msgs::Point drivetrain_msg;
        
    drivetrain_msg.x = 0;
    drivetrain_msg.y = 0;
    drivetrain_msg.z = 1;

    _pub_drivetrain.publish(drivetrain_msg);

    ros::spinOnce();
    loop_rate.sleep();
}


// Description: make the movement option of the subscribers
void Controller::makeMovement() {
// Pre: --
// Post: make the movement options of the subscribers

    //move(1000, 35);


    ros::Rate loop_rate(50);

    std::stringstream ss;
    ss << "/move | value: " << _move_msg.first << ", " << _move_msg.second;
    ROS_WARN_STREAM(ss.str());

    if (_move_msg.first != 0) {

        std_msgs::Bool state;
        state.data = true;
        _pub_drivetrain.publish(state);

        ros::spinOnce();
        loop_rate.sleep();

        ROS_WARN_STREAM("makeMovement | dins del sub_move");
        move(_move_msg.first, _move_msg.second);

        state.data = false;
        _pub_drivetrain.publish(state);

        ros::spinOnce();
        loop_rate.sleep();

    } else if (_turn_msg.first != 0) {
        // turn
    }

}
#include "controller_02.h"


/* --- CONSTRUCTOR --- */

// Description: initialize all the class attributes
Controller02::Controller02() {
// Pre: --
// Post: initialize all the class attributes
}


/* --- PRIVATE METHODS --- */

// Description: initialize all the publishers and subscribers
void Controller02::init(ros::NodeHandle& n) {
// Pre: --
// Pro: initialize all the publishers and subscribers

    // Initialize publishers
    _pub_arm_l = n.advertise<geometry_msgs::Point>("bras_esquerre", 1000);
    _pub_arm_r = n.advertise<geometry_msgs::Point>("bras_dret", 1000);
    _pub_rack = n.advertise<std_msgs::Int16>("rack", 1000);
}


/* --- PUBLIC METHODS --- */

// Description: move the arm on the desired position
void Controller02::moveArm(int arm, int pos_arm, int pos_plier) {
// Pre: --
// Post: move the arm on the posArm and posPlier positions

    /* --- POSIBLE PARAMETERS FOR FUNCTION ---
     *
     * Arm position:
     *    - Position 0 --> take plant
     *    - Position 1 --> initial position 
     *    - Position 2 --> leave plant
     * 
     * Plier position:
     *    - Position 0 --> open before take plants  (z: 100)
     *    - Position 1 --> open to leave plants     (z: 130)
     *    - Position 2 --> close to take plants     (z: 150)
     * 
     * 
     * --- MOVEMENTS SEQUENCE OF THE ARM --- 
     * 
     * 1. Pose robot in front of the plant before take it
     *      moveArm(X, 0, 0)
     * 2. Close the plier to take the plant
     *      moveArm(X, 0, 2)
     * 3. Put the arm to the initial position
     *      moveArm(X, 1, 2)
     * 4. Put the arm on the other side
     *      moveArm(X, 2, 2)
     * 5. Leave the plant on the corresponent side
     *      moveArm(X, 2, 1)
     *
    */

    ros::Rate loop_rate(50); 

    geometry_msgs::Point arm_position;

    // Save de plier parameter message
    if (pos_plier == 0) arm_position.z = 100;
    else if (pos_plier == 1) arm_position.z = 130;
    else arm_position.z = 150;

    // Save the other message values (0: left arm; 1: right arm)
    if (arm == 0) {
        // Save the message values
        if (pos_arm == 0) { arm_position.x = 0, arm_position.y = 180; }
        else if (pos_arm == 1) { arm_position.x = 90; arm_position.y = 0; }
        else { arm_position.x = 180; arm_position.y = 0; }

        // Publish the message
        _pub_arm_l.publish(arm_position);
    } else {
        // Save the message values
        if (pos_arm == 0) { arm_position.x = 0, arm_position.y = 0; }
        else if (pos_arm == 1) { arm_position.x = 90; arm_position.y = 180; }
        else { arm_position.x = 180; arm_position.y = 180; }

        // Publish the message
        _pub_arm_r.publish(arm_position);
    }

    ros::spinOnce();
    
    loop_rate.sleep();

}

// Description: move the rack on the desired direction
void Controller02::moveRack(int pos) {
// Pre: --
// Post: move the rack on the pos direction

    /* --- POSIBLE PARAMETERS FOR FUNCTION ---
     *
     * Position:
     *    - Position 0 --> left
     *    - Position 1 --> middle position 
     *    - Position 2 --> right
     *
    */

    ros::Rate loop_rate(50); 

    std_msgs::Int16 rack_position;

    // Save the message data
    if (pos == 0) rack_position.data = 91;
    else if (pos == 1) rack_position.data = 92;
    else rack_position.data = 93;

    // Publish the message
    _pub_rack.publish(rack_position);

    ros::spinOnce();
    
    loop_rate.sleep();
}

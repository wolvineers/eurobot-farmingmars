#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

void drivetrainCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Check if linear.x is greater than 0
    if (msg->linear.x > 0) {
        ROS_INFO("published");
        // Simulate increasing encoder value
        static float encoder_value = 0;
        encoder_value += 1; // Increment by a fixed value, adjust as needed

        // Publish fake encoder message
        std_msgs::Float32 encoder_msg;
        encoder_msg.data = encoder_value;

        ros::NodeHandle nh;
        ros::Publisher encoder_pub = nh.advertise<std_msgs::Float32>("/robot/encoder_value", 1000);
        encoder_pub.publish(encoder_msg);

        ros::spin();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoder_simulator");
    ros::NodeHandle nh;

    // Subscribe to the drivetrain topic
    ros::Subscriber drivetrain_sub = nh.subscribe("/robot/drivetrain", 1000, drivetrainCallback);

    ros::spin();

    return 0;
}

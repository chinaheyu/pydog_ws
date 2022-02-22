#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>


ros::Publisher pitch_pub;
ros::Publisher roll_pub;

ros::Publisher pose_pub;


double current_pitch, current_roll, current_yaw;


void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std_msgs::Float64 pitch_msg, roll_msg;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3 m(q);
    tf::Matrix3x3(q).getRPY(current_roll, current_pitch, current_yaw);

    pitch_msg.data = current_pitch;
    roll_msg.data = current_roll;

    pitch_pub.publish(pitch_msg);
    roll_pub.publish(roll_msg);
}


void pose_msg_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    geometry_msgs::Pose pose_msg;

    double target_pitch, target_roll, target_yaw;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3 m(q);
    tf::Matrix3x3(q).getRPY(target_roll, target_pitch, target_yaw);

    double diff_pitch = 1.0 * (target_pitch - current_pitch);
    double diff_row = 1.0 * (target_roll - current_roll);

    pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(diff_row, diff_pitch, 0);

    pose_msg.position = msg->position;

    pose_pub.publish(pose_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pydog_balance_node");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imu_data_callback);
    ros::Subscriber pose_sub = pnh.subscribe("body_pose", 1, pose_msg_callback);

    pitch_pub = pnh.advertise<std_msgs::Float64>("pitch", 1);
    roll_pub = pnh.advertise<std_msgs::Float64>("roll", 1);
    pose_pub = nh.advertise<geometry_msgs::Pose>("body_pose", 1);

    ros::spin();

    return 0;
}
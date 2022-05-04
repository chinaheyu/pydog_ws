#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/server.h>
#include <pydog_balance/BalanceConfig.h>
#include <ros/package.h>

#include <iostream>
#include <fstream> 

std::ofstream logfile;


ros::Publisher pitch_pub;
ros::Publisher roll_pub;

ros::Publisher pose_pub;


double current_pitch, current_roll, current_yaw;
double target_pitch, target_roll, target_yaw;
geometry_msgs::Point target_position;
double roll_last_error, pitch_last_error;
double roll_history_error, pitch_history_error;

double Kroll[3] = {1.1, 0.2, 0.02};
double Kpitch[3] = {1.1, 0.2, 0.02};

void callback(pydog_balance::BalanceConfig &config, uint32_t level) {
    Kroll[0] = config.kroll1;
    Kroll[1] = config.kroll2;
    Kroll[2] = config.kroll3;
    Kpitch[0] = config.kpitch1;
    Kpitch[1] = config.kpitch2;
    Kpitch[2] = config.kpitch3;
}

void imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std_msgs::Float64 pitch_msg, roll_msg;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3 m(q);
    tf::Matrix3x3(q).getRPY(current_roll, current_pitch, current_yaw);

    pitch_msg.data = current_pitch;
    roll_msg.data = current_roll;

    logfile << ros::Time::now().toSec() << "," << current_roll << "," << current_pitch << "\n";

    pitch_pub.publish(pitch_msg);
    roll_pub.publish(roll_msg);

    geometry_msgs::Pose pose_msg;
    double roll_error = target_roll - current_roll;
    double pitch_error = target_pitch - current_pitch;
    roll_history_error += roll_error;
    pitch_history_error += pitch_error;

    double roll_out = Kroll[0] * roll_error + Kroll[1] * roll_history_error + Kroll[2] * (roll_error - roll_last_error);
    double pitch_out =  Kpitch[0] * pitch_error + Kpitch[1] * pitch_history_error + Kpitch[2] * (pitch_error - pitch_last_error);

    roll_last_error = roll_error;
    pitch_last_error = pitch_error;
    pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_out, pitch_out, 0);
    pose_msg.position = target_position;

   // pose_pub.publish(pose_msg);
}


void pose_msg_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3 m(q);
    tf::Matrix3x3(q).getRPY(target_roll, target_pitch, target_yaw);
    target_position = msg->position;
}


int main(int argc, char** argv)
{
    logfile.open(ros::package::getPath("pydog_balance") + "/log.csv");

    ros::init(argc, argv, "pydog_balance_node");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imu_data_callback);
    ros::Subscriber pose_sub = pnh.subscribe("body_pose", 1, pose_msg_callback);

    pitch_pub = pnh.advertise<std_msgs::Float64>("pitch", 1);
    roll_pub = pnh.advertise<std_msgs::Float64>("roll", 1);
    pose_pub = nh.advertise<geometry_msgs::Pose>("body_pose", 1);

    dynamic_reconfigure::Server<pydog_balance::BalanceConfig> server;
    dynamic_reconfigure::Server<pydog_balance::BalanceConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();

    logfile.close();

    return 0;
}
#include <ros/ros.h>
#include <champ_msgs/ContactsStamped.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Contacts.h>
#include <champ_msgs/ContactsStamped.h>
#include <champ/utils/urdf_loader.h>
#include <olfaction_msgs/anemometer.h>
#include <olfaction_msgs/gas_sensor.h>


ros::Publisher foot_contacts_publisher_;
ros::Publisher joint_states_publisher_;
ros::Publisher joint_commands_publisher_;

ros::Publisher gas_sensor_publisher_;
ros::Publisher anemometer_publisher_;

std::vector<std::string> joint_names_;


void jointStatesRawCallback_(const champ_msgs::Joints::ConstPtr& msg)
{
    sensor_msgs::JointState joints_msg;

    joints_msg.header.stamp = ros::Time::now();
    joints_msg.name.resize(joint_names_.size());
    joints_msg.position.resize(joint_names_.size());
    joints_msg.name = joint_names_;

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {    
        joints_msg.position[i]= msg->position[i];
    }

    joint_states_publisher_.publish(joints_msg);
}

void footContactCallback_(const champ_msgs::Contacts::ConstPtr& msg)
{
    champ_msgs::ContactsStamped contacts_msg;
    contacts_msg.header.stamp = ros::Time::now();
    contacts_msg.contacts.resize(4);

    for(size_t i = 0; i < 4; i++)
    {
        contacts_msg.contacts[i] = msg->contacts[i];
    }

    foot_contacts_publisher_.publish(contacts_msg);
}

void gasSensorCallback_(const olfaction_msgs::gas_sensor::ConstPtr& msg)
{
    olfaction_msgs::gas_sensor gas_sensor_msg;
    gas_sensor_msg.header.stamp = ros::Time::now();
    gas_sensor_msg.raw = msg->raw;
    gas_sensor_msg.raw_units = msg->raw_units;

    gas_sensor_publisher_.publish(gas_sensor_msg);
}

void anemometerCallback_(const olfaction_msgs::anemometer::ConstPtr& msg)
{
    olfaction_msgs::anemometer anemometer_msg;
    anemometer_msg.header.stamp = ros::Time::now();
    anemometer_msg.wind_direction = msg->wind_direction;
    anemometer_msg.wind_speed = msg->wind_speed;

    anemometer_publisher_.publish(anemometer_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pydog_base");
    ros::NodeHandle nh;

    joint_names_ = champ::URDF::getJointNames(&nh);

    foot_contacts_publisher_ = nh.advertise<champ_msgs::ContactsStamped>("foot_contacts", 1);
    joint_states_publisher_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    joint_commands_publisher_ = nh.advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 1);

    gas_sensor_publisher_ = nh.advertise<olfaction_msgs::gas_sensor>("/PID/Sensor_reading", 1);
    anemometer_publisher_ = nh.advertise<olfaction_msgs::anemometer>("/Anemometer/WindSensor_reading", 1);

    ros::Subscriber joints_raw_subscriber_ = nh.subscribe("joint_states/raw", 1, jointStatesRawCallback_);
    ros::Subscriber foot_contacts_subscriber_ = nh.subscribe("foot_contacts/raw", 1, footContactCallback_);

    ros::Subscriber gas_sensor_subscriber_ = nh.subscribe("gas_sensor/raw", 1, gasSensorCallback_);
    ros::Subscriber anemometer_subscriber_ = nh.subscribe("anemometer/raw", 1, anemometerCallback_);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}

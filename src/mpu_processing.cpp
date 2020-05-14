#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#define M_PI 3.14159265

geometry_msgs::Vector3 accel_raw_data;
geometry_msgs::Vector3 gyro_raw_data;

void accelRawCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    accel_raw_data = *msg;
}

void gyroRawCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    gyro_raw_data = *msg;
}

int main(int argc, char** argv)
{
    // init ros
    ros::init(argc, argv, "mpu_processing");
    ros::NodeHandle nh;

    // sub to accel and gyro raw
    ros::Subscriber accel_sub = nh.subscribe("/accel_raw_data", 100, &accelRawCallback);
    ros::Subscriber gyro_sub = nh.subscribe("/gyro_raw_data", 100, &gyroRawCallback);

    // declare imu data
    sensor_msgs::Imu imu_data;

    ros::Rate r(100);
    while(ros::ok())
    {
        // calculate rotational velocities in rad/s
        double gxf = gyro_raw_data.x * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
        double gyf = gyro_raw_data.y * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
        double gzf = gyro_raw_data.z * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

        // calculate accelerations in m/s²
        double axf = accel_raw_data.x * (8.0 / 65536.0) * 9.81;
        double ayf = accel_raw_data.y * (8.0 / 65536.0) * 9.81;
        double azf = accel_raw_data.z * (8.0 / 65536.0) * 9.81;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
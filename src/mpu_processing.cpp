#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

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
        double gxf = gyro_raw_data.x / 131.0;
        double gyf = gyro_raw_data.y / 131.0;
        double gzf = gyro_raw_data.z / 131.0;

        // calculate accelerations in m/s²
        double axf = accel_raw_data.x / 16384.0;
        double ayf = accel_raw_data.y / 16384.0;
        double azf = accel_raw_data.z / 16384.0;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
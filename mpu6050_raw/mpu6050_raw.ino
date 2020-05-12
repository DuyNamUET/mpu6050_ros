/*
Connect MPU6050 with Arduino Uno
    UNO     MPU6050
    5V      VCC
    GND     GND
    A5      SCL
    A4      DSA
    2       INT
*/

// ROS library
#include "ros.h"
#include "sensor_msgs/Imu.h"
#include "ros/time.h"
#include "tf/transform_broadcaster.h"

// MPU6050 independence library
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// ROS initialize
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster br;

char base_link[] = "/base_link";
char imu[] = "/imu";

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t vx, vy, vz;

#define LED_PIN 13
#define PI 3.14159265
bool blinkState = false;

void processLinearAccelData()
{
    ax /= 16384.0;
    ay /= 16384.0;
    az /= 16384.0;
}

void processAngularVelData()
{
    vx = vx/131.0*PI/180.0;
    vy = vy/131.0*PI/180.0;
    vz = vz/131.0*PI/180.0;
}

void setup()
{
    // ROS setup
    nh.initNode();
    br.init(nh);

    Wire.begin();
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &vx, &vy, &vz);
    processLinearAccelData();
    processAngularVelData();

    Serial.print("Linear Acceleration:\n");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.println(az);
    Serial.print("Angular Velocity:\n");
    Serial.print(vx); Serial.print("\t");
    Serial.print(vy); Serial.print("\t");
    Serial.println(vz);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
    t.header.frame_id = base_link;
    t.header.stamp = nh.now();
    t.child_frame_id = imu;
    
    

    delay(10);
}

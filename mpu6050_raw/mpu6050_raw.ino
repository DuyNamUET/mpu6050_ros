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
#include "geometry_msgs/Vector3.h"

// MPU6050 independence library
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// ROS initialize
ros::NodeHandle nh;
geometry_msgs::Vector3 accel_raw_data;
geometry_msgs::Vector3 gyro_raw_data;
ros::Publisher accelpub("/accel_raw", &accel_raw_data);
ros::Publisher gyropub("/gyro_raw", &gyro_raw_data);

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
 
#define LED_PIN 13
bool blinkState = false;

void setup()
{
    // ROS setup
    nh.initNode();
    nh.advertise(accelpub);
    nh.advertise(gyropub);

    Wire.begin();
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // supply your own gyro offsets here, scaled for min sensitivity
    accelgyro.setXAccelOffset(-1169);
    accelgyro.setYAccelOffset(744);
    accelgyro.setZAccelOffset(1620);
    accelgyro.setXGyroOffset(48);
    accelgyro.setYGyroOffset(47);
    accelgyro.setZGyroOffset(-8);

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // print data on Serial monitor
    Serial.print("Data: ");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    // get data to publish
    accel_raw_data.x = ax;
    accel_raw_data.y = ay;
    accel_raw_data.z = az;

    gyro_raw_data.x = gx;
    gyro_raw_data.y = gy;
    gyro_raw_data.z = gz;

    accelpub.publish(&accel_raw_data);
    gyropub.publish(&gyro_raw_data);


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(10);
}

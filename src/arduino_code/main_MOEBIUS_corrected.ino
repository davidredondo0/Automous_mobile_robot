#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <std_msgs/Int16.h>
#include <stdio.h>
#include <std_msgs/String.h>

/*
************************************************************************************
                        // FORWARD DIRECTION //
                        
                               
                                _     _
                        -------|-|---|-|------- 
                ||\\|| |                       | ||//||
M2 (FL)         ||\\||=|                       |=||//||  M3 (FR)
                ||\\||=|                       |=||//||
                ||\\|| |                       | ||//||
                       |                       |
                       |                       |
                     [-|                       |-]  
                     [-|                       |-]
                       |                       |
                       |                       |
                       |                       |] Power Switch
                       |                       |
                        =======================
                ||//|| |                       | ||\\||
M1 (RL)         ||//||=|                       |=||\\||  M4 (RR)
                ||//||=|                       |=||\\||
                ||//||  -------|-|---|-|-------  ||\\||
                                ‾     ‾ 
                              

                       // BACKWARD DIRECTION //
************************************************************************************
*/

// CONFIGURATION
// Set this to false for Differential Drive (Skid Steer)
const bool IS_HOLONOMIC = true; 

// Motor and encoder Parameters
#define I2C_ADDR 0x34
#define ADC_BAT_ADDR 0
#define MOTOR_TYPE_ADDR 20
#define MOTOR_ENCODER_POLARITY_ADDR 21
#define MOTOR_FIXED_SPEED_ADDR 51
#define MOTOR_ENCODER_TOTAL_ADDR 0x3C

const float pulse_per_revolution = 1325.0;
const float wheel_radius = 0.04;
const float max_speed = 0.5;
const float max_speed_moebius = 5; // Scaling factor for motor command
const float L1 = 0.20; // Half Width
const float L2 = 0.175; // Half Length

int8_t MotorType = 3;
int8_t MotorEncoderPolarity = 0;
int32_t EncodeTotal[4];
int8_t speed_array[4]; // [M3, M2, M1, M4] based on original code usage? Let's check.
// Original code cmd_vel:
// speed_array[2] = M1 (RL)
// speed_array[1] = M2 (FL)
// speed_array[0] = M3 (FR)
// speed_array[3] = M4 (RR)

// Store measured speeds
// Order for ROS: [FL, FR, RL, RR]
float wheel_velocities[4]; 
uint8_t data[20];

ros::NodeHandle nh;
void messageCb(const geometry_msgs::Twist& cmd_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &messageCb);

std_msgs::Float32MultiArray wheel_velocity_msg;
ros::Publisher wheel_velocity_pub("wheel_velocity", &wheel_velocity_msg);

std_msgs::Int8MultiArray speed_array_msg;
ros::Publisher speed_array_pub("cmd_recieved", &speed_array_msg);

std_msgs::Int32MultiArray encoder_msg;
ros::Publisher encoder_pub("encoder_data", &encoder_msg);

std_msgs::Float32 frecuency_msg;
ros::Publisher frecuency_pub("frecuency", &frecuency_msg);

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

// Global variables for encoder counts
volatile int intCount1_pre = 0;
volatile int intCount2_pre = 0;
volatile int intCount3_pre = 0;
volatile int intCount4_pre = 0;

volatile int intCount1 = 0; // RL
volatile int intCount2 = 0; // FL
volatile int intCount3 = 0; // FR
volatile int intCount4 = 0; // RR

double vx, vy, w;
unsigned long prevUpdateTime;
double long updateOldness;
double long now;
double long updateRate = 70.0; // ms

// Utility functions to handle I2C communication
bool WireWriteByte(int8_t val) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) return false;
    return true;
}

bool WireWriteDataArray(uint8_t reg,int8_t *val,unsigned int len) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    for(unsigned int i = 0; i < len; i++) Wire.write(val[i]);
    if( Wire.endTransmission() != 0 ) return false;
    return true;
}

bool WireReadDataByte(uint8_t reg, uint8_t &val) {
    if (!WireWriteByte(reg)) return false;
    Wire.requestFrom(I2C_ADDR, 1);
    while (Wire.available()) val = Wire.read();
    return true;
}

int WireReadDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
    unsigned char i = 0;
    if (!WireWriteByte(reg)) return -1;
    Wire.requestFrom(I2C_ADDR, len);
    while (Wire.available()) {
        if (i >= len) return -1;
        val[i] = Wire.read();
        i++;
    }
    return i;
}

// Callback for Twist message
void messageCb(const geometry_msgs::Twist& msg) {
  vx = (float)msg.linear.x;
  vy = (float)msg.linear.y;
  w = (float)msg.angular.z;

  float v1, v2, v3, v4; // target speeds in m/s

  if (IS_HOLONOMIC) {
      // Mecanum
      // M1 (RL)
      v1 = 1 * vx + 1 * vy - (L1 + L2) * w;
      // M2 (FL)
      v2 = 1 * vx - 1 * vy - (L1 + L2) * w;
      // M3 (FR)
      v3 = 1 * vx + 1 * vy + (L1 + L2) * w;
      // M4 (RR)
      v4 = 1 * vx - 1 * vy + (L1 + L2) * w;
  } else {
      // Differential
      // Left = Vx - W * L
      // Right = Vx + W * L
      // M1 (RL), M2 (FL) = Left
      // M3 (FR), M4 (RR) = Right
      float v_left = vx - w * L1; // L1 used as half-width here
      float v_right = vx + w * L1;
      
      v1 = v_left;
      v2 = v_left;
      v3 = v_right;
      v4 = v_right;
  }

  // Convert m/s limit to motor values
  // M1 (RL) -> Index 2
  speed_array[2] = -int8_t(v1 * max_speed_moebius / max_speed); 
  // M2 (FL) -> Index 1
  speed_array[1] = int8_t(v2 * max_speed_moebius / max_speed); 
  // M3 (FR) -> Index 0
  speed_array[0] = int8_t(v3 * max_speed_moebius / max_speed); 
  // M4 (RR) -> Index 3
  speed_array[3] = -int8_t(v4 * max_speed_moebius / max_speed); 
}

void getWheelVel() {
  float factor = ((float)1000/updateOldness) * TWO_PI * wheel_radius;
  
  // Calculate velocities in m/s
  // ROS Order: [FL, FR, RL, RR]
  
  // FL (M2, intCount2)
  wheel_velocities[0] = ((float)intCount2 / pulse_per_revolution) * factor; 
  // FR (M3, intCount3)
  wheel_velocities[1] = ((float)intCount3 / pulse_per_revolution) * factor; 
  // RL (M1, intCount1)
  wheel_velocities[2] = ((float)intCount1 / pulse_per_revolution) * factor; 
  // RR (M4, intCount4)
  wheel_velocities[3] = ((float)intCount4 / pulse_per_revolution) * factor; 
  
  /* 
  Debug notes:
  M1 (RL) matches intCount1 (EncodeTotal[2])
  M2 (FL) matches intCount2 (EncodeTotal[1])
  M3 (FR) matches intCount3 (EncodeTotal[0])
  M4 (RR) matches intCount4 (EncodeTotal[3])
  */
}

bool readAndPublish() {
  sensors_event_t orientationData, angVelocityData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  imu::Quaternion quat = bno.getQuat();

  ros::Time time = nh.now();

  imu_msg.header.stamp = time;
  imu_msg.header.frame_id = "imu_link";

  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  // Normalize (good practice)
  double fused_orientation_norm = std::sqrt(
      imu_msg.orientation.w*imu_msg.orientation.w +
      imu_msg.orientation.x*imu_msg.orientation.x +
      imu_msg.orientation.y*imu_msg.orientation.y +
      imu_msg.orientation.z*imu_msg.orientation.z);

  imu_msg.orientation.w /= fused_orientation_norm;
  imu_msg.orientation.x /= fused_orientation_norm;
  imu_msg.orientation.y /= fused_orientation_norm;
  imu_msg.orientation.z /= fused_orientation_norm;

  imu_msg.angular_velocity.x = angVelocityData.gyro.x;
  imu_msg.angular_velocity.y = angVelocityData.gyro.y;
  imu_msg.angular_velocity.z = angVelocityData.gyro.z;

  imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
  imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
  imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

  imu_pub.publish(&imu_msg);
  return true;
}

void setup() {
  Serial.begin(9600); // For rosserial? No, rosserial usually takes Serial. 
  // If communicating with ROS via USB, Serial is used by node handle. 
  // Don't use Serial.print for debug. Use nh.loginfo

  nh.getHardware()->setBaud(115200); // Explicitly set baud rate if needed
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(encoder_pub);
  nh.advertise(imu_pub);
  nh.advertise(wheel_velocity_pub);
  nh.advertise(speed_array_pub);
  nh.advertise(frecuency_pub);

  Wire.begin();
  delay(200);
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);

  if (!bno.begin()) {
     // Handle error
     nh.logerror("No BNO055 detected");
  } else {
     nh.loginfo("IMU connected");
     bno.setExtCrystalUse(true);
  }
}

void loop() {
  now = millis();
  updateOldness = now - prevUpdateTime;
  if (updateOldness >= updateRate)
  {
    // Read encoder data
    WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, (uint8_t*)EncodeTotal, 16);
    intCount1 = EncodeTotal[2] - intCount1_pre; //M1
    intCount1_pre = EncodeTotal[2];

    intCount2 = EncodeTotal[1] - intCount2_pre; //M2
    intCount2_pre = EncodeTotal[1];

    intCount3 = EncodeTotal[0] - intCount3_pre; //M3
    intCount3_pre = EncodeTotal[0];

    intCount4 = EncodeTotal[3] - intCount4_pre; //M4
    intCount4_pre = EncodeTotal[3];

    // Calculate wheel velocities
    getWheelVel();

    // Send new speed commands to motors
    WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,speed_array,4);
    
    prevUpdateTime = now;  

    readAndPublish();

    // Publish wheel velocities
    wheel_velocity_msg.data_length = 4; // Use 4, exclude time/extra
    wheel_velocity_msg.data = wheel_velocities;
    wheel_velocity_pub.publish(&wheel_velocity_msg);
    
    // speed_array_msg.data_length = 4;
    // speed_array_msg.data = speed_array;
    // speed_array_pub.publish(&speed_array_msg);

    // encoder_msg.data_length = 4;
    // encoder_msg.data = EncodeTotal;
    // encoder_pub.publish(&encoder_msg);
  }
  nh.spinOnce();
}

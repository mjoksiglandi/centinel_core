#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>


#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

#include <ESP32Servo.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#define LED_PIN 2
#define PWM_MIN 1250
#define PWM_MAX 1750

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor_sub;

rcl_publisher_t publisher, pubtf, pubimu, pubenco;
std_msgs__msg__Int32 msg_th, msg_enco;
tf2_msgs__msg__TFMessage * tf_message;
rclc_executor_t executor_pub;
rcl_timer_t timer;

Servo Motor1;
Servo Motor2;
Servo Linear;

struct Datos{
  float x;
  float z;
}Datos;

struct Motor{
  const int Left =4;
  const int Right = 17;
  const int Sens = 12;
}Motor;

const int channelPinA = 13;
const int channelPinB = 26;

int count = 0;
int countA;

const int L1 = 16;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);
Adafruit_MPU6050 IMU;

const void euler_to_quat(float x, float y, float z, double* q) {
    float c1 = cos((y*3.14/180.0)/2);
    float c2 = cos((z*3.14/180.0)/2);
    float c3 = cos((x*3.14/180.0)/2);

    float s1 = sin((y*3.14/180.0)/2);
    float s2 = sin((z*3.14/180.0)/2);
    float s3 = sin((x*3.14/180.0)/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  Datos.x = constrain(msg->linear.x, -1, 1);
  Datos.z= constrain(msg->angular.z, -1, 1);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);

}

void setup() {
  set_microros_transports();
  pinMode(L1, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Motor1.attach(Motor.Left, PWM_MIN, PWM_MAX);
  Motor2.attach(Motor.Right, PWM_MIN, PWM_MAX);
  Linear.attach(Motor.Sens, 1000, 2000);
  attachInterrupt(digitalPinToInterrupt(channelPinA), encodeA, CHANGE);
   attachInterrupt(digitalPinToInterrupt(channelPinB), encodeB, CHANGE);
    pinMode(channelPinA, INPUT_PULLUP);
    pinMode(channelPinB, INPUT_PULLUP);
  delay(2000);

  IMU.begin();
  IMU.setAccelerometerRange(MPU6050_RANGE_16_G);
  IMU.setGyroRange(MPU6050_RANGE_1000_DEG);
  IMU.setFilterBandwidth(MPU6050_BAND_21_HZ);


  allocator = rcl_get_default_allocator();
   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

      // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &pubenco,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_encoder"));
    
 RCCHECK(rclc_publisher_init_default(
    &pubtf,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));


 // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor subscriber
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // create executor publisher
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string1[] = "/base_link";
  memcpy(tf_message->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
  tf_message->transforms.data[0].header.frame_id.size = strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  char string2[] = "/inertial_unit";
  tf_message->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(tf_message->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
  tf_message->transforms.data[0].child_frame_id.size = strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;


}//end Setup


void loop() {
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rcl_publish(&publisher, tf_message, NULL));
  RCSOFTCHECK(rcl_publish(&publisher, &msg_th, NULL));
  RCSOFTCHECK(rcl_publish(&pubenco, &msg_enco, NULL));
  Move();

  
  //////// encoder count
  if(count != countA){
      countA = count;
       msg_enco.data = count;
    }

  ///////////imu/////////
  struct timespec tv = {0};
  clock_gettime(0, &tv);

   sensors_event_t a, g, temp;
  IMU.getEvent(&a, &g, &temp);

  double q[4];
  euler_to_quat(g.gyro.x, g.gyro.y, g.gyro.z, q);

  tf_message->transforms.data[0].transform.rotation.x = (double) q[1];
	tf_message->transforms.data[0].transform.rotation.y = (double) q[2];
	tf_message->transforms.data[0].transform.rotation.z = (double) q[3]; 
	tf_message->transforms.data[0].transform.rotation.w = (double) q[0];
  tf_message->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
	tf_message->transforms.data[0].header.stamp.sec = tv.tv_sec;

  

}//end Loop

void Move(){
  if (Datos.x==0 && Datos.z==0){
    Motor1.write(90);
    Motor2.write(90);
  } 
  else if (Datos.x<0 && Datos.z==0){
    Motor1.write(0);
    Motor2.write(0);
  }
  else if (Datos.x>0 && Datos.z==0 ){
    Motor1.write(180);
    Motor2.write(180);    
  }
  else if (Datos.z<0 && Datos.x == 0){
    Motor1.write(0);
    Motor2.write(180);
  }
  else if (Datos.z>0 && Datos.x == 0){
    Motor1.write(180);
    Motor2.write(0);    
  }
  else if (Datos.x < 0 && Datos.z <0){
    Motor1.write(135);
    Motor2.write(180);
  }
  else if (Datos.x < 0 && Datos.z >0){
    Motor1.write(180);
    Motor2.write(135);
  }
    else if (Datos.x > 0 && Datos.z <0){
    Motor1.write(45);
    Motor2.write(0);
  }
  else if (Datos.x > 0 && Datos.z >0){
    Motor1.write(0);
    Motor2.write(45);
  }

}

void encodeA(){
  if(digitalRead(channelPinA) == digitalRead(channelPinB)) {
    count ++;  
   }
  }
void encodeB(){
if(digitalRead(channelPinA) == digitalRead(channelPinB)) {
    count --;   
    }
  }

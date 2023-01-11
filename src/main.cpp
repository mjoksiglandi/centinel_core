#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
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

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_th;
rclc_executor_t executor_pub;
rcl_timer_t timer;

Servo Motor1;
Servo Motor2;

struct Datos{
  float x;
  float z;
}Datos;

struct Motor{
  const int Left =4;
  const int Right =17;
}Motor;

const int L1 = 16;

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
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_th, NULL));
    msg_th.data=(float)Datos.x;
  }
}

void setup() {
  set_microros_transports();
  pinMode(L1, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Motor1.attach(Motor.Left, PWM_MIN, PWM_MAX);
  Motor2.attach(Motor.Right, PWM_MIN, PWM_MAX);
  delay(2000);

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

 // create timer,
  const unsigned int timer_timeout = 1000;
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

}
void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));

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

}//end Loop

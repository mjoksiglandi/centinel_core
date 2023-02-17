#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/vector3.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>
#include <Wire.h>

#include <ESP32Servo.h>
#define LED_PIN 2
#define PWM_MIN 1250
#define PWM_MAX 1750


Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanz;

double kalAngleX, kalAngleY;  // Calculated angle using a Kalman filter
uint32_t timer2;


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { rclErrorLoop(); } \
  }

#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)


rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rcl_node_t node;

///////subscribers
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor_sub;


/////publishers
rcl_publisher_t publisher, pubenco, pubtf, imuPub, pbjoint, pbodom;
std_msgs__msg__Int32 msg_th, msg_enco;
tf2_msgs__msg__TFMessage *tf_message;
sensor_msgs__msg__Imu *imu_msg;
sensor_msgs__msg__JointState *joint;
nav_msgs__msg__Odometry *odom_msg;
geometry_msgs__msg__Vector3 gyro_cal_;
rclc_executor_t executor_pub;

Servo Luces;

struct Datos {
  float x;
  float z;
} Datos;

struct Pump {
  int agua = 27;
  int gel = 12;
} Pump;

struct Motor {
  Servo M1;
  Servo M2;
  Servo sLin;
  Servo sRad;
  const int Left = 31;
  const int Right = 32;
  const int Lineal = 25;
  const int Radial = 26;
} Motor;
const int L1 = 16;
const int luces = 14;

struct encoder {
  int enc1A = 36;  // encoder izquierda canal A
  int enc1B = 39;  // encoder izquierda canal B
  int enc2A = 34;  // encoder derecha canal A
  int enc2B = 35;  // encoder derecha canal B
  int count1 = 0;
  int count2 = 0;
} encoder;

const float g_to_accel_ = 9.81;
const float mgauss_to_utesla_ = 0.1;
const float utesla_to_tesla_ = 0.000001;
float accel_cov_ = 0.00001;
float gyro_cov_ = 0.00001;
const int sample_size_ = 40;

int count = 0;
int countA;
unsigned long long time_offset = 0;
Adafruit_MPU6050 IMU;

bool micro_ros_init_successful;

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  Datos.x = constrain(msg->linear.x, -1, 1);
  Datos.z = constrain(msg->angular.z, -1, 1);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


const void euler_to_quat(float x, float y, float z, double *q) {
  float c1 = cos((y * 3.14 / 180.0) / 2);
  float c2 = cos((z * 3.14 / 180.0) / 2);
  float c3 = cos((x * 3.14 / 180.0) / 2);

  float s1 = sin((y * 3.14 / 180.0) / 2);
  float s2 = sin((z * 3.14 / 180.0) / 2);
  float s3 = sin((x * 3.14 / 180.0) / 2);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "microRos_Node", "", &support));


  RCCHECK(rclc_publisher_init_default(
    &pubenco,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/wheel"));

  RCCHECK(rclc_publisher_init_default(
    &pubtf,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));


  RCCHECK(rclc_publisher_init_default(
    &imuPub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu_link"));

  RCCHECK(rclc_publisher_init_default(
    &pbodom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom/unfiltered"));

  // twist subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  // executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  //create executor publisher
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  odom_msg = nav_msgs__msg__Odometry__create();

  imu_msg = sensor_msgs__msg__Imu__create();
  imu_msg->header.frame_id.data = "/base_link";

  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = (char *)malloc(100 * sizeof(char));
  char string1[] = "/base_link";
  memcpy(tf_message->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
  tf_message->transforms.data[0].header.frame_id.size = strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  char string2[] = "/left_back_wheel";
  tf_message->transforms.data[0].child_frame_id.data = (char *)malloc(100 * sizeof(char));
  memcpy(tf_message->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
  tf_message->transforms.data[0].child_frame_id.size = strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor_sub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  pinMode(L1, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(Pump.agua, OUTPUT);
  pinMode(Pump.gel, OUTPUT);
  Luces.attach(luces, 1000, 2000);
  Motor.M1.attach(Motor.Left, PWM_MIN, PWM_MAX);
  Motor.M2.attach(Motor.Right, PWM_MIN, PWM_MAX);
  Motor.sLin.attach(Motor.Lineal, 1000, 2000);
  Motor.sRad.attach(Motor.Radial, 1000, 2000);
  attachInterrupt(digitalPinToInterrupt(encoder.enc1A), encodeA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder.enc1B), encodeB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder.enc2A), encodeA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder.enc2B), encodeB2, CHANGE);


  IMU.begin();
  IMU.setAccelerometerRange(MPU6050_RANGE_8_G);
  IMU.setGyroRange(MPU6050_RANGE_500_DEG);
  IMU.setFilterBandwidth(MPU6050_BAND_21_HZ);

  state = WAITING_AGENT;
}

void loop() {
  Move();
  if (encoder.count1 != countA) {
    countA = encoder.count1;
    msg_enco.data = encoder.count1;
  }


  int rot = 0;


  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        Imudat();
        // Tf_dat();
        // odometria();
        odomDat();
        struct timespec tv = { 0 };
        clock_gettime(0, &tv);

        double q[4];
        euler_to_quat(0, 0, 0, q);

        tf_message->transforms.data[0].transform.rotation.w = encoder.count1;
        tf_message->transforms.data[0].transform.rotation.x = encoder.count1;
        tf_message->transforms.data[0].transform.rotation.y = 0;
        tf_message->transforms.data[0].transform.rotation.z = 0;
        tf_message->transforms.data[0].transform.translation.x = 0.1323;
        tf_message->transforms.data[0].transform.translation.y = -0.2;
        tf_message->transforms.data[0].transform.translation.z = 0.0235;
        tf_message->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
        tf_message->transforms.data[0].header.stamp.sec = tv.tv_sec;




        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));
        rcl_publish(&pubenco, &msg_enco, NULL);
        // rcl_publish(&pubenco, &msg_enco, NULL);
        rcl_publish(&imuPub, imu_msg, NULL);
        rcl_publish(&pbodom, odom_msg, NULL);
        rcl_publish(&pubtf, tf_message, NULL);
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    // digitalWrite(estado.Led1, HIGH);
  } else {
    // digitalWrite(estado.Led1, LOW);
  }
}
void calibrateGyro() {
  geometry_msgs__msg__Vector3 gyro;

  for (int i = 0; i < sample_size_; i++) {
    sensors_event_t a, g, temp;
    IMU.getEvent(&a, &g, &temp);
    gyro_cal_.x += g.gyro.x;
    gyro_cal_.y += g.gyro.y;
    gyro_cal_.z += g.gyro.z;

    delay(50);
  }

  gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
  gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
  gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
}

void Imudat() {
  struct timespec time_stamp = getTime();

  calibrateGyro();

  sensors_event_t a, g, temp;
  IMU.getEvent(&a, &g, &temp);

  imu_msg->header.frame_id.data = "/imu_link";
  imu_msg->angular_velocity.x -= gyro_cal_.x;
  imu_msg->angular_velocity.y -= gyro_cal_.y;
  imu_msg->angular_velocity.z -= gyro_cal_.z;

  if (imu_msg->angular_velocity.x > -0.01 && imu_msg->angular_velocity.x < 0.01)
    imu_msg->angular_velocity.x = 0;

  if (imu_msg->angular_velocity.y > -0.01 && imu_msg->angular_velocity.y < 0.01)
    imu_msg->angular_velocity.y = 0;

  if (imu_msg->angular_velocity.z > -0.01 && imu_msg->angular_velocity.z < 0.01)
    imu_msg->angular_velocity.z = 0;

  imu_msg->angular_velocity_covariance[0] = gyro_cov_;
  imu_msg->angular_velocity_covariance[4] = gyro_cov_;
  imu_msg->angular_velocity_covariance[8] = gyro_cov_;

  //imu_msg->linear_acceleration = a.acceleration();
  imu_msg->linear_acceleration.x = a.acceleration.x;
  imu_msg->linear_acceleration.y = a.acceleration.y;
  imu_msg->linear_acceleration.z = a.acceleration.z;

  imu_msg->linear_acceleration_covariance[0] = accel_cov_;
  imu_msg->linear_acceleration_covariance[4] = accel_cov_;
  imu_msg->linear_acceleration_covariance[8] = accel_cov_;
  imu_msg->header.stamp.sec = time_stamp.tv_sec;
  imu_msg->header.stamp.nanosec = time_stamp.tv_nsec;
}

void Move() {
  if (Datos.x == 0 && Datos.z == 0) {
    Motor.M1.write(90);
    Motor.M2.write(90);
  } else if (Datos.x < 0 && Datos.z == 0) {
    Motor.M1.write(0);
    Motor.M2.write(0);
  } else if (Datos.x > 0 && Datos.z == 0) {
    Motor.M1.write(180);
    Motor.M2.write(180);
  } else if (Datos.z < 0 && Datos.x == 0) {
    Motor.M1.write(0);
    Motor.M2.write(180);
  } else if (Datos.z > 0 && Datos.x == 0) {
    Motor.M1.write(180);
    Motor.M2.write(0);
  } else if (Datos.x < 0 && Datos.z < 0) {
    Motor.M1.write(135);
    Motor.M2.write(180);
  } else if (Datos.x < 0 && Datos.z > 0) {
    Motor.M1.write(180);
    Motor.M2.write(135);
  } else if (Datos.x > 0 && Datos.z < 0) {
    Motor.M1.write(45);
    Motor.M2.write(0);
  } else if (Datos.x > 0 && Datos.z > 0) {
    Motor.M1.write(0);
    Motor.M2.write(45);
  }
}

void odomDat() {

  struct timespec time_stamp = getTime();

  odom_msg->header.frame_id.data = "/odom";
  odom_msg->child_frame_id.data = "/base_footprint";

  odom_msg->pose.pose.position.x = encoder.count1;
  odom_msg->pose.pose.position.y = 0.0;
  odom_msg->pose.pose.position.z = 0.0;

  odom_msg->pose.pose.orientation.w = 1.0;  //quaternion
  odom_msg->pose.pose.orientation.x = 0.0;
  odom_msg->pose.pose.orientation.y = 0.0;
  odom_msg->pose.pose.orientation.z = 0.0;

  odom_msg->pose.covariance[0] = 0.0001;
  odom_msg->pose.covariance[7] = 0.0001;
  odom_msg->pose.covariance[35] = 0.0001;

  //linear speed encoder
  odom_msg->twist.twist.linear.x = 0.3;  //velocidad encoder
  odom_msg->twist.twist.linear.y = 0.0;
  odom_msg->twist.twist.linear.z = 0.0;

  //angular speed encoder
  odom_msg->twist.twist.angular.x = 0;
  odom_msg->twist.twist.angular.y = 0;
  odom_msg->twist.twist.angular.z = 0;  //angular speed

  odom_msg->twist.covariance[0] = 0.0001;
  odom_msg->twist.covariance[7] = 0.0001;
  odom_msg->twist.covariance[35] = 0.0001;


  odom_msg->header.stamp.sec = time_stamp.tv_sec;
  odom_msg->header.stamp.nanosec = time_stamp.tv_nsec;
}

void syncTime() {
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}
struct timespec getTime() {
  struct timespec tp = { 0 };
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}
void encodeA1() {
  if (digitalRead(encoder.enc1A) == digitalRead(encoder.enc1B)) {
    encoder.count1 = encoder.count1 + 1;
  }
}
void encodeB1() {
  if (digitalRead(encoder.enc1A) == digitalRead(encoder.enc1B)) {
    encoder.count1--;
  }
}
void encodeA2() {
  if (digitalRead(encoder.enc2A) == digitalRead(encoder.enc2B)) {
    encoder.count2++;
  }
}
void encodeB2() {
  if (digitalRead(encoder.enc2A) == digitalRead(encoder.enc2B)) {
    encoder.count2--;
  }
}
void rclErrorLoop() {
  while (true) {
    //
  }
}
#include <math.h>
#include <stdlib.h>

#include "IMU.h"
#include "delay.h"
#include "drv_include.h"
#include "drv_led.h"
#include "sys.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <turtlebot3_msgs/SensorState.h>

#include "24cxx.h"
#include "CKalman.h"
#include "CMotor.h"
#include "CPackage.h"
#include "CSonar.h"
#include "DataScope_DP.h"
#include "UARTClass.h"
#include "USBSerial.h"
#include "drv_exti.h"
#include "main.h"
#include "mpu.h"
#include "mpu9250_iic.h"

#define MAIN_DEBUG
#ifdef MAIN_DEBUG
#define fprintf(f) uart1_print(f)
#else
#define fprintf(f)
#endif

// static bool flag_odom = false;
// static char flag_sensor = 0;
// USBSerial Serial;
volatile int ttt_mp = 0;

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
 * Subscriber
 *******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void InitPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped& init_pose_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel",
                                                  commandVelocityCallback);
ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> init_pose_sub(
    "initialpose", InitPoseCallback);
/*******************************************************************************
 * Publisher
 *******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

/*******************************************************************************
 * Transform Broadcaster
 *******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tfbroadcaster;

/*******************************************************************************
 * Declaration for IMU
 *******************************************************************************/
cIMU imu;
CKalman kalman;

/*******************************************************************************
 * Declaration for MOTOR
 *******************************************************************************/
double goal_linear_x_velocity = 0.0;
double goal_linear_y_velocity = 0.0;
double goal_angular_velocity = 0.0;
CMotor motor;
Speed speed_motor = {0};

static int32_t old_wheel_encoder[3] = {0};
static int32_t diff_wheel_encoder[3] = {0};
double odometry_x = 0;
double odometry_y = 0;
double odometry_th = 0;
static double last_theta = 0.0;

bool init_encoder_[2] = {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];

/*******************************************************************************
 * Declaration for Sonar
 *******************************************************************************/
CSonar sonar;
/*******************************************************************************
 * Declaration for SYSTERM
 *******************************************************************************/
static unsigned long prev_update_time = 0;
unsigned long watch_dog = 0, last_time = 0;
char ex_log_msg[256];
USBSerial Serial;
extern UARTClass Serial1;
extern UARTClass Serial2;  // Arduino Serial
uint32_t stateLed_Hz = 2;

robot_serial ros_serial;
IMU_MSG imu_info = {0};
ODOM_MSG odom_info = {0};
uint8_t ros_readbuffer[256] = {0};

/*-------------------------------------------------------------------------------------------------------------*/
/*
        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա�?
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��?
*/
double ProcessNiose_Q = 0.002;
double MeasureNoise_R = 0.02;
double KalmanFilter(const double ResrcData) {
  double R = MeasureNoise_R;
  double Q = ProcessNiose_Q;

  static double x_last;

  double x_mid = x_last;
  double x_now;

  static double p_last;

  double p_mid;
  double p_now;
  double kg;

  x_mid = x_last;  // x_last=x(k-1|k-1),x_mid=x(k|k-1)
  p_mid = p_last + Q;  // p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
  kg = p_mid / (p_mid + R);  // kgΪkalman filter��RΪ����
  x_now =
      x_mid + kg * (ResrcData - x_mid);  //���Ƴ�������ֵ

  p_now = (1 - kg) * p_mid;  //����ֵ��Ӧ��covariance

  p_last = p_now;  //����covarianceֵ
  x_last = x_now;  //����ϵͳ״ֵ̬

  return x_now;
}

/*-------------------------------------------------------------------------------------------------------------*/

void system_Init(void) {
  Cache_Enable();  //��L1-Cache
  HAL_Init();      //��ʼ��HAL��
  Stm32_Clock_Init(432, 25, 2, 9);  //����ʱ��,216Mhz
  delay_init(216);  //��ʱ��ʼ��

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

/*******************************************************************************
 * Callback function for cmd_vel msg
 *******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  int32_t f_wheel_value[3] = {0, 0, 0};
  double wheel_angular_velocity[3] = {0.0, 0.0, 0.0};

  goal_linear_x_velocity = cmd_vel_msg.linear.x;
  goal_linear_y_velocity = cmd_vel_msg.linear.y;
  goal_angular_velocity = cmd_vel_msg.angular.z;

  // right
  wheel_angular_velocity[RIGHT_CH] =
      (goal_linear_x_velocity * (sqrt((double)3.0) / (2 * WHEEL_RADIUS))) +
      (goal_linear_y_velocity * (1 / (2 * WHEEL_RADIUS))) +
      (goal_angular_velocity * (DISTANCE_CENTER_TO_WHEEL / WHEEL_RADIUS));
  // left
  wheel_angular_velocity[LEFT_CH] =
      (goal_linear_x_velocity * (sqrt((double)3.0) / (-2 * WHEEL_RADIUS))) +
      (goal_linear_y_velocity * (1 / (2 * WHEEL_RADIUS))) +
      (goal_angular_velocity * (DISTANCE_CENTER_TO_WHEEL / WHEEL_RADIUS));
  // back
  wheel_angular_velocity[BACK_CH] =
      (goal_linear_x_velocity * 0) +
      (goal_linear_y_velocity * (-1 / WHEEL_RADIUS)) +
      (goal_angular_velocity * (DISTANCE_CENTER_TO_WHEEL / WHEEL_RADIUS));

  for (int id = 0; id < 3; id++) {
    f_wheel_value[id] = wheel_angular_velocity[id] * 9.54 / RPM_CONSTANT_VALUE;

    if (f_wheel_value[id] > LIMIT_X_MAX_VALUE)
      f_wheel_value[id] = LIMIT_X_MAX_VALUE;
    else if (f_wheel_value[id] < -LIMIT_X_MAX_VALUE)
      f_wheel_value[id] = -LIMIT_X_MAX_VALUE;
  }

  speed_motor.v_motor1 = f_wheel_value[0];
  speed_motor.v_motor2 = f_wheel_value[1];
  speed_motor.v_motor3 = f_wheel_value[2];

  //   sprintf(ex_log_msg, "M1:%f   M2:%f
  //   M3:%f",speed_motor.v_motor1,speed_motor.v_motor2,speed_motor.v_motor3);
  //   nh.loginfo(ex_log_msg);
  //  pidflag = true;
  watch_dog = millis();
}

void InitPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped& init_pose_msg) {
  odometry_x = init_pose_msg.pose.pose.position.x;
  odometry_y = init_pose_msg.pose.pose.position.y;
  odometry_th = atan2f(init_pose_msg.pose.pose.orientation.y *
                               init_pose_msg.pose.pose.orientation.z +
                           init_pose_msg.pose.pose.orientation.x *
                               init_pose_msg.pose.pose.orientation.w,
                       0.5f -
                           init_pose_msg.pose.pose.orientation.z *
                               init_pose_msg.pose.pose.orientation.z -
                           init_pose_msg.pose.pose.orientation.w *
                               init_pose_msg.pose.pose.orientation.w);

  imu.filter.q0 = init_pose_msg.pose.pose.orientation.w;
  imu.filter.q1 = init_pose_msg.pose.pose.orientation.y;
  imu.filter.q2 = init_pose_msg.pose.pose.orientation.y;
  imu.filter.q3 = init_pose_msg.pose.pose.orientation.z;

  last_theta = odometry_th;
}

/*******************************************************************************
 * Start Gyro Calibration
 *******************************************************************************/
void updateGyroCali(void) {
  static bool gyro_cali = false;
  uint32_t pre_time;
  uint32_t t_time;

  char log_msg[50];

  if (nh.connected()) {
    if (gyro_cali == false) {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      imu.SEN.gyro_cali_start();

      t_time = millis();
      pre_time = millis();
      while (!imu.SEN.gyro_cali_get_done()) {
        imu.update();

        if (millis() - pre_time > 5000) {
          break;
        }
        if (millis() - t_time > 100) {
          t_time = millis();
          // LED_TOGGLE(1);
        }
      }
      gyro_cali = true;

      sprintf(log_msg, "Calibrattion End");
      nh.loginfo(log_msg);
    }
  } else {
    gyro_cali = false;
  }
}

/*******************************************************************************
 * Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
 *******************************************************************************/
void publishSensorStateMsg(void) {
  bool dxl_comm_result = false;

  //  int32_t current_tick;
  int32_t new_encoder[3];
  sensor_state_msg.stamp = nh.now();

  float out;
  out = (float)getBatteryVol() / 20.408163;

  sensor_state_msg.battery = out;  // checkVoltage();

  // dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder,
  // sensor_state_msg.right_encoder);
  dxl_comm_result =
      motor.readEncoder(new_encoder[0], new_encoder[1], new_encoder[2]);
  if (dxl_comm_result == true) {
    sensor_state_pub.publish(&sensor_state_msg);
  } else {
    return;
  }

  diff_wheel_encoder[0] = new_encoder[0] - old_wheel_encoder[0];
  diff_wheel_encoder[1] = new_encoder[1] - old_wheel_encoder[1];
  diff_wheel_encoder[2] = new_encoder[2] - old_wheel_encoder[2];

  old_wheel_encoder[0] = new_encoder[0];
  old_wheel_encoder[1] = new_encoder[1];
  old_wheel_encoder[2] = new_encoder[2];
}

/*******************************************************************************
 * Calculate the odometry
 *******************************************************************************/
bool updateOdometry(double diff_time) {
  float angle = 0.0;
  //  float angle_kalman = 0.0;

  float wheel_dis[3];
  float x, y, th;
  wheel_dis[RIGHT_CH] = diff_wheel_encoder[RIGHT_CH] * TICK2RAD;
  wheel_dis[LEFT_CH] = diff_wheel_encoder[LEFT_CH] * TICK2RAD;
  wheel_dis[BACK_CH] = diff_wheel_encoder[BACK_CH] * TICK2RAD;
  x = (wheel_dis[RIGHT_CH] - wheel_dis[LEFT_CH]) / 1.732;
  y = (-2 * wheel_dis[BACK_CH] + wheel_dis[RIGHT_CH] + wheel_dis[LEFT_CH]) /
      3.0;
  // th = (wheel_dis[0]+wheel_dis[1]+wheel_dis[2])/(3*DISTANCE_CENTER_TO_WHEEL);

  angle = atan2f(imu.quat[1] * imu.quat[2] + imu.quat[0] * imu.quat[3],
                 0.5f - imu.quat[2] * imu.quat[2] - imu.quat[3] * imu.quat[3]);
  // angle_kalman = kalman.update(angle);
  th = angle - last_theta;

  odometry_th += th;
  odometry_x += (x * cos(odometry_th) - y * sin(odometry_th));
  odometry_y += (y * cos(odometry_th) + x * sin(odometry_th));
#if 0
  if(odometry_th > 2*PI)
    odometry_th -= 2*PI;
  if(odometry_th < -2*PI)
    odometry_th += 2*PI;
#endif
  // position
  odom.pose.pose.position.x = odometry_x;
  odom.pose.pose.position.y = odometry_y;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odometry_th);

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  double v_motor[3];
  for (int i = 0; i < 3; i++) {
    v_motor[i] = motor.speed[i] * 0.00304;
  }
  // double dt = (current_time - last_time).toSec();
  vx = (v_motor[RIGHT_CH] - v_motor[LEFT_CH]) / 1.732;
  vy = (2 * v_motor[BACK_CH] - v_motor[RIGHT_CH] - v_motor[LEFT_CH]) / 3.0;
  vth = (v_motor[0] + v_motor[1] + v_motor[2]) / (3 * DISTANCE_CENTER_TO_WHEEL);

  // velocity
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = vth;

  odom.pose.covariance[0] = float(ttt_mp * 1.0);
  odom.pose.covariance[7] = 0.1;
  odom.pose.covariance[14] = 999999;
  odom.pose.covariance[21] = 999999;
  odom.pose.covariance[28] = 999999;
  odom.pose.covariance[35] = 0.05;

  last_theta = angle;
  return true;
}

/*******************************************************************************
 * Calculate the TF
 *******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf) {
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
 * Publish msgs (odometry, joint states, tf)
 *******************************************************************************/
void publishDriveInformation(void) {
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = nh.now();

  // odom
  updateOdometry((double)(step_time * 0.001));
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // tf
  updateTF(odom_tf);
  tfbroadcaster.sendTransform(odom_tf);
}

void publishImuMsg(void) {
  // short ax,ay,az;
  // short gx,gy,gz;
  // MPU_Get_Accelerometer(&ax,&ay,&az);
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = imu.SEN.gyroADC[0];
  imu_msg.angular_velocity.y = imu.SEN.gyroADC[1];
  imu_msg.angular_velocity.z = imu.SEN.gyroADC[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

#if 1
  imu_msg.linear_acceleration.x = imu.SEN.accADC[0];
  imu_msg.linear_acceleration.y = imu.SEN.accADC[1];
  imu_msg.linear_acceleration.z = imu.SEN.accADC[2];
#else
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
#endif
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[1];
  imu_msg.orientation.y = imu.quat[2];
  imu_msg.orientation.z = imu.quat[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);

  tfs_msg.header.stamp = nh.now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id = "imu_link";
  tfs_msg.transform.rotation.w = 1.0;  // imu.quat[0];
  tfs_msg.transform.rotation.x = 0.0;  // imu.quat[1];
  tfs_msg.transform.rotation.y = 0.0;  // imu.quat[2];
  tfs_msg.transform.rotation.z = 0.0;  // imu.quat[3];

  tfs_msg.transform.translation.x = -0.032;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.068;

  tfbroadcaster.sendTransform(tfs_msg);
}

void loop() {
  static uint64_t sonar_update_time = 0;
  static uint64_t sonar_update_hz = 1;
  static uint64_t current_time = 0;
  current_time = millis();
  if ((current_time - sonar_update_time) >= (1000 / sonar_update_hz - 30)) {
    sonar.update();
    if ((current_time - sonar_update_time) >= (1000 / sonar_update_hz)) {
      sonar_update_time = current_time;
      sonar.upload();
      LED_TOGGLE(3);
    }
  }
}

void timer_Handle(void) {
  //	static uint8_t timer_count = 0;
  //	timer_count++;

  __disable_irq();
  motor.speed[0] = velocity_calculate(&motor.omni_wheel[0]);
  motor.speed[1] = velocity_calculate(&motor.omni_wheel[1]);
  motor.speed[2] = velocity_calculate(&motor.omni_wheel[2]);
  __enable_irq();
  motor.setSpeed(speed_motor.v_motor1, speed_motor.v_motor2,
                 speed_motor.v_motor3);
  motor.FeedbackSpeed(motor.speed[0], motor.speed[1], motor.speed[2]);
  motor.controller();
}

extern "C" {
// interrupt function of encoder1
void Encoder_Counter_1() { Count_and_Direction(&motor.omni_wheel[0]); }

// interrupt function of encoder2
void Encoder_Counter_2() { Count_and_Direction(&motor.omni_wheel[1]); }

// interrupt function of encoder3
void Encoder_Counter_3() {
  // LED_TOGGLE(1);
  Count_and_Direction(&motor.omni_wheel[2]);
}
}

int main(void) {
  /*************************************************************
                                                  Hardware Init
  ***************************************************************/
  bsp_init();
  drv_uart_init();

  drv_spi_init();
  drv_micros_init();
  drv_Led_Init();
  USB_Init();

/*************************************************************
                                                Ros Init
***************************************************************/
#if 0
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	
	nh.subscribe(cmd_vel_sub);
	nh.subscribe(init_pose_sub);
	nh.advertise(imu_pub);
	nh.advertise(odom_pub);
	nh.advertise(sensor_state_pub);
	tfbroadcaster.init(nh);
#endif
/*************************************************************
                                                Serial Init
***************************************************************/
#if 1
  Serial1.begin((uint32_t)115200);
#endif
/*************************************************************
                                                Motor Drive Init
***************************************************************/
#if 0
	drv_motor_pwm_init();
	motor.Init();
	speed_motor.v_motor1 = 0.0;
	speed_motor.v_motor2 = 0.0;
	speed_motor.v_motor3 = 0.0;
#endif
  /*************************************************************
                                                  Motor Encoder Init
  ***************************************************************/

  drv_exti_init();
#if 0
	pinMode(80,INPUT);
	pinMode(81,INPUT);
	pinMode(82,INPUT);
	pinMode(83,INPUT);
	pinMode(84,INPUT);
	pinMode(85,INPUT);
	pinMode(86,INPUT);
	pinMode(87,INPUT);
  
	attachInterrupt(2, Encoder_Counter_1, RISING);
	attachInterrupt(4, Encoder_Counter_2, RISING);
	attachInterrupt(6, Encoder_Counter_3, RISING);
#endif

#if 1
  if (imu.begin() == IMU_OK) {
    kalman.setMeasureNoise_R(0.002);
    kalman.setMeasureNoise_R(0.2);
    BEEP_LONG;
  } else {
    BEEP_ONCE;
  }
#else
  if (MPU9250_Init() == 0) {
    BEEP_LONG;
  } else {
    BEEP_ONCE;
  }
#endif

/*************************************************************
                                                Battery Init
***************************************************************/
#if 0
  drv_battery_init();
#endif

/*************************************************************
                                                Timer Init
***************************************************************/
#if 0
  drv_timer_init();
	drv_timer_pause(TIMER_CH2);
	drv_timer_set_period(TIMER_CH2,60000);
	drv_timer_attachInterrupt(TIMER_CH2,timer_Handle);
	drv_timer_resume(TIMER_CH2);
#endif

/*************************************************************
                                                Sonar Init
***************************************************************/
#if 1
  sonar.Init(&Serial1);
#endif
  prev_update_time = millis();

  while (1) {
    loop();
  }
}

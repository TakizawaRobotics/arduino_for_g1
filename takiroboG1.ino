/*======================================================================
  Project Name    : arduino_for_g1
  File Name       : takiroboG1.ino
  Encoding        : utf-8
  Creation Date   : c++
  author          : Hayato Tajiri
  update date     : 2021/11/16

  Copyright © <2021> TakizawaRoboticsLLC. All rights reserved.

  This source code or any portion thereof must not be
  reproduced or used in any manner whatsoever.
  ======================================================================*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <VescUart.h>

/*定数*/
#define BASE_WIDTH  (0.6) //(m)トレッド幅
#define WHEEL_SIZE  (0.2) //(m)8インチタイヤ
#define MOTOR_POLES  (15.0) //極数
#define MOTOR_STEPS  (4.0) //4°

/*フレーム名*/
const char *odom_frame = "/g1_odom";
const char *base_frame = "/g1_base_footprint";

/*ROSのインスタンス*/
ros::NodeHandle nh;
ros::Time current_time, last_time;

/*インホイールモータのインスタンス*/
VescUart LEFT_MOTOR;
VescUart RIGHT_MOTOR;

/*ROSメッセージの型*/
nav_msgs::Odometry odom_msg;
tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::TransformStamped odom_tf;

/*グローバル変数*/
volatile uint8_t flag_firsttime = 1;

/*プロトタイプ宣言*/
void callBackMotorControl(const geometry_msgs::Twist& twist);
void callBackResetFlag(const std_msgs::Empty& flag);
void motorDriverSetSpeed(const float vel_left, const float vel_right);
void getOdometry(void);
geometry_msgs::Quaternion createQuaternionFromRPY(float roll, float pitch, float yaw);

/*ROS Publisherのインスタンス*/
/*トピック名:"g1/odometry"*/
ros::Publisher odomPublisher("g1/odometry", &odom_msg);

/*ROS Subscriberのインスタンス*/
/*トピック名:"g1/cmd_vel"*/
ros::Subscriber<geometry_msgs::Twist> twistSubscriber("g1/cmd_vel", &callBackMotorControl);
ros::Subscriber<std_msgs::Empty> resetSubscriber("g1/position_reset", &callBackResetFlag);

/////////////
/*初期化設定*/
////////////
void setup()
{
  /*ROSの初期化*/
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  odom_broadcaster.init(nh);
  nh.advertise(odomPublisher);
  nh.subscribe(twistSubscriber);
  nh.subscribe(resetSubscriber);
  odom_tf.transform.rotation.w = 1.0;

  /** Setup Serial port to enter commands */
  Serial.begin(9600);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial2.begin(115200);//Left Motor Driver
  Serial3.begin(115200);//Right Motor Driver

  while (!Serial2) {
    ;
  }
  while (!Serial3) {
    ;
  }

  /** Define which ports to use as UART */
  LEFT_MOTOR.setSerialPort(&Serial2);
  RIGHT_MOTOR.setSerialPort(&Serial3);
}

/////////
/*メイン*/
/////////
void loop()
{
  getOdometry();
  nh.spinOnce();
  delay(1);
}

///////////////////////////////////////////////////////////////////////////
/*callback(トピックが飛んできた段階でモータに指令を出す)*/
///////////////////////////////////////////////////////////////////////////
void callBackMotorControl(const geometry_msgs::Twist& twist)
{
  /*Twist 情報の取得*/\
  char *buf = "";
  const float linear_x = twist.linear.x;//前後の移動方向
  const float angle_z = twist.angular.z;//z軸を中心としたときの回転、つまり旋回

  /*モーター速度の計算*/
  float vel_left_speed =  (angle_z * BASE_WIDTH - 2 * linear_x) / (-2.0);
  float vel_right_speed = (angle_z * BASE_WIDTH + 2 * linear_x) / 2.0;
  vel_left_speed /= (WHEEL_SIZE * 2.0 * PI);
  vel_right_speed /= (WHEEL_SIZE * 2.0 * PI);

  /*左右のモーターのスピードをセットする*/
  motorDriverSetSpeed(vel_left_speed, vel_right_speed);

  /*受信とモーターのスピートをセット出来たら成功*/
  sprintf(buf, "set twist:( %f, %f )", vel_left_speed, vel_right_speed);
  nh.loginfo(buf);
}

///////////////////////////////////////
/*速度からrpmに変換しモーターを回転させる*/
///////////////////////////////////////
void motorDriverSetSpeed(float vel_left, float vel_right)
{
  /*0.2m = Wheel diameter, 15 = motor poles*/
  const float vel_left_rpm = vel_left / (WHEEL_SIZE * PI) * 60.0 * MOTOR_POLES;
  const float vel_right_rpm = vel_right / (WHEEL_SIZE * PI) * 60.0 * MOTOR_POLES;

  /*モーターの回転数をセットする*/
  LEFT_MOTOR.setRPM((float)vel_left_rpm);
  RIGHT_MOTOR.setRPM((float)vel_right_rpm);
}

////////////////////
/*初期位置のリセット*/
///////////////////
void callBackResetFlag(const std_msgs::Empty& flag)
{
  flag_firsttime = 1;
  return;
}

//////////////////////////////////////
/*オドメトリ情報を取得してトピックに入れる*/
//////////////////////////////////////
void getOdometry(void)
{
  /*ローカル変数　一時的に位置情報と速度情報を保存しておく*/
  static long LEFT_MOTOR_tacho_old = 0;
  static long RIGHT_MOTOR_tacho_old = 0;
  double LEFT_MOTOR_odom = 0;
  double RIGHT_MOTOR_odom = 0;
  float LEFT_MOTOR_velocity = 0;
  float RIGHT_MOTOR_velocity = 0;
  double distance = 0;
  double theta = 0;
  static double distance_old = 0;
  static double theta_old = 0;
  float d_x, d_y, d_t;
  static double pos_x = 0;
  static double pos_y = 0;
  geometry_msgs::Quaternion odom_quat;

  if (LEFT_MOTOR.getVescValues() && RIGHT_MOTOR.getVescValues()) {
    /* getValues data struct
      float tempFET;
      float tempMotor;
      float avgMotorCurrent;
      float avgInputCurrent;
      float dutyCycleNow;
      long rpm;
      float inpVoltage;
      float ampHours;
      float ampHoursCharged;
      long tachometer;
      long tachometerAbs;
    */

    ///////////////////////
    /*スタート位置のリセット*/
    //////////////////////
    if ( flag_firsttime )
    {
      flag_firsttime = 0;

      LEFT_MOTOR_tacho_old = LEFT_MOTOR.data.tachometer;
      RIGHT_MOTOR_tacho_old = RIGHT_MOTOR.data.tachometer;
      distance_old = 0;
      theta_old = 0;
      pos_x = 0;
      pos_y = 0;
      last_time = nh.now();
      return;
    }

    /*現在の時間を保存する*/
    current_time = nh.now();
    d_t = current_time.toSec() - last_time.toSec();

    ///////////////////////////////////
    /*距離と角度の計算*/
    /*R = Rモーター回転角度×車輪直径×π÷360
      　  L = Lモーター回転角度×車輪直径×π÷360
      　  d = (L+R)÷2
      　  θ= atan((R-L)÷W)
      このモーターは4°=360/90毎に回転量が取れる*/
    ///////////////////////////////////
    LEFT_MOTOR_odom = ((float)(LEFT_MOTOR.data.tachometer  -  LEFT_MOTOR_tacho_old) * MOTOR_STEPS) * WHEEL_SIZE * PI / 360.0;
    RIGHT_MOTOR_odom = ((float)(RIGHT_MOTOR.data.tachometer - RIGHT_MOTOR_tacho_old) * MOTOR_STEPS) * WHEEL_SIZE * PI / 360.0;
    distance = (LEFT_MOTOR_odom + RIGHT_MOTOR_odom) / 2.0;
    theta = (RIGHT_MOTOR_odom - LEFT_MOTOR_odom) / BASE_WIDTH;

    //////////////////////////////
    /*直交座標の位置の計算
      dx = velocity*cosθ
      dy = velocity*sinθ
      xt+1=xt+x˙Δtcos(θt+θ˙Δt/2)
      yt+1=yt+y˙Δtsin(θt+θ˙Δt/2)*/
    /////////////////////////////
    /*pos_x += (dx * cos(theta) - _y * sin(theta));
      pos_y += (dx * sin(theta) + _y * cos(theta));*/
    d_x = (distance - distance_old) * cos(theta);
    d_y = (distance - distance_old) * sin(theta);
    pos_x += d_x * d_t * cos(theta + ((theta - theta_old) * d_t / 2.0));
    pos_y += d_y * d_t * sin(theta + ((theta - theta_old) * d_t / 2.0));

    /*クオータニオンに変換する*/
    odom_quat = createQuaternionFromRPY(0, 0, theta);
    /*odom_quat.x = 0.0;
      odom_quat.y = 0.0;
      odom_quat.z = 0.0;
      odom_quat.w = 1.0;*/

    ///////////
    /*TFの排出*/
    ///////////
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = odom_frame;
    odom_tf.child_frame_id = base_frame;

    /*位置変換情報を入れる*/
    odom_tf.transform.translation.x = pos_x;
    odom_tf.transform.translation.y = pos_y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;

    ////////////////////////
    /*Odometryトピックの排出*/
    ////////////////////////
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    /*位置情報を入れる*/
    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    /*速度情報を入れる
      velocity = (left_velocity + right_velosity) / 2
      ω = (right_velocity - left_velocity) / トレッド幅*/
    LEFT_MOTOR_velocity =  (LEFT_MOTOR.data.rpm / MOTOR_POLES) * WHEEL_SIZE * PI / 60.0;
    RIGHT_MOTOR_velocity = (RIGHT_MOTOR.data.rpm / MOTOR_POLES) * WHEEL_SIZE * PI / 60.0;
    odom_msg.twist.twist.linear.x = (LEFT_MOTOR_velocity + RIGHT_MOTOR_velocity) / 2.0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.z = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = (RIGHT_MOTOR_velocity - LEFT_MOTOR_velocity) / BASE_WIDTH;

    ////////////////////
    /*トピックとtfを送る*/
    ///////////////////
    odomPublisher.publish(&odom_msg);
    odom_broadcaster.sendTransform(odom_tf);

    /*現在の時間を保存しておくことによって速度情報を計算する*/
    last_time = current_time;
    distance_old = distance;
    theta_old = theta;

    return;
  }
}

///////////////////////////////
/*RPYからクオータニオンを計算する*/
//////////////////////////////
geometry_msgs::Quaternion createQuaternionFromRPY(float roll, float pitch, float yaw)
{
  geometry_msgs::Quaternion ret;

  float c1 = cos(pitch / 2);
  float c2 = cos(yaw / 2);
  float c3 = cos(roll / 2);

  float s1 = sin(pitch / 2);
  float s2 = sin(yaw / 2);
  float s3 = sin(roll / 2);

  ret.x = c1 * c2 * c3 + s1 * s2 * s3;
  ret.y = c1 * c2 * s3 - s1 * s2 * c3;
  ret.z = s1 * c2 * c3 + c1 * s2 * s3;
  ret.z = c1 * s2 * c3 - s1 * c2 * s3;
  return ret;
}

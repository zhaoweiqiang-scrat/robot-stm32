#include <stdio.h>
#include "hardwareserial.h"
#include "gy85.h"
#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "led.h"
#include "PID.h"
#include "Kinematics.h"
#include <ros.h>
#include <riki_msgs/Velocities.h>
#include <geometry_msgs/Twist.h>
#include <riki_msgs/PID.h>
#include <riki_msgs/Imu.h>
#include <riki_msgs/Battery.h>
#include <riki_msgs/Humiture.h>
#include <riki_msgs/Ultrasonic.h>
#include <riki_msgs/Illuminance.h>
#include <riki_msgs/Infrared.h>
#include <riki_msgs/Collision.h>
#include <riki_msgs/Lifter.h>
#include <riki_msgs/LED.h>
#include <riki_msgs/Angle.h>
#include <riki_msgs/Charge.h>
#include <riki_msgs/Action.h>
#include <riki_msgs/DetResult.h>
#include <std_msgs/Char.h>

#include <std_srvs/SetBool.h>
#include <riki_srvs/SetCmd.h>

#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include "usart.h"
//#include "bsp_mpu6050.h"
//#include "bsp_i2c_gpio.h"
#include "GYMPU680.h"
#include "ks103.h"
#include "JXBS3001GZ.h"
#include "IR.h"
#include "mcu2yt.h"
#include "mcu2dy.h"
#include "delay.h"
#include "IR_Receive.h"
#include "IR_Send.h"
#include "fuzzypid .h"
#include "mcu2ult.h"

#define H_ANGLE_ADJUSTMENT 0  //103-- -25  //105--- -7
#define V_ANGLE_ADJUSTMENT 0  //103-- 0   //105--- -20
#define LINEAR_ADJUSTMENT 1.0//41.993811615078//40.64048351520634
#define ANGULAR_ADJUSTMENT  1.0//18.119978674289//20.37179682044995

double required_angular_vel = 0.0;
double required_linear_vel = 0.0;
uint32_t previous_command_time = 0;

float encoder_count1;
float encoder_count2;
bool is_first = true;
bool accel, gyro, mag,zero_flag;
bool ROBOT_ACTION;
bool RESET_FLAG;
#define K_P    3.08//3.08 // P constant 0.08
#define K_I    5.15//1.15 // I constant 0.15
#define K_D    1.91//1.91 // D constant 0.01


PID motor1_pid(-999, 999, K_P, K_I, K_D);
PID motor2_pid(-999, 999, K_P, K_I, K_D);

Motor motor1(MOTOR1, 999, 5);
Motor motor2(MOTOR2, 999, 5);

Encoder encoder1(ENCODER1, 0xffff, 0, COUNTS_PER_REV);
Encoder encoder2(ENCODER2, 0xffff, 0, COUNTS_PER_REV);
Battery bat(25, 10.6, 12.6);
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);
HardwareSerial Serial1(SERIAL1);
HardwareSerial Serial3(SERIAL3);
HardwareSerial Serial4(SERIAL4);


Gy85  imu;
Led led;

void pid_callback( const riki_msgs::PID& pid);
void command_callback( const geometry_msgs::Twist& cmd_msg);
void lifter_callback( const riki_msgs::Lifter& Lifter);
void led_callback(const riki_msgs::LED& led);
void angle_callback(const riki_msgs::Angle& angle);
void action_callback(const riki_msgs::Action& action);
void det_callback(const riki_msgs::DetResult& det);
void lifter_auto_callback(const riki_msgs::Action& action);
void yt_auto_callback(const riki_msgs::Action& action);
void reset_yt_lifter(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void lifter_auto_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res);
void yt_auto_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res);
void em_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res);
void el_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res);


ros::NodeHandle  nh;

riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;
riki_msgs::Battery raw_battery_msg;
riki_msgs::Humiture raw_humiture_msg;
riki_msgs::Ultrasonic raw_ultrasonic_msg;
riki_msgs::Illuminance raw_illuminance_msg;
riki_msgs::Infrared infrared_obstacle_msg;
riki_msgs::Collision hardware_collision_msg;
riki_msgs::Lifter lifter_height_msg;
riki_msgs::Angle yt_angle_msg;
riki_msgs::Charge charge_pos_msg;
std_msgs::Char shut_down_msg;
//riki_msgs::DetResult det_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Subscriber<riki_msgs::Lifter> lifter_sub("lifter_height", lifter_callback);
//ros::Subscriber<riki_msgs::Action> lifter_auto_sub("lifter_auto", lifter_auto_callback);
ros::Subscriber<riki_msgs::LED> led_sub("led_level", led_callback);
ros::Subscriber<riki_msgs::Angle> angle_sub("angle", angle_callback);
//ros::Subscriber<riki_msgs::Action> yt_auto_sub("angle_auto", yt_auto_callback);
ros::Subscriber<riki_msgs::Action> action_sub("robot_action", action_callback);
ros::Subscriber<riki_msgs::DetResult> det_sub("det_result", det_callback);

ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
//ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);
ros::Publisher raw_humiture_pub("humiture", &raw_humiture_msg);
ros::Publisher raw_ultrasonic_pub("ultrasonic", &raw_ultrasonic_msg);
ros::Publisher raw_illuminance_pub("illuminance", &raw_illuminance_msg);
ros::Publisher infrared_obstacle_pub("infrared", &infrared_obstacle_msg);
ros::Publisher hardware_collision_pub("collision", &hardware_collision_msg);
ros::Publisher lifter_height_pub("lifter", &lifter_height_msg);
ros::Publisher yt_angle_pub("yt_angle", &yt_angle_msg);
ros::Publisher charging_point_pub("charging_point", &charge_pos_msg);
ros::Publisher shut_down_pub("shut_down",&shut_down_msg);

//ros::ServiceServer<std_srvs::SetBool::Request,std_srvs::SetBool::Response> reset_server("reset",&reset_yt_lifter);
ros::ServiceServer<riki_srvs::SetCmd::Request,riki_srvs::SetCmd::Response> lifter_server("lifter_auto_srv",&lifter_auto_fun);
ros::ServiceServer<riki_srvs::SetCmd::Request,riki_srvs::SetCmd::Response> yt_server("yt_auto_srv",&yt_auto_fun);
ros::ServiceServer<riki_srvs::SetCmd::Request,riki_srvs::SetCmd::Response> em_server("em_srv",&em_fun);//电磁铁
ros::ServiceServer<riki_srvs::SetCmd::Request,riki_srvs::SetCmd::Response> el_server("el_srv",&el_fun);//继电器
void move_base();
void pid_callback( const riki_msgs::PID& pid)
{
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
    ROBOT_ACTION = 1;
    required_linear_vel = -cmd_msg.linear.x;//*40.64048351520634;//45.055968420371;//41.98
		required_angular_vel = -cmd_msg.angular.z;//*20.37179682044995;//22.77576019211;
    previous_command_time = millis();
}
uint16_t Pulse_Cnt;
uint8_t height[4];
uint8_t Recevie_Lifter,Recevie_Angle,Lifter_Ok,Angle_Ok;
float Lif_H,V_A,H_A;
uint8_t Lifter_Auto_Up_Flag,Lifter_Auto_Down_Flag,Lifter_Auto_Manual_Flag;//自动向上、向下标志、手动标志
float Lif_Cur;
void lifter_callback( const riki_msgs::Lifter& Lifter)
{
	  uint8_t Home_Cmd[3] = {0XFF,0XCC,0XBB};
	
		printf("接收到的升降杆高度为：%fmm\r\n",Lifter.height);
		if(Lifter.height > 800.0)
			Lif_H = 800.0;
	  else if(Lifter.height < 10.0) 
		{
			Lif_H = 10.0;
			//MCU2YT(Home_Cmd,3);//接收到小于10的命令时，升降杆收缩，同时发送云台归位的命令
		}	
	  else 
			Lif_H = Lifter.height;
		Pulse_Cnt = (uint16_t)(Lif_H*28);
	  if(Pulse_Cnt > 22500)
			Pulse_Cnt = 22500;
	  height[0] = 0x5A;
	  height[1] = 0xA5;
		height[2] = Pulse_Cnt>>8;
		height[3] = Pulse_Cnt&0xff;
		Recevie_Lifter = 1;//接收到升降杆命令
	  Height_Flag1 = 1;//接收到命令
		Height_Flag2 = 0;
		Lifter_Ok = 0;
		
		Lifter_Auto_Up_Flag = 0;
		Lifter_Auto_Down_Flag = 0;
		Lifter_Auto_Manual_Flag = 0;
		
	  MCU2YT(height,4);
}

void lifter_auto_callback(const riki_msgs::Action& action)
{
	uint8_t Auto_Up_Cmd[3] = {0XFF,0XBB,0XAA};
	uint8_t Auto_Down_Cmd[3] = {0XFF,0XAA,0X99};
	uint8_t Manual_Cmd[3] = {0XFF,0X99,0X88};
	
	Lif_Cur = Height;//获取当前高度值
	
	Recevie_Lifter = 0;//清除手动命令
	Lifter_Ok = 1;//清除手动命令
	
	if(action.robot_action == 1)//自动上升
	{
		Lifter_Auto_Up_Flag = 1;
		Lifter_Auto_Down_Flag = 0;
		Lifter_Auto_Manual_Flag = 0;
		MCU2YT(Auto_Up_Cmd,3);
	}
	else if(action.robot_action == 2)//自动下降
	{
		Lifter_Auto_Up_Flag = 0;
		Lifter_Auto_Down_Flag = 1;
		Lifter_Auto_Manual_Flag = 0;
		MCU2YT(Auto_Down_Cmd,3);
	}
	else if(action.robot_action == 3)//手动控制
	{
		Lifter_Auto_Up_Flag = 0;
		Lifter_Auto_Down_Flag = 0;
		Lifter_Auto_Manual_Flag = 1;
		MCU2YT(Manual_Cmd,3);
	}
	if(Lifter_Auto_Up_Flag || Lifter_Auto_Down_Flag)
	{
		Recevie_Lifter = 0;//清除手动命令
		Lifter_Ok = 1;//清除手动命令
	}
}
uint8_t led_l[6];
typedef union {
        float real;
        uint8_t base[4];
      } yt_led;
yt_led YT_LED;
u32 Light_Pre;
bool Led_Flag = false;
void led_callback(const riki_msgs::LED& led)
{
	YT_LED.real = led.led_level;
	Light_Pre = Light;
	Led_Flag = 1;
	led_l[0] = 0xcf;
	led_l[1] = 0x01;
	led_l[2] = YT_LED.base[0];
	led_l[3] = YT_LED.base[1];
	led_l[4] = YT_LED.base[2];
	led_l[5] = YT_LED.base[3];
	MCU2YT(led_l,6);
}
typedef union {
        float real;
        uint8_t base[4];
      } ANGLE;
ANGLE V_ANGLE,H_ANGLE;

uint8_t angle_value[10];
uint8_t CW_Auto_Flag,CCW_Auto_Flag,Stop_Auto_Flag;//自动向上、向下标志、手动标志
float H_V_Cur;
uint8_t first_flag = 0;
float Last_H,Last_V;
void angle_callback(const riki_msgs::Angle& angle)
{
	V_ANGLE.real = angle.V_Angle + V_ANGLE_ADJUSTMENT;
	H_ANGLE.real = angle.H_Angle + H_ANGLE_ADJUSTMENT;
	
	V_A = angle.V_Angle + V_ANGLE_ADJUSTMENT;
	H_A = angle.H_Angle + H_ANGLE_ADJUSTMENT;
  if(H_A >= 360){
		H_A -= 360;
	}
	angle_value[0] = 0xcf;
	angle_value[1] = 0x05;
	angle_value[2] = V_ANGLE.base[0];
	angle_value[3] = V_ANGLE.base[1];
	angle_value[4] = V_ANGLE.base[2];
	angle_value[5] = V_ANGLE.base[3];

	angle_value[6] = H_ANGLE.base[0];
	angle_value[7] = H_ANGLE.base[1];
	angle_value[8] = H_ANGLE.base[2];
	angle_value[9] = H_ANGLE.base[3];
	Recevie_Angle = 1;//接收到云台角度命令
	Yt_Flag1 = 1;//接收到命令
	Yt_Flag2 = 0;
	Angle_Ok = 0;
	
	CW_Auto_Flag = 0;
	CCW_Auto_Flag = 0;
	Stop_Auto_Flag = 0;
//	if(!first_flag){
//		first_flag = 1;
//		Last_H = H_A;
//		Last_V = V_A;
//		MCU2YT(angle_value,10);	
//	}
//	else{
//		if((fabs(Last_H - H_angle) < 0.2) || (fabs(Last_V - V_angle) < 0.2))
//		{
//			Last_H = H_A;
//		  Last_V = V_A;
//			MCU2YT(angle_value,10);
//		}
//	}
	MCU2YT(angle_value,10);	
}

void yt_auto_callback(const riki_msgs::Action& action)
{
	uint8_t Auto_Up_Cmd[3] = {0XFF,0X88,0X77};
	uint8_t Auto_Down_Cmd[3] = {0XFF,0X77,0X66};
	uint8_t Auto_Left_Cmd[3] = {0XFF,0X66,0X55};
	uint8_t Auto_Right_Cmd[3] = {0XFF,0X55,0X44};
	uint8_t Manual_Cmd[3] = {0XFF,0X44,0X33};
	
	H_V_Cur = H_angle;
	
	
	if(action.robot_action == 1)//自动上升
	{
		MCU2YT(Auto_Up_Cmd,3);
	}
	else if(action.robot_action == 2)//自动下降
	{
		MCU2YT(Auto_Down_Cmd,3);
	}
	else if(action.robot_action == 3)//自动左转---逆时针
	{
		CW_Auto_Flag = 0;
		CCW_Auto_Flag = 1;
		Stop_Auto_Flag = 0;
		MCU2YT(Auto_Left_Cmd,3);
	}
	else if(action.robot_action == 4)//自动右转---顺时针
	{
		CW_Auto_Flag = 1;
		CCW_Auto_Flag = 0;
		Stop_Auto_Flag = 0;
		MCU2YT(Auto_Right_Cmd,3);
	}
	else if(action.robot_action == 5)//手动控制
	{
		CW_Auto_Flag = 0;
		CCW_Auto_Flag = 0;
		Stop_Auto_Flag = 1;
		MCU2YT(Manual_Cmd,3);
	}
	if(CW_Auto_Flag || CCW_Auto_Flag)
	{
		Recevie_Angle = 0;//清除手动标志
		Angle_Ok = 1;//清除手动标志
	}
}
uint8_t robot_start,robot_stop;
void action_callback(const riki_msgs::Action& action)
{
	if(action.robot_action)
	{
		robot_start = 1;
		robot_stop = 0;
	}
	else
	{
		robot_start = 0;
		robot_stop = 1;
	}
}
int test_vol1;
int test_vol2;
float current_rpm1;
float current_rpm2;
Kinematics::rpm req_rpm;
double test_pid1,test_pid2;
void move_base()
{
	char vel_buffer[]= " enter move base2 ";
	
	
  req_rpm = kinematics.getRPM(required_linear_vel, 0, required_angular_vel);
	
//	current_rpm1 = encoder1.getRPM();
//  current_rpm2 = encoder2.getRPM();
	current_rpm1 = encoder_count1;
	current_rpm2 = encoder_count2;
	
	printf("左轮转速设定值：%d\n\r",req_rpm.motor1);
	printf("左轮转速实时值：%d\n\r",current_rpm1);
	printf("右轮转速设定值：%d\n\r",req_rpm.motor2);
	printf("右轮转速实时值：%d\n\r",current_rpm2);
	
	test_pid1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
	test_pid2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);

    motor1.spin(test_pid1);
	motor2.spin(test_pid2);
//	motor1.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
//	motor2.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
	
	Kinematics::velocities current_vel;
	
	current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
	
	 //fill in the object
	raw_vel_msg.linear_x = -current_vel.linear_x/1;//40.64048351520634;//45.055968420371;//40.99223529411763;
  raw_vel_msg.linear_y = 0.0;
  raw_vel_msg.angular_z = -current_vel.angular_z/1;//20.37179682044995;//22.77576019211;//20.49818417289958;

	if(!(raw_vel_msg.linear_x || raw_vel_msg.angular_z ))
			zero_flag = 1;//速度为0
		else
			zero_flag = 0;
	if(fabs(raw_vel_msg.linear_x) > fabs(2*required_linear_vel))
		raw_vel_msg.linear_x = required_linear_vel;
	if(fabs(raw_vel_msg.angular_z) > fabs(2*required_angular_vel))
		raw_vel_msg.angular_z = required_angular_vel;
    //publish raw_vel_msg object to ROS
	raw_vel_pub.publish(&raw_vel_msg);
  nh.logwarn(vel_buffer);
}

void check_imu()
{
    gyro = imu.check_gyroscope();
    accel = imu.check_accelerometer();
    mag = imu.check_magnetometer();

    if (!accel){
        nh.logerror("Accelerometer NOT FOUND!");
    }   

    if (!gyro){
        nh.logerror("Gyroscope NOT FOUND!");
    }   

    if (!mag){
        nh.logerror("Magnetometer NOT FOUND!");
    }
		if(accel && gyro && mag)//加速度、角度、磁力计都check ok后，不再check
        is_first = false;
}
int zero_cnt;

void publish_imu()
{
    //geometry_msgs::Vector3 acceler, gyro, mag;
    //this function publishes raw IMU reading
    //measure accelerometer
    if (accel){
        imu.measure_acceleration();
        raw_imu_msg.linear_acceleration = imu.raw_acceleration;
    }

    //measure gyroscope
    if (gyro){
        imu.measure_gyroscope();
				if(zero_flag)
					zero_cnt++;
				else
					zero_cnt = 0;
				if(zero_cnt >= 5)
					imu.raw_rotation.z = 0;
        raw_imu_msg.angular_velocity = imu.raw_rotation;
    }

    //measure magnetometer
    if (mag){
        imu.measure_magnetometer();
        raw_imu_msg.magnetic_field = imu.raw_magnetic_field;
    }

    //publish raw_imu_msg object to ROS
    //raw_imu_pub.publish(&raw_imu_msg);

}
#if 0
void publish_imu_mpu6050()
{
    //MPU6050_ReadData();		/* 读取 MPU-6050的数据到全局变量 g_tMPU6050 */
   MPU6050();
    raw_imu_msg.linear_acceleration.x = g_tMPU6050.Accel_X;
    raw_imu_msg.linear_acceleration.y = g_tMPU6050.Accel_Y;
	raw_imu_msg.linear_acceleration.z = g_tMPU6050.Accel_Z;

    raw_imu_msg.angular_velocity.x = g_tMPU6050.GYRO_X;
	raw_imu_msg.angular_velocity.y = g_tMPU6050.GYRO_Y;
	raw_imu_msg.angular_velocity.z = g_tMPU6050.GYRO_Z;

    //publish raw_imu_msg object to ROS
    raw_imu_pub.publish(&raw_imu_msg);

}
#endif
void publish_humiture()
{
	char humiture_buffer[]= "The temperture and humidity were not updated! ";
	
	if(!calculate_TH())
	{
		raw_humiture_msg.temperature = Temperature;
	  raw_humiture_msg.humidity = Humidity;
		raw_humiture_pub.publish(&raw_humiture_msg);
	}
	else
	{
	//	nh.logwarn(humiture_buffer);
	}	
}

void publish_ultr_distance()
{
	char ultr_distance_left_buffer[]= "Ultrasonic data on the left is not updated! ";
	char ultr_distance_right_buffer[]= "Ultrasonic data on the right is not updated! ";
	float ultr_f_left,ultr_f_middle,ultr_f_right,ultr_left,ultr_right,ultr_a_left,ultr_a_right;
	
  	Sensors_Ultrasonic_task();
		ultr_f_left = Ultr_distance_F_left/1000.0;//mm->m
		ultr_f_right = Ultr_distance_F_right/1000.0;//mm->m
			
		ultr_a_left = Ultr_distance_A_left/1000.0;//mm->m
		ultr_a_right = Ultr_distance_A_right/1000.0;
		
		ultr_left = Ultr_distance_left/1000.0;//mm->m
		ultr_right = Ultr_distance_right/1000.0;//mm->m
	
		raw_ultrasonic_msg.ultr_FL = ultr_f_left;
		raw_ultrasonic_msg.ultr_FR = ultr_f_right;
		
		raw_ultrasonic_msg.ultr_AL = ultr_a_left;
		raw_ultrasonic_msg.ultr_AR = ultr_a_right;
		
		raw_ultrasonic_msg.ultr_L = ultr_left;
		raw_ultrasonic_msg.ultr_R = ultr_right;
		raw_ultrasonic_pub.publish(&raw_ultrasonic_msg);
}

void publish_illuminance()
{
	char illuminance_buffer[]= "illuminance is not updated! ";
	
	if(Led_Flag && !Led_Ack)
	{
			MCU2YT(led_l,6);
			//if(abs((int)(Light - Light_Pre)) > 20)
				//Led_Flag = 0;
	}
	else if(Led_Ack)
	{
			Led_Flag = 0;
			Led_Ack = 0;
	}
	if(!calculate())
	{	
		raw_illuminance_msg.light_intensity = Light;
		raw_illuminance_pub.publish(&raw_illuminance_msg);
	}
	else
	{
	//	nh.logwarn(illuminance_buffer);
	}
		
}

void publish_infrared_obstacle()
{
	  infrared_obstacle_msg.obstacle_left = IR_RIGHT;
	  infrared_obstacle_msg.obstacle_right = IR_LEFT;
		
		infrared_obstacle_pub.publish(&infrared_obstacle_msg);
}
void publish_hardware_collision()
{
		hardware_collision_msg.hardware_collision_front_l = Collision_Front_L;
	  hardware_collision_msg.hardware_collision_front_r = Collision_Front_R;
	  hardware_collision_msg.hardware_collision_later_l = Collision_Later_L;
	  hardware_collision_msg.hardware_collision_later_r = Collision_Later_R;
	
		hardware_collision_pub.publish(&hardware_collision_msg);
}
float Height_Pre;
static uint8_t lifter_cmd_num;

void publish_lifter_height()
{
	  uint8_t Ack_lifter[3] = {0xFF,0XEE,0XDD};
		
		uint8_t Auto_Up_Cmd[3] = {0XFF,0XBB,0XAA};
		uint8_t Auto_Down_Cmd[3] = {0XFF,0XAA,0X99};
		uint8_t Manual_Cmd[3] = {0XFF,0X99,0X88};
		
		LIFTER_Contr();
		
//		if(Lifter_Auto_Up_Flag || Lifter_Auto_Down_Flag || Lifter_Auto_Manual_Flag)//自动模式，清除手动标志
//		{
//			Recevie_Lifter = 0;
//			Lifter_Ok = 1;
//		}
		if(Lifter_Auto_Up_Flag && ((Lif_Cur - Height) < 5.0))//未响应自动上升命令
		{
			Lifter_Auto_Down_Flag = 0;
			Lifter_Auto_Manual_Flag = 0;
			MCU2YT(Auto_Up_Cmd,3);//继续发送
		}
		else if(Lifter_Auto_Up_Flag && ((Lif_Cur - Height) >= 5.0))
		{
			Lifter_Auto_Up_Flag = 0;
		}
		if(Lifter_Auto_Down_Flag && ((Lif_Cur - Height) < -5.0))//未响应自动下降命令
		{
			Lifter_Auto_Up_Flag = 0;
			Lifter_Auto_Manual_Flag = 0;
			MCU2YT(Auto_Down_Cmd,3);//继续发送下降指令
		}
		else if(Lifter_Auto_Down_Flag && ((Lif_Cur - Height) >= -5.0))
		{
			Lifter_Auto_Down_Flag = 0;
		}
		if(Lifter_Auto_Manual_Flag && (fabs(Lif_Cur - Height) >= 5.0))//未响应手动控制
		{
			Lifter_Auto_Up_Flag = 0;
			Lifter_Auto_Down_Flag = 0;
			MCU2YT(Manual_Cmd,3);//继续发送停止命令
		}
		else if(Lifter_Auto_Manual_Flag && (fabs(Lif_Cur - Height) < 5.0))
		{
			Lifter_Auto_Manual_Flag = 0;
		}
		
		if(Recevie_Lifter&&Height_Flag1&&!Height_Flag2)//接收到命令且未返回ACK
		{
			printf("LIFTER AGAIN!!!\r\n");
			MCU2YT(height,4);//再次下发命令
		}

		if(Recevie_Lifter&&(fabs(Lif_H-Height)<5.0)&&!Lifter_Ok)//抵达目标点
		{
			Recevie_Lifter = 0;
			Lifter_Ok = 1;
			MCU2YT(Ack_lifter,3);//接收数据成功ACK，STM43F429->STM32F103
		}	
		//else if(Recevie_Lifter&&(!Height_Flag1&&Height_Flag2)&&(fabs(Lif_H-Height)>5.0))
		else if(Recevie_Lifter&&(fabs(Lif_H-Height)>5.0))
		{
			lifter_cmd_num++;
			if(lifter_cmd_num>=50)//等待一段时间后还未达到预定高度
			{
				lifter_cmd_num = 0;
			  MCU2YT(height,4);//再次下发数据
			}
		}
	  lifter_height_msg.height = Height;
		lifter_height_pub.publish(&lifter_height_msg);	
}
static uint8_t angle_cmd_num;

void publish_yt_angle()
{
	  uint8_t Ack_angle[3] = {0xFF,0XDD,0XCC};
		
		uint8_t Auto_Left_Cmd[3] = {0XFF,0X66,0X55};
		uint8_t Auto_Right_Cmd[3] = {0XFF,0X55,0X44};
		uint8_t Manual_Cmd[3] = {0XFF,0X44,0X33};
	
		YT_Contr();
		/*
		if(CW_Auto_Flag || CCW_Auto_Flag || Stop_Auto_Flag)//自动模式，清除手动标志
		{
			Recevie_Angle = 0;
			Angle_Ok = 1;
		}
    */		
		if(CW_Auto_Flag && ((H_V_Cur - H_angle) < 0.5))//未响应顺时针旋转--自动
		{
			CCW_Auto_Flag = 0;
			Stop_Auto_Flag = 0;
			//MCU2YT(Auto_Right_Cmd,3);//继续发送
		}
		else if(CW_Auto_Flag && ((H_V_Cur - H_angle) >= 0.5))
		{
			CW_Auto_Flag = 0;
		}
		if(CCW_Auto_Flag && ((H_V_Cur - H_angle) > -0.5))//未响应自动逆时针旋转
		{
			CW_Auto_Flag = 0;
			Stop_Auto_Flag = 0;
			//MCU2YT(Auto_Left_Cmd,3);//继续发送下降指令
		}
		else if(CCW_Auto_Flag && ((H_V_Cur - H_angle) <= -0.5))
		{
			CCW_Auto_Flag = 0;
		}
		if(Stop_Auto_Flag && (fabs(H_V_Cur - H_angle) >= 0.5))//未响应手动控制
		{
			CCW_Auto_Flag = 0;
			CW_Auto_Flag = 0;
			//MCU2YT(Manual_Cmd,3);//继续发送停止命令
		}
		else if(Stop_Auto_Flag && (fabs(H_V_Cur - H_angle) < 0.5))
		{
			Stop_Auto_Flag = 0;
		}
		
		if(Recevie_Angle&&Yt_Flag1&&!Yt_Flag2)//接收到命令且未返回ACK
		{
			printf("ANGLE AGAIN!!!\r\n");
			//MCU2YT(angle_value,10);//再次下发数据
		}
		if((!Yt_Flag1&&Yt_Flag2)&&(fabs(V_A-V_angle)<0.5)&&(fabs(H_A-H_angle)<0.5)&&!Angle_Ok)//抵达目标点
		{
			Recevie_Angle = 0;
			Angle_Ok = 1;
			MCU2YT(Ack_angle,3);
		}		
		else if(Recevie_Angle&&(!Yt_Flag1&&Yt_Flag2)&&((fabs(V_A-V_angle)>0.5)||(fabs(H_A-H_angle)>0.5)))
		{
			angle_cmd_num++;
			if(angle_cmd_num>=50)
			{
				angle_cmd_num = 0;
			  //MCU2YT(angle_value,10);//再次下发数据
			}
		}
		
		if((H_A >=0) && (H_A < 1))
		{
			if(H_angle > 359)
				H_angle = 360 - H_angle;
		}
		if((H_A > 359) && (H_A <= 360))
		{
			if((H_angle > 0) && (H_angle < 1))
				H_angle = 360 - H_angle;
		}
	  yt_angle_msg.V_Angle = V_angle - V_ANGLE_ADJUSTMENT;//实时俯仰角
	  yt_angle_msg.H_Angle = H_angle - H_ANGLE_ADJUSTMENT;//实时水平角
		//yt_angle_pub.publish(&yt_angle_msg);	
}
void publish_charging_point()
{
	  Charge_Contr();
	  charge_pos_msg.charge_pos = Charge_Pos;
	  
		charging_point_pub.publish(&charge_pos_msg);	
}
void stop_base()
{
    required_linear_vel = 0;
    required_angular_vel = 0;
	  ROBOT_ACTION = 0;
	  #if 0
		{
		  GPIO_ResetBits(GPIOE, GPIO_Pin_6);
		  GPIO_ResetBits(GPIOH,GPIO_Pin_15);
			GPIO_ResetBits(GPIOH,GPIO_Pin_14);
			GPIO_ResetBits(GPIOB,GPIO_Pin_14);
			//TIM_Cmd(TIM5, DISABLE);
			//TIM_Cmd(TIM4, DISABLE);
	  }
		#endif
}

void publishBAT()
{
	raw_battery_msg.voltage = bat.get_volt();
	raw_battery_msg.current = bat.get_curr();
	raw_battery_msg.state_protection = State_protection;
	raw_battery_msg.surplus_capacity = Surplus_capacity;
	raw_battery_msg.surplus_capacity_percent = Surplus_capacity_percent;
	raw_battery_msg.total_capacity = Total_capacity;
	raw_battery_pub.publish(&raw_battery_msg);
}
void publishShutDown()
{
	shut_down_msg.data = 1;
	shut_down_pub.publish(&shut_down_msg);
}
void print_debug()
{
    char buffer[50]; 
    sprintf (buffer, "Encoder Left: %ld", encoder1.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder Right: %ld", encoder2.read());
    nh.loginfo(buffer);
    //sprintf (buffer, "get line speed : %f, pwm: %d", required_linear_vel, pwm);
    //nh->loginfo(buffer);
}
void reset_yt_lifter(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	RESET_FLAG = 1;
	res.success = true;
	res.message = "ok";
}

void lifter_auto_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res)
{

	uint8_t Auto_Up_Cmd[3] = {0XFF,0XBB,0XAA};
	uint8_t Auto_Down_Cmd[3] = {0XFF,0XAA,0X99};
	uint8_t Manual_Cmd[3] = {0XFF,0X99,0X88};
	
	Lif_Cur = Height;//获取当前高度值
	
	Recevie_Lifter = 0;//清除手动命令
	Lifter_Ok = 1;//清除手动命令
	
	if(req.data == 1)//自动上升
	{
		res.success = true;
		res.message = "ok";
		
		Lifter_Auto_Up_Flag = 1;
		Lifter_Auto_Down_Flag = 0;
		Lifter_Auto_Manual_Flag = 0;
		MCU2YT(Auto_Up_Cmd,3);
	}
	else if(req.data == 2)//自动下降
	{
		res.success = true;
		res.message = "ok";
		
		Lifter_Auto_Up_Flag = 0;
		Lifter_Auto_Down_Flag = 1;
		Lifter_Auto_Manual_Flag = 0;
		MCU2YT(Auto_Down_Cmd,3);
	}
	else if(req.data == 3)//手动控制
	{
		res.success = true;
		res.message = "ok";
		
		Lifter_Auto_Up_Flag = 0;
		Lifter_Auto_Down_Flag = 0;
		Lifter_Auto_Manual_Flag = 1;
		MCU2YT(Manual_Cmd,3);
	}
	if(Lifter_Auto_Up_Flag || Lifter_Auto_Down_Flag)
	{
		Recevie_Lifter = 0;//清除手动命令
		Lifter_Ok = 1;//清除手动命令
	}
}
void yt_auto_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res)
{
	uint8_t Auto_Up_Cmd[3] = {0XFF,0X88,0X77};
	uint8_t Auto_Down_Cmd[3] = {0XFF,0X77,0X66};
	uint8_t Auto_Left_Cmd[3] = {0XFF,0X66,0X55};
	uint8_t Auto_Right_Cmd[3] = {0XFF,0X55,0X44};
	uint8_t Manual_Cmd[3] = {0XFF,0X44,0X33};
	uint8_t Auto_Zero_Cmd[3] = {0XFF,0X33,0X22};
	
	H_V_Cur = H_angle;
	
	if(req.data == 1)//自动仰
	{
		res.success = true;
	  res.message = "ok";
		MCU2YT(Auto_Up_Cmd,3);
	}
	else if(req.data == 2)//自动俯
	{
		res.success = true;
		res.message = "ok";
		MCU2YT(Auto_Down_Cmd,3);
	}
	else if(req.data == 3)//自动左转---逆时针
	{
		res.success = true;
		res.message = "ok";
		CW_Auto_Flag = 0;
		CCW_Auto_Flag = 1;
		Stop_Auto_Flag = 0;
		MCU2YT(Auto_Left_Cmd,3);
	}
	else if(req.data == 4)//自动右转---顺时针
	{
		res.success = true;
		res.message = "ok";
		CW_Auto_Flag = 1;
		CCW_Auto_Flag = 0;
		Stop_Auto_Flag = 0;
		MCU2YT(Auto_Right_Cmd,3);
	}
	else if(req.data == 5)//手动控制
	{
		res.success = true;
		res.message = "ok";
		CW_Auto_Flag = 0;
		CCW_Auto_Flag = 0;
		Stop_Auto_Flag = 1;
		MCU2YT(Manual_Cmd,3);
	}
	else if(req.data == 6)//归中
	{
		res.success = true;
		res.message = "ok";
		CW_Auto_Flag = 0;
		CCW_Auto_Flag = 0;
		Stop_Auto_Flag = 1;
		MCU2YT(Auto_Zero_Cmd,3);
	}
	if(CW_Auto_Flag || CCW_Auto_Flag)
	{
		Recevie_Angle = 0;//清除手动标志
		Angle_Ok = 1;//清除手动标志
	}
}
const uint16_t polynom = 0xA001;
 
uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
    uint8_t i;
    uint16_t crc = 0xffff;
 
    if (len == 0) {
        len = 1;
    }
    while (len--) {
        crc ^= *ptr;
        for (i = 0; i<8; i++)
        {
            if (crc & 1) {
                crc >>= 1;
                crc ^= polynom;
            }
            else {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return(crc);
}

u8 auto_flag = false;

void em_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res)
{
//	u8  OpenCmd1[8]={0x03,0x05,0x00,0x00,0xff,0x00,0x8d,0xd8};//控制第一路开
//	u8  OpenCmd2[8]={0x03,0x05,0x00,0x01,0xff,0x00,0xdc,0x18};//控制第二路开
//	u8  OpenCmd3[8]={0x03,0x05,0x00,0x02,0xff,0x00,0x2c,0x18};//控制第三路开
//	u8  OpenCmd4[8]={0x03,0x05,0x00,0x03,0xff,0x00,0x7d,0xd8};//控制第四路开
//	
//	u8  CloseCmd1[8]={0x03,0x05,0x00,0x00,0x00,0x00,0xcc,0x28};//控制第一路关
//	u8  CloseCmd2[8]={0x03,0x05,0x00,0x01,0x00,0x00,0x9d,0xe8};//控制第二路关
//	u8  CloseCmd3[8]={0x03,0x05,0x00,0x02,0x00,0x00,0x6d,0xe8};//控制第三路关
//	u8  CloseCmd4[8]={0x03,0x05,0x00,0x03,0x00,0x00,0x3c,0x28};//控制第四路关
	u8  OpenCmd4[8]={0x02,0x05,0x00,0x03,0xff,0x00,0x7c,0x09};//控制第四路开
	u8  CloseCmd4[8]={0x02,0x05,0x00,0x03,0x00,0x00,0x3d,0xf9};//控制第四路关
	//if(!auto_flag)
	{
		if(req.data == 1)//打开电磁铁
		{
			res.success = true;
			res.message = "ok";
			//MCU2DY(1);
			JXBS3001GZ_Send(OpenCmd4);
			//JXBS3001GZ_Send(CloseCmd2);
		}
		else if(req.data == 0)//关闭电磁铁
		{
			res.success = true;
			res.message = "ok";
			//MCU2DY(0);
			JXBS3001GZ_Send(CloseCmd4);
			//JXBS3001GZ_Send(OpenCmd2);
		}
	}
}
void det_callback(const riki_msgs::DetResult& det)
{
	u8  OpenCmd4[8]={0x02,0x05,0x00,0x03,0xff,0x00,0x7c,0x09};//控制第四路开
	u8  CloseCmd4[8]={0x02,0x05,0x00,0x03,0x00,0x00,0x3d,0xf9};//控制第四路关
	if(det.B_Flag)
	{
		auto_flag = true;
		//MCU2DY(1);
		JXBS3001GZ_Send(OpenCmd4);
	}
	else if(!det.B_Flag)
	{
		auto_flag = false;
		//MCU2DY(0);
	}
	if(!det.D_Score)
	{
		//MCU2DY(0);
		JXBS3001GZ_Send(CloseCmd4);
	}
}
u8 OpenFlag1,OpenFlag2,OpenFlag3,OpenFlag4;
void el_fun(const riki_srvs::SetCmd::Request &req, riki_srvs::SetCmd::Response &res)
{
	u8  OpenCmd1[8]={0x02,0x05,0x00,0x00,0xff,0x00,0x8c,0x09};//控制第一路开
	u8  OpenCmd2[8]={0x02,0x05,0x00,0x01,0xff,0x00,0xdd,0xc9};//控制第二路开
	u8  OpenCmd3[8]={0x02,0x05,0x00,0x02,0xff,0x00,0x2d,0xc9};//控制第三路开
	u8  OpenCmd4[8]={0x02,0x05,0x00,0x03,0xff,0x00,0x7c,0x09};//控制第四路开
	
	u8  CloseCmd1[8]={0x02,0x05,0x00,0x00,0x00,0x00,0xcd,0xf9};//控制第一路关
	u8  CloseCmd2[8]={0x02,0x05,0x00,0x01,0x00,0x00,0x9c,0x39};//控制第二路关
	u8  CloseCmd3[8]={0x02,0x05,0x00,0x02,0x00,0x00,0x6c,0x39};//控制第三路关
	u8  CloseCmd4[8]={0x02,0x05,0x00,0x03,0x00,0x00,0x3d,0xf9};//控制第四路关
	
	if(req.data == 1)//打开继电器1
	{
		res.success = true;
	  res.message = "ok";
		OpenFlag1 = 1;
		JXBS3001GZ_Send(OpenCmd1);
	} 
	else if(req.data == 2)//打开继电器2
	{
		res.success = true;
	  res.message = "ok";
		OpenFlag2 = 1;
		JXBS3001GZ_Send(OpenCmd2);
	}
	else if(req.data == 3)//打开继电器3
	{
		res.success = true;
	  res.message = "ok";
		OpenFlag3 = 1;
		JXBS3001GZ_Send(OpenCmd3);
	}
	else if(req.data == 4)//打开继电器4
	{
		res.success = true;
	  res.message = "ok";
		OpenFlag4 = 1;
		JXBS3001GZ_Send(OpenCmd4);
	}
	else if(req.data == 0)//关闭继电器
	{
		res.success = true;
	  res.message = "ok";
		if(OpenFlag1)
		{
			OpenFlag1 = 0;
			JXBS3001GZ_Send(CloseCmd1);
		}
		if(OpenFlag2)
		{
			OpenFlag2 = 0;
			JXBS3001GZ_Send(CloseCmd2);
		}
		if(OpenFlag3)
		{
			OpenFlag3 = 0;
			JXBS3001GZ_Send(CloseCmd3);
		}
		if(OpenFlag4)
		{
			OpenFlag4 = 0;
			JXBS3001GZ_Send(CloseCmd4);
		}
	}
}
int CalCrc(int crc, const char *buf, int len)
{
    unsigned int byte;
    unsigned char k;
    unsigned short ACC,TOPBIT;
//    unsigned short remainder = 0x0000;
    unsigned short remainder = crc;
    TOPBIT = 0x8000;
    for (byte = 0; byte < len; ++byte)
    {
        ACC = buf[byte];
        remainder ^= (ACC <<8);
        for (k = 8; k > 0; --k)
        {
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^0x8005;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
    remainder=remainder^0x0000;
    return remainder;
}
char buffer[8] = {0xfe,0x05,0x00,0x00,0xff,0x00};
int main(void) 
{
	bool OnOff = true;
	uint32_t previous_battery_debug_time = 0;
	uint32_t previous_humiture_debug_time = 0;
	uint32_t previous_ultrasonic_debug_time = 0;
	uint32_t previous_illuminance_debug_time = 0;
	uint32_t previous_infrared_debug_time = 0;
	uint32_t previous_collision_debug_time = 0;
	uint32_t previous_height_debug_time = 0;
	uint32_t previous_angle_debug_time = 0;
	uint32_t previous_charge_debug_time = 0;
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t publish_vel_time = 0;
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";
	u8	AskCmd1[3]={0xda,0x02,0xBC};//KS103
	u8  AskCmd[8]={0x01,0x03,0x00,0x07,0x00,0x02,0x75,0xca};//JXBS3001GZ
  char vel_buffer1[]= " enter move base1 ";
	uint8_t Reset_Cmd[3] = {0XFF,0X33,0X22};
	SystemInit();
	initialise(); 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//???????????2	
	delay_init(168);
	Encode_TIM7_Init(500-1,8400-1);	//?????84M,????8400,??84M/8400=10Khz?????,??1000??100ms
	GYMPU680_Init(9600);
	JXBS3001GZ_Init(9600);
	UART7_Init(115200);
	IR_Init();
	TIM3_PWM_Init(2210,0);	 
	Serial1.begin(115200);//调试串口1
	USART3_Init(115200);//串口3，与电源控制芯片通信
	UART4_Init(115200);//串口4，与云台升降杆通信
	//Serial3.begin(115200);//
	//Serial4.begin(115200);//
	IR_Receive_Init();
  motor1.init();
  motor2.init();
	encoder1.init();
  encoder2.init();
	imu.init();
  
	
  unsigned short crc = CalCrc(0, buffer, 6);//?????16?CRC???
  buffer[7] = (char)crc;//????????
  buffer[6] = (char)(crc >> 8);//????????

	nh.initNode();	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // 开启串口2IDLE中断
	DMA_Use_USART2_Rx_Init();//配置串口2的DMA接收
	nh.advertise(raw_vel_pub);
  //nh.advertise(raw_imu_pub);
	nh.advertise(raw_battery_pub);
	nh.advertise(raw_ultrasonic_pub);
	//nh.advertise(infrared_obstacle_pub);
	nh.advertise(raw_humiture_pub);
	nh.advertise(raw_illuminance_pub);
	nh.advertise(hardware_collision_pub);
	nh.advertise(lifter_height_pub);
	nh.advertise(yt_angle_pub);
	nh.advertise(charging_point_pub);
	nh.advertise(shut_down_pub);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
	nh.subscribe(lifter_sub);
	nh.subscribe(led_sub);
	nh.subscribe(angle_sub);
	nh.subscribe(action_sub);
	nh.subscribe(det_sub);
	//nh.subscribe(lifter_auto_sub);
	//nh.subscribe(yt_auto_sub);
	
	//nh.advertiseService(reset_server);//复位云台、升降杆的服务
	nh.advertiseService(lifter_server);
	//nh.advertiseService(yt_server);
	nh.advertiseService(em_server);
	nh.advertiseService(el_server);
	#if 1
	while (!nh.connected()){
		nh.spinOnce();
	} 
	#endif
	nh.loginfo("Rikibase Connected!");
	printf("中电科西北集团有限公司---巡检机器人");
	//KS103_Send(AskCmd1);
	JXBS3001GZ_Send(AskCmd);
	send_Instruction();
	
	delay(1000);
	while(1){
	  if (((millis() - previous_control_time) >= (1000 / COMMAND_RATE))){
			 nh.logwarn(vel_buffer1);
			 move_base();
       previous_control_time = millis();
    }
		if ((millis() - previous_command_time) >= 400){
       stop_base();
    }
    
		if( (millis() - previous_battery_debug_time) >= (1000 / BAT_PUBLISH_RATE)){
			if(bat.get_volt() < 11.300000){
				OnOff = !OnOff;
				led.on_off(OnOff);
//				nh.logwarn(battery_buffer);			
			}
			publishBAT();
			previous_battery_debug_time = millis();		
		}
	  if(DEBUG){			
		  if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE)) {						
				print_debug();						
		    previous_debug_time = millis();			
		  }    
		}
		#if 0
	  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){	
	    //publish the IMU data
		if(is_first){
						//sanity check if the IMU exits
						check_imu();
				} else{
						//publish the IMU data
						publish_imu();
				}
		previous_imu_time = millis();
   }
		#endif
		if( (millis() - previous_ultrasonic_debug_time) >= (1000 / ULTR_PUBLISH_RATE)){
			//publish ultrasonic data on the left and right
			publish_ultr_distance();
			previous_ultrasonic_debug_time = millis();		
		}
		#if 0
		if( (millis() - previous_infrared_debug_time) >= (1000 / IR_PUBLISH_RATE)){
			//publish infrared-obstacle
			publish_infrared_obstacle();
			previous_infrared_debug_time = millis();		
		}
		#endif
		if( (millis() - previous_humiture_debug_time) >= (1000 / TH_PUBLISH_RATE)){
			//publish the temperature and humidity data
			publish_humiture();
			previous_humiture_debug_time = millis();		
		}
	  if( (millis() - previous_illuminance_debug_time) >= (1000 / ILLU_PUBLISH_RATE)){
			//publish illuminance data 
			publish_illuminance();
			previous_illuminance_debug_time = millis();		
		}
		if( (millis() - previous_collision_debug_time) >= (1000 / COLL_PUBLISH_RATE)){
			//publish hardware collision data 
			publish_hardware_collision();
			previous_collision_debug_time = millis();		
		}
		if( (millis() - previous_height_debug_time) >= (1000 / LIFTER_PUBLISH_RATE)){
			//publish lifter height data 
			publish_lifter_height();
			previous_height_debug_time = millis();		
		}
		if( (millis() - previous_angle_debug_time) >= (1000 / ANGLE_PUBLISH_RATE)){
			//publish yt Horizontal ande vertical angle data 
			publish_yt_angle();
			previous_angle_debug_time = millis();		
		}
		if( (millis() - previous_charge_debug_time) >= (1000 / CHARGE_PUBLISH_RATE)){
			
			publish_charging_point();
			previous_charge_debug_time = millis();
		}
		#if 0
		if(robot_start)
		{
			IR_Send_ContrlNum(0x39);//停止充电
		}
		if(robot_stop)
		{
			IR_Send_ContrlNum(0x3A);//开始充电
		}
		//IR_Send_ContrlNum(0x3A);
		#endif
		if(RESET_FLAG)
		{
			RESET_FLAG = 0;
			MCU2YT(Reset_Cmd,3);
		}
		if(SD_Rx_Flag)
		{
			publishShutDown();
		}
	  nh.spinOnce();
  }
}



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


double required_angular_vel = 0.0;
double required_linear_vel = 0.0;
uint32_t previous_command_time = 0;

bool is_first = true;
bool accel, gyro, mag;

PID motor1_pid(-499, 499, K_P, K_I, K_D);
PID motor2_pid(-499, 499, K_P, K_I, K_D);

Motor motor1(MOTOR1, 499, 20);
Motor motor2(MOTOR2, 499, 7);

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

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Subscriber<riki_msgs::Lifter> lifter_sub("lifter_height", lifter_callback);

ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);
ros::Publisher raw_humiture_pub("humiture", &raw_humiture_msg);
ros::Publisher raw_ultrasonic_pub("ultrasonic", &raw_ultrasonic_msg);
ros::Publisher raw_illuminance_pub("illuminance", &raw_illuminance_msg);
ros::Publisher infrared_obstacle_pub("infrared", &infrared_obstacle_msg);
ros::Publisher hardware_collision_pub("collision", &hardware_collision_msg);
ros::Publisher lifter_height_pub("lifter", &lifter_height_msg);

void pid_callback( const riki_msgs::PID& pid)
{
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
    required_linear_vel = -cmd_msg.linear.x*41.98;
    required_angular_vel = -cmd_msg.angular.z*20.16942611845979;

    previous_command_time = millis();
}
uint16_t Pulse_Cnt;
uint8_t height[4];
void lifter_callback( const riki_msgs::Lifter& Lifter)
{
		Pulse_Cnt = (uint16_t)(Lifter.height/0.85953841);
	  //Pulse_Cnt = (uint16_t)(Lifter.height/0.825962690859375);
	  height[0] = 0x5A;
	  height[1] = 0xA5;
		height[2] = Pulse_Cnt>>8;
		height[3] = Pulse_Cnt&0xff;
	  
	  MCU2YT(height,4);
}

void move_base()
{
	Kinematics::rpm req_rpm = kinematics.getRPM(required_linear_vel, 0, required_angular_vel);
	
	int current_rpm1 = encoder1.getRPM();
    int current_rpm2 = encoder2.getRPM();
	motor1.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
	
	Kinematics::velocities current_vel;
	current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);
	
	 //fill in the object
    raw_vel_msg.linear_x = -current_vel.linear_x/41.98;
    raw_vel_msg.linear_y = 0.0;
    raw_vel_msg.angular_z = -current_vel.angular_z/20.16942611845979;

    //publish raw_vel_msg object to ROS
    raw_vel_pub.publish(&raw_vel_msg);

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
    is_first = false;
}

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
        raw_imu_msg.angular_velocity = imu.raw_rotation;
    }

    //measure magnetometer
    if (mag){
        imu.measure_magnetometer();
        raw_imu_msg.magnetic_field = imu.raw_magnetic_field;
    }

    //publish raw_imu_msg object to ROS
    raw_imu_pub.publish(&raw_imu_msg);

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
		nh.logwarn(humiture_buffer);
	}
		
}

void publish_ultr_distance()
{
	char ultr_distance_left_buffer[]= "Ultrasonic data on the left is not updated! ";
	char ultr_distance_right_buffer[]= "Ultrasonic data on the right is not updated! ";
	float ultr_left,ultr_right;
	
	if(!Sensors_Ultrasonic_task1())
	{	
		if(!Sensors_Ultrasonic_task2())
		{
			ultr_left = Ultr_distance_left/1000.0;//mm->m
			ultr_right = Ultr_distance_right/1000.0;
			raw_ultrasonic_msg.ultr_left = ultr_left;
			raw_ultrasonic_msg.ultr_right = ultr_right;
			
			raw_ultrasonic_pub.publish(&raw_ultrasonic_msg);
		}
		else
		{
			//nh.logwarn(ultr_distance_right_buffer);
		}
	}
	else
	{
		nh.logwarn(ultr_distance_left_buffer);
	}
		
}

void publish_illuminance()
{
	char illuminance_buffer[]= "illuminance is not updated! ";
	
	if(!calculate())
	{	
		raw_illuminance_msg.light_intensity = Light;
		raw_illuminance_pub.publish(&raw_illuminance_msg);
	}
	else
	{
		nh.logwarn(illuminance_buffer);
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
	  DY2MCU();
		hardware_collision_msg.hardware_collision_front = Collision_Front;
	  hardware_collision_msg.hardware_collision_later = Collision_Later;
		
		hardware_collision_pub.publish(&hardware_collision_msg);
		
}
void publish_lifter_height()
{
		YT_Contr();
	  lifter_height_msg.height = Height;
		lifter_height_pub.publish(&lifter_height_msg);	
}

void stop_base()
{
    required_linear_vel = 0;
    required_angular_vel = 0;
}

void publishBAT()
{
	raw_battery_msg.battery = bat.get_volt();
	raw_battery_pub.publish(&raw_battery_msg);
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
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t publish_vel_time = 0;
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";
	u8	AskCmd1[3]={0xE8,0x02,0xBC};//KS103
	u8  AskCmd[8]={0x01,0x03,0x00,0x07,0x00,0x02,0x75,0xca};//JXBS3001GZ
  
	SystemInit();
	initialise(); 
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  //bsp_InitI2C();
	//bsp_InitMPU6050();
	
	GYMPU680_Init(9600);
	JXBS3001GZ_Init(9600);
	KS103_Init(9600);
	IR_Init();
	
	Serial1.begin(115200);//调试串口1
	USART3_Init(115200);
	UART4_Init(115200);
	//Serial3.begin(115200);//串口3，与STM32L431通信
	//Serial4.begin(115200);//串口4，与STM32F103通信
  motor1.init();
  motor2.init();
	encoder1.init();
  encoder2.init();
	imu.init();
	bat.init();
    
	nh.initNode();	
	nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);
	nh.advertise(raw_battery_pub);
	nh.advertise(raw_ultrasonic_pub);
	nh.advertise(infrared_obstacle_pub);
	nh.advertise(raw_humiture_pub);
	nh.advertise(raw_illuminance_pub);
	nh.advertise(hardware_collision_pub);
	nh.advertise(lifter_height_pub);
	
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
	nh.subscribe(lifter_sub);
	#if 0
	while (!nh.connected()){
		nh.spinOnce();
	} 
	#endif
	nh.loginfo("Rikibase Connected!");
	
	KS103_Send(AskCmd1);
	JXBS3001GZ_Send(AskCmd);
	send_Instruction();
	
	delay(1000);
	while(1){
		
	  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
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
				nh.logwarn(battery_buffer);			
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
		
		if( (millis() - previous_ultrasonic_debug_time) >= (1000 / ULTR_PUBLISH_RATE)){
			//publish ultrasonic data on the left and right
			publish_ultr_distance();
			previous_ultrasonic_debug_time = millis();		
		}
		
		if( (millis() - previous_infrared_debug_time) >= (1000 / IR_PUBLISH_RATE)){
			//publish infrared-obstacle
			publish_infrared_obstacle();
			previous_infrared_debug_time = millis();		
		}
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

		//printf("中电科西北集团有限公司---巡检机器人:%d\r\n",_counter);
	  nh.spinOnce();
  }
}



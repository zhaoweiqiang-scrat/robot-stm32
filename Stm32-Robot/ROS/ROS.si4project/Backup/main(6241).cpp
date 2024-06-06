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

#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include "usart.h"
//#include "bsp_mpu6050.h"
//#include "bsp_i2c_gpio.h"
#include "GYMPU680.h"
#include "ks103.h"
#include "JXBS3001GZ.h"

int Motor::counts_per_rev_ = COUNTS_PER_REV;

double required_angular_vel = 0.0;
double required_linear_vel = 0.0;
uint32_t previous_command_time = 0;

bool is_first = true;
bool accel, gyro, mag;

PID motor1_pid(-499, 499, K_P, K_I, K_D);
PID motor2_pid(-499, 499, K_P, K_I, K_D);

Motor motor1(MOTOR1, 499, 20);
Motor motor2(MOTOR2, 499, 7);

Encoder encoder1(ENCODER1, 0xffff, 0);
Encoder encoder2(ENCODER2, 0xffff, 0);
Battery bat(25, 10.6, 12.6);
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);
Gy85  imu;
Led led;

void pid_callback( const riki_msgs::PID& pid);
void command_callback( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle  nh;

riki_msgs::Imu raw_imu_msg;
riki_msgs::Velocities raw_vel_msg;
riki_msgs::Battery raw_battery_msg;
riki_msgs::Humiture raw_humiture_msg;
riki_msgs::Ultrasonic raw_ultrasonic_msg;
riki_msgs::Illuminance raw_illuminance_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);
ros::Publisher raw_humiture_pub("humiture", &raw_humiture_msg);
ros::Publisher raw_ultrasonic_pub("ultrasonic", &raw_ultrasonic_msg);
ros::Publisher raw_illuminance_pub("illuminance", &raw_illuminance_msg);

void pid_callback( const riki_msgs::PID& pid)
{
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
    required_linear_vel = cmd_msg.linear.x;
    required_angular_vel = cmd_msg.angular.z;

    previous_command_time = millis();
}


void move_base()
{
    Kinematics::output req_rpm;
    //get the required rpm for each motor based on required velocities
    req_rpm = kinematics.getRPM(required_linear_vel, 0.0, required_angular_vel);
	  //printf("req_rpm=:%d\r\n",req_rpm);

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1.spin(motor1_pid.compute(constrain(req_rpm.motor1, -MAX_RPM, MAX_RPM), motor1.rpm));
    motor2.spin(motor2_pid.compute(constrain(req_rpm.motor2, -MAX_RPM, MAX_RPM), motor2.rpm));
}

void publish_linear_velocity()
{
	motor1.updateSpeed(encoder1.read());
    motor2.updateSpeed(encoder2.read());
    
    Kinematics::velocities vel;
    vel = kinematics.getVelocities(motor1.rpm, motor2.rpm);

    //fill in the object
    raw_vel_msg.linear_x = vel.linear_x;
    raw_vel_msg.linear_y = 0.0;
    raw_vel_msg.angular_z = vel.angular_z;
	  //printf("raw_vel_msg.linear_x=:%f\r\n",raw_vel_msg.linear_x);
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
	
	if(!Sensors_Ultrasonic_task1())
	{	
		if(!Sensors_Ultrasonic_task2())
		{
			raw_ultrasonic_msg.ultr_distance1 = Ultr_distance_left;
			raw_ultrasonic_msg.ultr_distance2 = Ultr_distance_right;
			raw_ultrasonic_pub.publish(&raw_ultrasonic_msg);
		}
		else
		{
			nh.logwarn(ultr_distance_right_buffer);
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
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t publish_vel_time = 0;
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";
	u8	AskCmd1[3]={0xE8,0x02,0xBC};//KS103
	u8  AskCmd[8]={0x01,0x03,0x00,0x07,0x00,0x02,0x75,0xca};//JXBS3001GZ

	SystemInit();
	initialise();
  //bsp_InitI2C();
	//bsp_InitMPU6050();
	//GYMPU680_Init(9600);
	//JXBS3001GZ_Init(9600);
	//KS103_Init(9600);
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
	//nh.advertise(raw_humiture_pub);
	//nh.advertise(raw_ultrasonic_pub);
	//nh.advertise(raw_illuminance_pub);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
	#if 0
	while (!nh.connected()){
		nh.spinOnce();
	} 
	#endif
	nh.loginfo("Rikibase Connected!");
	//KS103_Send(AskCmd1);
	//JXBS3001GZ_Send(AskCmd);
	//send_Instruction();
	delay(1000);
	while(1){
		#if 0
		if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
			 move_base();
       previous_control_time = millis();
    }

    if ((millis() - previous_command_time) >= 400){
      // stop_base();
    }


		if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
				publish_linear_velocity();
				publish_vel_time = millis();
		}


		if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
				if(is_first){
						//sanity check if the IMU exits
						check_imu();
				} else{
						//publish the IMU data
						publish_imu();
				}
				previous_imu_time = millis();
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
						//print_debug();
						previous_debug_time = millis();
			}
    }


		
		#endif
		 
	  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE)){
			 move_base();
       previous_control_time = millis();
    }
		if ((millis() - previous_command_time) >= 400){
       //stop_base();
    }
    if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
				publish_linear_velocity();
				publish_vel_time = millis();
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
		//publish_imu_mpu6050();
		if(is_first){
						//sanity check if the IMU exits
						check_imu();
				} else{
						//publish the IMU data
						publish_imu();
				}
		previous_imu_time = millis();
   }
	  if( (millis() - previous_humiture_debug_time) >= (1000 / TH_PUBLISH_RATE)){
			//publish the temperature and humidity data
			publish_humiture();
			previous_humiture_debug_time = millis();		
		}
	  if( (millis() - previous_ultrasonic_debug_time) >= (1000 / ULTR_PUBLISH_RATE)){
			//publish ultrasonic data on the left and right
			publish_ultr_distance();
			previous_ultrasonic_debug_time = millis();		
		}
	  if( (millis() - previous_illuminance_debug_time) >= (1000 / ILLU_PUBLISH_RATE)){
			//publishilluminance data 
			publish_illuminance();
			previous_illuminance_debug_time = millis();		
		}
	  nh.spinOnce();
  }
}



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
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include "usart.h"
#include "bsp_mpu6050.h"
#include "bsp_i2c_gpio.h"

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

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<riki_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher raw_battery_pub("battery", &raw_battery_msg);


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
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t previous_control_time = 0;
	uint32_t publish_vel_time = 0;
	char battery_buffer[]= "The voltage is lower than 11.3V,Please charge! ";
	RCC_ClocksTypeDef* RCC_Clocks;

	SystemInit();
	
	RCC_GetClocksFreq(RCC_Clocks);
	initialise();
    //bsp_InitI2C();
	//bsp_InitMPU6050();
    motor1.init();
    motor2.init();
	encoder1.init();
    encoder2.init();
	led.init();
	imu.init();
	bat.init();

	nh.initNode();	
	nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
	nh.advertise(raw_battery_pub);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
	while (!nh.connected()){
		nh.spinOnce();
	} 
	nh.loginfo("Rikibase Connected!");
	led.on_off(OnOff);
	
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
       stop_base();
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
	  nh.spinOnce();
  }
}



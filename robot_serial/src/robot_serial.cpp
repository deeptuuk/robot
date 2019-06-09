#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <serial/serial.h>
#include <sstream>

serial::Serial ser("/dev/myserial", 115200, serial::Timeout::simpleTimeout(1000));

void send_ab_func(uint8_t fun,uint8_t*data,uint8_t len)
{
	static uint8_t send_buf[32];
	static uint8_t i;
	if(len>28)return;	
	send_buf[len+3]=0;	
	send_buf[0]=0x88;	
	send_buf[1]=fun;	
	send_buf[2]=len;	
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];		
	for(i=0;i<len+4;i++)ser.write((uint8_t *)(send_buf+i),1);
}

void write_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    static int16_t temp_vel[2];
    temp_vel[0]=(int16_t)((msg->linear.x)*1000);
    temp_vel[1]=(int16_t)((msg->angular.z)*1000);
    //ROS_INFO("write the data:%d,%d",temp_vel[0],temp_vel[1]);
    send_ab_func(0x55,(uint8_t *)temp_vel,4);
    //static unsigned char l_x=0,a_z=0;
    //static unsigned char end_a=0x0d,end_b=0x0a;

    //l_x=(unsigned char)((msg->linear.x)*100+100);
    //a_z=(unsigned char)((msg->angular.z)*100+100);
    //ROS_INFO("write the data:%d,%d",l_x,a_z);
    //ser.write((unsigned char *)(&l_x),1);
    //ser.write((unsigned char *)(&a_z),1);
    //ser.write((unsigned char *)(&end_a),1);
    //ser.write((unsigned char *)(&end_b),1);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "robot_serial");
	ros::NodeHandle n;
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("/vel_pub", 500);
    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 500, write_callback);

    ros::Publisher imu_data_raw_pub = n.advertise<sensor_msgs::Imu>("imu/data", 500);

	geometry_msgs::Vector3 v_result;
    sensor_msgs::Imu imu_data_raw;

    imu_data_raw.header.frame_id = "imu_data";

	v_result.x = 0;
	v_result.y = 0;
	v_result.z = 0;
    
    try{

    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else{
        return -1;
    }



	ros::Rate loop_rate(10);

    ser.flush();
	while (ros::ok()){

        if(ser.available()){

			static unsigned char start=0,flag=0;
            static unsigned char recive_data[64]={0};
            static short int temp_left=0,temp_righ=0;
            static unsigned char temp=0;

            while( ser.available() && (!(start==0x88))  ){
                ser.read(&start,1);
            }

			if(start==0x88){
				flag = ser.read(recive_data,33);
			}

            //ROS_INFO("Read: %x,%x,%x,%x,%x,%x,%x,%x",recive_data[0],recive_data[1],recive_data[2],recive_data[3],recive_data[4],recive_data[5],recive_data[6],recive_data[7]);

            if( (flag==33) &&  (recive_data[0]==0x55) && (recive_data[1]==30)  ){
                temp=0;
                for(int i=0;i<32;i++){
                    temp += recive_data[i];
                }
				temp += start;
                if(temp == recive_data[32]){

                    v_result.x = (int16_t)(((uint16_t)recive_data[3]<<8)|recive_data[2]) * 0.001f;
                    v_result.y = (int16_t)(((uint16_t)recive_data[5]<<8)|recive_data[4]) * 0.001f;

                    imu_data_raw.linear_acceleration.x = (int16_t)(((uint16_t)recive_data[7]<<8)|recive_data[6]) * 0.000598f;
                    imu_data_raw.linear_acceleration.y = (int16_t)(((uint16_t)recive_data[9]<<8)|recive_data[8]) * 0.000598f;
                    imu_data_raw.linear_acceleration.z = (int16_t)(((uint16_t)recive_data[11]<<8)|recive_data[10]) * 0.000598f;
                    
                    imu_data_raw.angular_velocity.x = (int16_t)(((uint16_t)recive_data[13]<<8)|recive_data[12]) * 0.001064f;
                    imu_data_raw.angular_velocity.y = (int16_t)(((uint16_t)recive_data[15]<<8)|recive_data[14]) * 0.001064f;
                    imu_data_raw.angular_velocity.z = (int16_t)(((uint16_t)recive_data[17]<<8)|recive_data[16]) * 0.001064f;

                    //if( (imu_data_raw.angular_velocity.z<=0.003) && (imu_data_raw.angular_velocity.z>=(-0.003)) ){
                    //    imu_data_raw.angular_velocity.z = 0.0;
                    //}

                    //imu_mag.magnetic_field.x = (int16_t)(((uint16_t)recive_data[19]<<8)|recive_data[18]) * 0.15f;
                    //imu_mag.magnetic_field.y = (int16_t)(((uint16_t)recive_data[21]<<8)|recive_data[20]) * 0.15f;
                    //imu_mag.magnetic_field.z = (int16_t)(((uint16_t)recive_data[23]<<8)|recive_data[22]) * 0.15f;

                    imu_data_raw.orientation.w = (int16_t)(((uint16_t)recive_data[25]<<8)|recive_data[24]) * 0.0001f;
                    imu_data_raw.orientation.x = (int16_t)(((uint16_t)recive_data[27]<<8)|recive_data[26]) * 0.0001f;
                    imu_data_raw.orientation.y = (int16_t)(((uint16_t)recive_data[29]<<8)|recive_data[28]) * 0.0001f;
                    imu_data_raw.orientation.z = (int16_t)(((uint16_t)recive_data[31]<<8)|recive_data[30]) * 0.0001f;                                        

                    v_result.z = imu_data_raw.angular_velocity.z;
                }
            }

			start = 0;
			flag = 0;
        }

        imu_data_raw.header.stamp = ros::Time::now();
        imu_data_raw_pub.publish(imu_data_raw);
        vel_pub.publish(v_result);        

        ros::spinOnce();
        loop_rate.sleep();				
	}

	ser.close();    
	return 0;
}
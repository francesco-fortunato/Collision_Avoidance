#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <cmath>
#define PARAM_VISUALE 450


sensor_msgs::LaserScan laser;
nav_msgs::Odometry odom;
geometry_msgs::Twist vel;


//Funzione per calcolare il minimo
float min_array(float array[], int len)
{
	float min = array[0];
	for(int k=0; k < len; k++)
	{
		if(array[k] < min )
		{
			min = array[k];
		}
	}
	return min;
} 

//Funzione che calcola la densita degli oggetti vicini
float avarage_density_array(float array[],int most_left,int most_right,int dist_aggiunta)
{
		float count=0;
		for(int i=most_left;i<most_right;i++){
			if(array[i] < dist_aggiunta) count++;
			}
		return count;
}

//Callback per il LaserScan
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//ROS_INFO("Lettura dei valori del LaserScan...");
	laser = *msg;	
}

//Callback per Odometry
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	//ROS_INFO("Lettura dei valori di Odometry...");
	odom = *msg;
}


int main(int argc , char* argv [])
{
	//ROS_INFO("Inizio del main");
	ros::init(argc,argv,"Collision_avoidance"); //Init del nodo nell'ambiente roscore
	ros::NodeHandle n; //NodeHandle is the main access point to communications with the ROS system

	ros::Publisher vel_pub;											//Publisher per il cmd_vel
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);	//Tell the master that we are going to publishing a message of type
                                                                    //Twist on the topic cmd_vel

	vel.linear.x = 0.0;		  										//Settiamo a 0.0 tutti i suoi campi del messaggio
	vel.linear.y = 0.0;
	vel.linear.z = 0.0;
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	vel.angular.z = 0.0;

	ros::Subscriber scanner_sub = n.subscribe("/base_scan", 1000, callback_scan); //Subscribe to the chatter topic /base_scan
                                                                                  //callback_scan() called anytime a new message arrives 

	ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, callback_odom);	//Subscribe to the chatter topic /odom
	ros::Rate loop_rate(5);
	int count = 0;
	bool check = true;	//check if is first time in if-else
	while(ros::ok())
	{
		float odom_x = odom.pose.pose.position.x;
		float odom_y = odom.pose.pose.position.y;
		//ROS_INFO("Posizione x: %f; y: %f\n", odom_x, odom_y);
		printf("Posizione x:%.2f; y:%.2f\n",odom_x,odom_y);

		int len = laser.ranges.size();	//length dell'array che dovrà contenere i valori del base_scan


			if (len > 0)
			{
				float destra[len/2];
				float centro[len - 2*PARAM_VISUALE];
				float sinistra[len - len/2];
				for(int i = 0; i < len; i++)
				{
					if(i<len/2) destra[i] = laser.ranges[i];
					else sinistra[i-len/2] = laser.ranges[i];
					//printf("Len totale : len=%d    Arrivato: i=%d \n",len,i);
					if(i >= PARAM_VISUALE ||  i < len-PARAM_VISUALE) centro[i - PARAM_VISUALE]=laser.ranges[i];
				}

				if(min_array(centro,len - 2*PARAM_VISUALE) > 0.8)
				{
					vel.linear.x = 1;
					vel.angular.z = 0.0;
					check=true;
					printf("Vado dritto\n");
				}
				else {
					if(check == true){
						if(min_array(destra,len/2) > min_array(sinistra,len - len/2)) vel.angular.z=-0.5;
						else vel.angular.z=0.5;
						check=false;
					}
					if(vel.angular.z > 0) printf("Gira a sinistra\n");
					else printf("Gira a destra\n");
					vel.linear.x=0.0;

					}

			vel_pub.publish(vel); //Publish del msg vel

		}

	    ros::spinOnce();
	    loop_rate.sleep();

	}

	return 0;

}
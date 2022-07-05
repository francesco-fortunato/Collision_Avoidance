#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <cmath>
#include "std_msgs/String.h"

#define PARAM_VISUALE 400       // Parametro che definisce il range dei valori da togliere sia da sinistra che da destra per mantenere solo i valori centrali
#define PARAM_LATERALE_INIZIO 100
#define PARAM_LATERALE_AMPIEZZA 100
#define WARNING_FRONT_PARAM 1.5  // Definisce il confine tra lo stato di WARNING e di DANGER
#define WARNING_SIDE_PARAM 0.3
#define K_OSTACOLI 0.00001
#define K_VELOCITA_IMPOSTA 100

struct Forza {
	float x_comp;
	float y_comp;
};

float angolo_base_mobile;
Forza* f_att = (Forza*) malloc( sizeof( Forza ) );
Forza* f_rep = (Forza*) malloc( sizeof( Forza ) );
Forza* f_ris = (Forza*) malloc( sizeof( Forza ) );
Forza* f_lat = (Forza*) malloc( sizeof( Forza ) );


// Variabili globali che puntano ai messaggi ricevuti dal nodo controller
sensor_msgs::LaserScan laser;    
nav_msgs::Odometry odom;
geometry_msgs::Twist vel_joystick;
geometry_msgs::Twist vel_stageros;

//Funzione ausiliaria che calcola il minimo su un array e ritorna quel valore. 
//Usato per capire se l'ostacolo proviene da sinistra o da destra
//e per capire se l'ostacolo si trova abbastanza vicino da poter reagire
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


//funzione callback per il LaserScan
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	laser = *msg;	
}

//funzione callback per Odometry
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;
}

//funzione callback per il Joystick
void callback_joystick(const geometry_msgs::Twist::ConstPtr& msg)
{
	vel_joystick = *msg;
}

//main
int main(int argc , char* argv [])
{
	
	//ROS_INFO("Inizio del main");
	ros::init(argc,argv,"Collision_avoidance"); //Init del nodo nell'ambiente roscore
	ros::NodeHandle n; //NodeHandle is the main access point to communications with the ROS system
		
	/*       #####      Publisher     #####          */
	ros::Publisher vel_pub;											//Publisher per il cmd_vel
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);	//Tell the master that we are going to publishing a message of type
                                                                //Twist on the topic cmd_vel (only 1 message in queue)
	
	/*       #####     Subscriber     #####			  */
	ros::Subscriber scanner_sub = n.subscribe("/base_scan", 1000, callback_scan); //Subscribe to the chatter topic /base_scan
                                                                                  //callback_scan() called anytime a new message arrives 

	ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, callback_odom);	//Subscribe to the chatter topic /odom
																				//callback_odom() call anytime a new message arrives


	ros::Subscriber command_sub = n.subscribe("/vel_joystick",1,callback_joystick);	//Subscribe to the chatter topic /vel_joystick
																					//callback_joystick() call anytime a new message arrives
	 
	ros::Rate r(1000);						
	while(ros::ok())
	{
		angolo_base_mobile = odom.pose.pose.orientation.z * M_PI;
		vel_stageros.linear.x=0;
		vel_stageros.angular.z=0;
		
		//float ang_gradi = odom.pose.pose.orientation.z * 180;
		
		//printf("Angolo in radian: %f   e  in gradi:%f  \n",angolo_base_mobile,ang_gradi);
		//printf("Cos(ang): %f  ;  Sin(ang):%f\n",cos(angolo_base_mobile) ,sin(angolo_base_mobile) );
		
		f_att->x_comp = vel_joystick.linear.x * cos(angolo_base_mobile);
		f_att->y_comp = vel_joystick.linear.x * sin(angolo_base_mobile);
	
		//printf("INFORMAZIONI SU /ODOM:\n");
		//printf("Pose: p.x:%f  , p.y:%f , p.z=%f , a.x:%f , a.y:%f  , a.z:%f \n",odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);
		//printf("Twist: l.x:%f  , l.y:%f , l.z=%f , a.x:%f , a.y:%f  , a.z:%f \n",odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z,odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z);
		
		int len= laser.ranges.size();				            // misura la lunghezza dell'array che mantiene i valori del laser (1081)
		int lunghezza_centro=len - 2*PARAM_VISUALE;
		if (len > 0)
		{
			/*
			 * In questa prima parte del codice vengono creati 3 array di float:
			 * Il primo array, quello chiamato centro, ha lungezza pari a len - 2*PARAM_VISUALE, e contiene i valori centrali della visuale del laser.
			 * Questo viene utilizzato per capire se ci sono ostacoli davanti al Robot
			 * 
			 * Il secondo e il terzo contengono esattamente la metà dei valori del laser e hanno lunghezza pari a len/2
			 * Quando il Robot si accorge di un ostacolo nelle vicinanze, analizzando i valori minimi degli array possiamo capire se l'ostacolo proviene 
			 * dalla sinistra o dalla destra del Robot
			 * */
			
			float centro[lunghezza_centro];
			float sinistro[PARAM_LATERALE_AMPIEZZA];
			float destro[PARAM_LATERALE_AMPIEZZA];
			
			//Questo ciclo fa l'assegnazione corretta dei valori che il laser rileva
			for(int i = 0; i < len; i++)
			{
				// Assegnazione dei valori centrali dei soli valori corretti
				if(i >= PARAM_VISUALE ||  i < len-PARAM_VISUALE)
				{
					centro[i - PARAM_VISUALE]=laser.ranges[i];
				}
				if(i >= PARAM_LATERALE_INIZIO && i< PARAM_LATERALE_AMPIEZZA + PARAM_LATERALE_INIZIO)
				{
					destro[i-PARAM_LATERALE_INIZIO]=laser.ranges[i];
				}
				if(i >= len - PARAM_LATERALE_AMPIEZZA - PARAM_LATERALE_INIZIO   &&  i < len - PARAM_LATERALE_INIZIO)
				{
					sinistro[i - len - PARAM_LATERALE_AMPIEZZA - PARAM_LATERALE_INIZIO] = laser.ranges[i];
				}
				
			}
			
			float incremento_radian = 270 / (len * 57.3) ;
			float angolo_estremo_destro = angolo_base_mobile - incremento_radian*lunghezza_centro/2;
			//printf("Incremento(radian): %f   ;  angolo_base_mobile:%f   ;  angolo_est_destro:%f\n",incremento_radian,angolo_base_mobile,angolo_estremo_destro);
			
			f_rep->x_comp=0;
			f_rep->y_comp=0;
			//printf("Lunghezza del vettore centro:%d\n",lunghezza_centro);
			
			for(int i=0;i<lunghezza_centro;i++)
			{
				//printf("Componente x:%f ;  Componente y:%f \n",f_rep->x_comp,f_rep->y_comp);
				if( centro[i] < WARNING_FRONT_PARAM )
				{
					float valore_secondo_distanza = WARNING_FRONT_PARAM - centro[i];
					valore_secondo_distanza = valore_secondo_distanza * valore_secondo_distanza * valore_secondo_distanza * valore_secondo_distanza* valore_secondo_distanza;
					f_rep->x_comp += K_OSTACOLI * (- valore_secondo_distanza * cos(angolo_estremo_destro + i*incremento_radian) );
					f_rep->y_comp += K_OSTACOLI * (- valore_secondo_distanza * sin(angolo_estremo_destro + i*incremento_radian) );
					//printf("La distanza:%f  , crea una comp_x:%f   e  comp_y:%f \n",centro[i],(- centro[i] * cos(angolo_estremo_destro + i*incremento_radian) ),(- centro[i] * sin(angolo_estremo_destro + i*incremento_radian) ) );
				}
			}
			printf("Distanza piu a destra:%f\n",centro[0]);
			printf("Distanza piu a sinistra:%f\n",centro[lunghezza_centro]);
			//printf("F_x_r:%f  ;  F_y_r:%f\n",f_rep->x_comp,f_rep->y_comp);
			//printf("F_x_a:%f  ;  F_y_a:%f\n",f_att->x_comp,f_att->y_comp);
			
			//Angolo imposto dagli ostacoli frontali
			float angolo_rep=0;
			if(f_rep->x_comp != 0 || f_rep->y_comp != 0)
			{
				angolo_rep = atan(f_rep->y_comp/f_rep->x_comp);
				//printf("Angolo rep non modificato:%f\n",angolo_rep);
				if(f_rep->x_comp < 0 )
				{ 
					if(f_rep->y_comp > 0 ) angolo_rep = angolo_rep + M_PI;
					else angolo_rep = angolo_rep - M_PI;
				}
			}
			f_lat->x_comp=0;
			f_lat->y_comp=0;
			for(int i=0;i<PARAM_LATERALE_AMPIEZZA;i++ )
			{
				if(destro[i] < WARNING_SIDE_PARAM)
				{
					float valore_distanza_d = WARNING_SIDE_PARAM - destro[i];
					valore_distanza_d = valore_distanza_d * valore_distanza_d * valore_distanza_d;
				
					//printf("Valore_distanza_d:%f\n",valore_distanza_d );
					f_lat->x_comp += K_OSTACOLI *(- valore_distanza_d * cos(angolo_base_mobile - M_PI/2) );
					f_lat->y_comp += K_OSTACOLI *(- valore_distanza_d * sin(angolo_base_mobile - M_PI/2) );
				}
				if(sinistro[i] < WARNING_SIDE_PARAM)
				{
					float valore_distanza_s = WARNING_SIDE_PARAM - sinistro[i];
					valore_distanza_s = valore_distanza_s * valore_distanza_s * valore_distanza_s;
				
					//printf("Valore_distanza_s:%f\n",valore_distanza_s );
					f_lat->x_comp += K_OSTACOLI *(- valore_distanza_s * cos(angolo_base_mobile + M_PI/2) );
					f_lat->y_comp += K_OSTACOLI *(- valore_distanza_s * sin(angolo_base_mobile + M_PI/2) );
				}
			}
			printf("F_x_l:%f  ;  F_y_l:%f\n",f_lat->x_comp,f_lat->y_comp);
			
			//Angolo imposto dagli ostacoli laterali
			//float angolo_lat=0;
			
		
			// Angolo imposto dalla velocità
			float angolo_att=0;
			if(f_att->x_comp != 0 || f_att->y_comp != 0 )
			{
				 angolo_att = atan(f_att->y_comp/f_att->x_comp);
				 if(angolo_base_mobile > M_PI/2 ) angolo_att += M_PI;
				 if(angolo_base_mobile < - M_PI/2 ) angolo_att -= M_PI;
			}
			
			//Angolo giusto
			float angolo_giusto=angolo_base_mobile;
			f_ris->x_comp = f_att->x_comp + f_rep->x_comp ;
			f_ris->y_comp = f_att->y_comp + f_rep->y_comp ;
			
			if(f_ris->x_comp != 0  || f_ris->y_comp != 0)
			{
				angolo_giusto = atan(f_ris->y_comp / f_ris->x_comp);
				if(f_ris->x_comp < 0 )
				{ 
					if(f_ris->y_comp > 0 ) angolo_giusto += M_PI;
					else angolo_giusto -= M_PI;
				}
			}
			printf("Angolo giusto:%f\n",angolo_giusto);
			printf("Angolo rep: %f\n",angolo_rep);
			printf("F_x_a:%f  ;  F_y_a:%f\n",f_rep->x_comp,f_rep->y_comp);
			printf("Angolo base mobile: %f\n",angolo_base_mobile);
			//vel_stageros.linear.x=f_att->x_comp / cos(angolo_giusto);
			float v_a_imposta = 0;
			if(vel_joystick.linear.x != 0) v_a_imposta = K_VELOCITA_IMPOSTA * fmodf(angolo_giusto - angolo_base_mobile  , M_PI);
			printf("Velocita imposta :%f\n",v_a_imposta);
			printf("\n");
			vel_stageros.angular.z = vel_joystick.angular.z + v_a_imposta;
			
			float k_l_imposta =1;
			float valore_min = min_array(centro,lunghezza_centro);
			if( valore_min < 0.3 ) k_l_imposta = 0;
			else if( valore_min <  WARNING_FRONT_PARAM )  k_l_imposta = valore_min / WARNING_FRONT_PARAM;
			k_l_imposta = k_l_imposta * k_l_imposta;
			
			if(vel_joystick.linear.x > 0.0 ) vel_stageros.linear.x = k_l_imposta * 2 * vel_joystick.linear.x;
			else vel_stageros.linear.x = vel_joystick.linear.x;
			
			vel_pub.publish(vel_stageros);
			
						
			/*
			 * La seconda parte è quella decisionale, ovvero la parte che permette al robot di evitare gli ostacoli e che assegna la velocità corretta al Robot
			 * 
			 * Abbiamo 3 possibili situazioni in cui può trovarsi il robot:
			 * 		1.Il Robot sta navigando e nelle vicinanze non si trova nessun ostacolo.   (SAFE)
			 * 
			 * 		2.Il Robot capisce che si sta avvicinando ad un ostacolo e reagisce diminuendo la sua velocita lineare.   (WARNING)
			 * 			NOTA: Dopo alcune prove fatte, risultava che, aumentando la velocita lineare, il Robot a non riusciva ad evitare 
			 * 				  gli ostacoli mantenendo invariato il coefficiente che determina la distanza dal quale il Robot doveva reagire.
			 * 				  L'aumento di tale coefficiente non porta ad una soluzione poiché il robot in questo modo non sarebbe potuto 
			 * 			      entrare negli spazi più stretti. Utilizzando questo terzo stato intermedio si permette dunque al Robot di navigare
			 *                con valori abbastanza grandi di velocità e, allo stesso tempo, di poter passare anche negli spazi stretti.
			 * 
			 * 		3.Il Robot reagisce agli ostacoli e li evita cambiando i valori della velocità lineare e angolare.     (DANGER)
			 * */
			 
			/*if(min_array(centro,len - 2*PARAM_VISUALE) >= SAFE_ZONE_PARAM)
			{
				vel_stageros.linear.x = vel_joystick.linear.x;
				if( !ostacolo_evitato_dopo_commando ) vel_stageros.angular.z=vel_joystick.angular.z;
				else vel_stageros.angular.z = 0.0;
				decisione_da_prendere = true;
				
				printf("Navigando con velocita lineare letto dal joystick \n");
			}*/
				
			/*else if(min_array(centro,len - 2*PARAM_VISUALE) > WARNING_ZONE_PARAM  &&  min_array(centro,len - 2*PARAM_VISUALE) < SAFE_ZONE_PARAM )
			{
				vel_stageros.linear.x = 0.2;
				if( !ostacolo_evitato_dopo_commando ) vel_stageros.angular.z=vel_joystick.angular.z;
				else vel_stageros.angular.z = 0.0;
				decisione_da_prendere = true;
				printf("Modificato la velocita lineare a 0.2 \n");
			}*/
			/*else
			{
				if(decisione_da_prendere == true){
					if(min_array(destra,len/2) > min_array(sinistra,len - len/2)) vel_stageros.angular.z=-vel_joystick.angular.z;
					else vel_stageros.angular.z=vel_joystick.angular.z;
					decisione_da_prendere = false;
				}
				if(vel_stageros.angular.z > 0) printf("Girando a sinistra\n");
				else printf("Girando a destra\n");
				vel_stageros.linear.x=0.0;
				ostacolo_evitato_dopo_commando = true;
			}*/
			
			/*
			 * Terza e ultima parte del codice dove si fa la publicazione della velocita verso il nodo /stageros
			 * */
			 
			//vel_pub.publish(vel_stageros);
		}
		
			
		
		
	    ros::spinOnce();
	    r.sleep();
	}
	
	return 0;
}

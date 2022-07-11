#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <cmath>
#include "std_msgs/String.h"

#define PARAM_FRONTE 475        
#define PARAM_VISUALE 400       
#define WARNING_CENTER_PARAM 1.5  // Distanza di sicurezza
#define WARNING_FRONT_PARAM 0.35
#define K_OSTACOLI 0.000001  // 10^(-6)
#define K_VELOCITA_IMPOSTA 10000 // 10^(4)

#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
#define RESET   "\033[0m"				   /* Reset color */

struct Forza {
	float x;
	float y;
};

float angolo_base_mobile;
Forza* f_att = (Forza*) malloc( sizeof( Forza ) );
Forza* f_rep = (Forza*) malloc( sizeof( Forza ) );
Forza* f_ris = (Forza*) malloc( sizeof( Forza ) );


// Variabili globali che puntano ai messaggi ricevuti dal nodo controller
sensor_msgs::LaserScan laser;    
nav_msgs::Odometry odom;
geometry_msgs::Twist vel_joystick;
geometry_msgs::Twist vel_stageros;

//Funzione ausiliaria che calcola il minimo su un array e ritorna quel valore. 
//Usato per capire se l'ostacolo proviene da sinistra o da destra
//e per capire se l'ostacolo si trova abbastanza vicino da poter reagire
float min_array(float array[], int len){
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
void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	laser = *msg;	
}

//funzione callback per Odometry
void callback_odom(const nav_msgs::Odometry::ConstPtr& msg){
	odom = *msg;
}

//funzione callback per il Joystick
void callback_joystick(const geometry_msgs::Twist::ConstPtr& msg){
	vel_joystick = *msg;
}

/*****************                MAIN                ****************/
int main(int argc , char* argv []){
	
	ros::init(argc,argv,"Collision_avoidance"); //Init del nodo nell'ambiente roscore
	ros::NodeHandle n; //NodeHandle is the main access point to communications with the ROS system
		
	/************      PUBLISHER          *************/	
	ros::Publisher vel_pub;											//Publisher per il cmd_vel
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);	//Tell the master that we are going to publishing a message of type
                                                                //Twist on the topic cmd_vel (only 1 message in queue)
	
	/************      SUBSCRIBER          *************/	
	ros::Subscriber scanner_sub = n.subscribe("/base_scan", 1000, callback_scan); //Subscribe to the chatter topic /base_scan
                                                                                  //callback_scan() called anytime a new message arrives 

	ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, callback_odom);	//Subscribe to the chatter topic /odom
																				//callback_odom() call anytime a new message arrives


	ros::Subscriber command_sub = n.subscribe("/vel_joystick",1,callback_joystick);	//Subscribe to the chatter topic /vel_joystick
																					//callback_joystick() call anytime a new message arrives
	 
	ros::Rate r(1000); //the Rate instance will attempt to keep the loop at 1000hz by accounting for the time used by the work done during the loop	

	//Le seguenti variabili servono per prevenire deadlock
	bool trap = true;
	float exit_trap = 0;	

	while(ros::ok()){
		angolo_base_mobile = odom.pose.pose.orientation.z * M_PI;
		vel_stageros.linear.x=0;
		vel_stageros.angular.z=0;
		
		int len = laser.ranges.size();				 // misura la lunghezza dell'array che mantiene tutti i valori del laser (1081)
		int lunghezza_centro = len - 2*PARAM_VISUALE;  // misura la lunghezza dell'array che mantiene solo i valori centrali del laser (281)
		int lunghezza_fronte = len - 2*PARAM_FRONTE;  // misura la lunghezza dell'array che mantiene solo i valori frontali del laser (81)

		
		/* In questa prima parte del codice vengono creati due array:
		 * Il primo, definito come centro, ha lungezza pari a len - 2*PARAM_VISUALE, e contiene i valori centrali della visuale del laser.
		 * Questo viene utilizzato per capire se ci sono ostacoli davanti al Robot.
		 * Il secondo, definito come fronte, ha lunghezza pari a len - 2*PARAM_FRONTE, e contine i valori frontali della visuale del laser.
		 * Questo viene utilizzato per capire se il robot ha di fronte a se un ostacolo
		 * */
		
		float centro[lunghezza_centro], fronte[lunghezza_fronte];
		
		//Popolazione di array centro e fronte
		for(int i = 0; i < len; i++){
			if(i >= PARAM_VISUALE &&  i < len-PARAM_VISUALE){
				centro[i - PARAM_VISUALE]=laser.ranges[i];
			}
			if(i >= PARAM_FRONTE &&  i < len-PARAM_FRONTE){
				fronte[i - PARAM_FRONTE]=laser.ranges[i];
			}
		}

		/* In questa seconda parte vengono calcolate le componenti x e y delle forze repulsive generate dagli ostacoli
		 * che agiscono sul robot.
		 */

		printf(BOLDWHITE "Velocità lineare joystick: %f. Velocità angolare joystick: %f.\n", vel_joystick.linear.x, vel_joystick.angular.z); //DEBUG

		float incremento_radian = 270 / (len * 56.5) ;		// differenza in radianti delle misure adiacenti del laser
		float angolo_estremo_destro = angolo_base_mobile - incremento_radian*lunghezza_centro/2; // valore dell'angolo per la misura più a destra del vettore centro
		
		f_rep->x=0;
		f_rep->y=0;
		//Ciclo che calcola le componenti x e y della forza repulsiva che ogni ostacolo crea			
		for(int i=0;i<lunghezza_centro;i++){
			if(centro[i] < WARNING_CENTER_PARAM){
				float valore_secondo_distanza = WARNING_CENTER_PARAM - centro[i];
				valore_secondo_distanza = pow(valore_secondo_distanza,5);
				f_rep->x += K_OSTACOLI * (-valore_secondo_distanza * cos(angolo_estremo_destro + i*incremento_radian));
				f_rep->y += K_OSTACOLI * (-valore_secondo_distanza * sin(angolo_estremo_destro + i*incremento_radian));
				//printf("L'ostacolo a distanza: %f, crea una comp_x:%f   e  comp_y:%f \n",centro[i],(- centro[i] * cos(angolo_estremo_destro + i*incremento_radian) ),(- centro[i] * sin(angolo_estremo_destro + i*incremento_radian) ) );
			}
		}
		
		f_att->x = vel_joystick.linear.x * cos(angolo_base_mobile);	
		f_att->y = vel_joystick.linear.x * sin(angolo_base_mobile);					
		
		//Angolo generato dalle componenti della forza repulsiva
		float angolo_giusto=angolo_base_mobile;
		f_ris->x = f_att->x + f_rep->x;	//Sommo le componenti secondo x e y delle forze agenti sulla base mobile
		f_ris->y = f_att->y + f_rep->y;
		
		//Calcolo l'angolo che la direzione della forza risultante forma con l'asse delle x 
		if(f_ris->x != 0  || f_ris->y != 0){
			angolo_giusto = atan(f_ris->y / f_ris->x);
			if(f_ris->x < 0 ){ 
				if(f_ris->y > 0 ) angolo_giusto += M_PI;
				else angolo_giusto -= M_PI;
			}
		}
		

		/*
		 * La seconda parte è quella decisionale, ovvero la parte che permette al robot di evitare gli ostacoli e che assegna la velocità corretta al Robot
		 * 
		 * Abbiamo 4 possibili situazioni in cui può trovarsi il robot:
		 * 		1.Il Robot sta navigando e nelle vicinanze non si trova nessun ostacolo.   (SAFE)
		 * 
		 * 		2.Il Robot capisce che si sta avvicinando ad un ostacolo e reagisce diminuendo la sua velocita lineare.   (WARNING)
		 * 			NOTA: Dopo alcune prove fatte, risultava che, aumentando la velocita lineare, il Robot a non riusciva ad evitare 
		 * 				  gli ostacoli mantenendo invariato il coefficiente che determina la distanza dal quale il Robot doveva reagire.
		 * 				  L'aumento di tale coefficiente non porta ad una soluzione poiché il robot in questo modo non sarebbe potuto 
		 * 			      entrare negli spazi più stretti. Utilizzando questo terzo stato intermedio si permette dunque al Robot di navigare
		 *                con valori relativamente grandi di velocità e, allo stesso tempo, di poter passare anche negli spazi stretti.
		 * 
		 * 		3. Il Robot capisce di trovarsi in una trappola e tenta di uscirne.		(DANGER)
		 * */

		float vel_imp = 0;	//La velocità angolare che viene sommata a quella del joystick 
							//per redirezionare la base mobile secondo il verso giusto

		if(vel_joystick.linear.x > 0){
			vel_imp = K_VELOCITA_IMPOSTA * fmodf(angolo_giusto - angolo_base_mobile  , M_PI);
		}

		printf("Velocità imposta:%f.\n\n" RESET,vel_imp); //DEBUG

		vel_stageros.angular.z = vel_joystick.angular.z + vel_imp;  //Velocità angolare da mandare al nodo /stageros
		
		float k = 1;	// coefficiente che moltiplica la velocità lineare della base mobile ed e' 
						// tanto piccolo quanto più è corta la distanza dagli ostacoli
		
		// Uso della funzione aux per calcolare la distanza minima rilevata
		float valore_min_fronte = min_array(fronte,lunghezza_fronte);	
		float valore_min_centro = min_array(centro,lunghezza_centro);	

		if( valore_min_fronte < WARNING_FRONT_PARAM ){			//Se davanti al robot si trova un ostacolo e non ha via di uscita 
			printf(BOLDRED "DANGER:\nObstacle distance: %f\n" RESET, valore_min_fronte);
		}
		else if( valore_min_centro <  WARNING_CENTER_PARAM ){
			printf(BOLDYELLOW "WARNING:\nObstacle distance: %f\n" RESET, valore_min_fronte);
		}
		else{
			printf(BOLDGREEN "SAFE\n" RESET);
		}

		if( valore_min_fronte < WARNING_FRONT_PARAM ){			//Se davanti al robot si trova un ostacolo e non ha via di uscita 
			k = 0;				// Evita che la base mobile sbatta contro l'ostacolo se non ha spazio ai lati
			if(trap == true){
				if (vel_joystick.linear.x>0){
					vel_joystick.linear.x=0;
					trap = false;
					if(vel_imp > 0) exit_trap = 0.5;
					else exit_trap = -0.5;
				}
			}
			vel_stageros.angular.z += exit_trap;
		}
		else{
			trap=true;
			exit_trap=0;
		}
		
		printf(BOLDWHITE "k:%f\n" RESET,k);

		if( valore_min_centro <  WARNING_CENTER_PARAM ){
			if(valore_min_fronte > WARNING_FRONT_PARAM ) k = valore_min_centro / (WARNING_CENTER_PARAM);     // Piu vicino è l'ostacolo, minore il coefficiente
		}
		
		k = pow(k,2);
		if(vel_joystick.linear.x > 0.0 ){
			vel_stageros.linear.x = 2 * k * vel_joystick.linear.x;   // La velocità lineare da mandare al nodo /stageros
		}
		else{
			vel_stageros.linear.x=vel_joystick.linear.x;
		}
		
		
		/* Terza e ultima parte del codice dove si fa la publicazione della velocita verso il nodo /stageros
		 */
		
		vel_pub.publish(vel_stageros);

		

	    ros::spinOnce();
	    r.sleep();

	}
	
	return 0;
}

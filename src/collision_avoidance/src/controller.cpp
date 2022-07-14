#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <cmath>
#include "std_msgs/String.h"

#define FRONT_PARAM 450     //numero di valori da togliere sia da destra che da sinistra dall'array laser.ranges
							//per ottenere i valori frontali
#define VISUAL_PARAM 380    //numero di valori da togliere sia da destra che da sinistra dall'array laser.ranges
							//per ottenere una porzione più ampia di valori   
#define WARNING_CENTER_PARAM 1.3  //distanza di sicurezza minima
#define DANGER_FRONT_PARAM 0.3   //distanza di sicurezza massima
#define K_OBSTACLE 0.000001  //coefficiente usato per ottenere la forza repulsiva generata dagli ostacoli. 10^(-6)
#define K_ANG_VEL 10000 //coefficiente usato per ottenere la velocità angolare generata dagli ostacoli. 10^(4)

#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
#define RESET   "\033[0m"				   /* Reset color */

//Struttura usata per definire la forza generata dal joystick e dagli ostacoli
struct Forza {
	float x;
	float y;
};

Forza* f_att = (Forza*) malloc(sizeof(Forza)); //forza generata dal joystick
Forza* f_rep = (Forza*) malloc(sizeof(Forza)); //forza generata dagli ostacoli
Forza* f_ris = (Forza*) malloc(sizeof(Forza)); //forza risultante


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
	for(int k=0; k < len; k++){
		if(array[k] < min ){
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

	float angolo_base_mobile;	//angolo della base mobile rispetto alla mappa

	//Le seguenti variabili servono per prevenire deadlock
	bool trap = true;
	float hard_trap = false;
	float exit_trap = 0;	

	while(ros::ok()){

		angolo_base_mobile = odom.pose.pose.orientation.z * M_PI; 
		vel_stageros.linear.x=0;
		vel_stageros.angular.z=0;
		
		
		int len = laser.ranges.size();	// misura la lunghezza dell'array che mantiene tutti i valori del laser (1081)
		int center_len = len - 2*VISUAL_PARAM;  // misura la lunghezza dell'array che mantiene solo i valori centrali del laser (321)
		int front_len = len - 2*FRONT_PARAM;  // misura la lunghezza dell'array che mantiene solo i valori frontali del laser (181)

		/* In questa prima parte vengono creati due array:
		 * Il primo, definito come centro, ha lungezza pari a len - 2*VISUAL_PARAM, e contiene i valori centrali della visuale del laser.
		 * Questo viene utilizzato per capire se ci sono ostacoli davanti al Robot.
		 * Il secondo, definito come fronte, ha lunghezza pari a len - 2*FRONT_PARAM, e contine i valori frontali della visuale del laser.
		 * Questo viene utilizzato per capire se il robot ha di fronte a se un ostacolo
		 * */
		
		float centro[center_len], fronte[front_len];
		
		//Popolazione degli array centro e fronte con i valori ricevuti dal laser
		for(int i = 0; i < len; i++){
			if(i >= VISUAL_PARAM &&  i < len-VISUAL_PARAM){
				centro[i - VISUAL_PARAM]=laser.ranges[i];
			}
			if(i >= FRONT_PARAM &&  i < len-FRONT_PARAM){
				fronte[i - FRONT_PARAM]=laser.ranges[i];
			}
		}

		printf(BOLDWHITE "Velocità lineare ricevuta dal joystick: %f. Velocità angolare ricevuta dal joystick: %f.\n", vel_joystick.linear.x, vel_joystick.angular.z); //DEBUG

		/* In questa seconda parte vengono calcolate le componenti x e y delle forze repulsive generate dagli ostacoli
		 * che agiscono sul robot.
		 */

		f_rep->x=0;
		f_rep->y=0;
		
		float incremento_radian = 270 / (len * 56.5) ;		// differenza in radianti delle misure adiacenti del laser
		float angolo_estremo_destro = angolo_base_mobile - incremento_radian*center_len/2; // valore dell'angolo più a destra del vettore centro

		//Ciclo che calcola le componenti x e y della forza repulsiva che ogni ostacolo crea 			
		for(int i=0;i<center_len;i++){
			if(centro[i] < WARNING_CENTER_PARAM){
				float valore_secondo_distanza = WARNING_CENTER_PARAM - centro[i];
				valore_secondo_distanza = pow(valore_secondo_distanza,5);
				f_rep->x += K_OBSTACLE * (-valore_secondo_distanza * cos(angolo_estremo_destro + i*incremento_radian));
				f_rep->y += K_OBSTACLE * (-valore_secondo_distanza * sin(angolo_estremo_destro + i*incremento_radian));
				//printf("L'ostacolo a distanza: %f, crea una comp_x:%f   e  comp_y:%f \n",centro[i],(- centro[i] * cos(angolo_estremo_destro + i*incremento_radian) ),(- centro[i] * sin(angolo_estremo_destro + i*incremento_radian) ) );
			}
		}
		
		//Calcolo delle componenti x e y della forza generata dalla forza mandata dal joystick
		f_att->x = vel_joystick.linear.x * cos(angolo_base_mobile);	
		f_att->y = vel_joystick.linear.x * sin(angolo_base_mobile);					
		
		
		float angolo_giusto=angolo_base_mobile; //Angolo generato dalle forze repulsive (se in gioco) 

		f_ris->x = f_att->x + f_rep->x;	//Sommo le componenti secondo x e y delle forze agenti sulla base mobile
		f_ris->y = f_att->y + f_rep->y;
		
		//Calcolo l'angolo che la direzione della forza risultante forma con l'asse delle x 
		if(f_ris->x != 0 || f_ris->y != 0){
			angolo_giusto = atan(f_ris->y / f_ris->x);	//Angolo generato dalle componenti della forza repulsiva
			if(f_ris->x < 0){
				if(f_ris->y > 0) angolo_giusto += M_PI;
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

		float vel_ang_imp = 0;	//La velocità angolare che viene sommata a quella ricevuta del joystick 
								//per redirezionare la base mobile secondo il verso giusto

		//Si agisce nel momento in cui la velocità lineare è maggiore di zero
		if(vel_joystick.linear.x > 0){
			vel_ang_imp = K_ANG_VEL * fmodf(angolo_giusto - angolo_base_mobile  , M_PI);
		}

		printf("Velocità imposta:%f.\n\n" RESET, vel_ang_imp); //DEBUG

		vel_stageros.angular.z = vel_joystick.angular.z + vel_ang_imp;  //Velocità angolare da mandare al nodo /stageros
		
		float k = 1;	// coefficiente che moltiplica la velocità lineare della base mobile ed e' 
						// tanto piccolo quanto più è corta la distanza dagli ostacoli
		
		// Uso della funzione aux per calcolare la distanza minima rilevata
		float valore_min_fronte = min_array(fronte,front_len);	
		float valore_min_centro = min_array(centro,center_len);	

		if( valore_min_fronte < DANGER_FRONT_PARAM ){			
			printf(BOLDRED "DANGER:\nObstacle distance: %f\n" RESET, valore_min_fronte);
		}
		else if( valore_min_centro <  WARNING_CENTER_PARAM ){
			printf(BOLDYELLOW "WARNING:\nObstacle distance: %f\n" RESET, valore_min_fronte);
		}
		else{
			printf(BOLDGREEN "SAFE\n" RESET);
		}

		if(valore_min_fronte < DANGER_FRONT_PARAM){	//Se davanti al robot si trova un ostacolo
			k = 0;		//Evita che la base mobile sbatta contro l'ostacolo se non ha spazio ai lati
			if(trap == true){
				if (vel_joystick.linear.x>0){
					trap = false;
					if(vel_ang_imp > 0) exit_trap = 0,5;
					else if (vel_ang_imp<0) exit_trap = -0,5;
					if(vel_ang_imp==0,064373 || vel_ang_imp==-0,064373) hard_trap=true;	//Se davanti al robot si trova un ostacolo e non ha via di uscita 
					vel_stageros.angular.z += exit_trap;
				}
				
				else if (vel_joystick.linear.x==0){
					trap =false;
					hard_trap=true;
					exit_trap=0;
					vel_stageros.angular.z = 0;
				}	
			}
		}
		else{
			hard_trap=false;
			trap=true;
			exit_trap=0;
		}

		if( valore_min_centro <  WARNING_CENTER_PARAM ){
			if(valore_min_fronte > DANGER_FRONT_PARAM ) k = valore_min_centro / (WARNING_CENTER_PARAM);     //Più vicino è l'ostacolo, minore il coefficiente
		}
		else{
			hard_trap=false;
		}
		
		k = pow(k,2);

		if (hard_trap==true){
			if (vel_joystick.linear.x>0){
				printf(BOLDRED "ATTEMPT TO EXIT THE TRAP\n" RESET);
				vel_stageros.angular.z = M_PI;
				vel_stageros.linear.x=0;
			}
			else{
				printf(BOLDRED "TRAP\n" RESET);
				vel_stageros.linear.x=vel_joystick.linear.x;
				vel_stageros.angular.z=vel_joystick.angular.z;
			}
			
		}
		else{
			if(vel_joystick.linear.x > 0.0 ){
				vel_stageros.linear.x = 2 * k * vel_joystick.linear.x;   // La velocità lineare da mandare al nodo /stageros
			}
			else{
				vel_stageros.linear.x=vel_joystick.linear.x;
			}
		}
		
		printf(BOLDWHITE "k:%f\n" RESET,k);

		/* Terza e ultima parte del codice dove si fa la publicazione della velocita verso il nodo /stageros
		 */
		
		vel_pub.publish(vel_stageros);

		

	    ros::spinOnce();
	    r.sleep();

	}
	
	return 0;
}

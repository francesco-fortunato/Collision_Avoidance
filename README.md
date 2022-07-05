# COLLISION AVOIDANCE

-Sistema che si basa su un laser per consentire l'anti-collisione

-Compilare eseguendo sul terminale:
	catkin build
-Si esegue in terminali diversi:
	1.  roscore      -> lancia il sistema ros
	2.  rosrun stage_ros stageros /opt/ros/melodic/share/stage_ros/world/willow-erratic.world    -> lancia il nodo /stageros
		Apre una finestra che mostra la navigazione del Robot sulla mappa
		
	3.  rosrun collision_avoidance controller_node    -> lancia il nodo /controller_node
		Controlla il Robot per evitare gli ostacoli e stampa costantemente la posizione del Robot nella mappa
		  
	4.  rosrun srrg_joystick_teleop joy_teleop_node -> lancia il nodo che manda messaggi del tipo geometry_msgs/Twist al nodo controllore e che legge 
		commandi dal dispositivo /opt/dev/js0
	MODIFICA iMPORTANTE: Nel nodo 'joy_teleop_node.cpp' ho sistituito il nome del topic su quale il nodo publica da '/cmd_vel' a '/vel_joystick'. 
	La ragione di questa modifica sta nel fatto di distinguere il topic  (joy_teleop_node.cpp) --[Nome topic: /vel_joystick]--> (/controller_node.cpp)  
	dal topic (controller_node.cpp) --[/cmd_vel]--> (stageros.cpp)
		
ATTENZIONE!! 
	Eseguire per ogni terminale: source devel/setup.bash


Debug:
	rosrun rqt_graph rqt_graph  -> Grafico che mostra i nodi,topic e la loro relazione
	rosmsg show <package>/<nome_messaggio>  -> mostra la definizione del messaggio
	rosmsg  list -> mostra la lista di tutti i messaggi disponibili   ( | grep <parte_del_messaggio>   -> per filtrare i messaggi)
	rostopic pub [-r] [frequenza(Hz)] <nome_topic> <tipo_messaggio>   -> publica da un certo topic 
	rostopic echo <nome_topic> [-nNUM]  -> legge un topic [opzione dove si leggono solo i NUM ultimi messaggi]	
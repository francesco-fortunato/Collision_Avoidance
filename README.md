# COLLISION AVOIDANCE

## Sistema di un robot basato su laser che impedise le collisioni.

### Passaggi da svolgere per testare il programma
	1. Compilare eseguendo sul terminale: `catkin build`.
	
	1. `roscore` -> lancia il sistema ros
	
	1.  `rosrun stage_ros stageros /opt/ros/melodic/share/stage_ros/world/willow-erratic.world `  -> lancia il nodo /stageros e apre una finestra che mostra la navigazione del Robot sulla mappa
	
	1.  rosrun collision_avoidance controller_node    -> lancia il nodo /controller_node che controlla il Robot per evitare gli ostacoli e stampa costantemente la posizione del Robot nella mappa
		  
	1.  `rosrun srrg_joystick_teleop joy_teleop_node` -> lancia il nodo che manda messaggi del tipo geometry_msgs/Twist al nodo controllore e che legge commandi dal dispositivo /opt/dev/js0. **MODIFICA IMPORTANTE**: Prima di eseguire quest'ultima istruzione, nel nodo 'joy_teleop_node.cpp' sostituire il nome del topic su quale il nodo publica da '/cmd_vel' a '/vel_joystick'. La ragione di questa modifica sta nel fatto che si ha la necessità di distinguere il topic  (joy_teleop_node.cpp) --[Nome topic: /vel_joystick]--> (/controller_node.cpp) dal topic (controller_node.cpp) --[/cmd_vel]--> (stageros.cpp).


Strumenti per il debug:
	rosrun rqt_graph rqt_graph  -> Grafico che mostra i nodi, itopic e la loro relazione
	rosmsg show <package>/<nome_messaggio>  ->  Mostra la definizione del messaggio
	rosmsg  list -> Mostra la lista di tutti i messaggi disponibili   ( | grep <parte_del_messaggio>   -> per filtrare i messaggi)
	rostopic pub [-r] [frequenza(Hz)] <nome_topic> <tipo_messaggio>   -> Pubblica da un certo topic 
	rostopic echo <nome_topic> [-nNUM]  -> lLegge un topic [opzione dove si leggono solo i NUM ultimi messaggi]	
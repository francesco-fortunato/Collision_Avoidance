# COLLISION AVOIDANCE

### Sistema di un robot basato su laser che impedise le collisioni.

![collision_avoidance](collision_avoidance.gif)

### How it works

Gli ostacoli creano forze repulsive che modificano la traiettoria del robot. Questo è un esempio delle varie forze che vengono applicate sul robot: una velocità angolare imposta dal joystick nulla, una certa velocità lineare imposta dal joystick, una forza repulsiva generata dall'ostacolo a distanza r con le sue due componenti x e y
![forze_repulsive](forze_repulsive.png)

Le forze repulsive si sommano con le forze angolari e lineari generate dal joystick, modificano quindi le velocità lineari e angolari del robot.
![forze_risultanti](forze_risultanti.png)

## Passaggi da svolgere per testare il programma
1.  Aprire il terminale.

1.  Eseguire il comando `cd ~/your/path/lab-IAGI-project`.
	
1.  Eseguire `catkin build`.

1.  A questo punto avviare roscore utilizzando il comando `roscore`.

1.  Aprire un nuovo terminale, clonare la repository del corso di lab. IAGI dell'anno 2020-21, disponibile [qui](https://gitlab.com/grisetti/labiagi_2020_21)

1.  Eseguire `cd ~/your/path/labiagi_2020_21/`.

1.  Digitare `./setup.sh`.

1.  Eseguire `cd ~/your/path/labiagi_2020_21/workspaces/srrg2_labiagi/src/srrg2_navigation_2d/config`.

1.  Da qui, aprire un terminale per ogni punto ed eseguire questi comandi:
	
	1.  `rosrun stage_ros stageros cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world`  -> lancia il nodo /stageros e apre una finestra che mostra la navigazione del Robot sulla mappa
	
	1.  `rosrun collision_avoidance controller_node`    -> lancia il nodo /controller_node che controlla il Robot per evitare gli ostacoli 
		  
	1.  `rosrun srrg_joystick_teleop joy_teleop_node` -> lancia il nodo che manda messaggi del tipo geometry_msgs/Twist al nodo controller e che legge commandi dal dispositivo /opt/dev/js0. 
	
	**MODIFICA IMPORTANTE**: Prima di eseguire quest'ultima istruzione, nel nodo 'joy_teleop_node.cpp' sostituire il nome del topic sul quale il nodo publica da '/cmd_vel' a '/vel_joystick'. La ragione di questa modifica sta nel fatto che si ha la necessità di distinguere il topic  (joy_teleop_node.cpp) --[Nome topic: /vel_joystick]--> (/controller_node.cpp) dal topic (controller_node.cpp) --[/cmd_vel]--> (stageros.cpp).


Strumenti per il debug:
	rosrun rqt_graph rqt_graph  -> Grafico che mostra i nodi, itopic e la loro relazione
	rosmsg show <package>/<nome_messaggio>  ->  Mostra la definizione del messaggio
	rosmsg  list -> Mostra la lista di tutti i messaggi disponibili   ( | grep <parte_del_messaggio>   -> per filtrare i messaggi)
	rostopic pub [-r] [frequenza(Hz)] <nome_topic> <tipo_messaggio>   -> Pubblica da un certo topic 
	rostopic echo <nome_topic> [-nNUM]  -> lLegge un topic [opzione dove si leggono solo i NUM ultimi messaggi]	

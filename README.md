# ROS node Collision Avoidance - Alessandro Ferrante 2020-2021

# Come testare 

* Copiare il contenuto di questa directory nella cartella 'src' del proprio workspace
* buildare con catkin_make all'interno del proprio workspace
* Avviare in una tab del terminale (con 'roscore' lanciato) tramite:
 'rosrun collision_avoidance avoider'
* avviare un controller come teleop_twist_keyboard tramite:
 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel_call'
* Recarsi in '/.../labiagi_2020_21/workspaces/srrg2_labiagi/src/srrg2_navigation_2d/config' e runnare
con '~/.../labiagi_2020_21/srrg2_webctl/proc_webctl run_navigation.webctl' il terminale web
* Aprire un qualsiasi browser e connettersi a 'localhost:9001'
* Cliccare 'start' su 01_roscore e 02_stage
* Con la tab del terminale di cui al punto 4 selezionata, muoversi liberamente coi comandi indicati! 

In order to test this node use a stage ros and a controller node like `teleop_twist_keyboard` that publish `Twist` messages. 


La mappa su cui sono stati fatti i test è la mappa del DIAG:  [cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world](https://gitlab.com//grisetti/labiagi_2020_21/-/raw/master/workspaces/srrg2_labiagi/src/srrg2_navigation_2d/config/cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world?inline=false)





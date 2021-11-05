#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_utils_fd.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

ros::Publisher vel;

bool cmdReceived = false;

//Inizializzo le variabili con cui lavoro
float vel_x=0;
float vel_y=0;
float vel_ang=0;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{ 
  cmdReceived = true;                       //Salvo gli input e attendo un laser scan prima di fare calcoli
  vel_x = msg->linear.x;
  vel_y = msg->linear.y;
  vel_ang = msg->angular.z;
}

void laser_cmd_vel_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (!cmdReceived ) return;                     //Verifico che  sia effettivamente un cmd, altrimenti ritorno
  cmdReceived = false;                           //Per non processare un comando di velocità più volte
                                                 //setto a false la variabile    
  tf::TransformListener listener;
  laser_geometry::LaserProjection laser_proj;
  sensor_msgs::PointCloud cloud;

 //Punti della nuvola che mi arrivano dal LaserScan (quindi nel S.R. del LaserScan)
  laser_proj.transformLaserScanToPointCloud("base_laser_link", *msg, cloud, listener);
  
 //Calcolo la trasformata per ottenere i punti della nuvola nel S.R. Robot 
  tf::StampedTransform transform_obstacle;
    try{
        //converto i punti dal S.R. LaserScan al S.R. Robot
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10.0)); 
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), transform_obstacle); 

    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", "Errore trasformazione");
        ros::Duration(1.0).sleep();         //In caso di errore stampo eccezione e metto in sleep
        return;
    }

//Prendo la matrice che definisce la trasformata per ricavare i punti nel S.R. del Robot
Eigen::Isometry2f laser_transform = convertPose2D(transform_obstacle);

//Inizializzo a 0 le forze repulsive degli ostacoli ricavati dal LaserScanner
float forza_x = 0;
float forza_y = 0;
Eigen::Vector2f p;    //Inizializzo la posizione dell'ostacolo rispetto al Laser
//Vado a ricavare le forza repulsive dalla nuvola di punti per ogni punto
for(auto& point: cloud.points){         //Ciclo su tutti i punti della nuvola
  p(0) = point.x;                       //Calcolo le componenti x,y che determinano la posizione dell'ostacolo
  p(1) = point.y;

  p = laser_transform * p;              //Ricavo posizione ostacolo nel S.R. del robot
  float distanza = sqrt(point.x * point.x + point.y * point.y);        //Calcolo distanza dell'ostacolo
  float forza_modulo = 1/(distanza*distanza);  //Modulo della forza repulsiva. E' inversamente proporzionale al quadrato della distanza dell'ostacolo. 
  forza_x += p(0) * forza_modulo;    //Aggiorno le componenti delle forze repulsive moltiplicandole ongi volta per il quadrato dell'inverso della distanza
  forza_y += p(1) * forza_modulo;
}

//Considero forze con direzione uguale e verso opposto ai vettori robot-ostacolo, perciò devo prendere l'opposto delle forze calcolate
forza_x = -forza_x;
forza_y = -forza_y;

geometry_msgs::Twist msg_send;
//In base alla velocità in input cambia anche la forza con cui agiamo; in questo modo dovremmo riuscire sempre a fermarci per qualunque velocità del Robot

//Calcolo della velocità angolare, agisco sulla componente in input del robot
msg_send.angular.z = vel_ang + forza_y * 0.00087 ;
                                                                    
//Modifico le due componenti della velocità del Robot
forza_x = forza_x * abs(vel_x)/485 ;                                                
forza_y = forza_y * abs(vel_y)/485 ;

msg_send.linear.x = forza_x + vel_x;
msg_send.linear.y = forza_y + vel_y;

vel.publish(msg_send);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "collision_avoidance");                   //inizializzo ROS specificando il nome del nodo
  ros::NodeHandle n;                                              //inizio del nodo roscpp
  //subscribe al topic cmd_vel                                                                
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel_call", 1, cmd_vel_callback);
  //subscriber al topic laser scan 
  ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1, laser_cmd_vel_callback);

  vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1000); 
  
  ros::spin();

  return 0;
}
#include <angles/angles.h>
#include "KinematicPositionController.h"
#include "tf_utils.hpp"

KinematicPositionController::KinematicPositionController(ros::NodeHandle& nh) :
  TrajectoryFollower(nh), transform_listener_( tfBuffer_ )
{
    expected_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
    
    ros::NodeHandle nhp("~");
    
    std::string goal_selection;
    nhp.param<std::string>("goal_selection", goal_selection, "TIME_BASED");
    nhp.param<double>("fixed_goal_x", fixed_goal_x_, 3);     
    nhp.param<double>("fixed_goal_y", fixed_goal_y_, 0);     
    nhp.param<double>("fixed_goal_a", fixed_goal_a_, -M_PI_2);     
    
    if(goal_selection == "TIME_BASED")
      goal_selection_ = TIME_BASED;
    else if(goal_selection == "PURSUIT_BASED")
      goal_selection_ = PURSUIT_BASED;
    else if(goal_selection == "FIXED_GOAL")
      goal_selection_ = FIXED_GOAL;
    else
      goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const ros::Time& t0, const ros::Time& t1, double y0, double y1, const ros::Time& t)
{
  return y0 + (t - t0).toSec() * (y1 - y0) / (t1 - t0).toSec();
}

bool KinematicPositionController::getCurrentPose(const ros::Time& t, double& x, double& y, double& a)
{
  tf2::Transform odom_to_robot;
  if (not lookupTransformSafe(tfBuffer_, "odom", "base_link", t, odom_to_robot))
    return false;

  x = odom_to_robot.getOrigin().getX();
  y = odom_to_robot.getOrigin().getY();

  a = tf2::getYaw(odom_to_robot.getRotation());

  return true;
}

#define K_X 0.1
#define K_Y 0.1
#define K_A 0.1


bool KinematicPositionController::control(const ros::Time& t, double& v_x, double& v_y, double& w_z)
{
  // Se obtiene la pose actual publicada por la odometria
  double current_x, current_y, current_a;
  if( not getCurrentPose(t, current_x, current_y, current_a) )
    return true;

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz

  // compute position differences with expected pose.
  double dx = goal_x - current_x;
  double dy = goal_y - current_y;
  double da = goal_a - current_a;

  // compute dx,dy,theta in goal reference frame
  double dx_rot = cos(-current_a) * dx - sin(-current_a) * dy;
  double dy_rot = sin(-current_a) * dx + cos(-current_a) * dy;

  // transform error data to polar coordinates.

  // use the control law to compute velocity commands.
  v_x = K_X * dx_rot;
  v_y = K_Y * dy_rot;
  w_z = K_A * da;

  ROS_INFO_STREAM("current_x: " << current_x << " current_y: " << current_y << " current_a: " << current_a << " v_x: " << v_x << " v_y: " << v_y << " w_z: " << w_z);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{ return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));}

bool KinematicPositionController::getPursuitBasedGoal(const ros::Time& t, double& x, double& y, double& a)
{
  // Los obtienen los valores de la posicion y orientacion actual.
  double current_x, current_y, current_a;
  if( not getCurrentPose(t, current_x, current_y, current_a) )
    return true;
   
  const robmovil_msgs::Trajectory& trajectory = getTrajectory();
  
  // Si nos encontramos "cerca" del final, se establece el ultimo wpoint como goal
  const robmovil_msgs::TrajectoryPoint& last_wpoint = trajectory.points.back(); 
  
  if(dist2(current_x, current_y, last_wpoint.transform.translation.x, last_wpoint.transform.translation.y) <= 0.25)
  {
    x = last_wpoint.transform.translation.x;
    y = last_wpoint.transform.translation.y;
    a = tf2::getYaw(last_wpoint.transform.rotation);
    
    return true;
  }
  
  // Se busca el waypoint mas cercano en terminos de x,y
  unsigned int closest_idx = 0;
  double closest_x = trajectory.points[0].transform.translation.x;
  double closest_y = trajectory.points[0].transform.translation.y;
  
  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    const robmovil_msgs::TrajectoryPoint& wpoint = trajectory.points[i];
    
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    
    if(dist2(current_x,current_y,wpoint_x,wpoint_y) < dist2(current_x,current_y,closest_x,closest_y)){
      closest_idx = i;
      closest_x = wpoint_x;
      closest_y = wpoint_y;
    }
  }
  
  /* Comenzando desde el wpoint mas cercano, se busca el siguiente wpoint que se distancie
   * en almenos 0.5 metros. Esto corresponde al concepto de lookahead en el controlador "PurePursuit"
   * NOTA: como el lookahead solo checkea distancia en x,y. Dado que no comprueba la diferencia
   *       de los angulos, esto solo es valido en trayectorias continuas con desplazamiento considerable en x,y. */
  for(unsigned int i = closest_idx; i < trajectory.points.size(); i++)
  {
    const robmovil_msgs::TrajectoryPoint& point = trajectory.points[i];
    
    double point_x = point.transform.translation.x;
    double point_y = point.transform.translation.y;
    double point_a = tf2::getYaw(point.transform.rotation);
    
    // Nos quedamos con el primer wpoint que cumpla con la distancia lookahead propuesta
    if(dist2(current_x,current_y,point_x,point_y) > 0.25){
      x = point_x;
      y = point_y;
      a = point_a;
      
      break;
    }
  }
  
  return true;
}






bool KinematicPositionController::getTimeBasedGoal(const ros::Time& t, double& x, double& y, double& a)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;
    
  ROS_INFO_STREAM("processing index: " << next_point_idx);

  const robmovil_msgs::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const ros::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const ros::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}

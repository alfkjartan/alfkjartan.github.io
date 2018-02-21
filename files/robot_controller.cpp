#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cstdlib>
#include <cstdio>

geometry_msgs::Pose current_pose;

void move_to_goal(geometry_msgs::Pose goal_pose,
		  double tol,
		  double Kv,
		  double Ka,
		  ros::Publisher pub);

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odometry_message);

int main(int argc, char **argv) {
    if (argc != 6) {
    ROS_INFO("Invalid number of parameters\nUsage: pioneer_controller X Y Theta Kv Ka");
    return 1;
  }

  ros::init(argc, argv, "pioneer_controller");
  ros::NodeHandle n;
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("pioneer2dx/cmd_vel", 10);
  ros::Subscriber odom_sub = n.subscribe("pioneer2dx/odom/", 10, odometry_callback);
  
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = atof(argv[1]);
  goal_pose.position.y = atof(argv[2]);
  goal_pose.position.z = 0;
  
  tf2::Quaternion quat;
  quat.setRPY(0, 0, atof(argv[3]));
  goal_pose.orientation.x = quat.x();
  goal_pose.orientation.y = quat.y();
  goal_pose.orientation.z = quat.z();
  goal_pose.orientation.w = quat.w();

  double tol = 0.1;
  double Kv = atof(argv[4]);
  double Ka = atof(argv[5]);
  
  move_to_goal(goal_pose, tol, Kv, Ka, twist_pub);
  
  return 0;
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odom_message){
  current_pose.position.x = odom_message->pose.pose.position.x;
  current_pose.position.y = odom_message->pose.pose.position.y;
  current_pose.orientation.x = odom_message->pose.pose.orientation.x;
  current_pose.orientation.y = odom_message->pose.pose.orientation.y;
  current_pose.orientation.z = odom_message->pose.pose.orientation.z;
  current_pose.orientation.w = odom_message->pose.pose.orientation.w;

}

  
void move_to_goal(geometry_msgs::Pose goal_pose,
		  double tol,
		  double Kv,
		  double Ka,
		  ros::Publisher pub){
  
  ros::Rate loop_rate(100);
  geometry_msgs::Twist vel_msg;

  double dist = 0;
  do {
    double dx = goal_pose.position.x - current_pose.position.x;
    double dy = goal_pose.position.y - current_pose.position.y;
    dist = sqrt( pow(dx,2) + pow(dy,2) );
    double speed = Kv*dist;

    tf2::Quaternion quat(current_pose.orientation.x,
			 current_pose.orientation.y,
			 current_pose.orientation.z,
			 current_pose.orientation.w);

    double currentOrientation = quat.getAngle();
    double desiredOrientation = atan2(dy, dx);
    double aerr = desiredOrientation - currentOrientation;
    aerr = atan2(sin(aerr), cos(aerr));

    std::cout << "Current angle =" << currentOrientation << std::endl;
    std::cout << "Desired angle =" << desiredOrientation << std::endl;
    std::cout << "Angle error =" << aerr << std::endl;



    vel_msg.linear.x = speed;
    vel_msg.angular.z = Ka*aerr;
    pub.publish(vel_msg);
    ros::spinOnce();
    
  } while ( (dist > tol) and ros::ok() );

  vel_msg.linear.x = 0;
  vel_msg.angular.z = 0;

  pub.publish(vel_msg);
  ros::spinOnce();
}




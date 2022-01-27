#include "ros/ros.h"
#include <geometric_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <mav_msgs/Actuators.h>
#include <geometric_controller.h>
#include <queue>
#include <nav_msgs/Odometry.h>

#define length 0.18
#define PI 3.1415926
double mp = 0.5, g = 9.8, kd = mp*g/(2*length), bd = 2.0, mF = 1.0, Md = 1.5, kf1 = 2.0;

geometry_msgs::PoseStamped desired_pose;
Eigen::Vector3d pose, vel, ang;
Eigen::Vector4d ori;
geometry_msgs::Point kappa;

bool flag = false;
bool force_control = false;

Eigen::Vector3d p_c2;
Eigen::Matrix3d payload_Rotation;
Eigen::Matrix3d uav_rotation;

geometry_msgs::PoseStamped force;
geometry_msgs::Point est_force;
geometry_msgs::Point data;

Eigen::Matrix3d payload_rotation_truth;
geometry_msgs::Point trigger;
bool triggered;

double payload_yaw;
void model_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){
  gazebo_msgs::LinkStates links = *msg;
  if(links.name.size()>0){

    for(unsigned int i=0; i<links.name.size(); i++){

      if (links.name[i].compare("payload::payload_rec_g_box") == 0 ){
        Eigen::Vector3d vec;
        double w,x,y,z;

        x = links.pose[i].orientation.x;
        y = links.pose[i].orientation.y;
        z = links.pose[i].orientation.z;
        w = links.pose[i].orientation.w;
        tf::Quaternion q(x,y,z,w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        //body to inertial frame
        payload_rotation_truth << cos(yaw), -sin(yaw),   0,
                                  sin(yaw),  cos(yaw),   0,
                                         0,         0,   1;
        data.z = payload_yaw - yaw;
      }
    }
  }
}

Eigen::VectorXd poly_t(double t){
  Eigen::VectorXd data;
  data.resize(6);
  data << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
  return data;
}

Eigen::Vector3d p_F_c2;
Eigen::Vector3d FF_I, FF_B, vb;
int count_ = 0;
std::queue<double> pos_x_buffer;
std::queue<double> pos_y_buffer;
Eigen::VectorXd coeff_x, coeff_y;

double last_time = 0;
double tmpx = 0, tmpy = 0;
double last_tmp_x = 0, last_tmp_y = 0;
double r, Fn, an;
double command;
double last_vec_x = 0, last_vec_y = 0, last_vx = 0, last_vy = 0;

void est_force_cb(const geometry_msgs::Point::ConstPtr& msg){
  est_force = *msg;
  FF_I << est_force.x, est_force.y, est_force.z;

  p_F_c2 = pose - uav_rotation * Eigen::Vector3d(0, 0, 0.05);  // offset x from uav to connector
  p_c2 = p_F_c2 + (FF_I/FF_I.norm()) * length;

  payload_yaw = atan2( tmpy - last_tmp_y, tmpx - last_tmp_x);

  if(payload_yaw < 0){
    payload_yaw += 2*PI;
  }

  payload_Rotation << cos(payload_yaw),  -sin(payload_yaw),   0,
                      sin(payload_yaw),   cos(payload_yaw),   0,
                                     0,                  0,   1;

  //change the force from inertial frame to body.
  if((count_ %1 == 0) && (pos_x_buffer.size()>7)){
    pos_x_buffer.pop();
    pos_y_buffer.pop();
    pos_x_buffer.push(p_c2(0));
    pos_y_buffer.push(p_c2(1));
    Eigen::VectorXd tx, ty;
    Eigen::MatrixXd w;
    tx.resize(7);
    ty.resize(7);
    w.resize(7,6);
    w << poly_t(0).transpose(),
         poly_t(0.02).transpose(),
         poly_t(0.04).transpose(),
         poly_t(0.06).transpose(),
         poly_t(0.08).transpose(),
         poly_t(0.10).transpose(),
         poly_t(0.12).transpose();

    double t1,t2,t3,t4,t5,t6,t7;

    t1 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t2 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t3 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t4 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t5 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t6 = pos_x_buffer.front();
    pos_x_buffer.pop();
    t7 = pos_x_buffer.front();
    pos_x_buffer.pop();

    tx << t1, t2, t3, t4, t5, t6, t7;
    coeff_x = (w.transpose() * w).inverse() * w.transpose()*tx;

    t1 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t2 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t3 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t4 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t5 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t6 = pos_y_buffer.front();
    pos_y_buffer.pop();
    t7 = pos_y_buffer.front();
    pos_y_buffer.pop();

    ty << t1, t2, t3, t4, t5, t6, t7;
    coeff_y = (w.transpose() * w).inverse() * w.transpose()* ty;

    double t = 0.14;
    tmpx = coeff_x(0)*1 + coeff_x(1)*t + coeff_x(2)*t*t + coeff_x(3)*t*t*t + coeff_x(4)*t*t*t*t + coeff_x(5)*t*t*t*t*t;
    tmpy = coeff_y(0)*1 + coeff_y(1)*t + coeff_y(2)*t*t + coeff_y(3)*t*t*t + coeff_y(4)*t*t*t*t + coeff_y(5)*t*t*t*t*t;
    tmpx = 0.7*last_tmp_x + 0.3*tmpx;    // filter
    tmpy = 0.7*last_tmp_y + 0.3*tmpy;

    double dt = ros::Time::now().toSec() - last_time;
    last_time = ros::Time::now().toSec();

    double vec_x = tmpx - last_tmp_x;
    double vec_y = tmpy - last_tmp_y;
    double vx = vec_x /dt;
    double vy = vec_y /dt;
    double ax = (vx - last_vx)/dt;
    double ay = (vy - last_vy)/dt;
    double v = sqrt(vx*vx + vy*vy);

    Eigen::Vector3d offset_b = payload_Rotation*(FF_I/FF_I.norm()) * length;

    r = (v*v*v)/fabs(vx*ay - vy*ax);
    an = (v*v)/r;
    Fn = mp/2 * an;

    //double fy = FF_B(1);
    double dy = -offset_b(1);

    if((atan2(vy,vx) - atan2(last_vy,last_vx))<0){
      //determine wheather the motion is CCW or CW, if CW Fn and dy is negative.
      Fn = Fn * -1.0;
    }

    command = mF*(bd*-vb(1) - kd*dy)/Md + Fn;

    kappa.x = r;
    kappa.y = Fn;
    kappa.z = dy;

    data.x = tmpx;
    data.y = tmpy;

    last_tmp_x = tmpx;
    last_tmp_y = tmpy;
    last_vec_x = vec_x;
    last_vec_y = vec_y;
    last_vx = vx;
    last_vy = vy;
  }
  else if (count_ %1 == 0){
    pos_x_buffer.push(p_c2(0));
    pos_y_buffer.push(p_c2(1));
  }
  count_++;
}

nav_msgs::Odometry odom;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
  odom = *msg;
  ori << odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z;
  pose << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
  vel << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
  ang << odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

  double w,x,y,z;
  x = msg->pose.pose.orientation.x;
  y = msg->pose.pose.orientation.y;
  z = msg->pose.pose.orientation.z;
  w = msg->pose.pose.orientation.w;

  uav_rotation << w*w+x*x-y*y-z*z,      2*x*y-2*w*z,     2*x*z+2*w*y,
                     2*x*y +2*w*z,  w*w-x*x+y*y-z*z,     2*y*z-2*w*x,
                     2*x*z -2*w*y,      2*y*z+2*w*x, w*w-x*x-y*y+z*z;
}

geometry_msgs::Point force2;
Eigen::Vector3d a_, b_;
void force2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  b_ << msg -> wrench.force.x, msg -> wrench.force.y, msg -> wrench.force.z;
  a_ = payload_rotation_truth * b_;

  force2.x = a_(0);
  force2.y = a_(1);
  force2.z = a_(2);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "geo2");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/firefly2/odometry_sensor1/odometry",3, odom_cb);
  ros::Subscriber est_force_sub = nh.subscribe<geometry_msgs::Point>("/follower_ukf/force_estimate",3, est_force_cb);
  ros::Subscriber force2_sub= nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor2_topic",2, force2_cb);
  ros::Subscriber model_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",1, model_cb);

  ros::Publisher traj_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly2/command/pose",2);
  ros::Publisher vc2_pub = nh.advertise<geometry_msgs::Point>("vc2_error",1);
  ros::Publisher trigger_pub = nh.advertise<geometry_msgs::Point>("/follower_trigger",2);
  ros::Publisher radius_pub = nh.advertise<geometry_msgs::Point>("/kappa",2);
  ros::Publisher force2_pub = nh.advertise<geometry_msgs::Point>("/force2",2);

  nh.setParam("/start2",false);
  nh.setParam("/force_control",false);

  triggered = false;
  ros::Rate loop_rate(50.0);

  desired_pose.pose.position.x = -0.3;
  desired_pose.pose.position.y = 0.0;
  desired_pose.pose.position.z = 1.3;

  while(ros::ok()){
    nh.getParam("/start2",flag);
    nh.getParam("/force_control",force_control);

    double ft = sqrt(FF_I(0)*FF_I(0) + FF_I(1)*FF_I(1));
    if((ft>0.3)){
      triggered = true;
    }
    else if((triggered)&&((ft<0.3)&&(ft>0.2))){
      triggered = true;
    }
    else{
      triggered = false;
    }

    if((triggered) && (force_control)){
      Eigen::Vector3d a, ab;

      FF_B = payload_Rotation * FF_I;
      vb = payload_Rotation * vel;

      ab <<                              kf1*-vb(0) + FF_B(0),
                                               command,
            5.0*(desired_pose.pose.position.z - pose(2)) + 2.0*(0 - vel(2)) + 0.5*mp*g;
      a = payload_Rotation.transpose()*ab;

      desired_pose.pose.position.x = pose(0);
      desired_pose.pose.position.y = pose(1);
      force.pose.position.x = a(0);
      force.pose.position.y = a(1);
      force.pose.position.z = a(2);

      trigger.x = 1;
    }
    else{
      force.pose.position.x = 3*(desired_pose.pose.position.x - pose(0)) + 1*(0 - vel(0));
      force.pose.position.y = 3*(desired_pose.pose.position.y - pose(1)) + 1*(0 - vel(1));
      force.pose.position.z = 3*(desired_pose.pose.position.z - pose(2)) + 1*(0 - vel(2)) + 0.5*mp*g;

      trigger.x = 0;
    }

    trigger.y = ft;

    traj_pub.publish(force);
    trigger_pub.publish(trigger);
    force2_pub.publish(force2);
    radius_pub.publish(kappa);
    vc2_pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

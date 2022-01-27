#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "lpf.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>

double l = 0.18, L = 0.5, g = 9.81;
Eigen::Vector3d pc1, pc2, pa, pb;
sensor_msgs::Imu imu_data;
Eigen::Matrix3d payload_rotation;
Eigen::Matrix3d uav_rotation;
Eigen::Vector3d omega_p;
void imu1_cb(const sensor_msgs::Imu::ConstPtr& msg){
  Eigen::Vector3d data;
  Eigen::Vector3d wdata;
  imu_data = *msg;
  data << imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z;
  wdata << imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z;

  double w,x,y,z;
  x = imu_data.orientation.x;
  y = imu_data.orientation.y;
  z = imu_data.orientation.z;
  w = imu_data.orientation.w;

  payload_rotation << w*w+x*x-y*y-z*z,     2*x*y-2*w*z,     2*x*z+2*w*y,
                          2*x*y+2*w*z, w*w-x*x+y*y-z*z,     2*y*z-2*w*x,
                          2*x*z-2*w*y,     2*y*z+2*w*x, w*w-x*x-y*y+z*z;

  pa = payload_rotation * data - Eigen::Vector3d(0,0,g);
  omega_p = payload_rotation*wdata;
}

Eigen::Vector3d PL;
void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg){

  PL << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  double w,x,y,z;
  x = msg->pose.pose.orientation.x;
  y = msg->pose.pose.orientation.y;
  z = msg->pose.pose.orientation.z;
  w = msg->pose.pose.orientation.w;

  uav_rotation << w*w+x*x-y*y-z*z,     2*x*y-2*w*z,     2*x*z+2*w*y,
                      2*x*y+2*w*z, w*w-x*x+y*y-z*z,     2*y*z-2*w*x,
                      2*x*z-2*w*y,     2*y*z+2*w*x, w*w-x*x-y*y+z*z;
}

void force_cb(const geometry_msgs::Point::ConstPtr& msg){
  Eigen::Vector3d f;
  f << msg->x, msg->y, msg->z;

  pb = PL - uav_rotation * Eigen::Vector3d(0,0,0.05);// offset x from uav to connector
  pc1 = pb + (f/f.norm())*l;

  //find pc2
  pc2 =  pc1 +  payload_rotation * Eigen::Vector3d(-1.0,0,0);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "so3");
  ros::NodeHandle nh;

  ros::Subscriber imu1_sub = nh.subscribe("/payload/IMU1",2,imu1_cb);
  ros::Subscriber odometry_sub = nh.subscribe("/firefly1/odometry_sensor1/odometry",2,odometry_cb);
  ros::Subscriber force_sub = nh.subscribe("/leader_ukf/force_estimate",2,force_cb);

  ros::Publisher point2_pub = nh.advertise<geometry_msgs::Point>("pointpc2",2);
  ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("pointvc2",2);
  ros::Publisher vel_est_pub = nh.advertise<geometry_msgs::Point>("est_vel",2);

  ros::Rate loop_rate(50);
  forceest forceest1(statesize,measurementsize);
  Eigen::MatrixXd mnoise;

  mnoise.setZero(measurementsize,measurementsize);
  mnoise = 3e-3*Eigen::MatrixXd::Identity(measurementsize , measurementsize);
  mnoise(mpc1_x,mpc1_x) = 2e-3;
  mnoise(mpc1_y,mpc1_y) = 2e-3;
  mnoise(mpc1_z,mpc1_z) = 2e-3;

  mnoise(mac1_x,mac1_x) = 2e-3;
  mnoise(mac1_y,mac1_y) = 2e-3;
  mnoise(mac1_z,mac1_z) = 2e-3;

  forceest1.set_measurement_noise(mnoise);

  Eigen::MatrixXd pnoise;
  pnoise.setZero(statesize,statesize);
  pnoise(pc1_x,pc1_x) = 1e-2;
  pnoise(pc1_y,pc1_y) = 1e-2;
  pnoise(pc1_z,pc1_z) = 1e-2;

  pnoise(vc1_x,vc1_x) = 1e-2;
  pnoise(vc1_y,vc1_y) = 1e-2;
  pnoise(vc1_z,vc1_z) = 1e-2;

  pnoise(ac1_x,ac1_x) = 1e-2;
  pnoise(ac1_y,ac1_y) = 1e-2;
  pnoise(ac1_z,ac1_z) = 1e-2;

  forceest1.set_process_noise(pnoise);

  Eigen::MatrixXd measurement_matrix;
  measurement_matrix.setZero(measurementsize,statesize);

  measurement_matrix(mpc1_x, pc1_x ) = 1;
  measurement_matrix(mpc1_y, pc1_y ) = 1;
  measurement_matrix(mpc1_z, pc1_z ) = 1;

  measurement_matrix(mac1_x, ac1_x ) = 1;
  measurement_matrix(mac1_y, ac1_y ) = 1;
  measurement_matrix(mac1_z, ac1_z ) = 1;

  forceest1.set_measurement_matrix(measurement_matrix);

  while(ros::ok()){
    forceest1.predict();

    Eigen::VectorXd measure;

    measure.setZero(measurementsize);

    measure << pc1(0), pc1(1), pc1(2), pa(0), pa(1), pa(2);

    forceest1.correct(measure);
    geometry_msgs::Point point, point2, point_vel;
    Eigen::Vector3d vc1_I = Eigen::Vector3d(forceest1.x[vc1_x], forceest1.x[vc1_y], 0);
    point_vel.x = forceest1.x[vc1_x];
    point_vel.y = forceest1.x[vc1_y];
    point_vel.z = forceest1.x[vc1_z];

    Eigen::Vector3d vc1_B = payload_rotation.transpose()*vc1_I;
    point.x = vc1_B(0);    //linear velocity in payload frame
    point.z = omega_p(2);

    point2.x = pc2(0);  //position in inertial frame
    point2.y = pc2(1);
    point2.z = 0;

    vel_est_pub.publish(point_vel);
    point_pub.publish(point);
    point2_pub.publish(point2);
    loop_rate.sleep();
    ros::spinOnce();
  }
  ROS_INFO("Hello world!");
}

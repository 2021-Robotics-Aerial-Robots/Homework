#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "hw11/kalman.h"

#include <queue>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "kf_test");
    ros::NodeHandle nh;

  int n = 3; // Number of states
  int m = 1; // Number of measurements

  double dt = 1.0 ; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix (3*3)
  Eigen::MatrixXd B(n, m); // input control (3*1)
  Eigen::MatrixXd C(m, n); // Output matrix (1*3)
  Eigen::MatrixXd Q(n, n); // Process noise covariance (3*3)
  Eigen::MatrixXd R(m, m); // Measurement noise covariance (1*1)
  Eigen::MatrixXd P(n, n); // Estimate error covariance (3*3)

  // Please set parameter in this part and use the dynamic model parameter.
  // Discrete LTI projectile motion, measuring position only

  //

  std::cout << "A: \n" << A << std::endl;
  std::cout << "C: \n" << C << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt,A, B, C, Q, R, P);

  // List of noisy position measurements (y)
  std::vector<double> measurements =
  {
        -41.3476,-10.56936,-0.09996,6.33969,45.89824,45.62117,44.61856,51.14404,60.73781,86.47371,131.77911,118.70147,198.3733,243.21885,223.93135,261.76944,295.08487,304.72116,347.15858,390.1038,445.32955,502.8035,544.69698,553.10905,619.93109,679.36774,691.80668,775.08624,819.29884,877.79287,979.42744,1056.82776,1073.63762,1145.40761,1212.50599,1308.97609,1343.82864,1447.64044,1513.31818,1633.95831,1702.98397,1746.72186,1868.41031,1910.65399,2030.43845,2135.39993
  };

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  x0 << measurements[0], 0, -9.81;
  kf.init(dt,x0);

  // Feed measurements into filter, output estimated states
  double t = 0;
  Eigen::VectorXd y(m);
  std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
  std::queue<double> answer;
  for (int i = 0; i < measurements.size(); i++) {
    t += dt;
    y << measurements[i];
    std::cout<<"y"<<y<<std::endl;
    kf.update(y);
    //show the position velocity acceleration
    //std::cout <<"kf.state().transpose()" <<kf.state().transpose() << std::endl;
    std::cout<<"kf.state()(0,0)"<<kf.state()(0,0)<<std::endl;
    answer.push(kf.state()(0,0));
  }

  std::ofstream in;
  //chage your path "home/ee405423/Desktop"
  in.open("/home/ncrl/robot_ws/src/Homework/hw11/src/data.csv",std::ios::out | std::ios::app);
  int len = answer.size();
  for(int i =0; i<len;i++)
  {
    std::cout<<"answer.front()"<<answer.front()<<std::endl;
    in << answer.front()<<",";

    answer.pop();
  }
    ros::spin();
  return 0;
}

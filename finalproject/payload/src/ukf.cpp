#include "ukf.h"

ukf::ukf(int state_size , int measurement_size){

  x_size = state_size;
  y_size = measurement_size;
  alpha = 1e-3;
  kappa = 0;
  beta = 2;
  lambda = 0.0;

  L=x_size;
  x_sigmavector_size = ?;

  lambda= alpha * alpha * (L + kappa) -L;

  x.setZero(x_size);
  y.setZero(y_size);

  x_hat.setZero(x_size);
  y_hat.setZero(y_size);

  x_a.setZero(x_size+x_size+y_size);

  x_sigmavector.setZero(x_size,x_sigmavector_size);
  y_sigmavector.setZero(x_sigmavector_size,y_size);

  H.setZero(y_size,x_size);  // measurement matrix

  y = H*x;

  w_c.setZero(x_sigmavector_size);
  w_m.setZero(x_sigmavector_size);

  w_c(0) = ?;
  w_m(0) = ?;

  for(int i=1 ; i<x_sigmavector_size ; i++){
    w_c(i) = ?;
    w_m(i) = ?;
  }

  // default Q R P matrix
  Q =5e-7*Eigen::MatrixXd::Identity(x_size, x_size);
  R =5e-4*Eigen::MatrixXd::Identity(y_size,y_size);
  P=1e-3*Eigen::MatrixXd::Identity(x_size, x_size);

  P_.setZero(x_size,x_size);
  P_yy.setZero(y_size,y_size);
  P_xy.setZero(x_size,y_size);
}

//time update
void ukf::predict(){

  //find sigma point
  P=(lambda+L)*P;
  Eigen::MatrixXd M= (P).llt().matrixL();
  Eigen::MatrixXd buffer;
  x_sigmavector.col(0) = x;

  for(int i=0;i<x_size;i++){
    Eigen::VectorXd sigma =(M.row(i)).transpose();
    x_sigmavector.col(i+1) = ?;
    x_sigmavector.col(i+x_size+1) = ?;
  }

  // process model
  x_sigmavector = ?;

  //x_hat (mean)
  x_hat.setZero(x_size);   //initialize x_hat

  for(int i=0;i<x_sigmavector_size;i++){
    x_hat += ?;
  }
  //covariance
  P_.setZero(x_size,x_size);

  for(int i=0 ; i<x_sigmavector_size ;i++){
    P_+= ?;
  }

  //add process noise covariance
  P_+= Q;

  // measurement model
  y_sigmavector = ?;


  //y_hat (mean)
  y_hat.setZero(y_size);

  for(int i=0;i< x_sigmavector_size;i++){
    y_hat += ?;
  }
}

//measurement update
void ukf::correct(Eigen::VectorXd measure){

    y=measure;

    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);

    for(int i=0;i<x_sigmavector_size;i++){
      Eigen::MatrixXd y_err;
      Eigen::MatrixXd y_err_t;
      y_err = ?;
      y_err_t = err.transpose();

      P_yy += ?;
    }

    //add measurement noise covarinace
    P_yy +=R;

    for(int i=0;i<x_sigmavector_size;i++){
      Eigen::VectorXd y_err , x_err;
      err_y = ?;
      err_x = ?;
      P_xy += ?;
    }

    Kalman_gain = ?;

    // correct states and covariance
    x = ?;
    P = ?;
}

Eigen::MatrixXd ukf::dynamics(Eigen::MatrixXd sigma_state){
  Eigen::MatrixXd a = sigma_state;
  return a;
}

void ukf::set_measurement_matrix(Eigen::MatrixXd matrix){
    H = matrix;
}
void ukf::set_process_noise(Eigen::MatrixXd matrix){
    this->Q = matrix;
}

void ukf::set_measurement_noise(Eigen::MatrixXd matrix){
    this->R = matrix;
}

ukf::~ukf(){
}

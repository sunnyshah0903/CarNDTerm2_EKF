#include "kalman_filter.h"
#include <string>
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  I = MatrixXd::Identity(2, 2);

}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	//new state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float x = x_(0);
    float y1 = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    float rho = sqrt(x*x+y1*y1);
    if(rho < 0.0001) {
   		rho = 0.0001;
    }
    float theta = atan2(y1,x);
    theta = atan2(sin(theta ),cos(theta ));
    float ro_dot = (x*vx+y1*vy)/rho;
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, theta,ro_dot;
    VectorXd y = z - z_pred;
    y[1] = atan2(sin(y[1]),cos(y[1]));

	long size = x_.size();
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;


  //new state
	x_ = x_ + (K * y);
	long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}


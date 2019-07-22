#include "kalman_filter.h"
using namespace std;


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
}

void KalmanFilter::Predict() {
  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
 
  VectorXd z_pred = H_ * x_;

  UpdateWithAlreadyPredictedMeasurements(z, z_pred);


}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px*px+py*py);
  double phi = atan2(py,px);
  double rho_dot = (px*vx + py*vy)/rho;

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  UpdateWithAlreadyPredictedMeasurements(z, z_pred);



}

void KalmanFilter::UpdateWithAlreadyPredictedMeasurements(const VectorXd& z, const VectorXd& z_pred) {
  /**
  TODO:
    * update after predicting
  */

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  
}

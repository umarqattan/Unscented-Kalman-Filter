#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

#define EPS 0.001 // Just a small number

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  // initial state vector
  x_ = VectorXd(5);
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.3;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.45;
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;
  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;
  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;
  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  // State dimension
  n_x_ = x_.size();
  // Augmented state dimension
  n_aug_ = n_x_ + 2;
  
  // Set the predicted sigma points matrix dimentions
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  
  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  
  // Measurement noise covariance matrices initialization
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;
  
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_,0,
            0,std_laspy_*std_laspy_;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  
  if (!is_initialized_) {
   
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    
    switch (measurement_pack.sensor_type_)
    {
      case MeasurementPackage::LASER:
      
        x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
        
        if (fabs(x_(0)) < 0.001 && fabs(x_(1)) < 0.001){
          x_(0) = 0.001;
          x_(1) = 0.001;
        }
        break;
      case MeasurementPackage::RADAR:
        
        float rho = measurement_pack.raw_measurements_[0];
        float phi = measurement_pack.raw_measurements_[1];
        float rho_dot = measurement_pack.raw_measurements_[2];
        
        float px = rho * cos(phi);
        float py = rho * sin(phi);
        float vx = rho_dot * cos(phi);
        float vy = rho_dot * sin(phi);
        float v  = sqrt(vx * vx + vy * vy);
        x_ << px, py, v, 0, 0;
        break;
    }
  
    
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
      weights_(i) = 0.5 / (n_aug_ + lambda_);
    }

    
    time_us_ = measurement_pack.timestamp_;
    
    is_initialized_ = true;

  }
  else {

    double dt = (measurement_pack.timestamp_ - time_us_)/1000000.0;
    time_us_ = measurement_pack.timestamp_;
    Prediction(dt);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      UpdateRadar(measurement_pack);
    }
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      UpdateLidar(measurement_pack);
    }
  }
}


void UKF::Prediction(double delta_t) {
  double delta_t2 = delta_t*delta_t;
 
  VectorXd x_aug = VectorXd(n_aug_);
 
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
 
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  MatrixXd L = P_aug.llt().matrixL();
  
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_)*L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*L.col(i);
  }


  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);


    double px_p, py_p;
    if (fabs(yawd) > 0.001) {
      px_p = p_x + (v/yawd) * (sin(yaw+yawd*delta_t) - sin(yaw));
      py_p = p_y + (v/yawd) * (cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    double v_p = v;
    double yaw_p = yaw+yawd*delta_t;
    double yawd_p = yawd;

    px_p += 0.5*nu_a*delta_t2*cos(yaw);
    py_p += 0.5*nu_a*delta_t2*sin(yaw);
    v_p += nu_a*delta_t;
    yaw_p += 0.5*nu_yawdd*delta_t2;
    yawd_p += nu_yawdd*delta_t;

    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  x_ = Xsig_pred_ * weights_; // vectorised sum
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while(x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;
    while(x_diff(3) >  M_PI) x_diff(3) -= 2.0*M_PI;
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {

  int n_z = 3;
  
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  
  for (int i = 0; i < 2*n_aug_+1; i++) {
    
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    
    double rho = sqrt(p_x*p_x + p_y*p_y);
    double phi = atan2(p_y,p_x);
    double rho_dot = (p_x*v1 + p_y*v2)/rho;
    Zsig(0,i) = rho;
    Zsig(1,i) = phi;
    Zsig(2,i) = rho_dot;
  }
  UpdateUKF(meas_package, Zsig, n_z);
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {

  int n_z = 2;
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2*n_aug_+1);
  UpdateUKF(meas_package, Zsig, n_z);
}

void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z){
  VectorXd z_pred = VectorXd(n_z);
  z_pred  = Zsig * weights_;
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {

    VectorXd z_diff = Zsig.col(i) - z_pred;

    while(z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;
    while(z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  MatrixXd R = MatrixXd(n_z, n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    R = R_radar_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    R = R_lidar_;
  }
  S = S + R;
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      while(z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;
      while(z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
    }
    
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    while(x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;
    while(x_diff(3) >  M_PI) x_diff(3) -= 2.0*M_PI;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  VectorXd z = meas_package.raw_measurements_;
  
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    while(z_diff(1) < -M_PI) z_diff(1) += 2.0*M_PI;
    while(z_diff(1) >  M_PI) z_diff(1) -= 2.0*M_PI;
  }
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
  
  double NIS = z.transpose() * S.inverse() * z;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    NIS_radar_ = NIS;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    NIS_laser_ = NIS;
  }
}


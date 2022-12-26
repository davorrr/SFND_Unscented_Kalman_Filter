#include<iostream>

#include "ukf.h"
#include "Eigen/Dense"

using std::cout;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Seting initialization flag
  is_initialized_ = false;

  // Timestamping
  t_previous_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 6.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.0;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // set measurement dimension, radar can measure r, phi, r_vel
  n_z_R_ = 3;

  // set measurement dimension, lidar can measure px, py
  n_z_L_ = 2;

  // Normalized Innovation Squared for Radar
  NIS_Radar_ = 0;

  // Normalized Innovation Squared for Lidar
  NIS_Lidar_ = 0;
  
  // Working variables for Radar NIS calculation
  total_Radar_measurements = 0;
  NIS_Radar_95 = 0;
  Radar_95_perc = 0; 
  NIS_Radar_5 = 0;
  Radar_5_perc = 0;

  // Working variables for Lidar NIS calculation
  total_Lidar_measurements = 0;
  NIS_Lidar_95 = 0;
  Lidar_95_perc = 0; 
  NIS_Lidar_5 = 0;
  Lidar_5_perc = 0;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // Radar Measurment noise covariance matrix
  R = MatrixXd(n_z_R_, n_z_R_);

  // Lidar Measurement noise covariance matrix
  L = MatrixXd(n_z_L_, n_z_L_);

  /**
   * Intermeditate matrices
   */

  // Weights of sigma points
  weights_ = VectorXd::Zero(2*n_aug_+1);

  // create augmented mean vector
  x_aug = VectorXd::Zero(n_aug_);

  // create augmented state covariance
  P_aug = MatrixXd::Zero(n_aug_, n_aug_);

  // Sigma points matrix -necessary to have 2 x state vector size + 1 sigma points
  Xsig = MatrixXd::Zero(n_x_, 2*n_x_ +1);

  // Create sigma point matrix
  Xsig_aug = MatrixXd::Zero(n_aug_, 2*n_aug_+1);

  // Sigma points in radar measurement space
  Zsig_r = MatrixXd::Zero(n_z_R_, 2*n_aug_+1);

  // Mean predicted radar measurement
  z_pred_r = VectorXd::Zero(n_z_R_);

  // Radar measurement covariance matrix S_r
  S_r = MatrixXd::Zero(n_z_R_, n_z_R_);

  // Sigma points in lidar measurement space
  Zsig_l = MatrixXd::Zero(n_z_L_, 2*n_aug_+1);

  // Mean predicted lidar measurement
  z_pred_l = VectorXd::Zero(n_z_L_);

  // Lidar measurement covariance matrix S_l
  S_l = MatrixXd::Zero(n_z_L_, n_z_L_);
   
  /**
   * Working matrices
   */

  // Predicted sigma points - output of PredictRadarMEasurements()
  Xsig_pred = MatrixXd::Zero(n_x_, 2*n_aug_+1);

  // Predicted mean state vector
  x = VectorXd::Zero(n_x_);

  // Predicted covariance matrix
  P = MatrixXd::Zero(n_x_, n_x_);

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (is_initialized_ == false){

    // Initialize covariance matrix
    P <<  0.15,    0,    0,    0,    0,
             0, 0.15,    0,    0,    0,
             0,    0,    0,    0,    0,
             0,    0,    0,    0,    0,
             0,    0,    0,    0,    0;
   
    // Initialize weights
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    double weight = 0.5/(n_aug_ + lambda_);
    for(int i = 1; i<2*n_aug_ + 1; ++i){   
      weights_(i) = weight;
    }
    
    // Initialize Radar Measurment noise covariance matrix
    R << std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0, std_radrd_*std_radrd_;

    // Initialize Lidar Measurement noise covariance matrix
    L << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
    
    if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;   

    }

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      float ro = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      x << ro*cos(phi), ro*sin(phi), 0, 0, 0;
    }
    t_previous_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double dt = static_cast<double>((meas_package.timestamp_ - t_previous_)*1e-6);
  t_previous_ = meas_package.timestamp_;
  
  Prediction(dt);

  // Process results according to the sensor type
  if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // Processing for Lidar data 
    UpdateLidar(meas_package);
  }

  if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
    // Processing for RADAR data 
   UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  GenerateSigmaPointsAugmented();

  SigmaPointPrediction(delta_t);

  PredictMeanAndCovariance();

}


void UKF::GenerateSigmaPointsAugmented(){

  // Generating sigma points on the of space distribution 

  // create augmented mean state
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // Covariance P square root
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  
  for(int i = 0; i < n_aug_; ++i){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_)*L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*L.col(i);
  }
}

void UKF::SigmaPointPrediction(const double delta_t_){

  // Sigma points prediction
  for(int i = 0; i < 2*n_aug_+1; ++i){
    // extracting values 
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yaw_vel = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yaw_acc = Xsig_aug(6,i);

    double px_p, py_p;

    if(fabs(yaw_vel) > 0.001){
      px_p = px + v/yaw_vel*(sin(yaw + yaw_vel*delta_t_) - sin(yaw));
      py_p = py + v/yaw_vel*( cos(yaw) - cos(yaw + yaw_vel*delta_t_));
    } else {
      px_p = px + v*delta_t_*cos(yaw);
      py_p = py + v*delta_t_*cos(yaw);
    }
  
    double v_p = v;
    double yaw_p = yaw + yaw_vel*delta_t_;
    double yaw_vel_p = yaw_vel;

    // Adding noise
    px_p = px_p + 0.5*nu_a*delta_t_*delta_t_*cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t_*delta_t_*sin(yaw);
    v_p = v_p + nu_a*delta_t_;

    yaw_p = yaw_p + 0.5*nu_yaw_acc*delta_t_*delta_t_;
    yaw_vel_p = yaw_vel_p + nu_yaw_acc*delta_t_; 

    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yaw_vel_p;
  
  }

}

void UKF::PredictMeanAndCovariance(){

  x.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    x = x + weights_(i) * Xsig_pred.col(i);
  }

  P.fill(0.0);
  for(int i = 0; i < 2*n_aug_ + 1; ++i){
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    
    // angle normalization
    while(x_diff(3)> M_PI)  x_diff(3) -= 2.*M_PI;
    while(x_diff(3)< -M_PI) x_diff(3) += 2.*M_PI;

    P = P + weights_(i)*x_diff*x_diff.transpose();
  }

}

void UKF::PredictRadarMeasurement(){

  for(int i = 0; i < 2*n_aug_+1; ++i){
    // extract values for better readibility
    double px = Xsig_pred(0,i);
    double py = Xsig_pred(1,i);
    double v = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // Radar measurement model
    Zsig_r(0,i) = sqrt(px*px +py*py);                   // r
    Zsig_r(1,i) = atan2(py, px);                        // phi
    Zsig_r(2,i) = (px*v1 + py*v2) / sqrt(px*px +py*py); // r_vel
  }

  // mean predicted measurement
  z_pred_r.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
    z_pred_r = z_pred_r + weights_(i)*Zsig_r.col(i);

  // innovation covariance matrix S
  S_r.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; ++i) {
    // residual
    VectorXd z_diff = Zsig_r.col(i) - z_pred_r;

    // angle normalization
    while (z_diff(1) > M_PI) 
      z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) 
      z_diff(1) += 2.*M_PI;

    S_r = S_r + weights_(i) * z_diff * z_diff.transpose();     
  }

  S_r = S_r + R;

}

void UKF::PredictLidarMeasurement(){

  for(int i = 0; i < 2*n_aug_+1; i++){
    // extract values for better readibility
    double px = Xsig_pred(0,i);
    double py = Xsig_pred(1,i);

    // Radar measurement model
    Zsig_l(0,i) = px;                 
    Zsig_l(1,i) = py;                 
  }

  // mean predicted measurement
  z_pred_l.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++)
    z_pred_l = z_pred_l + weights_(i)*Zsig_l.col(i);

  // innovation covariance matrix S
  S_l.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++) {
    // residual
    VectorXd z_diff = Zsig_l.col(i) - z_pred_l;

    S_l = S_l + weights_(i) * z_diff * z_diff.transpose();     
  }
  S_l = S_l + L;

}

void UKF::UpdateStateRadar(const VectorXd& z){

  // create matrix fro cross correlation TC
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_R_);
  
  // calculate cross correlation matrix
 
  for(int i = 0; i < 2*n_aug_+1; ++i) {
    // residual
    VectorXd z_diff = Zsig_r.col(i) - z_pred_r;

    // angle normalization
    while (z_diff(1) > M_PI) 
      z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) 
      z_diff(1) += 2.*M_PI;

    // state difference 
    VectorXd x_diff = Xsig_pred.col(i) - x;
      
    // angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc * S_r.inverse();

  // residual
  VectorXd z_diff = z - z_pred_r;

  // angle normalization
  while (z_diff(1) > M_PI) 
    z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) 
    z_diff(1) += 2.*M_PI;

  // update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K * S_r *K.transpose();

}

void UKF::UpdateStateLidar(const VectorXd& z){

  // create matrix fro cross correlation TC
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_L_);
  
  // calculate cross correlation matrix
  for(int i = 0; i < 2*n_aug_+1; i++) {
    // residual
    VectorXd z_diff = Zsig_l.col(i) - z_pred_l;

    // state difference 
    VectorXd x_diff = Xsig_pred.col(i) - x;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc * S_l.inverse();

  // residual
  VectorXd z_diff = z - z_pred_l;

  // update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K * S_l *K.transpose();

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  PredictLidarMeasurement();
  NIS_Lidar_ = (meas_package.raw_measurements_ - z_pred_l).transpose() * S_l.inverse() * (meas_package.raw_measurements_ - z_pred_l);
  std::cout << "NIS_Lidar = " << NIS_Lidar_ << std::endl;
  total_Lidar_measurements++;
  
  if(NIS_Lidar_ > 0.103){
    NIS_Lidar_95++;
    Lidar_95_perc = NIS_Lidar_95*100/total_Lidar_measurements;
  }
   if(NIS_Lidar_ > 5.991){
    NIS_Lidar_5++;
    Lidar_5_perc = NIS_Lidar_5*100/total_Lidar_measurements;
  }
  std::cout << "Lidar measurements above 95% line = " << Lidar_95_perc << std::endl;
  std::cout << "Lidar measurements above 5% line = " << Lidar_5_perc << std::endl;
  
  UpdateStateLidar(meas_package.raw_measurements_);

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  PredictRadarMeasurement();

  NIS_Radar_ = (meas_package.raw_measurements_ - z_pred_r).transpose() * S_r.inverse() * (meas_package.raw_measurements_ - z_pred_r);
  std::cout << "NIS_Radar = " << NIS_Radar_ << std::endl;
  total_Radar_measurements++;
  if(NIS_Radar_ > 0.352){
    NIS_Radar_95++;
    Radar_95_perc = NIS_Radar_95*100/total_Radar_measurements;
  }
   if(NIS_Radar_ > 7.815){
    NIS_Radar_5++;
    Radar_5_perc = NIS_Radar_5*100/total_Radar_measurements;
  }
  std::cout << "Radar measurements above 95% line = " << Radar_95_perc << std::endl; 
  std::cout << "Radar measurements above 5% line = " << Radar_5_perc << std::endl;
 
  UpdateStateRadar(meas_package.raw_measurements_);

}
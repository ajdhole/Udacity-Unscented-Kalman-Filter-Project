#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    
  // state dimension
    n_x_ = 5;
    
  // Augmented state dimension
    n_aug_ = 7;
    
  // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;
    
    lambda_aug_ = sqrt(lambda_ + n_aug_);
    sigma_points_ = 2 * n_aug_ + 1;

  // initial state vector[px, py, v, yaw, yaw_dot]
    x_ = VectorXd(n_x_);
    x_.fill(0.0);

  // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);
    P_.fill(0.0);
    for (int i=0; i<n_x_; i++) {P_(i, i) = 0.5;}
    
    
    H_ = MatrixXd(2, n_x_);
    H_.fill(0.0);
    H_(0,0) = 1.0;
    H_(1,1) = 1.0;
    
    //create vector for weights

    weights_ = VectorXd(sigma_points_); // initial weights matrix
    for (int i=1; i<sigma_points_; i++) {weights_[i] = 0.5/(n_aug_+lambda_);} // generate weights values
    weights_[0] = lambda_/(n_aug_+lambda_);


  // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 1.5;

  // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.1;

  // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.1;

  // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.25;

  // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
<<<<<<< HEAD
  // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;
    
  // time when the state is true, in us
    time_us_ = 0.0;
    
  // predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, sigma_points_);
    Xsig_pred_.fill(0.0); // initialize X sigma prediction matrix to 0.0

  // the current NIS for radar
  nis_radar_ = 0.0;
    
  // the current NIS for laser
  nis_laser_ = 0.0;
    
=======
    // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // time when the state is true, in us
  time_us_ = 0.0;

  // state dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);

  // the current NIS for radar
  NIS_radar_ = 0.0;

  // the current NIS for laser
  NIS_laser_ = 0.0;
  
>>>>>>> a07874862bdd19a036bcb3336242a56359deaef2
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
    
    double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
    time_us_ = meas_package.timestamp_;
    if (!is_initialized_) {
        cout << "Initializing UKF using first measurement... \n";
        if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
            x_[0] = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
            x_[1] = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
            is_initialized_ = true;
        }
        else if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
            x_[0] = meas_package.raw_measurements_[0];
            x_[1] = meas_package.raw_measurements_[1];
            is_initialized_ = true;
        }
    }
    else {
        Prediction(delta_t);
        if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
            UpdateRadar(meas_package);
        }
        else if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
            UpdateLidar(meas_package);
        }
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
    //create and populate augmented mean state
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.fill(0.0);
    x_aug.head(n_x_) = x_;
    
    //create and populate augmented covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_*std_a_;
    P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;
    
    //calculate square root matrix
    MatrixXd A_aug = P_aug.llt().matrixL();
    
    //create augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(n_aug_, sigma_points_);
    MatrixXd col = x_aug;
    Xsig_aug.col(0) = col;
    for (int i=0; i<n_aug_; i++) {
        col = A_aug.col(i) * lambda_aug_;
        Xsig_aug.col(i+1) = x_aug + col;
        Xsig_aug.col(i+1+n_aug_) = x_aug - col;
    }
    
    //predict augmented sigma points
    double half_dt_square = 0.5 * delta_t * delta_t;
    for (int i=0; i<sigma_points_; i++) {
        VectorXd col = Xsig_aug.col(i);
        Xsig_pred_.col(i) = col.head(5);
        
        // add noise values
        Xsig_pred_(0, i) += half_dt_square * cos(col[3]) * col[5];
        Xsig_pred_(1, i) += half_dt_square * sin(col[3]) * col[5];
        Xsig_pred_(2, i) += delta_t * col[5];
        Xsig_pred_(3, i) += half_dt_square * col[6];
        Xsig_pred_(4, i) += delta_t * col[6];
        
        // check for divide by zero
        if (fabs(Xsig_aug(4, i)) <= 0.001) {
            Xsig_pred_(0, i) += col[2] * cos(col[3]) * delta_t;
            Xsig_pred_(1, i) += col[2] * sin(col[3]) * delta_t;
        }
        else {
            double fract = col[2] / col[4];
            double psi_dt = col[4] * delta_t;
            Xsig_pred_(0, i) += fract * ((sin(col[3]+psi_dt))-sin(col[3]));
            Xsig_pred_(1, i) += fract * ((-cos(col[3]+psi_dt))+cos(col[3]));
            Xsig_pred_(3, i) += psi_dt;
        }
    }
    
    //predict state mean
    x_.fill(0.0);
    for (int i=0; i<sigma_points_; i++) {x_ += Xsig_pred_.col(i) * weights_[i];}
    
    //predict state covariance matrix
    P_.fill(0.0);
    for (int i=0; i<sigma_points_; i++) {
        MatrixXd x_diff = (Xsig_pred_.col(i) - x_);
        x_diff(3) = NormalizeAngle(x_diff(3));
        
        P_ += weights_[i] * x_diff * x_diff.transpose();
    }
}
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    //calculate measurement covariance matrix S
    MatrixXd S = MatrixXd(2, 2);
    S << std_laspx_*std_laspx_, 0,
    0, std_laspy_*std_laspy_; // add measurement noise covariance
    S += H_ * P_ * H_.transpose();
    
    // calculate error term y
    VectorXd y = meas_package.raw_measurements_ - H_ * x_;
    
    // calculate Kalman gain K
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    
    // update state mean and covariance matrix
    x_ += K * y;
    x_[3] = NormalizeAngle(x_[3]);
    P_ = (MatrixXd::Identity(5, 5) - K * H_) * P_;
    
    // calculate NIS value
    nis_laser_ = y.transpose() * S.inverse() * y;
    cout << "\nL\t" << nis_laser_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    //transform sigma points into measurement space
    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, sigma_points_);
    Zsig.fill(0.0);
    for (int i=0; i<sigma_points_; i++) {
        VectorXd col = Xsig_pred_.col(i);
        Zsig(0, i) = sqrt(col[0]*col[0] + col[1]*col[1]);
        Zsig(1, i) = atan2(col[1], col[0]);
        Zsig(2, i) = ((col[0]*cos(col[3])*col[2]) + (col[1]*sin(col[3])*col[2]))/Zsig(0,i);
    }
    
    //calculate mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i<sigma_points_; i++) {z_pred += weights_[i] * Zsig.col(i);}
    
    //calculate measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S << std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0, std_radrd_*std_radrd_; // add measurement noise covariance
    
    for (int i=0; i<sigma_points_; i++) {
        MatrixXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = NormalizeAngle(z_diff(1));
        S += weights_[i] * z_diff * z_diff.transpose();
    }
    
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    
    for (int i=0; i<sigma_points_; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = NormalizeAngle(x_diff(3));
        
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = NormalizeAngle(z_diff(1));
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    // calculate error term y
    VectorXd y = meas_package.raw_measurements_ - z_pred;
    y(1) = NormalizeAngle(y(1));
    
    // calculate Kalman gain K
    MatrixXd K = Tc * S.inverse();
    
    // update state mean and covariance matrix
    x_ += K * y;
    x_[3] = NormalizeAngle(x_[3]);
    P_ -= K * S * K.transpose();
    
    // calculate NIS value
    nis_radar_ = y.transpose() * S.inverse() * y;
    cout << "\nR:\t" << nis_radar_;
}

double UKF::NormalizeAngle(double angle) {
    while (angle> M_PI) angle-=2.0*M_PI;
    while (angle<-M_PI) angle+=2.0*M_PI;
    return angle;
}

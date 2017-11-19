#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
    bool is_initialized_; ///* initially set to false, set to true in first call of ProcessMeasurement
    bool use_laser_; ///* if this is false, laser measurements will be ignored (except for init)
    bool use_radar_; ///* if this is false, radar measurements will be ignored (except for init)
    
    VectorXd x_; ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    MatrixXd P_; ///* state covariance matrix
    MatrixXd H_; ///* Measurement matrix for laser
    
    MatrixXd Xsig_pred_; ///* predicted sigma points matrix
    
    long long time_us_; ///* time when the state is true, in us
    
    double std_a_; ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_yawdd_; ///* Process noise standard deviation yaw acceleration in rad/s^2
    
    double std_laspx_; ///* Laser measurement noise standard deviation position1 in m
    double std_laspy_; ///* Laser measurement noise standard deviation position2 in m
    
    double std_radr_; ///* Radar measurement noise standard deviation radius in m
    double std_radphi_; ///* Radar measurement noise standard deviation angle in rad
    double std_radrd_ ; ///* Radar measurement noise standard deviation radius change in m/s
    
    VectorXd weights_; ///* Weights of sigma points
    
    int n_x_; ///* State dimension
    int n_aug_; ///* Augmented state dimension
    int sigma_points_; ///* Total sigma points to generate
    
    double lambda_; ///* Sigma point spreading parameter
    double lambda_aug_; ///* Precomputed augmented lambda value
    
    double nis_radar_; ///* Listing of radar NIS values
    double nis_laser_; ///* Listing of laser NIS values
    
    
    /**
     * Constructor
     */
    UKF();
    
    /**
     * Destructor
     */
    virtual ~UKF();
    
    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);
    
    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);
    
    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);
    
    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);
    
    
    double NormalizeAngle(double angle);
};

#endif /* UKF_H */

#include <iostream>
#include "Eigen/Dense"
#include "ukf.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
    // LiDAR measurements will be ignored if this is false
    use_laser_ = true;

    // RADAR measurements will be ignored if this is false
    use_radar_ = true;

//    is_initialized_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 1.5;

//    time_us_ = 0;

    /***************************************************************************
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
     ****************************************************************************/

    // set state dimension
    n_x_ = 5;

    // set augmented dimension
    n_aug_ = 7;

    // lambda spreading parameter
    lambda_ = 3 - n_aug_;

    // initialise the weights
    weights_ = VectorXd(2 * n_aug_ + 1);

    // initialise the sigma prediction matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}


UKF::~UKF()
= default;

void UKF::ProcessMeasurement(MeasurementPackage measurement_package)
{
    cout << "Processing Measurement Package " << (measurement_package.sensor_type_ == 0 ? "LASER" : "RADAR") << endl;

    if(!is_initialized_)
    {
        time_us_ = measurement_package.timestamp_;

        if(measurement_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            cout << measurement_package.raw_measurements_[0] << " " << measurement_package.raw_measurements_[1] << " "
                 << measurement_package.raw_measurements_[2] << endl;
            float rho = measurement_package.raw_measurements_[0];
            float phi = measurement_package.raw_measurements_[1];
            float rho_dot = measurement_package.raw_measurements_[2];

            // convert to cartesian
            float px = rho * cos(phi);
            float py = rho * sin(phi);
            // note, not using the RADAR velocity
            x_ << px, py, 0, 0, 0;
        }
        else if(measurement_package.sensor_type_ == MeasurementPackage::LASER)
        {
            /**
             Initialize state.
             */

            cout << measurement_package.raw_measurements_[0] << " " << measurement_package.raw_measurements_[1] << endl;

            float px = measurement_package.raw_measurements_[0];
            float py = measurement_package.raw_measurements_[1];

            //       check is px and py are close to 0
            if(fabs(px) < 0.001 && fabs(py) < 0.001)
            {
                px = 0.001;
                py = 0.001;
            }

            x_ << px, py, 0, 0, 0;
        }
        else
        {
            cout << "Not LIDAR or RADAR measurement?" << endl;
        }

        // Initialise P_ the covariance matrix
        // P_ << MatrixXd::Identity(5, 5);

        // replaced Identify matrix version with one that uses square of sensor variance as per knowledge link:
        // https://knowledge.udacity.com/questions/62841
        double seensor_variance_squared = std_laspx_ * std_laspx_;
        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, seensor_variance_squared, 0,
              0, 0, 0, 0, seensor_variance_squared;


        time_us_ = measurement_package.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    } // end of if (!is_initialized_)

    double dt = (measurement_package.timestamp_ - time_us_) / 1000000.0; // dt - expressed in seconds
    time_us_ = measurement_package.timestamp_;

    Prediction(dt);

    if(measurement_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
        UpdateRadar(measurement_package);
    }
    else if(measurement_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {
        UpdateLidar(measurement_package);
    }
}

void UKF::Prediction(double delta_t)
{
    cout << "Prediction: delta_t = " << delta_t << endl;
    MatrixXd Xsig_aug = AugmentedSigmaPoints();

    // Predict the sigma points
    // create matrix with predicted sigma points as columns
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    PredictSigmaPoints(&Xsig_pred_, delta_t, Xsig_aug);

    // Predicted Mean and covariance
    PredictMeanAndCovariance();
}

MatrixXd UKF::AugmentedSigmaPoints() const
{// create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    // create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for(int i = 0; i < n_aug_; i++)
    {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }
    return Xsig_aug;
}

void UKF::PredictSigmaPoints(Eigen::MatrixXd *Xsig_pred, double delta_t, const MatrixXd & Xsig_aug)
{
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        // extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // predicted state values
        double px_p, py_p, v_p, yaw_p, yawd_p;

        // avoid division by zero
        if(fabs(yawd) > 0.001)
        {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        }
        else
        {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        v_p = v;
        yaw_p = yaw + yawd * delta_t;
        yawd_p = yawd;

        // add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;
        // populate the sigma pred matrix
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }

    *Xsig_pred = Xsig_pred_;
}

void UKF::PredictMeanAndCovariance()
{// set weights
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for(int i = 1; i < 2 * n_aug_ + 1; i++)
    { // 2n+1 weights
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
    // predicted state mean
    x_.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // predicted state covariance matrix
    P_.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        if(x_diff(3) > M_PI)
            x_diff(3) -= 2. * M_PI;
        if(x_diff(3) < -M_PI)
            x_diff(3) += 2. * M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

void UKF::UpdateLidar(const MeasurementPackage & measurement_package)
{
    cout << "Updating Lidar" << endl;

    // for LIDAR z_ is 2 dimension
    int n_z = 2;

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // transform sigma points into measurement space
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // 2n+1 sigma points
        // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        // measurement model
        Zsig(0, i) = p_x;
        Zsig(1, i) = p_y;
    }

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // 2n+1 sigma points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while(z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while(z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
    S = S + R;

    //  retrieve the actual measurements
    VectorXd z = measurement_package.raw_measurements_;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // 2n+1 sigma points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}

void UKF::UpdateRadar(const MeasurementPackage & measurement_package)
{
    cout << "Updating Radar" << endl;

    // for RADAR z_ is 3 dimension
    int n_z = 3;

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // transform sigma points into measurement space
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // 2n+1 sigma points
        // set measurement dimension, radar can measure r, phi, and r_dot

        // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                         // r
        Zsig(1, i) = atan2(p_y, p_x);                                     // phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); // r_dot
    }

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // 2n+1 sigma points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while(z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while(z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_ * std_radrd_;
    S = S + R;

    //  retrieve the actual measurements
    VectorXd z = measurement_package.raw_measurements_;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    // calculate cross correlation matrix
    Tc.fill(0.0);

    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    { // 2n+1 sigma points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while(z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while(z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // angle normalization
        while(x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while(x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while(z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while(z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}

/**
 *
 * @return state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
 */
const Eigen::VectorXd & UKF::State() const
{
    return x_;
}


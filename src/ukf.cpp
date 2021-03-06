#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.0; //1.8; //0.27; //30;  // 0.2?

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.3; //0.027; //30; // 0.2?

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03; // 0.0175

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;   // 0.1
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */

    //set state dimension
    n_x_ = 5;
    n_z_ = 3;
    n_z_lidar_ = 2;

    //set augmented dimension
    n_aug_ = 7;

    //define spreading parameter
    lambda_ = 3 - n_aug_;

    is_initialized_ = false;

    previous_timestamp_ = 0;

    //create vector for weights
    weights_ = VectorXd(2 * n_aug_ + 1);

    //create example matrix with sigma points in measurement space
    Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
    Zsig_lidar = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);

    // initializing matrices
    //R_laser_ = MatrixXd(2, 2);
    //R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 5);
    H_radar_ = MatrixXd(3, 5);

    /**
    TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
    */

    //create a 4D state vector, we don't know yet the values of the x state
    VectorXd x_in = VectorXd(5);
    //x_in << 1, 1, 1, 1, 0;
    x_in << 1, 1, 0, 0, 0;

    //state covariance matrix P
    MatrixXd P_in = MatrixXd(5, 5);

    P_in << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 10, 0, 0,
            0, 0, 0, 10, 0,
            0, 0, 0, 0, 1;

    H_laser_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0;

    MatrixXd Q_in = MatrixXd(5, 5);

    //ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);
    x_ = x_in;
    P_ = P_in;
    Q_ = Q_in;
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
    cout << endl << "# Iter: " << Iteration_ << " Sendor_type = " << meas_package.sensor_type_ << endl;

    Iteration_++;

    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
        * Initialize the state this->x_ with the first measurement.
        * Create the covariance matrix.
        * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "UKF: " << endl;
        //this->x_ = VectorXd(4);
        //this->x_ << 1, 1, 0, 0, 0;

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            float ro, theta;
            // Don't need ro_dot;
            ro = meas_package.raw_measurements_[0];
            theta = meas_package.raw_measurements_[1];    // Phi?
            this->x_ << ro * cos(theta), ro * sin(theta), 0, 0, 0;

            //cout << "ro, theta = " << ro << " " << theta << endl;
            //cout << "x_ (init R) = " << this->x_ << endl;
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            this->x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
            //cout << "px, py = " << meas_package.raw_measurements_[0] << " " << meas_package.raw_measurements_[1] << endl;
            //cout << "x_ (init L) = " << this->x_ << endl;
        }

        previous_timestamp_ = meas_package.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    if (((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_ == false))
     || ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_ == false)))
    {
        cout << "sendor_type mismatch = " << meas_package.sensor_type_ << endl;
        return;
    }
    float noise_ax = 9;
    float noise_ay = 9;

    //compute the time elapsed between the current and previous measurements
    float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    cout << "dt, prev_TS, meas_TS: " << dt << " " << previous_timestamp_ << " " << meas_package.timestamp_ << endl;

    previous_timestamp_ = meas_package.timestamp_;
    //cout << " Predict 1: dt:" << dt << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        float dt_2 = dt * dt;
        float dt_3 = dt_2 * dt;
        float dt_4 = dt_3 * dt;

        this->Q_ = MatrixXd(5, 5);
        this->Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay, 0,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay, 0,
                0, 0, 0, 0, 0;
    }

    cout << " Predict Start" << endl;

    //if (dt > 0.001)
    {
  		if ((use_laser_ && meas_package.sensor_type_ == meas_package.LASER) ||
            (use_radar_ && meas_package.sensor_type_ == meas_package.RADAR))
            // Predict
            this->Prediction(meas_package, dt); // x_, F_, P_, Q_ => Update: x_, P_
  	}

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        cout << "Update Start: (R)" << endl;

        VectorXd z_out = VectorXd(3);
        MatrixXd S_out = MatrixXd(3, 3);
        this->PredictRadarMeasurement(meas_package, &z_out, &S_out); // x_, P_, H_ R_, z; Update: x_, P_
        this->UpdateRadar(meas_package, z_out, S_out); // x_, P_, H_ R_, z; Update: x_, P_
        cout << " Update Done: (R)" << endl;
    } else {
      cout << "Update: (L)" << endl;

      VectorXd z_out = VectorXd(3);
      MatrixXd S_out = MatrixXd(3, 3);
      this->PredictLidarMeasurement(meas_package, &z_out, &S_out); // x_, P_, H_ R_, z; Update: x_, P_
      this->UpdateLidar(meas_package, z_out, S_out); // x_, P_, H_ R_, z; Update: x_, P_
      cout << " Update Done: (L)" << endl;
    }

    // print the output
    //cout << "x_ = " << this->x_ << endl;
    //cout << "P_ = " << this->P_ << endl;

    cout << "Loop Done!!!" << endl << endl;
    //cin.get();
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(MeasurementPackage meas_package, double delta_t) {
    /**
    TODO:
    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    // x_, F_, P_, Q_ => Update: x_, P_
    // Augment Sigma Points
    cout << "## Prediction Start Radar 1:" << endl;

    MatrixXd Xsig_aug = MatrixXd(7, 15);
    this->AugmentedSigmaPoints(&Xsig_aug);
    MatrixXd xsig_pred = MatrixXd(15, 5);
    this->SigmaPointPrediction(delta_t, Xsig_aug, &xsig_pred);  // In: Xsig_aug, Out:xsig_pred
    Xsig_pred_ = xsig_pred; //.transpose();
    this->PredictMeanAndCovariance(&x_, &P_);

    cout << "Prediction Done:" << endl;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

    //set state dimension
    int n_x = n_x_;

    //set augmented dimension
    int n_aug = n_aug_;

    //Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a = std_a_;

    //Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd = std_yawdd_;

    //define spreading parameter
    double lambda = lambda_;

    //set example state
    VectorXd x = VectorXd(n_x);
    x = x_;

    cout << endl << "# Augmented Sigma Point:"  << endl;

    VectorXd x_aug_ = VectorXd(n_aug_);
    //x_aug_.fill(0.0);
    //x_aug_.head(5) = x;
    //x_aug_(5) = 0;
    //x_aug_(6) = 0;

    //create example covariance matrix
    //MatrixXd P = MatrixXd(n_x, n_x);
    //cout << "P:" << P << endl;
    //cout << "P_:" << P_ << endl;
    //P << P_;

    //create augmented mean vector
    //VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

    /*******************************************************************************
    * Student part begin
    ******************************************************************************/

    //cout << "n_aug_:" << n_aug_ << endl;

    //create augmented mean state
    x_aug_(0) = x_(0);
    x_aug_.head(5) = x_;
    x_aug_(5) = 0;
    x_aug_(6) = 0;

    //create augmented covariance matrix
    P_aug.fill(0.0);

    //cout << "P_:" << P_ << endl;

    P_aug.topLeftCorner(5,5) = P_;

    //cout << "P_aug:" << P_aug << endl;

    P_aug(5,5) = std_a * std_a;
    P_aug(6,6) = std_yawdd * std_yawdd;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug_;
    double const_sqrt = sqrt(lambda+n_aug);

    for (int i = 0; i < n_aug; i++)
    {
        Xsig_aug.col(i + 1)       = x_aug_ + const_sqrt * L.col(i);
        Xsig_aug.col(i + 1 + n_aug) = x_aug_ - const_sqrt * L.col(i);
    }

    /*******************************************************************************
    * Student part end
    ******************************************************************************/

    std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
    *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(double delta_t, MatrixXd Xsig_aug, MatrixXd* Xsig_out) {
    // In: Xsig_aug
    // Out: Xsig_out

    //set state dimension
    int n_x = n_x_;

    //set augmented dimension
    int n_aug = n_aug_;

    //create example sigma point matrix
    //MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

    cout << endl << "# SigmaPoint Prediction:"  << endl;

    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

    //double delta_t = 0.1; //time diff in sec
    /*******************************************************************************
    * Student part begin
    ******************************************************************************/

    //predict sigma points
    for (int i = 0; i< 2*n_aug+1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred(0,i) = px_p;
        Xsig_pred(1,i) = py_p;
        Xsig_pred(2,i) = v_p;
        Xsig_pred(3,i) = yaw_p;
        Xsig_pred(4,i) = yawd_p;
    }

/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

    //write result
    *Xsig_out = Xsig_pred;

}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {
    // Input: Xsig_pred [5 * 15]
    // Output: x_out [5 * 1], P_out [5 * 5]

    //set state dimension
    int n_x = n_x_;

    //set augmented dimension
    int n_aug = n_aug_;

    //define spreading parameter
    double lambda = lambda_;

    cout << endl << "# Predict Mean & Cov:"  << endl;

    //create example matrix with predicted sigma points
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_pred = Xsig_pred_;
    //cout << "PredMeanCovar 1:" << endl;

    //cout << "PredMeanCovar 2:" << endl;

    //create vector for predicted state
    VectorXd x = VectorXd(n_x);

    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x, n_x);

    /*******************************************************************************
    * Student part begin
    ******************************************************************************/

    // set weights
    double weight_0 = lambda / (lambda + n_aug);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (n_aug + lambda);
        weights_(i) = weight;
    }

    //predicted state mean
    x.fill(0.0);
    //cout << "PredMeanCovar 4.1:" << x << endl << endl;
    //cout << "PredMeanCovar 4.1:" << weights << endl << endl;
    //cout << "PredMeanCovar 4.1:" << Xsig_pred_ << endl << endl;
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
        x = x + weights_(i) * Xsig_pred.col(i);
    }

    //predicted state covariance matrix
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
        //angle normalization
        //cout << "x_diff(3):" << x_diff(3) << endl;

        while (x_diff(3)> M_PI)
        {
            //x_diff(3)-=2.*M_PI;
            double temp = fmod((x_diff(3) - M_PI), (2 * M_PI)); // -= 2. * M_PI;
            x_diff(3) = temp - M_PI;
        }
        while (x_diff(3)<-M_PI)
        {
            //x_diff(3)+=2.*M_PI;
            double temp = fmod((x_diff(3) + M_PI), (2 * M_PI)); // -= 2. * M_PI;
            x_diff(3) = temp + M_PI;
        }
        //cout << endl << "x_diff(3):" << x_diff(3) << endl;

        P = P + weights_(i) * x_diff * x_diff.transpose() ;
    }

/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "x (predicted state):" << std::endl;
    std::cout << x << std::endl;
    std::cout << "P (predicted covariance):" << std::endl;
    std::cout << P << std::endl;

    //write result
    *x_out = x;
    *P_out = P;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::PredictLidarMeasurement(MeasurementPackage meas_package,
                                  VectorXd* z_out, MatrixXd* S_out) {
    // Input Xsig_pred: [5 * 15]

    //set state dimension
    int n_x = n_x_;

    //set augmented dimension
    int n_aug = n_aug_;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = n_z_lidar_;

    //define spreading parameter
    double lambda = lambda_;

    cout << endl << "# Predict Lidar Meas:"  << endl;



    //radar measurement noise standard deviation radius in m
    double std_laspx = std_laspx_;

    //radar measurement noise standard deviation angle in rad
    double std_laspy = std_laspy_;


    cout << endl << "# Predict Lidar Meas 3:"  << endl;
    // Radar updates
    float meas_x, meas_y;
    // Don't need ro_dot;
    meas_x = meas_package.raw_measurements_[0];
    meas_y = meas_package.raw_measurements_[1];
    //this->x_ << ro * cos(theta), ro * sin(theta), 0, 0;

    cout << endl << "# Predict Lidar Meas 4:"  << endl;



    VectorXd z = VectorXd(n_z);
    z << meas_x, meas_y;

    //create matrix for sigma points in measurement space
    //MatrixXd Zsig_lidar = MatrixXd(n_z, 2 * n_aug + 1);

    cout << endl << "# Predict Lidar Meas 5:"  << endl;

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig_lidar(0,i) = p_x;                        //r
        Zsig_lidar(1,i) = p_y;                                 //phi
        //Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig_lidar.col(i);
    }

    //innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig_lidar.col(i) - z_pred;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    cout << endl << "# Predict Lidar Meas 6:"  << n_z << endl;

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_laspx * std_laspx, 0,
          0, std_laspy * std_laspy;
    S = S + R;

    cout << endl << "# Predict Lidar Meas 7:"  << endl;

/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "z_pred (mean): " << std::endl << z_pred << std::endl;
    std::cout << "S (cov): " << std::endl << S << std::endl;

    //write result
    *z_out = z_pred;
    *S_out = S;
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package,
                      VectorXd z_pred, MatrixXd S_in) {
    // Input Xsig_pred [5 * 15], z_pred [3], S_in [3 * 3]
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */

    //set state dimension
    int n_x = n_x_;

    //set augmented dimension
    int n_aug = n_aug_;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = n_z_lidar_;

    //define spreading parameter
    double lambda = lambda_;

    cout << endl << "# Updated Radar: 0.1 "  << endl;


    //create example vector for predicted state mean
    VectorXd x = VectorXd(n_x);
    x = x_;

    //create example matrix for predicted state covariance
    MatrixXd P = MatrixXd(n_x,n_x);
    P = P_;

    //create example vector for mean predicted measurement
    //VectorXd z_pred = VectorXd(n_z);
    //z_pred = z_pred;

    //create example matrix for predicted measurement covariance
    MatrixXd S = MatrixXd(n_z, n_z);
    S = S_in;

    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    float meas_x, meas_y; //rho, theta, rho_dot;
    meas_x = meas_package.raw_measurements_[0];
    meas_y = meas_package.raw_measurements_[1];    // Phi?
    //rho_dot = meas_package.raw_measurements_[2];    // Phi?

    z << meas_x, meas_y;

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z);

  /*******************************************************************************
   * Student part begin
   ******************************************************************************/

    //calculate cross correlation matrix
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig_lidar.col(i) - z_pred;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;

    std::cout << "z (Meas Input): " << z << std::endl;
    std::cout << "z_pred (predicted z): " << z_pred << std::endl;

    //update state mean and covariance matrix
    x = x + K * z_diff;
    P = P - K * S * K.transpose();

    /*******************************************************************************
    * Student part end
    ******************************************************************************/

    //print result
    std::cout << "Updated state x: " << std::endl << x << std::endl;
    std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

    //write result
    x_ = x;
    P_ = P;
}

void UKF::PredictRadarMeasurement(MeasurementPackage meas_package,
                                  VectorXd* z_out, MatrixXd* S_out) {
    // Input Xsig_pred: [5 * 15]

    //set state dimension
    int n_x = n_x_;

    //set augmented dimension
    int n_aug = n_aug_;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = n_z_;

    //define spreading parameter
    double lambda = lambda_;

    cout << endl << "# Predict Radar Meas:"  << endl;



    //radar measurement noise standard deviation radius in m
    double std_radr = std_radr_;

    //radar measurement noise standard deviation angle in rad
    double std_radphi = std_radphi_;

    //radar measurement noise standard deviation radius change in m/s
    double std_radrd = std_radrd_;

    // Radar updates
    //float rho, theta, rho_dot;
    // Don't need ro_dot;
    //rho = meas_package.raw_measurements_[0];
    //theta = meas_package.raw_measurements_[1];    // Phi?
    //rho_dot = meas_package.raw_measurements_[2];    // Phi?
    //this->x_ << ro * cos(theta), ro * sin(theta), 0, 0;

    //VectorXd z = VectorXd(3);
    //z << rho, theta, rho_dot;

    //create matrix for sigma points in measurement space
    //MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI)
        {
            //z_diff(1)-=2.*M_PI;
            double temp = fmod((z_diff(1) - M_PI), (2 * M_PI)); // -= 2. * M_PI;
            z_diff(1) = temp - M_PI;
        }
        while (z_diff(1)<-M_PI)
        {
            //z_diff(1)-=2.*M_PI;
            double temp = fmod((z_diff(1) + M_PI), (2 * M_PI)); // += 2. * M_PI;
            z_diff(1) = temp + M_PI;
        }

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
    S = S + R;


/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "z_pred (mean): " << std::endl << z_pred << std::endl;
    std::cout << "S (cov): " << std::endl << S << std::endl;

    //write result
    *z_out = z_pred;
    *S_out = S;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package,
                      VectorXd z_pred, MatrixXd S_in) {
    // Input Xsig_pred [5 * 15], z_pred [3], S_in [3 * 3]
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
    //set state dimension
    int n_x = n_x_;

    //set augmented dimension
    int n_aug = n_aug_;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    //define spreading parameter
    double lambda = lambda_;

    cout << endl << "# Updated Radar: 0.1 "  << endl;


    //create example vector for predicted state mean
    VectorXd x = VectorXd(n_x);
    x = x_;

    //create example matrix for predicted state covariance
    MatrixXd P = MatrixXd(n_x,n_x);
    P = P_;

    //create example matrix for predicted measurement covariance
    MatrixXd S = MatrixXd(n_z, n_z);
    S = S_in;

    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    float rho, theta, rho_dot;
    rho = meas_package.raw_measurements_[0];
    theta = meas_package.raw_measurements_[1];    // Phi?
    rho_dot = meas_package.raw_measurements_[2];    // Phi?
    //this->x_ << ro * cos(theta), ro * sin(theta), 0, 0;

    z << rho, theta, rho_dot;

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z);

  /*******************************************************************************
   * Student part begin
   ******************************************************************************/

    //calculate cross correlation matrix
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI)
        {
            //z_diff(1)-=2.*M_PI;
            double temp = fmod((z_diff(1) - M_PI), (2 * M_PI)); // -= 2. * M_PI;
            z_diff(1) = temp - M_PI;
        }

        while (z_diff(1)<-M_PI)
        {
            //z_diff(1)-=2.*M_PI;
            double temp = fmod((z_diff(1) + M_PI), (2 * M_PI)); // += 2. * M_PI;
            z_diff(1) = temp + M_PI;
        }

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x;
        //angle normalization
        while (x_diff(3)> M_PI)
        {
            //x_diff(3)-=2.*M_PI;
            double temp = fmod((x_diff(3) - M_PI), (2 * M_PI)); // -= 2. * M_PI;
            x_diff(3) = temp - M_PI;
        }
        while (x_diff(3)<-M_PI)
        {
            //x_diff(3)+=2.*M_PI;
            double temp = fmod((x_diff(3) + M_PI), (2 * M_PI)); // += 2. * M_PI;
            x_diff(3) = temp + M_PI;
        }

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;

    std::cout << "z (Meas Input): " << z << std::endl;
    std::cout << "z_pred (predicted z): " << z_pred << std::endl;

    //angle normalization
    while (z_diff(1) > M_PI)
    {
        //z_diff(1)-=2.*M_PI;
        double temp = fmod((z_diff(1) - M_PI), (2 * M_PI)); // -= 2. * M_PI;
        z_diff(1) = temp - M_PI;
    }
    while (z_diff(1) < -M_PI)
    {
        //z_diff(1)-=2.*M_PI;
        double temp = fmod((z_diff(1) + M_PI), (2 * M_PI)); // += 2. * M_PI;
        z_diff(1) = temp + M_PI;
    }

    //update state mean and covariance matrix
    x = x + K * z_diff;
    P = P - K * S * K.transpose();

  /*******************************************************************************
   * Student part end
   ******************************************************************************/

    //print result
    std::cout << "Updated state x: " << std::endl << x << std::endl;
    std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

    //write result
    x_ = x;
    P_ = P;
}

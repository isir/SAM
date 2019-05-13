#ifndef LAWOPTI_H
#define LAWOPTI_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class LawOpti {

public:
    LawOpti();
    ~LawOpti();
    void initialization(Eigen::Vector3d posA, Eigen::Vector3d posEE, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, unsigned int freq);
    void initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, Eigen::Quaterniond qFA_record, int initCounter, int initCounts);
    void rotationMatrices(Eigen::Quaterniond qHip, Eigen::Quaterniond qFA_record, int initCounter, int initCounts);
    void computeEEfromFA(Eigen::Vector3d posFA, int _l, Eigen::Quaterniond qFA_record);
    void projectionInHip(Eigen::Vector3d posA, Eigen::Vector3d posElbow, Eigen::Vector3d posHip, int initCounter, int initCounts);
    void filter_optitrackData(Eigen::Vector3d posA, Eigen::Vector3d posEE);
    void bufferingOldValues();
    void controlLaw(Eigen::Vector3d posEE, double beta, double Lua, double Lfa, double l, int lambda, double threshold);
    void controlLawWrist(int lambdaW, double thresholdW);
    void writeDebugData(double debug[], Eigen::Vector3d posEE, double beta);
    void displayData(Eigen::Vector3d posEE, double beta);
    double returnBetaDot_deg() { return betaDot * 180. / M_PI; }
    double returnWristVel_deg() { return wristVel * 180. / M_PI; }
    Eigen::Vector3d returnPosAinHip() { return posAinHip; }
    Eigen::Vector3d returnPosElbowinHip() { return posElbowinHip; }

private:
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posA0; // initial position of the acromion
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posA0inHip; // initial position of the acromion in hip frame
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posAinHip, posElbowinHip; // position of the acromion and the elbow in hip frame
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posA_filt, posA_filt_old; // filtered acromion position
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posEEinHip; // position of the end-effector in hip frame
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posEEfromFA; // position of the end-effector reconstructed from forearm cluster
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posEE_filt, posEE_filt_old; // filtered EE position
    double delta; // distance between initial (=reference) acromion position and current end-effector position
    double Lee; // length from elbow to EE
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> posHip0; // initial position of hip
    Eigen::Quaternion<double, Eigen::DontAlign> qHip0, qHip_filt, qHip_filt_old; // quaternions for hip frame rotation
    double beta_new; // desired elbow flexion/extension angle
    double dBeta; // difference between current beta and desired beta
    double betaDot; // elbow angular velocity
    double samplePeriod;
    double coeff; // coefficient for low-pass filtering

    Eigen::Quaternion<double, Eigen::DontAlign> qFA, qFA0, qFA_relative; // quaternion of forearm cluster

    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> R, R_FA; // term of rotation matrix of hip frame in initial hip frame anf of FA in initial FA frame
    double R11, R12, R13, R21, R22, R23, R31, R32, R33;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> ea;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> z0, zFA;
    double phi, theta, wristAngle_new; // euler angles for wrist frame
    double wristVel;
};

#endif // LAWOPTI_H

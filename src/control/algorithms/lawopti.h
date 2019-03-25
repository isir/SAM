#ifndef LAWOPTI_H
#define LAWOPTI_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class LawOpti
{

public:
    LawOpti();
    ~LawOpti();
    void initialization(Eigen::Vector3d posA, Eigen::Vector3d posEE, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, unsigned int freq);
    void initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, Eigen::Quaterniond qFA_record, int initCounter, int initCounts);
    void rotationMatrices(Eigen::Quaterniond qHip, Eigen::Quaterniond qFA_record);
    void initialAcromionPositionInHip();
    void projectionInHip(Eigen::Vector3d posA, Eigen::Vector3d posEE, Eigen::Vector3d posHip);
    void filter_optitrackData(Eigen::Vector3d posA, Eigen::Vector3d posEE);
    void bufferingOldValues();
    void controlLaw(Eigen::Vector3d posEE, double beta, double Lua, double Lfa, int lambda, double threshold);
    void controlLawWrist(int lambdaW, double thresholdW);
    void writeDebugData(double debug[], Eigen::Vector3d posEE, double beta);
    void displayData(Eigen::Vector3d posEE, double beta);
    double returnBetaDot_deg() { return betaDot*180./M_PI; }
    double returnWristVel_deg() { return wristVel*180./M_PI; }

private:
    Eigen::Vector3d posA0; // initial position of the acromion
    Eigen::Vector3d posA0inHip; // initial position of the acromion in hip frame
    Eigen::Vector3d posAinHip; // position of the acromion in hip frame
    Eigen::Vector3d posA_filt, posA_filt_old; // filtered acromion position
    Eigen::Vector3d posEEinHip; // position of the end-effector in hip frame
    Eigen::Vector3d posEE_filt, posEE_filt_old; // filtered EE position
    double delta; // distance between initial (=reference) acromion position and current end-effector position
    Eigen::Vector3d posHip0; // initial position of hip
    Eigen::Quaterniond qHip0, qHip_filt, qHip_filt_old, qHip_relative; // quaternions for hip frame rotation
    double beta_new; // desired elbow flexion/extension angle
    double dBeta; // difference between current beta and desired beta
    double betaDot; // elbow angular velocity
    double samplePeriod;
    double coeff; // coefficient for low-pass filtering

    Eigen::Quaterniond qFA, qFA0, qFA_relative; // quaternion of forearm cluster

    Eigen::Matrix3d R, R_FA; // term of rotation matrix of hip frame in initial hip frame anf of FA in initial FA frame
    double R11, R12, R13, R21, R22, R23, R31, R32, R33;
    Eigen::Vector3d ea;
    double phi, theta, wristAngle_new; // euler angles for wrist frame
    double wristVel;
};

#endif // LAWOPTI_H

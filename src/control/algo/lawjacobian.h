#ifndef LAWJACOBIAN_H
#define LAWJACOBIAN_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string.h>

/// For 3DOF (=wrist flex/ext, wrist pronosup, elbow flex/ext) configuration
//static const int nbFrames = 4;
//static const int nbLinks = 3;
//static const std::string rel = "0yyz";

/// For 2DOF (=wrist pronosup, elbow flex/ext) configuration
static const int nbFrames = 3;
static const int nbLinks = 2;
static const std::string rel = "0yz";
#define ROW ;

class LawJacobian {
public:
    LawJacobian();
    ~LawJacobian();
    void initialization(Eigen::Quaterniond qHip, unsigned int freq); // common initialization
    void initializationOpti(Eigen::Vector3d posA); // init special for Optitrack
    void initializationIMU(); // init special for IMU
    void initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts);
    void initialQuat(Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk, Eigen::Quaterniond qArm, int initCounter, int initCounts);
    void idealFrames(Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk, int initCounter, int initCounts);
    void rotationMatrices(Eigen::Quaterniond qHand, Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk);
    void rotationMatrices2(Eigen::Quaterniond qHandOpti, Eigen::Quaterniond qHandIMU, Eigen::Quaterniond qHipOpti, Eigen::Quaterniond qHipIMU, Eigen::Quaterniond qTrunk);
    void projectionInHip(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts);
    void orientationInHand(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts);
    void projectionInHipIMU(int lt, int lsh, int initCounter, int initCounts);
    void bufferingOldValues();
    void updateTrunkFrame(Eigen::Quaterniond qTrunk);
    void updateFrames(double theta[]);
    void updateFramesinEE(double theta[]);
    void computeOriginsVectors(int l[], int nbDOF);
    void computeTrunkAngles(Eigen::Quaterniond qHand, Eigen::Quaterniond qTrunk, Eigen::Quaterniond qHip);
    void computeArmAngles(Eigen::Quaterniond qHand, Eigen::Quaterniond qTrunk, Eigen::Quaterniond qArm);
    void controlLaw_v1(Eigen::Vector3d posA, int k, int useIMU, double lambda[], double threshold[], int _cnt);
    void controlLaw_orientation(double k, double lambda[], double threshold[], int _cnt);
    void controlLaw_v2(int k, double lambda[], double threshold[], int _cnt);
    void controlLaw_v3(int lt, int lsh, int k, double lambda[], double threshold[], int _cnt);
    void controlLaw_v4(int lt, int lsh, int k, double lambda[], double threshold[], int _cnt);
    void scaleDisplacementIMU(int lt, int _cnt);
    void scaleDisplacementHip(int _cnt);
    void writeDebugData(double debug[], double theta[]);
    void displayData(Eigen::Vector3d posEE, double beta);
    /// RETURN DATA
    Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> returnthetaDot_deg() { return thetaDot * 180. / M_PI; }
    Eigen::Vector3d returnPosAinHip() { return posAinHip; }

private:
    Eigen::Matrix<double, 3, nbFrames, Eigen::DontAlign> x, y, z; // frames
    Eigen::Matrix<double, 3, nbLinks, Eigen::DontAlign> J;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Jp;
    Eigen::Matrix<double, 3, nbLinks - 1, Eigen::DontAlign> Jo; // jacobian matrices
    Eigen::Matrix<double, nbLinks, 3, Eigen::DontAlign> dlsJ; // damped least square solutions from jacobian matrix
    Eigen::RowVector3d dlsJp;
#ifdef ROW
    Eigen::RowVector3d dlsJo; // for Eigen, matrix of one line must be defined as a RowVector
#else
    Eigen::Matrix<double, nbLinks - 1, 3, Eigen::DontAlign> dlsJo;
#endif
    //    Eigen::MatrixXd pinvJ; // pseudo inverse of jacobian matrix
    Eigen::Matrix<double, 3, nbLinks, Eigen::DontAlign> OO; // vectors between the centers of the frames
    Eigen::Vector3d xref, yref, zref, Ytrunk0, Ytrunk, Xtrunk, Ztrunk;
    Eigen::Vector3d posA0; // initial position of the acromion
    Eigen::Vector3d posA0inHip, posA0inHipOpti; // initial position of the acromion in hip frame
    Eigen::Vector3d posAinHip, posAinHipOpti, posAinHand, IO; // position of the acromion and the elbow in hip frame
    Eigen::Vector3d HA0, HA; // hip to acromion vectors (initial and current)
    Eigen::Vector3d delta, deltaOpti, disp, dispOpti; // displacement between acromion initial (=reference) position and current position
    Eigen::Vector3d posHip0; // initial position of hip
    double samplePeriod;
    double coeff; // coefficient for low-pass filtering
    Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> thetaNew, thetaNewOpti, thetaDot; // joint angles and angular velocities command
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> eulerT, eulerA, eulerHA; // Trunk, Arm and Hip-Acromion Euler angles, expressed in hand frame

    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Rhip, RhipOpti, Rtrunk, Rhand, RhandOpti, Rframe, R0, RtrunkInHand, RtrunkHipInHand, RArmInHand, I3, Rhand_rel, RHA; // rotation matrices
    //    Eigen::Matrix<int, 3, 3, Eigen::DontAlign> I3; // identity matrix
    Eigen::Matrix<double, 2, 2, Eigen::DontAlign> I2;

    double theta0H, theta0T, thetaHA; // angles to correct trunk and hip IMU initial orientation
    Eigen::Quaterniond qRecalH, qRecalT, qIdealH, qIdealT, qHA; // quaternions to correct trunk and hip IMU initial orientation + corrected quaternions of trunk and hip IMU
    Eigen::Quaternion<double, Eigen::DontAlign> qTrunk0, qHip0, qHip_filt, qHip_filt_old, qArm0, qHand_relative; // quaternions for hip, arm and trunk frames
    Eigen::Quaternion<double, Eigen::DontAlign> Yinit, Y0, Xinit, X0, Zinit, Z0; // quaternions to compute initial and current trunk frame
    int scale;
};

#endif // LAWJACOBIAN_H

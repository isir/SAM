#ifndef LAWIMU_WE_H
#define LAWIMU_WE_H

#include "math.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


class LawIMU_WE
{

public:
    LawIMU_WE();
    ~LawIMU_WE();

    /// Initialization
    void initialization(int Lt, int Lua, int Lfa, int lsh);
    void recordInitialPosition(Eigen::Quaterniond qTronc, Eigen::Quaterniond qBras, int initCounts, int initCounter);

    /// transform brut Eigen::Quaternion<double, Eigen::DontAlign>ernions
    void moveToIdealFrames(Eigen::Quaterniond qTrunk, int initCounts, int initCounter);
    void brutQuatToNewFrame(Eigen::Quaterniond qTronc);
//    void brutQuatToRelative(Eigen::Quaternion<double, Eigen::DontAlign> qBras, Eigen::Quaternion<double, Eigen::DontAlign> qTronc);
    void rotationMatrices(Eigen::Quaterniond qBras);

    // control algorithm
    void computeAxis(Eigen::Quaterniond qTronc, Eigen::Quaterniond qBras);
    void new_coordEE(int Lt, int Lua, int Lfa, int lsh, double beta);
    void computeBetaDot(int Lua, int Lfa, double lambda, double threshold, double beta);
    void computeWristVel(double lambdaW, double thresholdW);
    void buffering();
    void display(int counter);
    void writeDebugData(double debug[], double beta);

    // access to computed values
    double getBetaDot_deg(){return betaDot*180/M_PI;}
    double getWristVel(){return wristVel;} // already in deg
    double getXx(){return Xtronc[0];}
    double getXy(){return Xtronc[1];}
    double getXz(){return Xtronc[2];}
    double getYx(){return Ytronc[0];}
    double getYy(){return Ytronc[1];}
    double getYz(){return Ytronc[2];}

private:
    // Initial Eigen::Quaternion<double, Eigen::DontAlign>ernion in global frame
    Eigen::Quaternion<double, Eigen::DontAlign> qTronc0, qBras0;
    // vectors and Eigen::Quaternion<double, Eigen::DontAlign>ernions of calibration, trunk and arm frames
    Eigen::Quaternion<double, Eigen::DontAlign> X0, Y0, Z0;
    Eigen::Quaternion<double, Eigen::DontAlign> Xinit, Yinit,Zinit;
    Eigen::Vector3d Xtronc_init_vect,Ytronc_init_vect, Ztronc_init_vect, Xbras_init_vect, Ybras_init_vect, Zbras_init_vect;
    Eigen::Quaternion<double, Eigen::DontAlign> XQuat, YQuat,ZQuat;
    Eigen::Quaternion<double, Eigen::DontAlign> Xbras_long, Ybras_long,Zbras_long;
    Eigen::Vector3d Xtronc, Ytronc, Ztronc;
    Eigen::Vector3d Xbras,Ybras, Zbras;
    Eigen::Vector3d Xbras_ref,Ybras_ref, Zbras_ref;
    Eigen::Quaternion<double, Eigen::DontAlign> qRecalT, qIdealT, qRecalA, qBras_relative;
    Eigen::Quaternion<double, Eigen::DontAlign> qinit, qInit_recal;
    Eigen::Vector3d X0_vect,Y0_vect,Z0_vect;
    Eigen::Vector3d crossP;

    double theta0;
    // shoulder coordinates in reference position, body frame
    double Xsh_ref, Ysh_ref, Zsh_ref;
    // shoulder-hand distance
    double delta;

    // initial end-effector coordinates in body frame
    double Xee_init, Yee_init, Zee_init;
    // current end-effector coordinates in body frame
    double Xee, Yee, Zee;
    // elbow angle values
    double beta_new, dBeta, betaDot;
    // wrist ang. velocity
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> R_Bras;
    double wristAngle_new, wristVel, theta, psi, phi;



};

#endif // LawIMU_WE_H

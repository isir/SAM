#ifndef LAWIMU_H
#define LAWIMU_H

#include "math.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class LawIMU {

public:
    LawIMU();
    ~LawIMU();
    void initialization();
    void initialPositions(Eigen::Quaterniond qFA_record, int initCounter, int initCounts);
    void rotationMatrices(Eigen::Quaterniond qFA_record);
    void controlLawWrist(int lambdaW, double thresholdW);
    void writeDebugData(double debug[]);
    void displayData();
    double returnWristVel_deg() { return wristVel * 180. / M_PI; }

private:
    Eigen::Quaternion<double, Eigen::DontAlign> qFA, qFA0, qFA_relative; // quaternion of forearm cluster

    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> R, R_FA; // term of rotation matrix of hip frame in initial hip frame anf of FA in initial FA frame
    double R11, R12, R13, R21, R22, R23, R31, R32, R33;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> ea;
    Eigen::Matrix<double, 3, 1, Eigen::DontAlign> z0, zFA;
    double wristAngle_new, theta, psi; // euler angles for wrist frame
    double wristVel;
};

#endif // LawIMU_H

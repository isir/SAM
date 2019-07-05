#include "lawimu.h"
#include <QDebug>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>

LawIMU::LawIMU()
{
}

LawIMU::~LawIMU()
{
}

void LawIMU::initialization()
{
    qFA0.w() = 0.;
    qFA0.x() = 0.;
    qFA0.y() = 0.;
    qFA0.z() = 0.;
    qFA_relative.w() = 0.;
    qFA_relative.x() = 0.;
    qFA_relative.y() = 0.;
    qFA_relative.z() = 0.;
    //    z0_quat.w() = 0.;
    //    z0_quat.x() = 0.;
    //    z0_quat.y() = 0.;
    //    z0_quat.z() = 1.;
    //    zFA_quat = z0_quat;
    zFA = Eigen::Vector3d::Zero();
    z0[0] = 0.;
    z0[1] = 0.;
    z0[2] = 1.;
    wristAngle_new = 0.;
    wristVel = 0.;
    R = Eigen::Matrix3d::Zero();
}
/**
 * @brief LawIMU::initialPositions stores the initial orientation of forearm IMU
 * @param qFA_record quaternion of forearm IMU
 * @param initCounter counter of time steps
 * @param initCounts number of time step to take into account to compute the initial position
 */
void LawIMU::initialPositions(Eigen::Quaterniond qFA_record, int initCounter, int initCounts)
{
    if (initCounter < initCounts) {
        qFA0.w() += qFA_record.w();
        qFA0.x() += qFA_record.x();
        qFA0.y() += qFA_record.y();
        qFA0.z() += qFA_record.z();
    }
    if (initCounter == initCounts) {
        qFA0.w() += qFA_record.w();
        qFA0.x() += qFA_record.x();
        qFA0.y() += qFA_record.y();
        qFA0.z() += qFA_record.z();
        qFA0.w() = qFA0.w() / initCounts;
        qFA0.x() = qFA0.x() / initCounts;
        qFA0.y() = qFA0.y() / initCounts;
        qFA0.z() = qFA0.z() / initCounts;
        qFA0 = qFA0.normalized();
    }
}
/**
 * @brief LawIMU::rotationMatrices compute the rotation matrices of the forearm frames with respect to the global frame
 * @param qFA_record quaternions of the forearm cluster
 */
void LawIMU::rotationMatrices(Eigen::Quaterniond qFA_record)
{
    qFA = qFA_record;
    qFA_relative = qFA.normalized() * qFA0.conjugate();
    /// For optitrack quaternion definition
    R_FA = qFA_relative.toRotationMatrix();
    //    ///  From quaternions to orientation
}
/**
 * @brief LawIMU::controlLawWrist control law for prono-supination
 * @param lambdaW gain of the integrator for wrist
 * @param thresholdW threshold of activation in rad
 */
void LawIMU::controlLawWrist(int lambdaW, double thresholdW)
{
    wristAngle_new = atan2(R_FA(1, 2), R_FA(2, 2)); //rotation around X-axis
    theta = -atan(R_FA(0, 2) / sqrt(1 - R_FA(0, 2) * R_FA(0, 2))); //rotation around Y-axis
    psi = atan2(R_FA(0, 1), R_FA(0, 0)); //rotation around Z-axis
    if (abs(wristAngle_new) < thresholdW)
        wristVel = 0.;
    else if (wristAngle_new <= -thresholdW) {
        wristVel = lambdaW * (wristAngle_new + thresholdW);
    } else if (wristAngle_new >= thresholdW) {
        wristVel = lambdaW * (wristAngle_new - thresholdW);
    }
}

void LawIMU::writeDebugData(double debug[])
{
    debug[0] = wristAngle_new;
    debug[1] = theta;
    debug[2] = psi;
    debug[3] = wristVel;
}

void LawIMU::displayData()
{
    qDebug("wristAngle (deg): %lf  -- theta: %lf  -- psi: %lf\n", wristAngle_new * 180 / M_PI, theta * 180 / M_PI, psi * 180 / M_PI);
    //    printf("wristAngle (deg): %lf  -- theta: %lf  -- psi: %lf\n", wristAngle_new * 180 / M_PI, theta * 180 / M_PI, psi * 180 / M_PI);
    qDebug("Wrist velocity (deg): %lf\n", wristVel * 180 / M_PI);
}

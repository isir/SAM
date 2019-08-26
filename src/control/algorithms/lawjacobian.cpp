#include "lawjacobian.h"
#include <QDebug>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <string.h>

LawJacobian::LawJacobian()
{
}

LawJacobian::~LawJacobian()
{
}

void LawJacobian::initialization(Eigen::Vector3d posA, Eigen::Quaterniond qHip, unsigned int freq)
{
    /// POSITIONS AND QUATERNIONS
    posA0 = Eigen::Vector3d::Zero();
    posAinHip = posA;
    posA0inHip = posA;
    posHip0 = Eigen::Vector3d::Zero();
    qHip0.w() = 0.;
    qHip0.x() = 0.;
    qHip0.y() = 0.;
    qHip0.z() = 0.;
    qHip_filt_old = qHip;
    qHip_filt.w() = 0.;
    qHip_filt.x() = 0.;
    qHip_filt.y() = 0.;
    qHip_filt.z() = 0.;
    /// FRAMES
    for (int i = 0; i < nbFrames; i++) {
        x(0, i) = 1.;
        x(1, i) = 0.;
        x(2, i) = 0.;
        y(0, i) = 0.;
        y(1, i) = 1.;
        y(2, i) = 0.;
        z(0, i) = 0.;
        z(1, i) = 0.;
        z(2, i) = 1.;
    }
    samplePeriod = 1. / freq;
    coeff = samplePeriod / (0.03 + samplePeriod);
    delta[0] = 0;
    delta[1] = 0;
    delta[2] = 0;
    R = Eigen::Matrix3d::Zero();
    Rframe = Eigen::Matrix3d::Zero();
    thetaNew = Eigen::MatrixXd::Zero(nbFrames, 1);
    thetaDot = Eigen::MatrixXd::Zero(nbFrames, 1);
}
/**
 * @brief LawJacobian::initialPositions computes the initial position of the acromion marker = mean over the initCounts first measures of the acromion position
 *  and the initial position + orientation of the hip cluster = mean over the initCounts first measures
 * @param posA cartesian coordinates of the acromion marker (from optitrack)
 * @param posHip cartesian coordinates of the hip marker (from optitrack)
 * @param qHip quaternions of the hip marker (from optitrack)
 * @param initCounter counter of time steps
 * @param initCounts number of time step to take into account to compute the initial position
 */
void LawJacobian::initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, int initCounter, int initCounts)
{
    if (initCounter < initCounts) {
        posA0 += posA;
        posHip0 += posHip;
        qHip0.w() += qHip.w();
        qHip0.x() += qHip.x();
        qHip0.y() += qHip.y();
        qHip0.z() += qHip.z();
    }
    if (initCounter == initCounts) {
        posA0 += posA;
        posA0 = posA0 / initCounts;
        posHip0 += posHip;
        posHip0 = posHip0 / initCounts;
        qHip0.w() += qHip.w();
        qHip0.x() += qHip.x();
        qHip0.y() += qHip.y();
        qHip0.z() += qHip.z();
        qHip0.w() = qHip0.w() / initCounts;
        qHip0.x() = qHip0.x() / initCounts;
        qHip0.y() = qHip0.y() / initCounts;
        qHip0.z() = qHip0.z() / initCounts;
        qHip0 = qHip0.normalized();
    }
}

/**
 * @brief LawOpti::rotationMatrices compute the rotation matrices of the hip and forearm frames with respect to the global frame
 * @param qHip quaternions of the hip cluster
 * @param qFA_record quaternions of the forearm cluster
 */
void LawJacobian::rotationMatrices(Eigen::Quaterniond qHip, int initCounter, int initCounts)
{
    qHip_filt.w() = qHip_filt_old.w() + coeff * (qHip.w() - qHip_filt_old.w());
    qHip_filt.x() = qHip_filt_old.x() + coeff * (qHip.x() - qHip_filt_old.x());
    qHip_filt.y() = qHip_filt_old.y() + coeff * (qHip.y() - qHip_filt_old.y());
    qHip_filt.z() = qHip_filt_old.z() + coeff * (qHip.z() - qHip_filt_old.z());

    //    qHip_relative = quat_multiply(qHip_filt,quat_conj(qHip0));
    ///  From quaternions to orientation
    /// for IMU quaternion definition
    //    R11 = 2*qHip_relative.w()*qHip_relative.w() - 1 + 2*qHip_relative.x()*qHip_relative.x();
    //    R21 = 2*(qHip_relative.x()*qHip_relative.y() - qHip_relative.w()*qHip_relative.z());
    //    R31 = 2*(qHip_relative.x()*qHip_relative.z() + qHip_relative.w()*qHip_relative.y());
    //    R12 = 2*(qHip_relative.x()*qHip_relative.y() + qHip_relative.w()*qHip_relative.z());
    //    R22 = 2* qHip_relative.w()*qHip_relative.w() - 1 + 2*qHip_relative.y()*qHip_relative.y();
    //    R32 = 2*(qHip_relative.y()*qHip_relative.z() - qHip_relative.w()*qHip_relative.x());
    //    R13 = 2*(qHip_relative.x()*qHip_relative.z() - qHip_relative.w()*qHip_relative.y());
    //    R23 = 2*(qHip_relative.y()*qHip_relative.z() + qHip_relative.w()*qHip_relative.x());
    //    R33 = 2* qHip_relative.w()*qHip_relative.w() - 1 + 2*qHip_relative.z()*qHip_relative.z();
    /// For optitrack quaternion definition
    if (initCounter == initCounts) {
        R = qHip0.toRotationMatrix();
    } else {
        R = qHip_filt.toRotationMatrix();
    }
}

/**
 * @brief LawOpti::projectionInHip project positions in the hip frame
 * @param posA position of the acromion
 * @param posElbow position of the elbow
 * @param posHip position of the hip
 * @param initCounter counter
 * @param initCounts number of counts that defined the initial position
 */
void LawJacobian::projectionInHip(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts)
{
    // project in hip frame
    posAinHip = R.transpose() * (posA - posHip);
    if (initCounter == initCounts) {
        posA0inHip = R.transpose() * (posA0 - posHip0);
    }
}

void LawJacobian::bufferingOldValues()
{
    qHip_filt_old = qHip_filt;
}

void LawJacobian::updateFrames(double theta[], double l[])
{
    for (int i = 1; i < nbFrames; i++) {

        /// COMPUTE ROTATION MATRIX
        if (rel[i] == 'z') {
            Rframe(0, 0) = cos(theta[i - 1]);
            Rframe(0, 1) = sin(theta[i - 1]);
            Rframe(0, 2) = 0;
            Rframe(1, 0) = -sin(theta[i - 1]);
            Rframe(1, 1) = cos(theta[i - 1]);
            Rframe(1, 2) = 0;
            Rframe(2, 0) = 0;
            Rframe(2, 1) = 0;
            Rframe(2, 2) = 1;
        }

        if (rel[i] == 'x') {
            Rframe(0, 0) = -sin(theta[i - 1]);
            Rframe(0, 1) = cos(theta[i - 1]);
            Rframe(0, 2) = 0;
            Rframe(1, 0) = 0;
            Rframe(1, 1) = 0;
            Rframe(1, 2) = 1;
            Rframe(2, 0) = cos(theta[i - 1]);
            Rframe(2, 1) = sin(theta[i - 1]);
            Rframe(2, 2) = 0;
        }

        if (rel[i] == 'y') {
            Rframe(0, 0) = cos(theta[i - 1]);
            Rframe(0, 1) = sin(theta[i - 1]);
            Rframe(0, 2) = 0;
            Rframe(1, 0) = 0;
            Rframe(1, 1) = 0;
            Rframe(1, 2) = 1;
            Rframe(2, 0) = sin(theta[i - 1]);
            Rframe(2, 1) = -cos(theta[i - 1]);
            Rframe(2, 2) = 0;
        }

        /// UPDATE FRAMES
        x(0, i) = Rframe(0, 0) * x(0, i - 1) + Rframe(0, 1) * y(0, i - 1) + Rframe(0, 2) * z(0, i - 1);
        x(1, i) = Rframe(0, 0) * x(1, i - 1) + Rframe(0, 1) * y(1, i - 1) + Rframe(0, 2) * z(1, i - 1);
        x(2, i) = Rframe(0, 0) * x(2, i - 1) + Rframe(0, 1) * y(2, i - 1) + Rframe(0, 2) * z(2, i - 1);

        y(0, i) = Rframe(1, 0) * x(0, i - 1) + Rframe(1, 1) * y(0, i - 1) + Rframe(1, 2) * z(0, i - 1);
        y(1, i) = Rframe(1, 0) * x(1, i - 1) + Rframe(1, 1) * y(1, i - 1) + Rframe(1, 2) * z(1, i - 1);
        y(2, i) = Rframe(1, 0) * x(2, i - 1) + Rframe(1, 1) * y(2, i - 1) + Rframe(1, 2) * z(2, i - 1);

        z(0, i) = Rframe(2, 0) * x(0, i - 1) + Rframe(2, 1) * y(0, i - 1) + Rframe(2, 2) * z(0, i - 1);
        z(1, i) = Rframe(2, 0) * x(1, i - 1) + Rframe(2, 1) * y(1, i - 1) + Rframe(2, 2) * z(1, i - 1);
        z(2, i) = Rframe(2, 0) * x(2, i - 1) + Rframe(2, 1) * y(2, i - 1) + Rframe(2, 2) * z(2, i - 1);
    }

    /// UPDATE POSITIONS OF CENTERS OF FRAMES
    OO(0, 0) = l[0] * y(0, 1) + l[1] * x(0, 2) + l[3] * x(0, 3);
    OO(1, 0) = l[0] * y(1, 1) + l[1] * x(1, 2) + l[3] * x(1, 3);
    OO(2, 0) = l[0] * y(2, 1) + l[1] * x(2, 2) + l[3] * x(2, 3);

    OO(0, 1) = l[1] * x(0, 2) + l[3] * x(0, 3);
    OO(1, 1) = l[1] * x(1, 2) + l[3] * x(1, 3);
    OO(2, 1) = l[1] * x(2, 2) + l[3] * x(2, 3);

    OO(0, 2) = l[3] * x(0, 3);
    OO(1, 2) = l[3] * x(1, 3);
    OO(2, 2) = l[3] * x(2, 3);
}

void LawJacobian::controlLaw(Eigen::Vector3d posA, int lambda, double threshold[])
{
    /// COMPUTE JACOBIAN
    for (int i = 0; i < nbLinks; i++) {
        J.block<3, 1>(0, i) = z.block<3, 1>(0, i).cross(OO.block<3, 1>(0, i));
    }
    /// COMPUTE delta, position error of acromion
    delta = posA - posA0;
    qDebug("nb of rows J: %d\n", J.RowsAtCompileTime);
    qDebug("nb of col J: %d\n", J.ColsAtCompileTime);
    /// COMPUTE ANG. VELOCITIES
    //    thetaNew = (J.completeOrthogonalDecomposition().pseudoInverse()) * delta;
    //    thetaNew = (J.pseudoInverse()) * delta;
    //    for (int i = 0; i < nbFrames; i++) {
    //        if (thetaNew[i] < threshold[i])
    //            thetaNew[i] = 0;
    //        else if (thetaNew[i] >= threshold[i])
    //            thetaNew[i] = thetaNew[i] - threshold[i];
    //        else if (thetaNew[i] <= threshold[i])
    //            thetaNew[i] = thetaNew[i] + threshold[i];
    //    }
    //    thetaDot = lambda * thetaNew;
}

void LawJacobian::writeDebugData(double debug[], double theta[])
{
    for (int i = 0; i < nbFrames; i++) {
        debug[i] = theta[i];
        debug[i + nbFrames] = thetaNew[i];
        debug[i + 2 * nbFrames] = thetaDot[i];
    }
}

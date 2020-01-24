#include "lawjacobian.h"
#include "utils/log/log.h"
#include "utils/pseudoinverse.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>
#include <iostream>
#include <math.h>
#include <string.h>

LawJacobian::LawJacobian()
{
}

LawJacobian::~LawJacobian()
{
}

void LawJacobian::initialization(Eigen::Quaterniond qHip, unsigned int freq)
{
    /// POSITIONS AND QUATERNIONS
    qHip0.w() = 0.;
    qHip0.x() = 0.;
    qHip0.y() = 0.;
    qHip0.z() = 0.;
    qRecalH.w() = 0.;
    qRecalH.x() = 0.;
    qRecalH.y() = 0.;
    qRecalH.z() = 0.;
    qIdealH.w() = 0.;
    qIdealH.x() = 0.;
    qIdealH.y() = 0.;
    qIdealH.z() = 0.;
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
    xref << 1., 0., 0.;
    yref << 0., 1., 0.;
    zref << 0., 0., 1.;
    samplePeriod = 1. / freq;
    coeff = samplePeriod / (0.03 + samplePeriod);
    delta[0] = 0;
    delta[1] = 0;
    delta[2] = 0;
    Rhip = Eigen::Matrix3d::Zero();
    Rhand = Eigen::Matrix3d::Zero();
    RtrunkInHand = Eigen::Matrix3d::Zero();
    RtrunkHipInHand = Eigen::Matrix3d::Zero();
    RArmInHand = Eigen::Matrix3d::Zero();
    thetaNew = Eigen::MatrixXd::Zero(nbLinks, 1);
    thetaDot = Eigen::MatrixXd::Zero(nbLinks, 1);
    eulerT = Eigen::MatrixXd::Zero(3, 1);
    J = Eigen::MatrixXd::Zero(3, nbLinks);
    Jp = Eigen::MatrixXd::Zero(3, 1);
    Jo = Eigen::MatrixXd::Zero(3, nbLinks - 1);
    dlsJ = Eigen::MatrixXd::Zero(nbLinks, 3);
    dlsJp = Eigen::RowVector3d::Zero();
    dlsJo = Eigen::MatrixXd::Zero(nbLinks - 1, 3);
    OO = Eigen::MatrixXd::Zero(3, nbLinks);
    IO = Eigen::Vector3d::Zero();

    // Rotation matrix from IMU or hand rigid body frame to theoretical arm
    if (nbLinks == 2) {
        R0 << 0., 1., 0.,
            0., 0., 1.,
            1., 0., 0.;
    } else if (nbLinks == 3) {
        R0 << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;
    }
    // Identity matrix
    I3 << 1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.;
    //    I3 = Eigen::Matrix<int, 3, 3>::Identity();
}

void LawJacobian::initializationOpti(Eigen::Vector3d posA)
{
    /// POSITIONS AND QUATERNIONS
    posA0 = Eigen::Vector3d::Zero();
    posAinHip = posA;
    posA0inHip = posA;
    posAinHand = Eigen::Vector3d::Zero();
    posHip0 = Eigen::Vector3d::Zero();
}

void LawJacobian::initializationIMU()
{
    qTrunk0.w() = 0.;
    qTrunk0.x() = 0.;
    qTrunk0.y() = 0.;
    qTrunk0.z() = 0.;
    qArm0.w() = 0.;
    qArm0.x() = 0.;
    qArm0.y() = 0.;
    qArm0.z() = 0.;
    qRecalT.w() = 0.;
    qRecalT.x() = 0.;
    qRecalT.y() = 0.;
    qRecalT.z() = 0.;
    qIdealT.w() = 0.;
    qIdealT.x() = 0.;
    qIdealT.y() = 0.;
    qIdealT.z() = 0.;
    Rframe = Eigen::Matrix3d::Zero();
}
/**
 * @brief LawJacobian::initialPositions computes the initial position of the acromion marker = mean over the initCounts first measures of the acromion position
 *  and the initial position + orientation of the hip cluster = mean over the initCounts first measures
 * @param posA cartesian coordinates of the acromion marker (from optitrack)
 * @param posHip cartesian coordinates of the hip marker (from optitrack)
 * @param initCounter counter of time steps
 * @param initCounts number of time step to take into account to compute the initial position
 */
void LawJacobian::initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts)
{
    if (initCounter < initCounts) {
        posA0 += posA;
        posHip0 += posHip;
    }
    if (initCounter == initCounts) {
        posA0 += posA;
        posA0 = posA0 / initCounts;
        posHip0 += posHip;
        posHip0 = posHip0 / initCounts;
    }
}

/**
 * @brief LawJacobian::initialQuat computes the initial quaternions = mean over the initCounts first measures (linear approximation)
 * @param qHip quaternions of the hip
 * @param qTrunk quaternions of the trunk
 * @param initCounter counter
 * @param initCounts number of time steps for the initialization
 */
void LawJacobian::initialQuat(Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk, Eigen::Quaterniond qArm, int initCounter, int initCounts)
{
    if (initCounter < initCounts) {
        qHip0.w() += qHip.w();
        qHip0.x() += qHip.x();
        qHip0.y() += qHip.y();
        qHip0.z() += qHip.z();
        qTrunk0.w() += qTrunk.w();
        qTrunk0.x() += qTrunk.x();
        qTrunk0.y() += qTrunk.y();
        qTrunk0.z() += qTrunk.z();
        qArm0.w() += qArm.w();
        qArm0.x() += qArm.x();
        qArm0.y() += qArm.y();
        qArm0.z() += qArm.z();
    }
    if (initCounter == initCounts) {
        qHip0.w() += qHip.w();
        qHip0.x() += qHip.x();
        qHip0.y() += qHip.y();
        qHip0.z() += qHip.z();
        qHip0.w() = qHip0.w() / initCounts;
        qHip0.x() = qHip0.x() / initCounts;
        qHip0.y() = qHip0.y() / initCounts;
        qHip0.z() = qHip0.z() / initCounts;
        qHip0 = qHip0.normalized();

        qTrunk0.w() += qTrunk.w();
        qTrunk0.x() += qTrunk.x();
        qTrunk0.y() += qTrunk.y();
        qTrunk0.z() += qTrunk.z();
        qTrunk0.w() = qTrunk0.w() / initCounts;
        qTrunk0.x() = qTrunk0.x() / initCounts;
        qTrunk0.y() = qTrunk0.y() / initCounts;
        qTrunk0.z() = qTrunk0.z() / initCounts;
        qTrunk0 = qTrunk0.normalized();

        qArm0.w() += qArm.w();
        qArm0.x() += qArm.x();
        qArm0.y() += qArm.y();
        qArm0.z() += qArm.z();
        qArm0.w() = qArm0.w() / initCounts;
        qArm0.x() = qArm0.x() / initCounts;
        qArm0.y() = qArm0.y() / initCounts;
        qArm0.z() = qArm0.z() / initCounts;
        qArm0 = qArm0.normalized();
    }
}

/**
 * @brief LawJacobian::idealFrames correct the IMU frames to make the axis that should be vertical really vertical
 * @param qHand quaternions of the hand IMU
 * @param qHip quaternions of the hip IMU
 * @param qTrunk quaternions of the trunk IMU
 */
void LawJacobian::idealFrames(Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk, int initCounter, int initCounts)
{

    /// compute tilt between initial IMu orientation and vertical
    if (initCounter == initCounts) {
        /// HIP
        Eigen::Quaterniond Yquat;
        Yquat.w() = 0;
        Yquat.vec() = yref;

        Eigen::Quaterniond Zquat;
        Zquat.w() = 0;
        Zquat.vec() = zref;

        Eigen::Quaterniond rotatedYquat = qHip0.inverse() * Yquat * qHip0;
        Eigen::Vector3d rotatedY = rotatedYquat.vec();

        Eigen::Vector3d crossP = rotatedY.cross(zref);
        crossP = crossP.normalized();
        theta0H = acos(rotatedY.dot(zref));
        qRecalH.w() = cos(-theta0H / 2);
        qRecalH.x() = sin(-theta0H / 2) * crossP(0);
        qRecalH.y() = sin(-theta0H / 2) * crossP(1);
        qRecalH.z() = sin(-theta0H / 2) * crossP(2);

        /// TRUNK
        rotatedYquat = qTrunk0.inverse() * Yquat * qTrunk0;
        rotatedY = rotatedYquat.vec();

        crossP = rotatedY.cross(zref);
        crossP = crossP.normalized();
        theta0T = acos(rotatedY.dot(zref));
        qRecalT.w() = cos(-theta0T / 2);
        qRecalT.x() = sin(-theta0T / 2) * crossP(0);
        qRecalT.y() = sin(-theta0T / 2) * crossP(1);
        qRecalT.z() = sin(-theta0T / 2) * crossP(2);

        qIdealH = qHip * qRecalH;
        qIdealH = qIdealH.normalized();

        qIdealT = qTrunk * qRecalT;
        qIdealT = qIdealT.normalized();

    } else if (initCounter > initCounts) {
        qIdealH = qHip * qRecalH;
        qIdealH = qIdealH.normalized();

        qIdealT = qTrunk * qRecalT;
        qIdealT = qIdealT.normalized();
    }
}

/**
 * @brief LawOpti::rotationMatrices compute the rotation matrices of the hip and forearm frames with respect to the global frame
 * @param qHip quaternions of the hip IMU
 * @param qhand quaternions of the hand IMU
 * @param qTrunk quaternions of the trunk IMU
 */
void LawJacobian::rotationMatrices(Eigen::Quaterniond qHand, Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk)
{
    ///  From quaternions to orientation
    //    Rhip = qIdealH.toRotationMatrix();
    //    Rtrunk = qIdealT.toRotationMatrix();

    Rhip = qHip.toRotationMatrix();
    Rtrunk = qTrunk.toRotationMatrix();
    Rhand = qHand.toRotationMatrix();
}

/**
 * @brief LawJacobian::projectionInHip project positions in the hip frame
 * @param posA position of the acromion
 * @param posElbow position of the elbow
 * @param posHip position of the hip
 * @param initCounter counter
 * @param initCounts number of counts that defined the initial position
 */
void LawJacobian::projectionInHip(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts)
{
    // project in hip frame
    if (initCounter == initCounts) {
        // for optitrack quaternions
        //        posA0inHip = Rhip.transpose() * (posA0 - posHip0);
        // for imu quaternions
        posA0inHip = Rhip * (posA0 - posHip0);
    }
    // for optitrack quaternions
    //    posAinHip = Rhip.transpose() * (posA - posHip);
    // for imu quaternions
    posAinHip = Rhip * (posA - posHip);

    if (initCounter % 50 == 0) {
        //        debug() << "posA in hip: " << posAinHip[0] << ", " << posAinHip[1] << ", " << posAinHip[2];
    }
}

/**
 * @brief LawJacobian::projectionInHipIMU compute acromion positions in the hip frame
 * @param posA position of the acromion
 * @param initCounter counter
 * @param initCounts number of counts that defined the initial position
 */
void LawJacobian::projectionInHipIMU(int lt, int lsh, int initCounter, int initCounts)
{
    // project in hip frame
    if (initCounter == initCounts) {
        // for imu quaternions
        posA0inHip = Rhip * Rtrunk.transpose() * (lt * yref + lsh * xref);
    }
    // for imu quaternions
    posAinHip = Rhip * Rtrunk.transpose() * (lt * yref + lsh * xref);

    if (initCounter % 50 == 0) {
        //        debug() << "posA in hip: " << posAinHip[0] << ", " << posAinHip[1] << ", " << posAinHip[2];
    }
}

/**
 * @brief LawJacobian::computeTrunkAngles compute Euler angles of the trunk, projected into hand frame
 * @param qHand quaternions of the hand IMU
 * @param qTrunk quaternions of the trunk IMU
 */
void LawJacobian::computeTrunkAngles(Eigen::Quaterniond qHand, Eigen::Quaterniond qTrunk, Eigen::Quaterniond qHip)
{
    RtrunkInHand = R0 * ((qHand * qTrunk0.conjugate() * qTrunk * qHand.conjugate()).toRotationMatrix()) * R0.transpose();
    eulerT(0) = atan2(RtrunkInHand(1, 2), RtrunkInHand(2, 2)); //rotation around X-axis
    eulerT(1) = -atan(RtrunkInHand(0, 2) / sqrt(1 - RtrunkInHand(0, 2) * RtrunkInHand(0, 2))); //rotation around Y-axis
    eulerT(2) = atan2(RtrunkInHand(0, 1), RtrunkInHand(0, 0)); //rotation around Z-axis

    //    RtrunkHipInHand = R0 * ((qHand * qHip.conjugate() * qHip0 * qTrunk0.conjugate() * qTrunk * qHand.conjugate()).toRotationMatrix()) * R0.transpose();
    //    eulerT(0) = atan2(RtrunkHipInHand(1, 2), RtrunkHipInHand(2, 2)); //rotation around X-axis
    //    eulerT(1) = -atan(RtrunkHipInHand(0, 2) / sqrt(1 - RtrunkHipInHand(0, 2) * RtrunkHipInHand(0, 2))); //rotation around Y-axis
    //    eulerT(2) = atan2(RtrunkHipInHand(0, 1), RtrunkHipInHand(0, 0)); //rotation around Z-axis
}

/**
 * @brief LawJacobian::computeArmAngles compute rotation angles of the shoulder, with respect to the trunk, projected in z0 axis of hand frame
 * @param qHand quaternions of hand IMU
 * @param qTrunk quaternions of trunk IMU
 * @param qArm quaternions of arm IMU
 */
void LawJacobian::computeArmAngles(Eigen::Quaterniond qHand, Eigen::Quaterniond qTrunk, Eigen::Quaterniond qArm)
{
    RArmInHand = R0 * ((qHand * qTrunk.conjugate() * qTrunk0 * qArm0.conjugate() * qArm * qHand.conjugate()).toRotationMatrix()) * R0.transpose();
    eulerA(0) = atan2(RArmInHand(1, 2), RArmInHand(2, 2)); //rotation around X-axis
    eulerA(1) = -atan(RArmInHand(0, 2) / sqrt(1 - RArmInHand(0, 2) * RArmInHand(0, 2))); //rotation around Y-axis
    eulerA(2) = atan2(RArmInHand(0, 1), RArmInHand(0, 0)); //rotation around Z-axis
}

void LawJacobian::bufferingOldValues()
{
    qHip_filt_old = qHip_filt;
}

/**
 * @brief LawJacobian::updateFrames compute the current orientation of the links frames
 * @param theta current angles of the joints
 */
void LawJacobian::updateFrames(double theta[])
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

        //        debug() << "x" << i << ": " << x(0, i) << "; " << x(1, i) << "; " << x(2, i) << "\n";
        //        debug() << "y" << i << ": " << y(0, i) << "; " << y(1, i) << "; " << y(2, i) << "\n";
        //        debug() << "z" << i << ": " << z(0, i) << "; " << z(1, i) << "; " << z(2, i) << "\n";
    }
}

/**
 * @brief LawJacobian::updateFramesinEE compute the current orientation of the links frames expressed in EE frame
 * @param theta current angles of the joints
 */
void LawJacobian::updateFramesinEE(double theta[])
{
    for (int i = nbFrames - 2; i > 0; i--) {

        /// COMPUTE ROTATION MATRIX
        if (rel[i + 1] == 'z') {
            Rframe(0, 0) = cos(theta[i]);
            Rframe(1, 0) = sin(theta[i]);
            Rframe(2, 0) = 0;
            Rframe(0, 1) = -sin(theta[i]);
            Rframe(1, 1) = cos(theta[i]);
            Rframe(2, 1) = 0;
            Rframe(0, 1) = 0;
            Rframe(1, 2) = 0;
            Rframe(2, 2) = 1;
        }

        if (rel[i + 1] == 'x') {
            Rframe(0, 0) = -sin(theta[i]);
            Rframe(1, 0) = cos(theta[i]);
            Rframe(2, 0) = 0;
            Rframe(0, 1) = 0;
            Rframe(1, 1) = 0;
            Rframe(2, 1) = 1;
            Rframe(0, 2) = cos(theta[i]);
            Rframe(1, 2) = sin(theta[i]);
            Rframe(2, 2) = 0;
        }

        if (rel[i + 1] == 'y') {
            Rframe(0, 0) = cos(theta[i]);
            Rframe(1, 0) = sin(theta[i]);
            Rframe(2, 0) = 0;
            Rframe(0, 1) = 0;
            Rframe(1, 1) = 0;
            Rframe(2, 1) = 1;
            Rframe(0, 2) = sin(theta[i]);
            Rframe(1, 2) = -cos(theta[i]);
            Rframe(2, 2) = 0;
        }

        /// UPDATE FRAMES
        x(0, i) = Rframe(0, 0) * x(0, i + 1) + Rframe(0, 1) * y(0, i + 1) + Rframe(0, 2) * z(0, i + 1);
        x(1, i) = Rframe(0, 0) * x(1, i + 1) + Rframe(0, 1) * y(1, i + 1) + Rframe(0, 2) * z(1, i + 1);
        x(2, i) = Rframe(0, 0) * x(2, i + 1) + Rframe(0, 1) * y(2, i + 1) + Rframe(0, 2) * z(2, i + 1);

        y(0, i) = Rframe(1, 0) * x(0, i + 1) + Rframe(1, 1) * y(0, i + 1) + Rframe(1, 2) * z(0, i + 1);
        y(1, i) = Rframe(1, 0) * x(1, i + 1) + Rframe(1, 1) * y(1, i + 1) + Rframe(1, 2) * z(1, i + 1);
        y(2, i) = Rframe(1, 0) * x(2, i + 1) + Rframe(1, 1) * y(2, i + 1) + Rframe(1, 2) * z(2, i + 1);

        z(0, i) = Rframe(2, 0) * x(0, i + 1) + Rframe(2, 1) * y(0, i + 1) + Rframe(2, 2) * z(0, i + 1);
        z(1, i) = Rframe(2, 0) * x(1, i + 1) + Rframe(2, 1) * y(1, i + 1) + Rframe(2, 2) * z(1, i + 1);
        z(2, i) = Rframe(2, 0) * x(2, i + 1) + Rframe(2, 1) * y(2, i + 1) + Rframe(2, 2) * z(2, i + 1);

        //        debug() << "x" << i << ": " << x(0, i) << "; " << x(1, i) << "; " << x(2, i) << "\n";
        //        debug() << "y" << i << ": " << y(0, i) << "; " << y(1, i) << "; " << y(2, i) << "\n";
        //        debug() << "z" << i << ": " << z(0, i) << "; " << z(1, i) << "; " << z(2, i) << "\n";
    }
}

/**
 * @brief LawJacobian::computeOriginsVectors compute the vectors from the origins of the frames to the end-effector position
 * @param l lengths of the arm segments
 * @param nbDOF number of DOF of the prosthetic arm
 */
void LawJacobian::computeOriginsVectors(int l[], int nbDOF)
{
    if (nbDOF == 2) {
        /// UPDATE POSITIONS OF CENTERS OF FRAMES
        OO(0, 0) = -l[0] * z(0, 0) - l[1] * y(0, 1) - l[2] * y(0, 2);
        OO(1, 0) = -l[0] * z(1, 0) - l[1] * y(1, 1) - l[2] * y(1, 2);
        OO(2, 0) = -l[0] * z(2, 0) - l[1] * y(2, 1) - l[2] * y(2, 2);

        OO(0, 1) = -l[2] * y(0, 2);
        OO(1, 1) = -l[2] * y(1, 2);
        OO(2, 1) = -l[2] * y(2, 2);

        //    debug() << "OO 04: " << OO(0, 0) << "; " << OO(1, 0) << "; " << OO(2, 0) << "\n";
        //    debug() << "OO 14: " << OO(0, 1) << "; " << OO(1, 1) << "; " << OO(2, 1) << "\n";
        //    debug() << "OO 24: " << OO(0, 2) << "; " << OO(1, 2) << "; " << OO(2, 2) << "\n";
    }

    if (nbDOF == 3) {
        /// UPDATE POSITIONS OF CENTERS OF FRAMES
        OO(0, 0) = (l[0] + l[1]) * z(0, 1) + l[2] * x(0, 3);
        OO(1, 0) = (l[0] + l[1]) * z(1, 1) + l[2] * x(1, 3);
        OO(2, 0) = (l[0] + l[1]) * z(2, 1) + l[2] * x(2, 3);

        OO(0, 1) = (l[0] + l[1]) * z(0, 1) + l[2] * x(0, 3);
        OO(1, 1) = (l[0] + l[1]) * z(1, 1) + l[2] * x(1, 3);
        OO(2, 1) = (l[0] + l[1]) * z(2, 1) + l[2] * x(2, 3);

        OO(0, 2) = l[2] * x(0, 3);
        OO(1, 2) = l[2] * x(1, 3);
        OO(2, 2) = l[2] * x(2, 3);

        //        debug() << "OO 04: " << OO(0, 0) << "; " << OO(1, 0) << "; " << OO(2, 0) << "\n";
        //        debug() << "OO 14: " << OO(0, 1) << "; " << OO(1, 1) << "; " << OO(2, 1) << "\n";
        //        debug() << "OO 24: " << OO(0, 2) << "; " << OO(1, 2) << "; " << OO(2, 2) << "\n";
    }
}

/**
 * @brief LawJacobian::controlLaw_v1 delta = acromion displacement in hip frame
 * @param k damping parameters
 * @param lambda gain
 * @param threshold
 * @param _cnt
 */
void LawJacobian::controlLaw_v1(int k, double lambda[], double threshold[], int _cnt)
{
    /// COMPUTE JACOBIAN
    for (int i = 0; i < nbLinks; i++) {
        J.block<3, 1>(0, i) = z.block<3, 1>(0, i).cross(OO.block<3, 1>(0, i));
    }
    /// COMPUTE delta, position error of acromion
    /// for optitrack quaternion for hand frame, no projection in hip
    //    delta = R0 * Rhand * (posA0 - posA);
    /// for IMU quaternion
    delta = R0 * Rhand * Rhip.transpose() * (posA0inHip - posAinHip);
    /// for optitrack quaternions
    //    delta = R0 * Rhand.transpose() * Rhip * (posA0 - posA);
    //    delta = Rhip * Rhand.transpose() * (posA0inHip - posAinHip);

    /// PSEUDO INVERSE SOLUTION
    //    pinvJ = pseudoinverse<Eigen::MatrixXd>(J, 1e-3);

    /// DAMPED LEAST SQUARE SOLUTION
    dlsJ = J.transpose() * (J * J.transpose() + k * k * I3).inverse();

    /// COMPUTE ANG. VELOCITIES
    thetaNew = dlsJ * delta;
    // deadzone
    for (int i = 0; i < nbLinks; i++) {
        if (abs(thetaNew(i)) < threshold[i])
            thetaNew(i) = 0;
        else if (thetaNew(i) >= threshold[i])
            thetaNew(i) = thetaNew(i) - threshold[i];
        else if (thetaNew(i) <= -threshold[i])
            thetaNew(i) = thetaNew(i) + threshold[i];
    }
    // display data
    if (_cnt % 50 == 0) {
        debug() << "delta: " << delta(0) << "; " << delta(1) << "; " << delta(2) << "\r\n";
        //        debug() << "pinvJ: " << pinvJ(0, 0) << "; " << pinvJ(0, 1) << "; " << pinvJ(0, 2) << "\r\n";
        debug() << "thetaNew(after threshold, deg): ";
        for (int i = 0; i < nbLinks; i++) {
            debug() << thetaNew(i) * 180 / M_PI;
        }
    }
    // Angular velocities
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
        // speed limits
        if (abs(thetaDot(i)) > 60 * M_PI / 180) {
            thetaDot(i) = thetaDot(i) / abs(thetaDot(i)) * 60 * M_PI / 180;
        }
    }
}

/**
 * @brief LawJacobian::controlLaw_v2 position and orientation separated with two different jacobians. Elbow(1DOF) -> position, wrist(1 or 2DOF) -> orientation.
 * @param k
 * @param lambda
 * @param threshold
 * @param _cnt
 */
void LawJacobian::controlLaw_v2(int k, double lambda[], double threshold[], int _cnt)
{
    // Jacobian matrix for position control, taking only elbow into account
    Jp.block<3, 1>(0, 1) = z.block<3, 1>(0, nbLinks).cross(OO.block<3, 1>(0, nbLinks));
    // Jacobian matrix for orientation control, taking only wrist into account
    for (int i = 0; i < nbLinks - 1; i++) {
        Jo.block<3, 1>(0, i) = z.block<3, 1>(0, i);
    }
    /// DAMPED LEAST SQUARE SOLUTION
    dlsJp = Jp.transpose() * (Jp * Jp.transpose() + k * k * I3).inverse();
    dlsJo = Jo.transpose() * (Jo * Jo.transpose() + k * k * I3).inverse();
    // wrist angles
    thetaNew.block<nbLinks - 1, 1>(0, 0) = dlsJo * eulerT;
    // OR other version for wrist
    thetaNew(0) = eulerT(2); // rotation around Zhand
    thetaNew(1) = eulerT(0); // rotation around Xhand

    // elbow angle
    delta = R0 * Rhand * Rhip.transpose() * (posA0inHip - posAinHip);
    thetaNew(nbLinks) = dlsJp * delta;

    // deadzone
    for (int i = 0; i < nbLinks; i++) {
        if (abs(thetaNew(i)) < threshold[i])
            thetaNew(i) = 0;
        else if (thetaNew(i) >= threshold[i])
            thetaNew(i) = thetaNew(i) - threshold[i];
        else if (thetaNew(i) <= -threshold[i])
            thetaNew(i) = thetaNew(i) + threshold[i];
    }
    // Angular velocities
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
        // speed limits
        if (abs(thetaDot(i)) > 60 * M_PI / 180) {
            thetaDot(i) = thetaDot(i) / abs(thetaDot(i)) * 60 * M_PI / 180;
        }
    }
}

/**
 * @brief LawJacobian::controlLaw_v3 each DOF is taken as independant. delta = angular velocity of the trunk expressed at acromion point
 * @param lt trunk length
 * @param lsh neck-> shoulder length
 * @param k
 * @param lambda
 * @param threshold
 * @param _cnt
 */
void LawJacobian::controlLaw_v3(int lt, int lsh, int k, double lambda[], double threshold[], int _cnt)
{
    for (int i = 0; i < nbLinks; i++) {
        J.block<3, 1>(0, i) = z.block<3, 1>(0, i).cross(OO.block<3, 1>(0, i));
    }
    for (int i = 0; i < nbLinks; i++) {
        dlsJ.block<1, 3>(i, 0) = (J.block<3, 1>(0, i)).transpose() * (J.block<3, 1>(0, i) * (J.block<3, 1>(0, i)).transpose() + k * k * I3).inverse();
    }

    IO = R0 * Rhand * Rtrunk.transpose() * (lt * yref + lsh * xref);
    delta = (-eulerT).cross(R0 * Rhand * Rtrunk.transpose() * (lt * yref + lsh * xref));
    thetaNew = dlsJ * delta;

    // deadzone
    for (int i = 0; i < nbLinks; i++) {
        if (abs(thetaNew(i)) < threshold[i])
            thetaNew(i) = 0;
        else if (thetaNew(i) >= threshold[i])
            thetaNew(i) = thetaNew(i) - threshold[i];
        else if (thetaNew(i) <= -threshold[i])
            thetaNew(i) = thetaNew(i) + threshold[i];
    }

    if (_cnt % 50 == 0) {
        debug() << "delta: " << delta(0) << "; " << delta(1) << "; " << delta(2) << "\r\n";
        debug() << "thetaNew(after threshold, deg): ";
        for (int i = 0; i < nbLinks; i++) {
            debug() << thetaNew(i) * 180 / M_PI;
        }
    }

    // Angular velocities
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
        // speed limits
        if (abs(thetaDot(i)) > 60 * M_PI / 180) {
            thetaDot(i) = thetaDot(i) / abs(thetaDot(i)) * 60 * M_PI / 180;
        }
    }
}

/**
 * @brief LawJacobian::controlLaw_v4 elbow + wrist pronation solved with 2DOF model with delta = ang. velocity of the trunk expressed at acromion;
 * wrist flexion = angle around hand z0-axis due to shoulder rotation w/r to the trunk
 * @param lt
 * @param lsh
 * @param k
 * @param lambda
 * @param threshold
 * @param _cnt
 */
void LawJacobian::controlLaw_v4(int lt, int lsh, int k, double lambda[], double threshold[], int _cnt)
{
    // jacobian only for 2DOF
    for (int i = 0; i < 2; i++) {
        J.block<3, 1>(0, i) = z.block<3, 1>(0, i).cross(OO.block<3, 1>(0, i));
    }
    for (int i = 0; i < 2; i++) {
        dlsJ.block<1, 3>(i, 0) = (J.block<3, 1>(0, i)).transpose() * (J.block<3, 1>(0, i) * (J.block<3, 1>(0, i)).transpose() + k * k * I3).inverse();
    }

    IO = R0 * Rhand * Rtrunk.transpose() * (lt * yref + lsh * xref);
    delta = (-eulerT).cross(R0 * Rhand * Rtrunk.transpose() * (lt * yref)); // + lsh * xref));
    if (nbLinks == 2) { // no wrist flexion
        thetaNew = dlsJ * delta;
    } else if (nbLinks == 3) { // wrist flexion -> from shoulder rotation
        thetaNew.block<2, 1>(1, 0) = dlsJ * delta;
        thetaNew(0) = eulerA(3);
    }

    // deadzone
    for (int i = 0; i < nbLinks; i++) {
        if (abs(thetaNew(i)) < threshold[i])
            thetaNew(i) = 0;
        else if (thetaNew(i) >= threshold[i])
            thetaNew(i) = thetaNew(i) - threshold[i];
        else if (thetaNew(i) <= -threshold[i])
            thetaNew(i) = thetaNew(i) + threshold[i];
    }

    if (_cnt % 50 == 0) {
        debug() << "delta: " << delta(0) << "; " << delta(1) << "; " << delta(2) << "\r\n";
        debug() << "thetaNew(after threshold, deg): ";
        for (int i = 0; i < nbLinks; i++) {
            debug() << thetaNew(i) * 180 / M_PI;
        }
    }

    // Angular velocities
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
        // speed limits
        if (abs(thetaDot(i)) > 60 * M_PI / 180) {
            thetaDot(i) = thetaDot(i) / abs(thetaDot(i)) * 60 * M_PI / 180;
        }
    }
}

void LawJacobian::writeDebugData(double d[], double theta[])
{
    for (int i = 0; i < nbLinks; i++) {
        d[i] = theta[i];
        d[i + nbLinks] = thetaNew(i);
        d[i + 2 * nbLinks] = thetaDot(i);
    }
    for (int i = 0; i < nbLinks; i++) {
        for (int j = 0; j < 3; j++) {
            d[3 * nbLinks + j + 3 * i] = OO(j, i);
        }
    }

    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + j] = delta(j);
    }
    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + 3 + j] = IO(j);
    }
    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + 6 + j] = eulerT(j);
    }
    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + 9 + j] = eulerA(j);
    }
}

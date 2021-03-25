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

    qAcr0.w() = 0.;
    qAcr0.x() = 0.;
    qAcr0.y() = 0.;
    qAcr0.z() = 0.;

    Y0.w() = 0.0;
    Y0.x() = 0.0;
    Y0.y() = 1.0;
    Y0.z() = 0.0;

    X0.w() = 0.0;
    X0.x() = 1.0;
    X0.y() = 0.0;
    X0.z() = 0.0;

    Z0.w() = 0.0;
    Z0.x() = 0.0;
    Z0.y() = 0.0;
    Z0.z() = 1.0;

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
    deltaOpti[0] = 0;
    deltaOpti[1] = 0;
    deltaOpti[2] = 0;
    RHA = Eigen::Matrix3d::Zero();
    Rhip = Eigen::Matrix3d::Zero();
    Rhand = Eigen::Matrix3d::Zero();
    RhipOpti = Eigen::Matrix3d::Zero();
    RhandOpti = Eigen::Matrix3d::Zero();
    RtrunkInHand = Eigen::Matrix3d::Zero();
    RtrunkHipInHand = Eigen::Matrix3d::Zero();
    RArmInHand = Eigen::Matrix3d::Zero();
    thetaNew = Eigen::MatrixXd::Zero(nbLinks, 1);
    thetaNewOpti = Eigen::MatrixXd::Zero(nbLinks, 1);
    thetaDot = Eigen::MatrixXd::Zero(nbLinks, 1);
    eulerT = Eigen::MatrixXd::Zero(3, 1);
    eulerHA = Eigen::MatrixXd::Zero(3, 1);
    thetaHA = Eigen::MatrixXd::Zero(2, 1);
    eulerAcr = Eigen::MatrixXd::Zero(3, 1);

    J = Eigen::MatrixXd::Zero(3, nbLinks);
    dlsJ = Eigen::MatrixXd::Zero(nbLinks, 3);
    OO = Eigen::MatrixXd::Zero(3, nbLinks);
    IO = Eigen::Vector3d::Zero();
    disp = Eigen::Vector3d::Zero();
    dispOpti = Eigen::Vector3d::Zero();

    // Rotation matrix from IMU or hand rigid body frame to theoretical arm
    if (nbLinks == 2) {
        // if IMU on hand when cable towards bottom
        //        R0 << 0., 1., 0.,
        //            0., 0., 1.,
        //            1., 0., 0.;
        // if IMU on hand when tare with elbow at 90°
        R0 << 0., -1., 0.,
            0., 0., -1.,
            1., 0., 0.;
        // if IMU on upper arm
        //        R0 << 0., 1., 0.,
        //            -1., 0., 0.,
        //            0., 0., 1.;

        I << 1., 0.,
            0., 1.;
    } else if (nbLinks == 3) {
        R0 << 1., 0., 0.,
            0., 0., -1.,
            0., 1., 0.;

        I << 1., 0., 0.,
            0., 1., 0,
            0., 0., 1.;
    }
    // Identity matrix

    //    I3 = Eigen::Matrix<int, 3, 3>::Identity();
}

void LawJacobian::initializationOpti(Eigen::Vector3d posA)
{
    /// POSITIONS AND QUATERNIONS
    posA0 = Eigen::Vector3d::Zero();
    posAinHip = posA;
    posA0inHip = posA;
    posAinHipOpti = posA;
    posA0inHipOpti = posA;
    posAinHand = Eigen::Vector3d::Zero();
    posHip0 = Eigen::Vector3d::Zero();
}

void LawJacobian::initializationIMU()
{

    qAcr0.w() = 0.;
    qAcr0.x() = 0.;
    qAcr0.y() = 0.;
    qAcr0.z() = 0.;
    //    qTrunk0.w() = 0.;
    //    qTrunk0.x() = 0.;
    //    qTrunk0.y() = 0.;
    //    qTrunk0.z() = 0.;
    //    qRecalT.w() = 0.;
    //    qRecalT.x() = 0.;
    //    qRecalT.y() = 0.;
    //    qRecalT.z() = 0.;
    //    qIdealT.w() = 0.;
    //    qIdealT.x() = 0.;
    //    qIdealT.y() = 0.;
    //    qIdealT.z() = 0.;
    //    Rframe = Eigen::Matrix3d::Zero();
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
void LawJacobian::initialQuat(Eigen::Quaterniond qAcr, int initCounter, int initCounts)
{
    if (initCounter < initCounts) {
        //        qHip0.w() += qHip.w();
        //        qHip0.x() += qHip.x();
        //        qHip0.y() += qHip.y();
        //        qHip0.z() += qHip.z();
        //        qTrunk0.w() += qTrunk.w();
        //        qTrunk0.x() += qTrunk.x();
        //        qTrunk0.y() += qTrunk.y();
        //        qTrunk0.z() += qTrunk.z();
        //        qArm0.w() += qArm.w();
        //        qArm0.x() += qArm.x();
        //        qArm0.y() += qArm.y();
        //        qArm0.z() += qArm.z();
        qAcr0.w() += qAcr.w();
        qAcr0.x() += qAcr.x();
        qAcr0.y() += qAcr.y();
        qAcr0.z() += qAcr.z();
    }
    if (initCounter == initCounts) {
        //        qHip0.w() += qHip.w();
        //        qHip0.x() += qHip.x();
        //        qHip0.y() += qHip.y();
        //        qHip0.z() += qHip.z();
        //        qHip0.w() = qHip0.w() / initCounts;
        //        qHip0.x() = qHip0.x() / initCounts;
        //        qHip0.y() = qHip0.y() / initCounts;
        //        qHip0.z() = qHip0.z() / initCounts;
        //        qHip0 = qHip0.normalized();

        //        qTrunk0.w() += qTrunk.w();
        //        qTrunk0.x() += qTrunk.x();
        //        qTrunk0.y() += qTrunk.y();
        //        qTrunk0.z() += qTrunk.z();
        //        qTrunk0.w() = qTrunk0.w() / initCounts;
        //        qTrunk0.x() = qTrunk0.x() / initCounts;
        //        qTrunk0.y() = qTrunk0.y() / initCounts;
        //        qTrunk0.z() = qTrunk0.z() / initCounts;
        //        qTrunk0 = qTrunk0.normalized();

        //        Yinit = qTrunk0.inverse() * Y0 * qTrunk0;
        //        Ytrunk0 = Yinit.vec();
        //        Ytrunk0 = Ytrunk0.normalized();

        qAcr0.w() += qAcr.w();
        qAcr0.x() += qAcr.x();
        qAcr0.y() += qAcr.y();
        qAcr0.z() += qAcr.z();
        qAcr0.w() = qAcr0.w() / initCounts;
        qAcr0.x() = qAcr0.x() / initCounts;
        qAcr0.y() = qAcr0.y() / initCounts;
        qAcr0.z() = qAcr0.z() / initCounts;
        qAcr0 = qAcr0.normalized();
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
    qHand_relative = qHand.normalized() * qArm0.conjugate();
    /// For optitrack quaternion definition
    Rhand_rel = qHand_relative.toRotationMatrix();
}

/**
 * @brief LawOpti::rotationMatrices compute the rotation matrices of the hip and forearm frames with respect to the global frame
 * @param qHip quaternions of the hip IMU
 * @param qhand quaternions of the hand IMU
 * @param qTrunk quaternions of the trunk IMU
 */
void LawJacobian::rotationMatrices2(Eigen::Quaterniond qHandOpti, Eigen::Quaterniond qHandIMU, Eigen::Quaterniond qHipOpti, Eigen::Quaterniond qHipIMU, Eigen::Quaterniond qTrunk)
{
    ///  From quaternions to orientation
    //    Rhip = qIdealH.toRotationMatrix();
    //    Rtrunk = qIdealT.toRotationMatrix();

    RhipOpti = qHipOpti.toRotationMatrix();
    Rhip = qHipIMU.toRotationMatrix();
    Rtrunk = qTrunk.toRotationMatrix();
    Rhand = qHandIMU.toRotationMatrix();
    RhandOpti = qHandOpti.toRotationMatrix();
    //    qHand_relative = qHand.normalized() * qArm0.conjugate();
    /// For optitrack quaternion definition
    //    Rhand_rel = qHand_relative.toRotationMatrix();
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
    /// Project in hip frame
    /// Initial acromion position in hip frame
    if (initCounter == initCounts) {
        /// for OPTITRACK quaternions
        posA0inHipOpti = RhipOpti.transpose() * (posA0 - posHip0);
        /// for IMU quaternions
        posA0inHip = Rhip * (posA0 - posHip0);
    }

    /// Current acromion position in hip frame
    /// for OPTITRACK quaternions
    posAinHipOpti = RhipOpti.transpose() * (posA - posHip);
    //    if (initCounter % 50 == 0) {
    //        debug() << "PosA0inhipOpti: " << posA0inHipOpti(0) << "; " << posA0inHipOpti(1) << "; " << posA0inHipOpti(2);
    //        debug() << "PosAinhipOpti" << posAinHipOpti(0) << "; " << posAinHipOpti(1) << "; " << posAinHipOpti(2);
    //    }

    /// for IMU quaternions
    posAinHip = Rhip * (posA - posHip);
    //    if (initCounter % 50 == 0) {
    //        debug() << "PosA0inHip: " << posA0inHip(0) << "; " << posA0inHip(1) << "; " << posA0inHip(2);
    //        debug() << "PosAinHip" << posAinHip(0) << "; " << posAinHip(1) << "; " << posAinHip(2);
    //    }
}

/**
 * @brief LawJacobian::orientationInHand computes the orientation change around Z-axis of the hip-acromion vector in hand frame
 * @param posA acromion position
 * @param posHip hip position
 * @param initCounter
 * @param initCounts
 */
void LawJacobian::orientationInHand(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts)
{
    /// Compute hip-aromion vector in current hand frame
    /// Initial vector
    HA0 = (posA0 - posHip0);
    HA0 = HA0.normalized();

    /// Current vector
    HA = (posA - posHip).normalized();

    ///Compute quaternion of the rotation
    if (initCounter > initCounts) {
        Eigen::Vector3d crossP = R0 * RhandOpti.transpose() * (HA0.cross(HA));
        crossP = crossP.normalized();
        // projection of the rotation vector in hand frame
        /// for OPTITRACK quaternions
        alpha = acos(HA0.dot(HA));
        qHA.w() = cos(-alpha / 2);
        qHA.x() = sin(-alpha / 2) * crossP(0);
        qHA.y() = sin(-alpha / 2) * crossP(1);
        qHA.z() = sin(-alpha / 2) * crossP(2);

        RHA = qHA.toRotationMatrix();
        //        eulerAcr(0) = -atan2(RHA(0, 1), RHA(0, 0)); //rotation around Z-axis

        eulerHA(0) = -atan2(RHA(1, 2), RHA(2, 2)); //rotation around X-axis
        eulerHA(1) = atan(RHA(0, 2) / sqrt(1 - RHA(0, 2) * RHA(0, 2))); //rotation around Y-axis
        eulerHA(2) = -atan2(RHA(0, 1), RHA(0, 0)); //rotation around Z-axis
        //        if (initCounter % 50 == 0)
        //            debug() << "Rotation around Z0: " << eulerAcr(0);
    }
}
/**
 * @brief LawJacobian::orientationInWrist computes the orientation change around Z-axis of the hip-acromion vector in wrist frame
 * @param posA acromion position in global frame
 * @param posHip hip position in global frame
 * @param initCounter
 * @param initCounts
 */
void LawJacobian::orientationInWrist(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts)
{
    /// Compute hip-aromion vector in wrist frame
    /// Initial vector
    if (initCounter == initCounts) {
        /// for OPTITRACK quaternions
        HA0 = (posA0 - posHip0).normalized();
    }
    /// Current vector
    HA = (posA - posHip).normalized();

    ///Compute quaternion of the rotation
    if (initCounter > initCounts) {
        // in wrist frame
        Eigen::Vector3d crossPw = R01 * R0 * RhandOpti.transpose() * (HA0.cross(HA));
        crossPw = crossPw.normalized();
        /// for OPTITRACK quaternions
        alpha = acos(HA0.dot(HA));
        qHA.w() = cos(-alpha / 2);
        qHA.x() = sin(-alpha / 2) * crossPw(0);
        qHA.y() = sin(-alpha / 2) * crossPw(1);
        qHA.z() = sin(-alpha / 2) * crossPw(2);

        RHA = qHA.toRotationMatrix();
        thetaHA(0) = atan(RHA(0, 2) / sqrt(1 - RHA(0, 2) * RHA(0, 2))); //rotation around Y-axis
        thetaHA(1) = -atan2(RHA(0, 1), RHA(0, 0)); //rotation around Z-axis
        if (initCounter % 50 == 0) {
            //            debug() << "HA0w: " << HA0(0) << ";" << HA0(1) << ";" << HA0(2);
            //            debug() << "RHA wrist: " << RHA(0, 0) << RHA(0, 1);
            //            debug() << "Rotation around Z1: " << thetaHA(1);
        }
    }
}

void LawJacobian::eulerAcromion(Eigen::Quaterniond qHand, Eigen::Quaterniond qAcr, int initCounter)
{
    Eigen::Quaterniond q = qHand.conjugate() * qAcr * qAcr0.conjugate() * qHand;
    RAcr = R0.transpose() * q.toRotationMatrix() * R0;
    eulerAcr(0) = -atan2(RAcr(1, 2), RAcr(2, 2)); //rotation around X-axis
    eulerAcr(1) = atan(RAcr(0, 2) / sqrt(1 - RAcr(0, 2) * RAcr(0, 2))); //rotation around Y-axis
    eulerAcr(2) = -atan2(RAcr(0, 1), RAcr(0, 0)); //rotation around Z-axis

    if (initCounter % 50 == 0) {
        debug() << "eulerAcr: " << eulerAcr(0) * 180 / M_PI << "; " << eulerAcr(1) * 180 / M_PI << "; " << eulerAcr(2) * 180 / M_PI;
    }
}

/**
 * @brief LawJacobian::projectionInHipIMU compute acromion positions in the hip frame when using only IMU (no Optitrack)
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
 * @brief LawJacobian::updateTrunkFrame compute the current orientation of the trunk IMU frame
 * @param qTrunk quaternion of the trunk IMU
 */
void LawJacobian::updateTrunkFrame(Eigen::Quaterniond qTrunk)
{
    Yinit = qTrunk.inverse() * Y0 * qTrunk;
    Ytrunk = Yinit.vec();
    Ytrunk = Ytrunk.normalized();

    Xinit = qTrunk.inverse() * X0 * qTrunk;
    Xtrunk = Xinit.vec();
    Xtrunk = Xtrunk.normalized();

    Zinit = qTrunk.inverse() * Z0 * qTrunk;
    Ztrunk = Zinit.vec();
    Ztrunk = Ztrunk.normalized();
    //    debug() << "Ytrunk: " << Ytrunk(0) << " ; " << Ytrunk(1) << " ; " << Ytrunk(2);
    //    debug() << "Xtrunk: " << Xtrunk(0) << " ; " << Xtrunk(1) << " ; " << Xtrunk(2);
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

        if (i == 1) {
            R01 = Rframe;
        }
    }
}

/**
 * @brief LawJacobian::updateFramesinEE compute the current orientation of the links frames expressed in acromion (=end-effector of the arm model) frame
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
        //        x(0, i) = Rframe(0, 0) * x(0, i + 1) + Rframe(0, 1) * y(0, i + 1) + Rframe(0, 2) * z(0, i + 1);
        //        x(1, i) = Rframe(0, 0) * x(1, i + 1) + Rframe(0, 1) * y(1, i + 1) + Rframe(0, 2) * z(1, i + 1);
        //        x(2, i) = Rframe(0, 0) * x(2, i + 1) + Rframe(0, 1) * y(2, i + 1) + Rframe(0, 2) * z(2, i + 1);

        //        y(0, i) = Rframe(1, 0) * x(0, i + 1) + Rframe(1, 1) * y(0, i + 1) + Rframe(1, 2) * z(0, i + 1);
        //        y(1, i) = Rframe(1, 0) * x(1, i + 1) + Rframe(1, 1) * y(1, i + 1) + Rframe(1, 2) * z(1, i + 1);
        //        y(2, i) = Rframe(1, 0) * x(2, i + 1) + Rframe(1, 1) * y(2, i + 1) + Rframe(1, 2) * z(2, i + 1);

        //        z(0, i) = Rframe(2, 0) * x(0, i + 1) + Rframe(2, 1) * y(0, i + 1) + Rframe(2, 2) * z(0, i + 1);
        //        z(1, i) = Rframe(2, 0) * x(1, i + 1) + Rframe(2, 1) * y(1, i + 1) + Rframe(2, 2) * z(1, i + 1);
        //        z(2, i) = Rframe(2, 0) * x(2, i + 1) + Rframe(2, 1) * y(2, i + 1) + Rframe(2, 2) * z(2, i + 1);

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
 * @brief LawJacobian::computeOriginsVectors compute the vectors from the origins of the frames to the acromion position
 * @param l lengths of the arm segments
 * @param nbDOF number of DOF of the prosthetic arm
 */
void LawJacobian::computeOriginsVectors(int l[], int nbDOF)
{
    if (nbDOF == 3) {
        /// UPDATE POSITIONS OF CENTERS OF FRAMES
        OO(0, 0) = -l[0] * x(0, 0) - (l[1] + l[2]) * z(0, 1) - l[3] * y(0, 3);
        OO(1, 0) = -l[0] * x(1, 0) - (l[1] + l[2]) * z(1, 1) - l[3] * y(1, 3);
        OO(2, 0) = -l[0] * x(2, 0) - (l[1] + l[2]) * z(2, 1) - l[3] * y(2, 3);

        OO(0, 1) = -l[2] * z(0, 1) - l[3] * y(0, 3);
        OO(1, 1) = -l[2] * z(1, 1) - l[3] * y(1, 3);
        OO(2, 1) = -l[2] * z(2, 1) - l[3] * y(2, 3);

        OO(0, 2) = -l[3] * y(0, 3);
        OO(1, 2) = -l[3] * y(1, 3);
        OO(2, 2) = -l[3] * y(2, 3);

        //        debug() << "OO 04: " << OO(0, 0) << "; " << OO(1, 0) << "; " << OO(2, 0) << "\n";
        //        debug() << "OO 14: " << OO(0, 1) << "; " << OO(1, 1) << "; " << OO(2, 1) << "\n";
        //        debug() << "OO 24: " << OO(0, 2) << "; " << OO(1, 2) << "; " << OO(2, 2) << "\n";
    }
}

/**
 * @brief LawJacobian::scaleDisplacementHip Penalize trunk extension to avoid too many harmful compensations
 */
void LawJacobian::scaleDisplacementHip(int _cnt)
{
    ///// With OPTITRACK quaternions
    /// Acromion displacement in hip frame
    dispOpti = (posA0inHipOpti - posAinHipOpti);
    //    if (_cnt % 50 == 0) {
    //        debug() << "original disp opti: " << dispOpti(0) << "; " << dispOpti(1) << "; " << dispOpti(2);
    //    }

    /// Penalize trunk extension
    //    // ATTENTION : éventuellement changer la coordonnée qu'on modifie en fonction du placement du cluster hanche !!!!
    if (dispOpti(2) < 0) // backward motion (trunk extension)
        dispOpti(2) = 2 * dispOpti(2);
    if (_cnt % 50 == 0) {
        debug() << "rescaled disp opti: " << dispOpti(0) << "; " << dispOpti(1) << "; " << dispOpti(2);
    }

    //// With IMU quaternions
    /// Displacement of the acromion in hip frame, from current to initial position
    disp = (posA0inHip - posAinHip);
    //    if (_cnt % 50 == 0) {
    //        debug() << "original disp: " << disp(0) << "; " << disp(1) << "; " << disp(2);
    //    }

    /// Penalize trunk extension. Coordinate when z axis is perpendicular to the trunk and points backward.
    /// <0 because acromion displacement is from current to initial position
    if (disp(2) < 0) // backward motion (trunk extension)
        disp(2) = 2 * disp(2);
    //    if (_cnt % 50 == 0) {
    //        debug() << "scaled disp: " << disp(0) << "; " << disp(1) << "; " << disp(2);
    //    }
}

/**
 * @brief LawJacobian::controlLaw_v1 delta = acromion displacement in hip frame
 * @param k damping parameters
 * @param lambda gain
 * @param threshold
 * @param _cnt
 */
void LawJacobian::controlLaw_v1(int useIMU, double lambda[], double k, double threshold[], int _cnt)
{
    /// COMPUTE JACOBIAN
    for (int i = 0; i < nbLinks; i++) {
        J.block<3, 1>(0, i) = z.block<3, 1>(0, i).cross(OO.block<3, 1>(0, i));
    }

    /// PSEUDO INVERSE SOLUTION
    //    pinvJ = pseudoinverse<Eigen::MatrixXd>(J, 1e-3);

    /// DAMPED LEAST SQUARE SOLUTION
    dlsJ = (J.transpose() * J + k * k * I).inverse() * J.transpose();

    /// PROJECT DISPLACEMENT OF ACROMION IN HAND FRAME
    /// For IMU quaternion
    delta = R0 * Rhand * Rhip.transpose() * disp;

    /// For OPTITRACK quaternions
    deltaOpti = R0 * RhandOpti.transpose() * RhipOpti * dispOpti;
    /// for optitrack quaternion for hand frame, no projection in hip
    //    delta = R0 * RhandOpti * (posA0 - posA);

    /// COMPUTE ANG. VELOCITIES
    if (useIMU == 1)
        thetaNew = dlsJ * delta;
    else if (useIMU == 0)
        thetaNew = dlsJ * deltaOpti;

    /// DEADZONE
    for (int i = 0; i < nbLinks; i++) {
        if (abs(thetaNew(i)) < threshold[i])
            thetaNew(i) = 0;
        else if (thetaNew(i) >= threshold[i])
            thetaNew(i) = thetaNew(i) - threshold[i];
        else if (thetaNew(i) <= -threshold[i])
            thetaNew(i) = thetaNew(i) + threshold[i];
    }

    /// DISPLAY DATA
    if (_cnt % 50 == 0) {
        //        debug() << "delta: " << delta(0) << "; " << delta(1) << "; " << delta(2) << "\r\n";
        //        debug() << "deltaOpti: " << deltaOpti(0) << "; " << deltaOpti(1) << "; " << deltaOpti(2) << "\r\n";
        //        debug() << "pinvJ: " << pinvJ(0, 0) << "; " << pinvJ(0, 1) << "; " << pinvJ(0, 2) << "\r\n";
        debug() << "thetaNew(after threshold, deg): ";
        for (int i = 0; i < nbLinks; i++) {
            debug() << thetaNew(i) * 180 / M_PI;
        }
        //        debug() << "thetaNewOpti (after threshold, deg): ";
        //        for (int i = 0; i < nbLinks; i++) {
        //            debug() << thetaNewOpti(i) * 180 / M_PI;
        //        }
    }

    /// COMPUTE ANGULAR VELOCITIES
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
        // speed limits
        if (abs(thetaDot(i)) > 60 * M_PI / 180) {
            thetaDot(i) = thetaDot(i) / abs(thetaDot(i)) * 60 * M_PI / 180;
        }
    }
}

/**
 * @brief LawJacobian::controlLaw_orientation2DOF control law for the 2DOF prosthesis (pronosupination and elbow flex/ext)
 * the error signal is the rotation of the hip-acromion vector
 * @param lambda gain vector of the control
 * @param threshold
 * @param _cnt
 */
void LawJacobian::controlLaw_orientation2DOF(double lambda[], double threshold[], int _cnt)
{

    thetaNew(0) = thetaHA(0);
    thetaNew(1) = thetaHA(1);
    if (_cnt % 50 == 0) {
        debug() << "thetaNew with I2: " << thetaHA(0) << "; " << thetaHA(1);
    }

    /// DEADZONE
    for (int i = 0; i < nbLinks; i++) {
        if (abs(thetaNew(i)) < threshold[i])
            thetaNew(i) = 0;
        else if (thetaNew(i) >= threshold[i])
            thetaNew(i) = thetaNew(i) - threshold[i];
        else if (thetaNew(i) <= -threshold[i])
            thetaNew(i) = thetaNew(i) + threshold[i];
    }

    /// DISPLAY DATA
    if (_cnt % 50 == 0) {
        //        debug() << "delta: " << delta(0) << "; " << delta(1) << "; " << delta(2) << "\r\n";
        //        debug() << "deltaOpti: " << deltaOpti(0) << "; " << deltaOpti(1) << "; " << deltaOpti(2) << "\r\n";
        //        debug() << "pinvJ: " << pinvJ(0, 0) << "; " << pinvJ(0, 1) << "; " << pinvJ(0, 2) << "\r\n";
        //        debug() << "thetaNew(after threshold, deg): ";
        //        for (int i = 0; i < nbLinks; i++) {
        //            debug() << thetaNew(i) * 180 / M_PI;
        //        }
        //        debug() << "thetaNewOpti (after threshold, deg): ";
        //        for (int i = 0; i < nbLinks; i++) {
        //            debug() << thetaNewOpti(i) * 180 / M_PI;
        //        }
    }

    /// COMPUTE ANGULAR VELOCITIES
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
        // speed limits
        if (abs(thetaDot(i)) > 60 * M_PI / 180) {
            thetaDot(i) = thetaDot(i) / abs(thetaDot(i)) * 60 * M_PI / 180;
        }
    }
}

/**
 * @brief LawJacobian::controlLaw_orientation3DOF delta = Euler angles sequence of acromion, expressed in hand frame
 * @param lambda gain
 * @param threshold
 * @param _cnt
 */
void LawJacobian::controlLaw_orientation3DOF(double lambda[], double k, double threshold[], int _cnt)
{
    /// COMPUTE JACOBIAN
    for (int i = 0; i < nbLinks; i++) {
        J.block<3, 1>(0, i) = z.block<3, 1>(0, i);
    }
    /// DAMPED LEAST SQUARE SOLUTION
    dlsJ = (J.transpose() * J + k * k * I).inverse() * J.transpose();

    /// COMPUTE ANG. VELOCITIES
    thetaNew = dlsJ * eulerAcr;

    /// DEADZONE
    for (int i = 0; i < nbLinks; i++) {
        if (abs(thetaNew(i)) < threshold[i])
            thetaNew(i) = 0;
        else if (thetaNew(i) >= threshold[i])
            thetaNew(i) = thetaNew(i) - threshold[i];
        else if (thetaNew(i) <= -threshold[i])
            thetaNew(i) = thetaNew(i) + threshold[i];
    }

    /// DISPLAY DATA
    if (_cnt % 50 == 0) {
        //        debug() << "delta: " << delta(0) << "; " << delta(1) << "; " << delta(2) << "\r\n";
        //        debug() << "deltaOpti: " << deltaOpti(0) << "; " << deltaOpti(1) << "; " << deltaOpti(2) << "\r\n";
        //        debug() << "pinvJ: " << pinvJ(0, 0) << "; " << pinvJ(0, 1) << "; " << pinvJ(0, 2) << "\r\n";
        debug() << "thetaNew(after threshold, deg): ";
        for (int i = 0; i < nbLinks; i++) {
            debug() << thetaNew(i) * 180 / M_PI;
        }
        //        debug() << "thetaNewOpti (after threshold, deg): ";
        //        for (int i = 0; i < nbLinks; i++) {
        //            debug() << thetaNewOpti(i) * 180 / M_PI;
        //        }
    }

    /// COMPUTE ANGULAR VELOCITIES
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
        // speed limits
        if (abs(thetaDot(i)) > 60 * M_PI / 180) {
            thetaDot(i) = thetaDot(i) / abs(thetaDot(i)) * 60 * M_PI / 180;
        }
    }
}

/**
 * @brief LawJacobian::scaleDisplacementIMU multiply trunk extension to avoid too many harmful compensations
 * @param lt
 */
void LawJacobian::scaleDisplacementIMU(int lt, int _cnt)
{
    if ((Ytrunk0 - Ytrunk).dot(Ztrunk) > 0)
        scale = 2;
    else if ((Ytrunk0 - Ytrunk).dot(Ztrunk) <= 0)
        scale = 1;

    disp = lt * ((Ytrunk0 - Ytrunk).dot(Xtrunk) * Xtrunk + (Ytrunk0 - Ytrunk).dot(Ytrunk) * Ytrunk + scale * (Ytrunk0 - Ytrunk).dot(Ztrunk) * Ztrunk);

    if (_cnt % 50 == 0) {
        debug() << "original disp: " << lt * (Ytrunk0 - Ytrunk)(0) << "; " << lt * (Ytrunk0(1) - Ytrunk(1)) << "; " << lt * (Ytrunk0(2) - Ytrunk(2));
        debug() << "scaled disp: " << disp(0) << "; " << disp(1) << "; " << disp(2);
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
            d[3 * nbLinks + j + 3 * i] = J(j, i);
        }
    }

    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + 3 + j] = dispOpti(j);
    }
    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + 6 + j] = deltaOpti(j);
    }
    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + 9 + j] = eulerAcr(j);
    }
    for (int j = 0; j < 3; j++) {
        d[6 * nbLinks + 12 + j] = posAinHipOpti(j);
    }
}

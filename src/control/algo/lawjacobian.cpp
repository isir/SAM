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

void LawJacobian::initialization(Eigen::Vector3d posA, Eigen::Quaterniond qHip, unsigned int freq)
{
    /// POSITIONS AND QUATERNIONS
    posA0 = Eigen::Vector3d::Zero();
    posAinHip = posA;
    posA0inHip = posA;
    posAinTrunk = Eigen::Vector3d::Zero();
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
    xref << 1., 0., 0.;
    yref << 0., 1., 0.;
    samplePeriod = 1. / freq;
    coeff = samplePeriod / (0.03 + samplePeriod);
    delta[0] = 0;
    delta[1] = 0;
    delta[2] = 0;
    Rhip = Eigen::Matrix3d::Zero();
    Rhand = Eigen::Matrix3d::Zero();
    Rframe = Eigen::Matrix3d::Zero();
    thetaNew = Eigen::MatrixXd::Zero(nbLinks, 1);
    thetaDot = Eigen::MatrixXd::Zero(nbLinks, 1);
    J = Eigen::MatrixXd::Zero(3, nbLinks);
    pinvJ = Eigen::MatrixXd::Zero(nbLinks, 3);
    OO = Eigen::MatrixXd::Zero(3, nbLinks);

    // Rotation matrix from IMU or hand rigid body frame to theoretical arm frame
    R0 << 0., 1., 0.,
        0., 0., 1.,
        1., 0., 0.;
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
void LawJacobian::initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk, int initCounter, int initCounts)
{
    if (initCounter < initCounts) {
        posA0 += posA;
        posHip0 += posHip;
        qHip0.w() += qHip.w();
        qHip0.x() += qHip.x();
        qHip0.y() += qHip.y();
        qHip0.z() += qHip.z();
        qTrunk0.w() += qTrunk.w();
        qTrunk0.x() += qTrunk.x();
        qTrunk0.y() += qTrunk.y();
        qTrunk0.z() += qTrunk.z();
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

        qTrunk0.w() += qTrunk.w();
        qTrunk0.x() += qTrunk.x();
        qTrunk0.y() += qTrunk.y();
        qTrunk0.z() += qTrunk.z();
        qTrunk0.w() = qTrunk0.w() / initCounts;
        qTrunk0.x() = qTrunk0.x() / initCounts;
        qTrunk0.y() = qTrunk0.y() / initCounts;
        qTrunk0.z() = qTrunk0.z() / initCounts;
        qTrunk0 = qTrunk0.normalized();
    }
}

/**
 * @brief LawOpti::rotationMatrices compute the rotation matrices of the hip and forearm frames with respect to the global frame
 * @param qHip quaternions of the hip cluster
 * @param qFA_record quaternions of the forearm cluster
 */
void LawJacobian::rotationMatrices(Eigen::Quaterniond qHand, Eigen::Quaterniond qHip, Eigen::Quaterniond qTrunk, int initCounter, int initCounts)
{
    //    qHip_filt.w() = qHip_filt_old.w() + coeff * (qHip.w() - qHip_filt_old.w());
    //    qHip_filt.x() = qHip_filt_old.x() + coeff * (qHip.x() - qHip_filt_old.x());
    //    qHip_filt.y() = qHip_filt_old.y() + coeff * (qHip.y() - qHip_filt_old.y());
    //    qHip_filt.z() = qHip_filt_old.z() + coeff * (qHip.z() - qHip_filt_old.z());

    ///  From quaternions to orientation
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
    posAinTrunk = lt * yref + lsh * xref;
    // project in hip frame
    if (initCounter == initCounts) {
        // for imu quaternions
        posA0inHip = Rhip * Rtrunk.transpose() * posAinTrunk;
    }
    // for imu quaternions
    posAinHip = Rhip * Rtrunk.transpose() * posAinTrunk;

    if (initCounter % 50 == 0) {
        //        debug() << "posA in hip: " << posAinHip[0] << ", " << posAinHip[1] << ", " << posAinHip[2];
    }
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

void LawJacobian::controlLaw(Eigen::Vector3d posA, int lambda[], double threshold[], int _cnt)
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

    pinvJ = pseudoinverse<Eigen::MatrixXd>(J, 1e-3);
    /// COMPUTE ANG. VELOCITIES
    thetaNew = pinvJ * delta;
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
        debug() << "thetaNew(after threshold): ";
        for (int i = 0; i < nbLinks; i++) {
            debug() << thetaNew(i) * 180 / M_PI;
        }
    }
    // Angular velocities
    for (int i = 0; i < nbLinks; i++) {
        thetaDot(i) = lambda[i] * thetaNew(i);
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

    //    for (int i = 0; i < nbLinks; i++) {
    //        for (int j = 0; j < 3; j++) {
    //            d[6 * nbLinks + 3 + j + 3 * i] = J(j, i);
    //        }
    //    }
    d[6 * nbLinks + 6 + 3 * nbLinks] = posAinHip[0];
    d[6 * nbLinks + 6 + 3 * nbLinks + 1] = posAinHip[1];
    d[6 * nbLinks + 6 + 3 * nbLinks + 2] = posAinHip[2];
}

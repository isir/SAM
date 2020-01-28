#include "lawimu_we.h"
#include "math.h"
#include <eigen3/Eigen/Geometry>

LawIMU_WE::LawIMU_WE()
{
}

LawIMU_WE::~LawIMU_WE()
{
}

/**
 * @brief LawIMU_WE::initialization set all used factor to their initial values or to zero
 * @param Lt Length of the trunk (from hip to shoulder)
 * @param Lua Length of the upper arm (from shoulder to elbow)
 * @param Lfa Length of the forearm (from elbow to hand)
 * @param lsh Length between the two shoulder
 */
void LawIMU_WE::initialization(int Lt, int Lua, int Lfa, int lsh)
{
    //Initial quaternion from trunk IMU in calibration frame
    qTronc0.w() = 0.;
    qTronc0.x() = 0.;
    qTronc0.y() = 0.;
    qTronc0.z() = 0.;

    // calibration frame quaternions and vectors
    X0.w() = 0.0;
    X0.x() = 1.0;
    X0.y() = 0.0;
    X0.z() = 0.0;
    Y0.w() = 0.0;
    Y0.x() = 0.0;
    Y0.y() = 1.0;
    Y0.z() = 0.0;
    Z0.w() = 0.0;
    Z0.x() = 0.0;
    Z0.y() = 0.0;
    Z0.z() = 1.0;
    X0_vect[0] = 1.0;
    X0_vect[1] = 0.0;
    X0_vect[2] = 0.0;
    Y0_vect[0] = 0.0;
    Y0_vect[1] = 1.0;
    Y0_vect[2] = 0.0;
    Z0_vect[0] = 0.0;
    Z0_vect[1] = 0.0;
    Z0_vect[2] = 1.0;

    // initial trunk and arm vectors and quaternions
    Xinit.w() = 0.0;
    Xinit.x() = 0.0;
    Xinit.y() = 0.0;
    Xinit.z() = 0.0;
    Yinit.w() = 0.0;
    Yinit.x() = 0.0;
    Yinit.y() = 0.0;
    Yinit.z() = 0.0;
    Zinit.w() = 0.0;
    Zinit.x() = 0.0;
    Zinit.y() = 0.0;
    Zinit.z() = 0.0;

    Xtronc_init_vect[0] = 0.0;
    Xtronc_init_vect[1] = 0.0;
    Xtronc_init_vect[2] = 0.0;
    Ytronc_init_vect[0] = 0.0;
    Ytronc_init_vect[1] = 0.0;
    Ytronc_init_vect[2] = 0.0;
    Ztronc_init_vect[0] = 0.0;
    Ztronc_init_vect[1] = 0.0;
    Ztronc_init_vect[2] = 0.0;
    Xbras_init_vect[0] = 0.0;
    Xbras_init_vect[1] = 0.0;
    Xbras_init_vect[2] = 0.0;
    Ybras_init_vect[0] = 0.0;
    Ybras_init_vect[1] = 0.0;
    Ybras_init_vect[2] = 0.0;
    Zbras_init_vect[0] = 0.0;
    Zbras_init_vect[1] = 0.0;
    Zbras_init_vect[2] = 0.0;

    // trunk and arm current vectors and quaternions
    XQuat.w() = 0.0;
    XQuat.x() = 0.0;
    XQuat.y() = 0.0;
    XQuat.z() = 0.0;
    YQuat.w() = 0.0;
    YQuat.x() = 0.0;
    YQuat.y() = 0.0;
    YQuat.z() = 0.0;
    ZQuat.w() = 0.0;
    ZQuat.x() = 0.0;
    ZQuat.y() = 0.0;
    ZQuat.z() = 0.0;
    Xbras_long.w() = 0.0;
    Xbras_long.x() = 0.0;
    Xbras_long.y() = 0.0;
    Xbras_long.z() = 0.0;
    Ybras_long.w() = 0.0;
    Ybras_long.x() = 0.0;
    Ybras_long.y() = 0.0;
    Ybras_long.z() = 0.0;
    Zbras_long.w() = 0.0;
    Zbras_long.x() = 0.0;
    Zbras_long.y() = 0.0;
    Zbras_long.z() = 0.0;
    Xtronc[0] = 0.0;
    Xtronc[1] = 0.0;
    Xtronc[2] = 0.0;
    Ytronc[0] = 0.0;
    Ytronc[1] = 0.0;
    Ytronc[2] = 0.0;
    Ztronc[0] = 0.0;
    Ztronc[1] = 0.0;
    Ztronc[2] = 0.0;
    Xbras[0] = 0.0;
    Xbras[1] = 0.0;
    Xbras[2] = 0.0;
    Ybras[0] = 0.0;
    Ybras[1] = 0.0;
    Ybras[2] = 0.0;
    Zbras[0] = 0.0;
    Zbras[1] = 0.0;
    Zbras[2] = 0.0;

    // reference arm frame
    Xbras_ref[0] = 0.0;
    Xbras_ref[1] = 0.0;
    Xbras_ref[2] = 0.0;
    Ybras_ref[0] = 0.0;
    Ybras_ref[1] = 0.0;
    Ybras_ref[2] = 0.0;
    Zbras_ref[0] = 0.0;
    Zbras_ref[1] = 0.0;
    Zbras_ref[2] = 0.0;

    // quaternions to move to ideal frame
    qinit.w() = 0.;
    qinit.x() = 0.;
    qinit.y() = 0.;
    qinit.z() = 0.;
    qRecalT.w() = 0.;
    qRecalT.x() = 0.;
    qRecalT.y() = 0.;
    qRecalT.z() = 0.;
    qRecalA.w() = 0.;
    qRecalA.x() = 0.;
    qRecalA.y() = 0.;
    qRecalA.z() = 0.;
    qInit_recal.w() = 0.;
    qInit_recal.x() = 0.;
    qInit_recal.y() = 0.;
    qInit_recal.z() = 0.;

    crossP(0) = 0.;
    crossP(1) = 0.;
    crossP(2) = 0.;

    // shoulder reference position
    Xsh_ref = lsh;
    Ysh_ref = Lt;
    Zsh_ref = 0.;

    // shoulder-hand distance
    delta = 0.;

    // initial end-effector coordinates in body frame
    Xee_init = lsh;
    Yee_init = Lt - Lua;
    Zee_init = Lfa;

    // end-effector coordinates in body frame
    Xee = 0.;
    Yee = 0.;
    Zee = 0.;

    //elbow data
    beta_new = 0.;
    dBeta = 0.;
    betaDot = 0.;

    // Wrist rotation ang.vel
    wristVel = 0.;
}

/**
 * @brief LawIMU_WE::recordInitialPosition Compute initial quaternions : sum up the values to then make an average (see compute_initialPosition)
 * @param qTronc  trunk IMU quaternion
 * @param qBras  arm IMU quaternion
 */
void LawIMU_WE::recordInitialPosition(Eigen::Quaterniond qTronc, Eigen::Quaterniond qBras, int initCounter, int initCounts)
{
    if (initCounter < initCounts) {
        qTronc0.w() += qTronc.w();
        qTronc0.x() += qTronc.x();
        qTronc0.y() += qTronc.y();
        qTronc0.z() += qTronc.z();

        qBras0.w() += qBras.w();
        qBras0.x() += qBras.x();
        qBras0.y() += qBras.y();
        qBras0.z() += qBras.z();
    }

    if (initCounter == initCounts) {
        qTronc0.w() += qTronc.w();
        qTronc0.x() += qTronc.x();
        qTronc0.y() += qTronc.y();
        qTronc0.z() += qTronc.z();

        qBras0.w() += qBras.w();
        qBras0.x() += qBras.x();
        qBras0.y() += qBras.y();
        qBras0.z() += qBras.z();

        qTronc0.w() = qTronc0.w() / initCounts;
        qTronc0.x() = qTronc0.x() / initCounts;
        qTronc0.y() = qTronc0.y() / initCounts;
        qTronc0.z() = qTronc0.z() / initCounts;

        qBras0.w() = qBras0.w() / initCounts;
        qBras0.x() = qBras0.x() / initCounts;
        qBras0.y() = qBras0.y() / initCounts;
        qBras0.z() = qBras0.z() / initCounts;

        // Norm quaternion
        qTronc0 = qTronc0.normalized();

        qBras0 = qBras0.normalized();

        // compute initial trunk and arm vertical axes
        Yinit = qTronc0.inverse() * Y0 * qTronc0;
        Ytronc_init_vect = Yinit.vec();
        Ytronc_init_vect = Ytronc_init_vect.normalized();

        qinit = qBras0.inverse() * X0 * qBras0;
        Xbras_init_vect = qinit.vec();
        Xbras_init_vect = Xbras_init_vect.normalized();
    }
}

/**
 * @brief LawIMU_WE::moveToIdealFrame compute quaternion of the rotation of the vertical axis of the IMUs frame to the vertical axis of the ground frame.
 * Compute the initial ideal trunk and arm vectors.
 */
void LawIMU_WE::moveToIdealFrames(Eigen::Quaterniond qTrunk, int initCounter, int initCounts)
{
    if (initCounter == initCounts) {
        // rotate the trunk frame so that the vertical axis of the IMU frame (Y) is the same as the vertical axis of the ground frame.
        crossP = Ytronc_init_vect.cross(Z0_vect);
        crossP = crossP.normalized();
        theta0 = acos(Ytronc_init_vect.dot(Z0_vect));
        qRecalT.w() = cos(-theta / 2);
        qRecalT.x() = sin(-theta / 2) * crossP(0);
        qRecalT.y() = sin(-theta / 2) * crossP(1);
        qRecalT.z() = sin(-theta / 2) * crossP(2);

        // rotate the arm frame so that the vertical axis of the IMU frame (X) is the same as the vertical axis of the ground frame.
        crossP = Xbras_init_vect.cross(Z0_vect);
        crossP = crossP.normalized();
        theta0 = acos(Xbras_init_vect.dot(Z0_vect));
        qRecalA.w() = cos(-theta / 2);
        qRecalA.x() = sin(-theta / 2) * crossP[0];
        qRecalA.y() = sin(-theta / 2) * crossP[1];
        qRecalA.z() = sin(-theta / 2) * crossP[2];

        qIdealT = qTrunk * qRecalT;
        qIdealT = qIdealT.normalized();
    } else if (initCounter > initCounts) {

        qIdealT = qTrunk * qRecalT;
        qIdealT = qIdealT.normalized();
    }
}

/**
 * @brief Law3D::computeAxis compute current trunk and arm axis, in the calibration frame.
 * @param qTronc trunk quaternion from IMU
 * @param qBras arm quaternion, from IMU
 */
void LawIMU_WE::computeAxis(Eigen::Quaterniond qTronc, Eigen::Quaterniond qBras)
{
    XQuat = qIdealT.inverse() * X0 * qIdealT;
    Xtronc = XQuat.vec();
    Xtronc = Xtronc.normalized();

    YQuat = qIdealT.inverse() * Y0 * qIdealT;
    Ytronc = YQuat.vec();
    Ytronc = Ytronc.normalized();

    ZQuat = qIdealT.inverse() * Z0 * qIdealT;
    Ztronc = ZQuat.vec();
    Ztronc = Ztronc.normalized();

    Xbras_long = qBras.inverse() * X0 * qBras;
    Xbras = Xbras_long.vec();
    Xbras = Xbras.normalized();

    Ybras_long = qBras.inverse() * Y0 * qBras;
    Ybras = Ybras_long.vec();
    Ybras = Ybras.normalized();

    Zbras_long = qBras.inverse() * Z0 * qBras;
    Zbras = Zbras_long.vec();
    Zbras = Zbras.normalized();
}

/**
 * @brief LawIMU_WE::new_coordEE Compute the new cartesian coordinates of the hand which has moved because of body compensations
 * @param Lt Length of the trunk
 * @param Lua Length of the upper ar
 * @param Lfa Length of the forearm
 * @param lsh Length between the two shoulders
 * @param beta Elbow flexion/extension angle
 */
void LawIMU_WE::new_coordEE(int Lt, int Lua, int Lfa, int lsh, double beta)
{
    // compute the new coordinates of the end-effector (hand) after trunk and scapula motions
    //    printf("(Yt,X0) : %f, (Xt,X0) :%f, (Zt,X0) : %f \n \n", dotProduct(Ytronc,X0_vect,3), dotProduct(Xtronc,X0_vect,3), dotProduct(Ztronc,X0_vect,3));

    Xee = (Lt)*Ytronc.dot(Xtronc_init_vect) + lsh * (Xtronc.dot(Xtronc_init_vect))
        - Lua * Xbras.dot(Xtronc_init_vect) + Lfa * (-cos(-beta) * Xbras.dot(Xtronc_init_vect) - sin(-beta) * Ybras.dot(Xtronc_init_vect));
    Yee = (Lt)*Ytronc.dot(Ytronc_init_vect) + lsh * (Xtronc.dot(Ytronc_init_vect))
        - Lua * Xbras.dot(Ytronc_init_vect) + Lfa * (-cos(-beta) * Xbras.dot(Ytronc_init_vect) - sin(-beta) * Ybras.dot(Ytronc_init_vect));
    Zee = (Lt)*Ytronc.dot(Ztronc_init_vect) + lsh * (Xtronc.dot(Ztronc_init_vect))
        - Lua * Xbras.dot(Ztronc_init_vect) + Lfa * (-cos(-beta) * Xbras.dot(Ztronc_init_vect) - sin(-beta) * Ybras.dot(Ztronc_init_vect));
}

/**
 * @brief LawIMU_WE::computeBetaDot Compute the flexion/extension angular velocity of the elbow, based on the desired position of the hand, when there is only the elbow to control
 * @param Lua Length of the upper arm
 * @param Lfa Length of the forearm
 * @param lambda gain of the integrator
 * @param beta Elbow flexion/extension angle
 */
void LawIMU_WE::computeBetaDot(int Lua, int Lfa, double lambda, double threshold, double beta)
{
    dBeta = 0.;

    //    //Method 1 : keep direction
    //        beta_new = -atan2(-YeeR2,-XeeR2);

    // Method 2 : keep position if subject correclty adapts its shoulder position
    // Compute shoulder-hand distance
    delta = sqrt((Xee - Xsh_ref) * (Xee - Xsh_ref) + (Yee - Ysh_ref) * (Yee - Ysh_ref) + (Zee - Zsh_ref) * (Zee - Zsh_ref));
    // limit conditions
    if (delta > (Lua + Lfa))
        delta = Lua + Lfa;
    if (delta < abs(Lfa - Lua))
        delta = abs(Lfa - Lua);

    // compute the unique angle that allows to form the triangle shoulder-elbow-hand
    beta_new = -(M_PI - acos((Lfa * Lfa + Lua * Lua - delta * delta) / (2 * Lua * Lfa)));

    // Threshold before activation of the prosthetic elbow
    if (abs(beta_new - beta) < threshold) {
        dBeta = 0;
    } else if ((abs(beta_new - beta) >= threshold) && beta_new - beta < 0) {
        dBeta = beta_new - beta + threshold;
    } else if ((abs(beta_new - beta) >= threshold) && beta_new - beta > 0) {
        dBeta = beta_new - beta - threshold;
    }
    betaDot = lambda * dBeta;
    if (abs(betaDot) > 50 * M_PI / 180) {
        betaDot = betaDot / abs(betaDot) * 50 * M_PI / 180;
    }
}

void LawIMU_WE::rotationMatrices(Eigen::Quaterniond qBras)
{
    qBras_relative = qBras.normalized() * qBras0.conjugate();
    /// For optitrack quaternion definition
    R_Bras = qBras_relative.toRotationMatrix();
    //    ///  From quaternions to orientation
}

/**
 * @brief LawIMU_WE::computeWristVel Compute wrist angular velocity, based on trunk bending
 */
void LawIMU_WE::computeWristVel(double lambdaW, double thresholdW)
{

    phi = atan2(R_Bras(1, 2), R_Bras(2, 2)); //rotation around X-axis
    //    debug() << "wrist angle: " << wristAngle_new * M_PI / 180 << "\n";
    wristAngle_new = -atan(R_Bras(0, 2) / sqrt(1 - R_Bras(0, 2) * R_Bras(0, 2))); //rotation around Y-axis
    psi = atan2(R_Bras(0, 1), R_Bras(0, 0)); //rotation around Z-axis
    if (abs(wristAngle_new) < thresholdW)
        wristVel = 0.;
    else if (wristAngle_new <= -thresholdW) {
        wristVel = lambdaW * (wristAngle_new + thresholdW);
    } else if (wristAngle_new >= thresholdW) {
        wristVel = lambdaW * (wristAngle_new - thresholdW);
    }
}

/**
 * @brief LawIMU_WE::display Display some data on the command window
 * @param counter
 * @param lambda gain of the integrator, defined trhough the main code
 */
void LawIMU_WE::display(int counter)
{
    if (counter % 50 == 0) {
        printf("betaDot:%lf\n", betaDot);
        printf("wrist velocity: %lf\n", wristVel);
        //printf("phi : %f, theta : %f, psi : %f \n dOx : %f, dOz : %f\n  lambda : %lf\n  dBeta : %f\n \n", phi_filt, theta_filt, psi_filt, deltaOx_filt, deltaOz_filt, lambda, dBeta*180/PI);
        //printf("Xee :%f, Yee : %f, Zee : %f\n distance hand : %f\n beta new : %f\n", Xee, Yee, Zee, sqrt((Xee-Xee_init)*(Xee-Xee_init) + (Yee-Yee_init)*(Yee-Yee_init) + (Zee-Zee_init)*(Zee-Zee_init)), beta_new*180/PI);
    }
}

void LawIMU_WE::writeDebugData(double debug[], double beta)
{
    debug[9] = beta_new;
    //    debug[19] = R11A;
    //    debug[20] = R12A;
    //    debug[21] = R13A;
    //    debug[22] = R21A;
    //    debug[23] = R22A;
    //    debug[24] = R23A;
    //    debug[25] = R31A;
    //    debug[26] = R32A;
    //    debug[27] = R33A;
    debug[17] = beta;
    debug[18] = dBeta;
    debug[19] = betaDot;
    debug[20] = Xee;
    debug[21] = Yee;
    debug[22] = Zee;

    debug[33] = Xbras[0];
    debug[34] = Xbras[1];
    debug[35] = Xbras[2];
    debug[36] = Ybras[0];
    debug[37] = Ybras[1];
    debug[38] = Ybras[2];
    debug[39] = Zbras[0];
    debug[40] = Zbras[1];
    debug[41] = Zbras[2];
}

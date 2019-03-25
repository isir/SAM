#include "lawopti.h"
#include <QDebug>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Geometry>

LawOpti::LawOpti()
{
}

LawOpti::~LawOpti()
{
}

void LawOpti::initialization(Eigen::Vector3d posA, Eigen::Vector3d posEE, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, unsigned int freq){
    posA0 = Eigen::Vector3d::Zero();
    posAinHip = posA;

    posEEinHip = posEE;

    posA_filt = posA;
    posA_filt_old = posA;

    posEE_filt = posEE;
    posEE_filt_old = posEE;

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

    qHip_relative.w() = 0.;
    qHip_relative.x() = 0.;
    qHip_relative.y() = 0.;
    qHip_relative.z() = 0.;

    qFA0.w() = 0.;
    qFA0.x() = 0.;
    qFA0.y() = 0.;
    qFA0.z() = 0.;

    qFA_relative.w() = 0.;
    qFA_relative.x() = 0.;
    qFA_relative.y() = 0.;
    qFA_relative.z() = 0.;

    samplePeriod = 1./freq;
    coeff = samplePeriod/(0.03 + samplePeriod);

    delta = 0;
    beta_new = -M_PI_2;
    dBeta = 0;
    betaDot = 0;

    R = Eigen::Matrix3d::Zero();
}

/**
 * @brief LawOpti::initialPositions computes the initial position of the acromion marker = mean over the initCounts first measures of the acromion position
 *  and the initial position + orientation of the hip cluster = mean over the initCounts first measures
 * @param posA cartesian coordinates of the acromion marker (from optitrack)
 * @param posHip cartesian coordinates of the hip marker (from optitrack)
 * @param qHip quaternions of the hip marker (from optitrack)
 * @param initCounter counter of time steps
 * @param initCounts number of time step to take into account to compute the initial position
 */
void LawOpti::initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, Eigen::Quaterniond qFA_record, int initCounter, int initCounts){
    if (initCounter<initCounts){
        posA0 += posA;
        posHip0 +=posHip;
        qHip0.w() += qHip.w();
        qHip0.x() += qHip.x();
        qHip0.y() += qHip.y();
        qHip0.z() += qHip.z();
        qFA0.w() += qFA_record.w();
        qFA0.x() += qFA_record.x();
        qFA0.y() += qFA_record.y();
        qFA0.z() += qFA_record.z();

    }

    if (initCounter == initCounts){
        posA0 += posA;
        posA0 = posA0/initCounts;

        posHip0 += posHip;
        posHip0 = posHip0/initCounts;

        qHip0.w() += qHip.w();
        qHip0.x() += qHip.x();
        qHip0.y() += qHip.y();
        qHip0.z() += qHip.z();

        qHip0.w() = qHip0.w()/initCounts;
        qHip0.x() = qHip0.x()/initCounts;
        qHip0.y() = qHip0.y()/initCounts;
        qHip0.z() = qHip0.z()/initCounts;

        qHip0 = qHip0.normalized();

        qFA0.w() += qFA_record.w();
        qFA0.x() += qFA_record.x();
        qFA0.y() += qFA_record.y();
        qFA0.z() += qFA_record.z();

        qFA0.w() = qFA0.w()/initCounts;
        qFA0.x() = qFA0.x()/initCounts;
        qFA0.y() = qFA0.y()/initCounts;
        qFA0.z() = qFA0.z()/initCounts;

        qFA0 = qFA0.normalized();
    }
}

void LawOpti::rotationMatrices(Eigen::Quaterniond qHip, Eigen::Quaterniond qFA_record){
    qHip_filt.w() = qHip_filt_old.w() + coeff*(qHip.w()  - qHip_filt_old.w());
    qHip_filt.x() = qHip_filt_old.x() + coeff*(qHip.x()  - qHip_filt_old.x());
    qHip_filt.y() = qHip_filt_old.y() + coeff*(qHip.y()  - qHip_filt_old.y());
    qHip_filt.z() = qHip_filt_old.z() + coeff*(qHip.z()  - qHip_filt_old.z());

//    qHip_filt = qHip_filt.normalized();
//    qHip_relative = qHip_filt*qHip0.conjugate(); // for IMU quaternion definition
    qHip_relative = qHip_filt.normalized().conjugate()*qHip0;
    qHip_relative = qHip_relative.normalized();
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
    R = qHip_relative.toRotationMatrix();
    //ea = R.eulerAngles(2,1,0).cast<double>();

    qFA = qFA_record;
    qFA_relative = qFA.normalized().conjugate()*qFA0;
    /// For optitrack quaternion definition
    R_FA = qFA_relative.toRotationMatrix();

//    ///  From quaternions to orientation


}

void LawOpti::initialAcromionPositionInHip(){
    // project initial position of acromion in hip frame
    posA0inHip = R*(posA0-posHip0);
}

void LawOpti::projectionInHip(Eigen::Vector3d posA, Eigen::Vector3d posEE, Eigen::Vector3d posHip){

    // project in hip frame
    posAinHip  = R*(posA-posHip);
    posEEinHip = R*(posEE-posHip);

}

void LawOpti::filter_optitrackData(Eigen::Vector3d posA, Eigen::Vector3d posEE){
    posA_filt = posA_filt_old + coeff*(posA - posA_filt_old);
    posEE_filt = posEE_filt_old + coeff*(posEE - posEE_filt_old);
}

void LawOpti::bufferingOldValues(){
    posA_filt_old = posA_filt;
    posEE_filt_old = posEE_filt;
    qHip_filt_old = qHip_filt;
}

/**
 * @brief LawOpti::controlLaw
 * @param posEE cartesian coordinates of the end-effector (from optitrack)
 * @param beta elbow angle
 * @param Lua length of the upper-arm
 * @param Lfa length of the forearm
 */
void LawOpti::controlLaw(Eigen::Vector3d posEE, double beta, double Lua, double Lfa, int lambda, double threshold){
    // delta = distance between the initial acromion position (considered as the reference position) and the actual end-effector position
//    delta = (posEE-posA0).norm();
    delta = (posEEinHip-posA0inHip).norm();

    // limit conditions
    if (delta>(Lua+Lfa))
        delta = Lua+Lfa;
    if (delta < abs(Lfa-Lua))
        delta = abs(Lfa-Lua);

    beta_new = -(M_PI-acos((Lfa*Lfa + Lua*Lua-delta*delta)/(2*Lua*Lfa)));

    // Threshold before activation of the prosthetic elbow
    if (abs(beta_new-beta)<threshold){
        dBeta = 0;
    }
    else if ((abs(beta_new-beta)>=threshold) && beta_new-beta<0){
        dBeta = beta_new-beta + threshold;
    }
    else if ((abs(beta_new-beta)>=threshold) && beta_new-beta>0){
        dBeta = beta_new-beta - threshold;
    }

    betaDot = lambda*dBeta;
}

/**
 * @brief LawOpti::controlLawWrist control law for prono-supination
 * @param lambdaW gain of the integrator for wrist
 * @param thresholdW threshold of activation in rad
 */
void LawOpti::controlLawWrist(int lambdaW, double thresholdW){

    phi = atan2(R_FA(1,2), R_FA(2,2));  //rotation around X-axis
    theta = -atan(R_FA(0,2)/sqrt(1-R_FA(0,2)*R_FA(0,2)));   //rotation around Y-axis
    wristAngle_new = atan2(R_FA(0,1), R_FA(0,0)); //rotation around Z-axis

    if (abs(wristAngle_new)<thresholdW)
        wristVel = 0.;
    else if (wristAngle_new<-thresholdW){
        //        wristVel = lambdaW*sign*(wristAngle_new-thresholdW);
        wristVel = lambdaW*(wristAngle_new+thresholdW);}
    else if (wristAngle_new>thresholdW){
        //        wristVel = lambdaW*sign*(wristAngle_new-thresholdW);
        wristVel = lambdaW*(wristAngle_new-thresholdW);}
}

void LawOpti::writeDebugData(double debug[], Eigen::Vector3d posEE, double beta){
    debug[0] = posA0inHip[0];
    debug[1] = posA0inHip[1];
    debug[2] = posA0inHip[2];
    debug[3] = posEEinHip[0];
    debug[4] = posEEinHip[1];
    debug[5] = posEEinHip[2];
    debug[6] = delta;
    debug[9] = beta_new;
    debug[10] = beta;
    debug[11] = dBeta;
    debug[12] = betaDot;
    debug[13] = phi;
    debug[14] = theta;
    debug[15] = wristAngle_new;
    debug[16] = wristVel;

}

void LawOpti::displayData(Eigen::Vector3d posEE, double beta){
    //qDebug("posA0 : %lf; %lf, %lf", posA0[0], posA0[1], posA0[2]);
    qDebug("posA0 in hip frame: %lf; %lf, %lf", posA0inHip[0], posA0inHip[1], posA0inHip[2]);
    qDebug("posEE: %lf; %lf, %lf", posEE[0], posEE[1], posEE[2]);
    qDebug("posEE in hip frame: %lf; %lf, %lf", posEEinHip[0], posEEinHip[1], posEEinHip[2]);
    qDebug("beta: %lf \n beta_new: %lf", beta, beta_new);
}

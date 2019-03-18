#include "lawopti.h"
#include <QDebug>
#include <math.h>

LawOpti::LawOpti()
{
}

LawOpti::~LawOpti()
{
}


void LawOpti::initialization(Eigen::Vector3d posA, Eigen::Vector3d posEE, unsigned int freq){
    posA0 = posA;
    posAinHip = posA;

    posEEinHip = posEE;

    posA_filt = posA;
    posA_filt_old = posA;

    posEE_filt = posEE;
    posEE_filt_old = posEE;

    samplePeriod = 1./freq;
    coeff = samplePeriod/(0.03 + samplePeriod);

    delta = 0;
    beta_new = -M_PI_2;
    dBeta = 0;
    betaDot = 0;

    R = Eigen::Matrix3d::Zero();
}

/**
 * @brief LawOpti::initialAcromionPosition compute the initial position of the acromion marker = mean over the initCounts first measures of the acromion position
 * @param posA cartesian coordinates of the acromion marker (from optitrack)
 * @param initCounter counter of time steps
 * @param initCounts number of time step to take into account to compute the initial position
 */
void LawOpti::initialAcromionPosition(Eigen::Vector3d posA, int initCounter, int initCounts){
    if (initCounter<initCounts){
        posA0 += posA;
    }

    if (initCounter == initCounts){
        posA0 += posA;
        posA0 = posA0/initCounts;
    }
}


void LawOpti::filter_optitrackData(Eigen::Vector3d posA, Eigen::Vector3d posEE){
    posA_filt = posA_filt_old + coeff*(posA - posA_filt_old);
    posEE_filt = posEE_filt_old + coeff*(posEE - posEE_filt_old);
}

void LawOpti::bufferingOldValues(){
    posA_filt_old = posA_filt;
    posEE_filt_old = posEE_filt;
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
    delta = (posEE-posA0).norm();

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

void LawOpti::writeDebugData(double debug[], Eigen::Vector3d posEE, double beta){
    debug[0] = posA0[0];
    debug[1] = posA0[1];
    debug[2] = posA0[2];
    debug[3] = posEE[0];
    debug[4] = posEE[1];
    debug[5] = posEE[2];
    debug[9] = delta;
    debug[10] = beta;
    debug[11] = dBeta;
    debug[12] = betaDot;

}

void LawOpti::displayData(Eigen::Vector3d posEE, double beta){
    qDebug("posA0: %lf; %lf, %lf\n", posA0[0], posA0[1], posA0[2]);
    qDebug("posA in hip frame: %lf; %lf, %lf\n", posAinHip[0], posAinHip[1], posAinHip[2]);
    qDebug("posEE: %lf; %lf, %lf\n", posEE[0], posEE[1], posEE[2]);
    qDebug("posEE in hip frame: %lf; %lf, %lf\n", posEEinHip[0], posEEinHip[1], posEEinHip[2]);
    qDebug("beta: %lf \n beta_new: %lf \n", beta, beta_new);
    qDebug("delta : %lf\n", delta);
}

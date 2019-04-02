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
    qDebug("posEE from FA: %lf; %lf, %lf", posEEfromFA[0], posEEfromFA[1], posEEfromFA[2]);
    //qDebug("posEE in hip frame: %lf; %lf, %lf", posEEinHip[0], posEEinHip[1], posEEinHip[2]);
    qDebug("beta: %lf \n beta_new: %lf", beta, beta_new);
    qDebug("phi: %lf -- theta: %lf  -- wristAngle: %lf", phi, theta, wristAngle_new*180/M_PI);
    qDebug("Wrist velocity: %lf",wristVel*180/M_PI);
}

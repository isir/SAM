#include "jfimu_v3.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "wiringPi.h"
#include <filesystem>
#include <iostream>

JFIMU_v3::JFIMU_v3(std::shared_ptr<SAM::Components> robot)
    : ControlIMU("Jacobian Formulation IMU 3", "JFIMU3", robot)
{
    _menu->set_description("Jacobian Formulation IMU v3");
    _menu->set_code("jfi3");
}

JFIMU_v3::~JFIMU_v3()
{
    stop_and_join();
}

void JFIMU_v3::initializationLaw(Eigen::Quaterniond qHi, double p)
{
    _lawJ.initialization(qHi, 1 / p);
    _lawJ.initializationIMU();
}

void JFIMU_v3::initialPositionsLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int cnt, int init_cnt)
{
    _lawJ.initialQuat(qHi, qT, qA, cnt, init_cnt);
    _lawJ.rotationMatrices(qHa, qHi, qT);
    _lawJ.updateFrames(theta);
    _lawJ.computeOriginsVectors(l, nbDOF);
}

void JFIMU_v3::controlLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int k, double lambda[], double threshold[], int cnt, int init_cnt)
{
    _lawJ.rotationMatrices(qHa, qHi, qT);
    _lawJ.updateFrames(theta);
    _lawJ.computeOriginsVectors(l, nbDOF);
    _lawJ.computeTrunkAngles(qHa, qT, qHi);
    _lawJ.controlLaw_v3(lt, lsh, k, lambda, threshold, cnt);
}

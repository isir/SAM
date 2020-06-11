#include "jfimu_v4.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "wiringPi.h"
#include <filesystem>
#include <iostream>

JFIMU_v4::JFIMU_v4(std::shared_ptr<SAM::Components> robot)
    : JacobianFormulationIMU_sk("Jacobian Formulation IMU 4", "JFIMU4", robot)
{
    _menu->set_description("Jacobian Formulation IMU v4");
    _menu->set_code("jfi4");
}

JFIMU_v4::~JFIMU_v4()
{
    stop_and_join();
}

void JFIMU_v4::initializationLaw(Eigen::Quaterniond qHi, double p)
{
    _lawJ.initialization(qHi, 1 / p);
    _lawJ.initializationIMU();
}

void JFIMU_v4::initialPositionsLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int cnt, int init_cnt)
{
    _lawJ.initialQuat(qHi, qT, qA, cnt, init_cnt);
    _lawJ.rotationMatrices(qHa, qHi, qT);
    _lawJ.updateFrames(theta);
    _lawJ.updateTrunkFrame(qT);
    _lawJ.computeOriginsVectors(l, nbDOF);
}

void JFIMU_v4::controlLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int k, double lambda[], double threshold[], int cnt, int init_cnt)
{
    _lawJ.rotationMatrices(qHa, qHi, qT);
    _lawJ.updateFrames(theta);
    _lawJ.updateTrunkFrame(qT);
    _lawJ.computeOriginsVectors(l, nbDOF);
    _lawJ.computeTrunkAngles(qHa, qT, qHi);
    _lawJ.computeArmAngles(qHa, qT, qA);
    _lawJ.scaleDisplacement(lt, cnt);
    _lawJ.controlLaw_v4(lt, lsh, k, lambda, threshold, cnt);
}

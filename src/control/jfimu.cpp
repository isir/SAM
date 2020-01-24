#include "jfimu.h"
#include "utils/check_ptr.h"
#include "utils/log/log.h"
#include "wiringPi.h"
#include <filesystem>
#include <iostream>

JacobianFormulationIMU::JacobianFormulationIMU(std::shared_ptr<SAM::Components> robot)
    : ControlIMU("Jacobian Formulation IMU 1", "JFIMU1", robot)
{
    _menu->set_description("Jacobian Formulation IMU v1");
    _menu->set_code("jfi1");
}

JacobianFormulationIMU::~JacobianFormulationIMU()
{
    stop_and_join();
}

void JacobianFormulationIMU::initializationLaw(Eigen::Quaterniond qHi, double p)
{
    _lawJ.initialization(qHi, 1 / p);
    _lawJ.initializationIMU();
}

void JacobianFormulationIMU::initialPositionsLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int cnt, int init_cnt)
{
    _lawJ.initialQuat(qHi, qT, qA, cnt, init_cnt);
    _lawJ.rotationMatrices(qHa, qHi, qT);
    _lawJ.projectionInHipIMU(lt, lsh, cnt, init_cnt);
    _lawJ.updateFrames(theta);
    _lawJ.computeOriginsVectors(l, nbDOF);
}

void JacobianFormulationIMU::controlLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int k, double lambda[], double threshold[], int cnt, int init_cnt)
{
    _lawJ.rotationMatrices(qHa, qHi, qT);
    _lawJ.projectionInHipIMU(lt, lsh, cnt, init_cnt);
    _lawJ.updateFrames(theta);
    _lawJ.computeOriginsVectors(l, nbDOF);
    _lawJ.controlLaw_v1(k, lambda, threshold, cnt);
}

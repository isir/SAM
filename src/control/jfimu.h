#ifndef JACOBIAN_FORMULATION_IMU_H
#define JACOBIAN_FORMULATION_IMU_H

#include "controlimu.h"
#include <fstream>

class JacobianFormulationIMU : public ControlIMU {
public:
    explicit JacobianFormulationIMU(std::shared_ptr<SAM::Components> robot);
    ~JacobianFormulationIMU() override;

private:
    void initializationLaw(Eigen::Quaterniond qHi, double p) override;
    void initialPositionsLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int cnt, int init_cnt) override;
    void controlLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int k, double lambda[], double threshold[], int cnt, int init_cnt) override;
};

#endif // GENERAL_FORMULATION_H

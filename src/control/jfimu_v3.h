#ifndef JFIMU_v3_H
#define JFIMU_v3_H

#include "jfimu_sk.h"
#include <fstream>

class JFIMU_v3 : public JacobianFormulationIMU_sk {
public:
    explicit JFIMU_v3(std::shared_ptr<SAM::Components> robot);
    ~JFIMU_v3() override;

private:
    void initializationLaw(Eigen::Quaterniond qHi, double p) override;
    void initialPositionsLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int cnt, int init_cnt) override;
    void controlLaw(Eigen::Quaterniond qHa, Eigen::Quaterniond qHi, Eigen::Quaterniond qT, Eigen::Quaterniond qA, double theta[], int lt, int lsh, int l[], int nbDOF, int k, double lambda[], double threshold[], int cnt, int init_cnt) override;
};

#endif // JFIMU_v3_H

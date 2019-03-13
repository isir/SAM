#ifndef LAWOPTI_H
#define LAWOPTI_H

#include <eigen3/Eigen/Dense>

class LawOpti
{

public:
    LawOpti();
    ~LawOpti();
    void initialization(Eigen::Vector3d posA, Eigen::Vector3d posEE, unsigned int freq);
    void initialAcromionPosition(Eigen::Vector3d posA, int initCounter, int initCounts);
    void filter_optitrackData(Eigen::Vector3d posA, Eigen::Vector3d posEE);
    void bufferingOldValues();
    void controlLaw(Eigen::Vector3d posEE, double beta, double Lua, double Lfa, int lambda, double threshold);
    void writeDebugData(double debug[], Eigen::Vector3d posEE, double beta);
    void displayData(Eigen::Vector3d posEE, double beta);
    double returnBetaDot_deg() { return betaDot*180./M_PI; }

private:
    Eigen::Vector3d posA0; // initial position of the acromion
    Eigen::Vector3d posAinHip; // position of the acromion in hip frame
    Eigen::Vector3d posA_filt, posA_filt_old; // filtered acromion position
    Eigen::Vector3d posEEinHip; // position of the end-effector in hip frame
    Eigen::Vector3d posEE_filt, posEE_filt_old; // filtered EE position
    double delta; // distance between initial (=reference) acromion position and current end-effector position
    double beta_new; // desired elbow flexion/extension angle
    double dBeta; // difference between current beta and desired beta
    double betaDot; // elbow angular velocity
    double samplePeriod;
    double coeff; // coefficient for low-pass filtering

    Eigen::Matrix3d R; // term of rotation matrix of hip frame in global frame
};

#endif // LAWOPTI_H

#ifndef LAWJACOBIAN_H
#define LAWJACOBIAN_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string.h>

static const int nbFrames = 5;
static const int nbLinks = 4;
static const std::string rel = "0zyyz";

class LawJacobian {
public:
    LawJacobian();
    ~LawJacobian();
    void initialization(Eigen::Vector3d posA, Eigen::Quaterniond qHip, unsigned int freq);
    void initialPositions(Eigen::Vector3d posA, Eigen::Vector3d posHip, Eigen::Quaterniond qHip, int initCounter, int initCounts);
    void rotationMatrices(Eigen::Quaterniond qHand, Eigen::Quaterniond qHip, int initCounter, int initCounts);
    void projectionInHip(Eigen::Vector3d posA, Eigen::Vector3d posHip, int initCounter, int initCounts);
    void bufferingOldValues();
    void updateFrames(double theta[], double l[]);
    void controlLaw(Eigen::Vector3d posA, int lambda, double threshold[]);
    void writeDebugData(double d[], double theta[]);
    void displayData(Eigen::Vector3d posEE, double beta);
    /// RETURN DATA
    Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> returnthetaDot_deg() { return thetaDot * 180. / M_PI; }
    Eigen::Vector3d returnPosAinHip() { return posAinHip; }

private:
    Eigen::Matrix<double, 3, nbFrames, Eigen::DontAlign> x, y, z; // frames
    Eigen::Matrix<double, 3, nbLinks, Eigen::DontAlign> J, pinvJ; // jacobian patrix
    Eigen::Matrix<double, 3, nbLinks, Eigen::DontAlign> OO; // vectors between the centers of the frames
    Eigen::Vector3d posA0; // initial position of the acromion
    Eigen::Vector3d posA0inHip; // initial position of the acromion in hip frame
    Eigen::Vector3d posAinHip; // position of the acromion and the elbow in hip frame
    Eigen::Vector3d delta; // displacement between acromion initial (=reference) position and current position
    Eigen::Vector3d posHip0; // initial position of hip
    Eigen::Quaternion<double, Eigen::DontAlign> qHip0, qHip_filt, qHip_filt_old; // quaternions for hip frame rotation
    double samplePeriod;
    double coeff; // coefficient for low-pass filtering
    Eigen::Matrix<double, nbLinks, 1, Eigen::DontAlign> thetaNew, thetaDot;

    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Rhip, Rhand, Rframe; // term of rotation matrix of hip frame in initial hip frame
    double R11, R12, R13, R21, R22, R23, R31, R32, R33;
};

#endif // LAWJACOBIAN_H

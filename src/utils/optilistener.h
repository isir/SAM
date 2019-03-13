#ifndef OPTILISTENER_H
#define OPTILISTENER_H

#include <QUdpSocket>
#include <QObject>
#include <unistd.h>

typedef struct {
    float posx;
    float posy;
    float posz;
} optitrack_marker_t;

typedef struct  {
    char name[256];
    int nMarkers;
    std::vector<optitrack_marker_t> markers;
} optitrack_market_set_t;

typedef struct {
    int ID;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
    float fError; // Version >= 2
    bool bTrackingValid; // Version >= 2.6
    short params; // Version >= 2.6
} optitrack_rigibody_t;

// Skeletons (NatNet version 2.1 and later)
typedef struct {
    int skeletonID;
    int nRigidBodies;
    std::vector<optitrack_rigibody_t> rigidBodies;
} optitrack_skeleton_t;

// labeled markers (NatNet version 2.3 and later)
typedef struct {
    int ID;
    int modelID, markerID;
    float x;
    float y;
    float z;
    float size;

    // Version >= 2.6
    bool bOccluded;
    bool bPCSolved;
    bool bModelSolved;

    // Version >= 3
    bool bHasModel;
    bool bUnlabeled;
    bool bActiveMarker;
    float residual;

    short params;
} optitrack_labeled_marker_t;

// Force Plate data (NatNet version 2.9 and later)
typedef struct {
    int ID;
    int nChannels;
    int nFrames;
} optitrack_force_plate_t;

// Device data (NatNet version 3.0 and later)
typedef struct {
} optitrack_device_data_t;

typedef struct {
    int16_t messageID;
    int16_t nBytes;
    int32_t frameNumber;
    int32_t nMarkerSets;
    std::vector<optitrack_market_set_t> markerSets;
    int nOtherMarkers;
    std::vector<optitrack_marker_t> otherMarkers;
    int nRigidBodies;
    std::vector<optitrack_rigibody_t> rigidBodies;
    int nSkeletons;
    std::vector<optitrack_skeleton_t> skeletons;
    int nLabeledMarkers;
    std::vector<optitrack_labeled_marker_t> labeledMarkers;
    int nForcePlates;
    std::vector<optitrack_force_plate_t> forcePlates;
    int nDevices;
    std::vector<optitrack_device_data_t> devicesData;

    float softwareLatency;

    unsigned int timecode;
    unsigned int timecodeSub;
    char szTimecode[128];
    double timestamp;
    float fTemp;

    uint64_t cameraMidExposureTimestamp;
    uint64_t cameraDataReceivedTimestamp;
    uint64_t transmitTimestamp;

    bool bIsRecording;
    bool bTrackedModelsChanged;

    short params;
    int eod;
} optitrack_data_t;

Q_DECLARE_METATYPE(optitrack_data_t)

class OptiListener : public QObject
{
    Q_OBJECT
public:
    OptiListener();

    void begin(int port = 1511);

private:
    optitrack_data_t unpack(char* pData);

    QUdpSocket _udpSocket;

private slots:
    void on_new_packet();

signals:
    void new_data(optitrack_data_t data);
};

#endif // OPTILISTENER_H

#include "opti_listener.h"
#include "utils/log/log.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

OptiListener::OptiListener()
{
}

void OptiListener::begin(int port)
{
    if (!_socket.bind("0.0.0.0", port)) {
        critical() << "OptiListener: Failed to bind socket";
    }
}

void OptiListener::update()
{
    std::vector<std::byte> packet;
    while (_socket.available()) {
        packet = _socket.receive();
        _last_data = unpack(reinterpret_cast<char*>(packet.data()));
    }
}

optitrack_data_t OptiListener::unpack(char* pData)
{
    // Checks for NatNet Version number. Used later in function. Packets may be different depending on NatNet version.
    int major = 3;
    int minor = 0;

    char* ptr = pData;
    optitrack_data_t tmpData;

    // First 2 Bytes is message ID
    tmpData.messageID = 0;
    memcpy(&tmpData.messageID, ptr, 2);
    ptr += 2;
    //qDebug("Message ID : %d", tmpData.messageID);

    // Second 2 Bytes is the size of the packet
    tmpData.nBytes = 0;
    memcpy(&tmpData.nBytes, ptr, 2);
    ptr += 2;
    //qDebug("Byte count : %d", tmpData.nBytes);

    if (tmpData.messageID == 7) // FRAME OF MOCAP DATA packet
    {
        // Next 4 Bytes is the frame number
        tmpData.frameNumber = 0;
        memcpy(&tmpData.frameNumber, ptr, 4);
        ptr += 4;
        //qDebug("Frame # : %d", tmpData.frameNumber);

        // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
        tmpData.nMarkerSets = 0;
        memcpy(&tmpData.nMarkerSets, ptr, 4);
        ptr += 4;
        //qDebug("Marker Set Count : %d", tmpData.nMarkerSets);

        // Loop through number of marker sets and get name and data
        for (int i = 0; i < tmpData.nMarkerSets; i++) {
            optitrack_market_set_t tmpMarkerSet;
            // Markerset name
            strcpy(tmpMarkerSet.name, ptr);
            unsigned int nDataBytes = strlen(tmpMarkerSet.name) + 1;
            ptr += nDataBytes;
            //qDebug("Model Name: %s", tmpMarkerSet.name);

            // marker data
            tmpMarkerSet.nMarkers = 0;
            memcpy(&tmpMarkerSet.nMarkers, ptr, 4);
            ptr += 4;
            //qDebug("Marker Count : %d", tmpMarkerSet.nMarkers);

            for (int j = 0; j < tmpMarkerSet.nMarkers; j++) {
                optitrack_marker_t tmpMarker;
                tmpMarker.posx = 0;
                memcpy(&tmpMarker.posx, ptr, 4);
                ptr += 4;
                tmpMarker.posy = 0;
                memcpy(&tmpMarker.posy, ptr, 4);
                ptr += 4;
                tmpMarker.posz = 0;
                memcpy(&tmpMarker.posz, ptr, 4);
                ptr += 4;
                tmpMarkerSet.markers.push_back(tmpMarker);
                //qDebug("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]",j,tmpMarker.posx,tmpMarker.posy,tmpMarker.posz);
            }
            tmpData.markerSets.push_back(tmpMarkerSet);
        }

        // Loop through unlabeled markers
        tmpData.nOtherMarkers = 0;
        memcpy(&tmpData.nOtherMarkers, ptr, 4);
        ptr += 4;
        // OtherMarker list is Deprecated
        //printf("Unidentified Marker Count : %d\n", nOtherMarkers);
        for (int j = 0; j < tmpData.nOtherMarkers; j++) {
            optitrack_marker_t tmpMarker;
            tmpMarker.posx = 0.0f;
            memcpy(&tmpMarker.posx, ptr, 4);
            ptr += 4;
            tmpMarker.posy = 0.0f;
            memcpy(&tmpMarker.posy, ptr, 4);
            ptr += 4;
            tmpMarker.posz = 0.0f;
            memcpy(&tmpMarker.posz, ptr, 4);
            ptr += 4;
            tmpData.otherMarkers.push_back(tmpMarker);
            // Deprecated
            //printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n",j,x,y,z);
        }

        // Loop through rigidbodies
        tmpData.nRigidBodies = 0;
        memcpy(&tmpData.nRigidBodies, ptr, 4);
        ptr += 4;
        //qDebug("Rigid Body Count : %d", tmpData.nRigidBodies);
        for (unsigned int j = 0; j < tmpData.nRigidBodies; j++) {
            optitrack_rigibody_t tmpRigidBody;
            // Rigid body position and orientation
            tmpRigidBody.ID = 0;
            memcpy(&tmpRigidBody.ID, ptr, 4);
            ptr += 4;
            tmpRigidBody.x = 0.0f;
            memcpy(&tmpRigidBody.x, ptr, 4);
            ptr += 4;
            tmpRigidBody.y = 0.0f;
            memcpy(&tmpRigidBody.y, ptr, 4);
            ptr += 4;
            tmpRigidBody.z = 0.0f;
            memcpy(&tmpRigidBody.z, ptr, 4);
            ptr += 4;
            tmpRigidBody.qx = 0;
            memcpy(&tmpRigidBody.qx, ptr, 4);
            ptr += 4;
            tmpRigidBody.qy = 0;
            memcpy(&tmpRigidBody.qy, ptr, 4);
            ptr += 4;
            tmpRigidBody.qz = 0;
            memcpy(&tmpRigidBody.qz, ptr, 4);
            ptr += 4;
            tmpRigidBody.qw = 0;
            memcpy(&tmpRigidBody.qw, ptr, 4);
            ptr += 4;
            //qDebug("ID : %d", tmpRigidBody.ID);
            //qDebug("pos: [%3.2f,%3.2f,%3.2f]", tmpRigidBody.x,tmpRigidBody.y,tmpRigidBody.z);
            //qDebug("ori: [%3.2f,%3.2f,%3.2f,%3.2f]", tmpRigidBody.qx,tmpRigidBody.qy,tmpRigidBody.qz,tmpRigidBody.qw);

            // NatNet version 2.0 and later
            if (major >= 2) {
                // Mean marker error
                tmpRigidBody.fError = 0.0f;
                memcpy(&tmpRigidBody.fError, ptr, 4);
                ptr += 4;
                //qDebug("Mean marker error: %3.2f", tmpRigidBody.fError);
            }

            // NatNet version 2.6 and later
            if (((major == 2) && (minor >= 6)) || (major > 2) || (major == 0)) {
                // params
                tmpRigidBody.params = 0;
                memcpy(&tmpRigidBody.params, ptr, 2);
                ptr += 2;
                tmpRigidBody.bTrackingValid = tmpRigidBody.params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
            }

            tmpData.rigidBodies.push_back(tmpRigidBody);
        } // Go to next rigid body
    } else if (tmpData.messageID == 5) // Data Descriptions
    {
        warning() << "WARNING ID message : 5 -- Not yet integrated";
    } else if (tmpData.messageID == 2) // skeleton
    {
        warning() << "WARNING ID message : 2 -- Not yet integrated";
    } else {
        warning() << "Unrecognized Packet Type: " << tmpData.messageID;
    }

    //    printf("From listener : nRB : %d\n", tmpData.nRigidBodies);
    return tmpData;
}

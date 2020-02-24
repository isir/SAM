/**
 * @file OscPacket.h
 * @author Seb Madgwick
 * @brief Functions and structures for constructing and deconstructing OSC
 * packets.
 * See http://opensoundcontrol.org/spec-1_0
 */

#ifndef OSC_PACKET_H
#define OSC_PACKET_H

#include "OscBundle.h"
#include "OscCommon.h"
#include "OscError.h"
#include "OscMessage.h"
#include <stddef.h>

// Maximum OSC packet size.  The OSC packet size is limited by the maximum packet size permitted by the transport layer.
#define MAX_OSC_PACKET_SIZE (MAX_TRANSPORT_SIZE)

class OscPacket {
public:
    OscPacket();
    ~OscPacket();
    size_t OscPacketGetSize() const;
    char OscPacketGetContents(unsigned int index) const;
    void OscPacketSetContents(unsigned int index, char value);
    OscError OscPacketProcessMessages(OscMessage *oscMessage, OscTimeTag *oscTimeTag);
    OscError OscPacketInitialiseFromContents(const void * const oscContents);
    OscError OscPacketInitialiseFromCharArray(const char * const source, const size_t numberOfBytes);

private:
    OscError DeconstructContents(OscTimeTag * const oscTimeTag, const char * const oscContents, const size_t contentsSize, OscMessage *currentMessage, OscTimeTag *currentTimeTag);

    char contents[MAX_OSC_PACKET_SIZE];
    size_t size;

};

#endif //OSC_PACKET_H

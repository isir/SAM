/**
 * @file OscSlip.h
 * @author Seb Madgwick
 * @brief Functions and structures for encoding and decoding OSC packets using
 * the SLIP protocol.
 * See http://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol
 */

#ifndef OSC_SLIP_H
#define OSC_SLIP_H

#include "OscCommon.h"
#include "OscError.h"
#include "OscPacket.h"


// OSC SLIP decoder buffer size.  If a packet size exceeds the buffer size then all bytes in the decoder buffer will be discarded.
#define OSC_SLIP_DECODER_BUFFER_SIZE (MAX_TRANSPORT_SIZE)

class OscSlipDecoder {

public:
    OscSlipDecoder();
    ~OscSlipDecoder();

    void OscSlipClearBuffer();
    OscError OscSlipProcessByte(const char byte, OscMessage * oscMessage, OscTimeTag * oscTimeTag);
    OscError OscSlipEncodePacket(const OscPacket * const oscPacket, size_t * const slipPacketSize, char * const destination, const size_t destinationSize);

private:
    char buffer[OSC_SLIP_DECODER_BUFFER_SIZE];
    unsigned int bufferIndex;
};

#endif //OSC_SLIP_H

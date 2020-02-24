/**
 * @file OscSlip.c
 * @author Seb Madgwick
 * @brief Functions and structures for encoding and decoding OSC packets using
 * the SLIP protocol.
 * See http://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol
 */

//------------------------------------------------------------------------------
// Includes

#include "OscSlip.h"
#include <stddef.h>
#include "utils/log/log.h"

//------------------------------------------------------------------------------
// Definitions

#define SLIP_END static_cast<char>(0xC0)
#define SLIP_ESC static_cast<char>(0xDB)
#define SLIP_ESC_END static_cast<char>(0xDC)
#define SLIP_ESC_ESC static_cast<char>(0xDD)

//------------------------------------------------------------------------------
// Functions

OscSlipDecoder::OscSlipDecoder()
    : bufferIndex(0)
{

}

OscSlipDecoder::~OscSlipDecoder()
{

}

/**
 * @brief Encodes an OSC packet as a SLIP packet.
 *
 * The OSC packet is encoded as a SLIP packet and written to the destination
 * address.  The size of the encoded SLIP packet is written to the
 * destinationSize address.  If the destination is too small to contain the
 * encoded SLIP packet then the written size will be 0.
 *
 * Example use:
 * @code
 * char slipPacket[1024];
 * size slipPacketSize;
 * OscSlipEncodePacket(&oscPacket, slipPacket, &slipPacketSize, sizeof(slipPacket));
 * @endcode
 *
 * @param oscPacket OSC packet to be encoded.
 * @param destination Destination address of the OSC SLIP packet.
 * @param destinationSize Size of the destination.
 * @return Error code (0 if successful).
 */
OscError OscSlipDecoder::OscSlipEncodePacket(const OscPacket * const oscPacket, size_t * const slipPacketSize, char * const destination, const size_t destinationSize) {
    *slipPacketSize = 0; // size will be 0 if function unsuccessful
    unsigned int encodedPacketSize = 0;
    unsigned int packetIndex;
    for (packetIndex = 0; packetIndex < oscPacket->OscPacketGetSize(); packetIndex++) {
        if ((encodedPacketSize + 1) > destinationSize) {
            return OscErrorDestinationTooSmall; // error: destination too small
        }
        switch (oscPacket->OscPacketGetContents(packetIndex)) {
            case SLIP_END:
                destination[encodedPacketSize++] = SLIP_ESC;
                destination[encodedPacketSize++] = SLIP_ESC_END;
                break;
            case SLIP_ESC:
                destination[encodedPacketSize++] = SLIP_ESC;
                destination[encodedPacketSize++] = SLIP_ESC_ESC;
                break;
            default:
                destination[encodedPacketSize++] = oscPacket->OscPacketGetContents(packetIndex);
        }
    }
    destination[encodedPacketSize++] = SLIP_END;
    *slipPacketSize = encodedPacketSize;
    return OscErrorNone;
}

/**
 * @brief Processes byte received within serial stream.
 *
 * This function should be called for each consecutive byte received within a
 * serial stream.  Each byte is added to SLIP decoder receive buffer.  If the
 * received byte is the last byte of a SLIP packet then the SLIP packet is
 * decoded and parsed to the application as an OSC packet via the ProcessPacket
 * function.  The decoded packet will be discarded if a ProcessPacket function
 * has not been assigned.
 *
 * Example use:
 * @code
 * while(MySerialDataReady()){
 *     OscSlipDecoderProcessByte(&oscSlipDecoder, MySerialGetByte());
 * }
 * @endcode
 *
 * @param oscSlipDecoder Address OSC SLIP decoder structure.
 * @param byte Byte received within serial stream.
 * @return Error code (0 if successful).
 */
OscError OscSlipDecoder::OscSlipProcessByte(const char byte, OscMessage * oscMessage, OscTimeTag * oscTimeTag) {

    // Add byte to buffer
    buffer[bufferIndex] = byte;

    // Increment index with overflow
    if (++bufferIndex+1 >= OSC_SLIP_DECODER_BUFFER_SIZE) {
        bufferIndex = 0;
        return OscErrorEncodedSlipPacketTooLong; // error: SLIP packet is too long
    }

    // Return if byte not END byte
    if (byte != SLIP_END) {
        return OscErrorNone;
    }

    // Reset index
    bufferIndex = 0;

    // Decode packet
    OscPacket oscPacket;
    unsigned int index = 0;
    while (buffer[index] != SLIP_END) {
        if (buffer[index] == SLIP_ESC) {
            switch (buffer[++index]) {
                case SLIP_ESC_END:
                    oscPacket.OscPacketSetContents(oscPacket.OscPacketGetSize(), SLIP_END);
                    break;
                case SLIP_ESC_ESC:
                    oscPacket.OscPacketSetContents(oscPacket.OscPacketGetSize(), SLIP_ESC);
                    break;
                default:
                    return OscErrorUnexpectedByteAfterSlipEsc; // error: unexpected byte value
            }
        } else {
            oscPacket.OscPacketSetContents(oscPacket.OscPacketGetSize(), buffer[index]);
        }
        if (oscPacket.OscPacketGetSize() > MAX_OSC_PACKET_SIZE) {
            return OscErrorDecodedSlipPacketTooLong; // error: decoded packet too large
        }
        index++;
    }

    OscError oscError = oscPacket.OscPacketProcessMessages(oscMessage, oscTimeTag);
    bufferIndex = 0;
    if (oscError != OscErrorNone) {
        return oscError;
    }
    return OscErrorNone;
}


void OscSlipDecoder::OscSlipClearBuffer() {
    bufferIndex = 0;
}

/**
 * @file OscPacket.c
 * @author Seb Madgwick
 * @brief Functions and structures for constructing and deconstructing OSC
 * packets.
 * See http://opensoundcontrol.org/spec-1_0
 */

#include "OscPacket.h"
#include <stdbool.h>
#include <iostream>

OscPacket::OscPacket()
    : size(0)
{

}

OscPacket::~OscPacket()
{

}

size_t OscPacket::OscPacketGetSize() const
{
    return size;
}

char OscPacket::OscPacketGetContents(unsigned int index) const
{
    return contents[index];
}

void OscPacket::OscPacketSetContents(unsigned int index, char value)
{
    contents[index] = value;
    size = index+1;
}

/**
 * @brief Initialises an OSC packet from either OSC message or OSC
 * bundle.
 *
 * An OSC packet must be initialised before use.  This function is used to
 * initialise an OSC packet from either OSC message or OSC  bundle and is
 * typically of use when constructing an OSC packet for transmission.
 *
 * Example use:
 * @code
 * OscMessage oscMessage;
 * OscMessageInitialise(&oscMessage, "/example");
 * OscPacket oscPacket;
 * OscPacketInitialiseFromContents(&oscPacket, &oscMessage);
 * @endcode
 *
 * @param oscPacket OSC packet to be initialised.
 * @param oscContents OSC message or OSC bundle.
 * @return Error code (0 if successful).
 */
OscError OscPacket::OscPacketInitialiseFromContents(const void * const oscContents) {
    if (OscContentsIsMessage(oscContents) == true) {
        OscMessage* oscMessage = (OscMessage *) oscContents;
        return oscMessage->OscMessageToCharArray(&size, contents, MAX_OSC_PACKET_SIZE);
    }
    if (OscContentsIsBundle(oscContents) == true) {
        OscBundle* oscBundle = (OscBundle *) oscContents;
        return oscBundle->OscBundleToCharArray(&size, contents, MAX_OSC_PACKET_SIZE);
    }
    return OscErrorInvalidContents; // error: invalid or uninitialised OSC contents
}

/**
 * @brief Initialises an OSC packet from byte array.
 *
 * An OSC packet must be initialised before use.  This function is used to
 * initialise an OSC packet from a byte array and is typically of use when
 * constructing an OSC packet from received bytes.
 *
 * Example use:
 * @code
 * OscPacket oscPacket;
 * const char source[] = "/example\0\0\0\0,\0\0"; // string terminating null character is part of OSC message
 * OscPacketInitialiseFromCharArray(&oscPacket, source, sizeof(source));
 * @endcode
 *
 * @param oscPacket OSC packet to be initialised.
 * @param source Byte array.
 * @param numberOfBytes Number of bytes in byte array.
 * @return Error code (0 if successful).
 */
OscError OscPacket::OscPacketInitialiseFromCharArray(const char * const source, const size_t numberOfBytes) {
    size = 0;
    if (numberOfBytes > MAX_OSC_PACKET_SIZE) {
        return OscErrorPacketSizeTooLarge; // error: size exceeds maximum packet size
    }
    while (size < numberOfBytes) {
        contents[size] = source[size];
        size++;
    }
    return OscErrorNone;
}

/**
 * @brief Processes the OSC packet to provide each OSC message contained within
 * the packet to the user application with the associated OSC time tag (if the
 * message is contained within a bundle).
 *
 * A ProcessMessage function must be implemented within the application and
 * assigned to the OSC packet structure after initialisation.  The
 * ProcessMessage function will be called for each OSC message found within the
 * OSC packet.
 *
 * Example use:
 * @code
 * void ProcessMessage(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage) {
 * }
 *
 * void Main() {
 *     OscPacket oscPacket;
 *     const char source[] = "/example\0\0\0\0,\0\0\0";
 *     OscPacketInitialiseFromCharArray(&oscPacket, source, sizeof(source) - 1);
 *     oscPacket.processMessage = ProcessPacket;
 *     OscPacketProcessMessages(&oscPacket);
 * }
 * @endcode
 *
 * @param oscPacket OSC packet to be processed.
 * @return Error code (0 if successful).
 */
OscError OscPacket::OscPacketProcessMessages(OscMessage *oscMessage, OscTimeTag *oscTimeTag) {
    return DeconstructContents(nullptr, contents, size, oscMessage, oscTimeTag);
}

/**
 * @brief Recursively deconstructs the OSC contents to provide each OSC message
 * to the user application with the associated OSC time tag (if the message is
 * contained within a bundle).
 *s
 * This is an internal function and cannot be called by the user application.
 *
 * @param oscPacket OSC packet.
 * @param oscTimeTag OSC time tag of the bundle containing the OSC contents.
 * Must be NULL if the contents is not within an OSC bundle.
 * @param oscContents OSC contents to be deconstructed.
 * @param contentsSize Size of the OSC contents.
 * @return Error code (0 if successful).
 */
OscError OscPacket::DeconstructContents(OscTimeTag * const oscTimeTag, const char * const oscContents, const size_t contentsSize, OscMessage *currentMessage, OscTimeTag *currentTimeTag) {
    if (contentsSize == 0) {
        return OscErrorContentsEmpty; // error: contents empty
    }

    // Contents is an OSC message
    if (OscContentsIsMessage(oscContents) == true) {
        OscMessage oscMessage;
        const OscError oscError = oscMessage.OscMessageInitialiseFromCharArray(oscContents, contentsSize);
        if (oscError != OscErrorNone) {
            return oscError; // error: message initialisation failed
        }
        *currentMessage = oscMessage;
        if(oscTimeTag)
            *currentTimeTag = *oscTimeTag;
        return OscErrorNone;
    }

    // Contents is an OSC bundle
    if (OscContentsIsBundle(oscContents) == true) {
        OscBundle oscBundle;
        OscError oscError = oscBundle.OscBundleInitialiseFromCharArray(oscContents, contentsSize);
        if (oscError != OscErrorNone) {
            return oscError; // error: bundle initialisation failed
        }
        do {
            OscBundleElement oscBundleElement;
            if (oscBundle.OscBundleIsBundleElementAvailable() == false) {
                break; // no more bundle elements
            }
            oscError = oscBundle.OscBundleGetBundleElement(&oscBundleElement);
            if (oscError != OscErrorNone) {
                return oscError; // error: get bundle element failed
            }
            OscTimeTag oscTime = oscBundle.OscBundleGetTimeTag();         
            oscError = DeconstructContents(&oscTime, oscBundleElement.contents, static_cast<unsigned int>(oscBundleElement.size.int32), currentMessage, currentTimeTag); // recursive deconstruction
            if (oscError != OscErrorNone) {
                return oscError; // error: contents deconstruction failed
            }
        } while (true);
        return OscErrorNone;
    }

    return OscErrorInvalidContents; // error: invalid or uninitialised contents
}

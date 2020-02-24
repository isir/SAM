/**
 * @file OscBundle.h
 * @author Seb Madgwick
 * @brief Functions and structures for constructing and deconstructing OSC
 * bundles.
 * See http://opensoundcontrol.org/spec-1_0
 */

#ifndef OSC_BUNDLE_H
#define OSC_BUNDLE_H

//------------------------------------------------------------------------------
// Includes

#include "OscError.h"
#include "OscCommon.h"
#include "OscMessage.h"
#include <stddef.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief OSC bundle header.  These are the first 8 bytes (including the
 * terminating null character) that appear at the start of every bundle.
 */
#define OSC_BUNDLE_HEADER "#bundle"

/**
 * @brief Minimum size (number of bytes) of an OSC bundle as per the OSC
 * specification.
 */
#define MIN_OSC_BUNDLE_SIZE ((sizeof(OSC_BUNDLE_HEADER) + sizeof(OscTimeTag)))

/**
 * @brief Maximum size (number of bytes) of an OSC bundle equal to the maximum
 * packet size permitted by the transport layer.
 */
#define MAX_OSC_BUNDLE_SIZE (MAX_TRANSPORT_SIZE)

/**
 * @brief Maximum combined size (number of bytes) of all OSC bundle elements
 * that may be contained within an OSC bundle.
 */
#define MAX_OSC_BUNDLE_ELEMENTS_SIZE (MAX_OSC_BUNDLE_SIZE - sizeof(OSC_BUNDLE_HEADER) - sizeof(OscTimeTag))

/**
 * @brief OSC bundle element structure.  This structure is used internally and
 * should not be used by the user application.
 */
typedef struct {
    OscArgument32 size; // int32
    char * contents; // pointer to bundle element contents
} OscBundleElement;


class OscBundle {
public:
    OscBundle(const OscTimeTag timeTag=oscTimeTagZero);
    ~OscBundle();   

    OscTimeTag OscBundleGetTimeTag();

    void OscBundleInitialise(const OscTimeTag oscTimeTag);
    OscError OscBundleAddContents(const char * const oscContents);
    void OscBundleEmpty();
    bool OscBundleIsEmpty();
    size_t OscBundleGetRemainingCapacity();
    size_t OscBundleGetSize();
    OscError OscBundleToCharArray(size_t * const oscBundleSize, char * const destination, const size_t destinationSize);
    OscError OscBundleInitialiseFromCharArray(const char * const source, const size_t numberOfBytes);
    bool OscBundleIsBundleElementAvailable();
    OscError OscBundleGetBundleElement(OscBundleElement * const oscBundleElement);

private:
    char header[sizeof (OSC_BUNDLE_HEADER)]; // must be the first member so that the first byte of structure is equal to '#'.
    OscTimeTag oscTimeTag;
    char oscBundleElements[MAX_OSC_BUNDLE_ELEMENTS_SIZE];
    size_t oscBundleElementsSize;
    unsigned int oscBundleElementsIndex;
};

#endif //OSC_BUNDLE_H

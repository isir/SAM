/**
 * @file OscBundle.c
 * @author Seb Madgwick
 * @brief Functions and structures for constructing and deconstructing OSC
 * bundles.
 * See http://opensoundcontrol.org/spec-1_0
 */

//------------------------------------------------------------------------------
// Includes

#include "OscBundle.h"

//------------------------------------------------------------------------------
// Functions


OscBundle::OscBundle(const OscTimeTag timeTag)
    : oscTimeTag(timeTag), oscBundleElementsSize(0)
{
    header[0] = OSC_BUNDLE_HEADER[0];
    header[1] = OSC_BUNDLE_HEADER[1];
    header[2] = OSC_BUNDLE_HEADER[2];
    header[3] = OSC_BUNDLE_HEADER[3];
    header[4] = OSC_BUNDLE_HEADER[4];
    header[5] = OSC_BUNDLE_HEADER[5];
    header[6] = OSC_BUNDLE_HEADER[6];
    header[7] = OSC_BUNDLE_HEADER[7];
}

OscBundle::~OscBundle()
{

}

OscTimeTag OscBundle::OscBundleGetTimeTag()
{
    return oscTimeTag;
}


/**
 * @brief Initialises an OSC bundle with a specified OSC time tag.
 *
 * An OSC bundle must be initialised before use.  The oscTimeTag argument may be
 * specified as oscTimeTagZero for an OSC time tag value of  zero. This may be
 * of use if the OSC time tag value is irrelevant to the user application, if
 * the contained OSC messages should be invoke immediately, or if the OSC time
 * tag value is intended to be overwritten after initialisation of the OSC
 * bundle.
 *
 * Example use:
 * @code
 * OscBundle oscBundle;
 * OscBundleInitialise(&oscBundle, oscTimeTagZero);
 * oscBundle.oscTimeTag.value = 0x100000000; // overwrite OSC time tag with value of 1 second
 * @endcode
 *
 * @param oscBundle OSC bundle to be initialised.
 * @param oscTimeTag OSC time tag.
 */
void OscBundle::OscBundleInitialise(const OscTimeTag timeTag) {
    header[0] = OSC_BUNDLE_HEADER[0];
    header[1] = OSC_BUNDLE_HEADER[1];
    header[2] = OSC_BUNDLE_HEADER[2];
    header[3] = OSC_BUNDLE_HEADER[3];
    header[4] = OSC_BUNDLE_HEADER[4];
    header[5] = OSC_BUNDLE_HEADER[5];
    header[6] = OSC_BUNDLE_HEADER[6];
    header[7] = OSC_BUNDLE_HEADER[7];
    oscTimeTag = timeTag;
    oscBundleElementsSize = 0;
}

/**
 * @brief Adds an OSC message or OSC bundle to an OSC bundle.
 *
 * The oscContents argument must point to an initialised OSC message or OSC
 * bundle.  This function may be called multiple times to add multiple  OSC
 * messages or OSC bundles to a containing OSC bundle.  If the remaining
 * capacity of the containing OSC bundle is insufficient to hold the additional
 * contents then the additional contents will be discarded and the function will
 * return an error.
 *
 * Example use:
 * @code
 * OscMessage oscMessageToAdd;
 * OscMessageInitialise(&oscMessageToAdd, "/example/address/pattern");
 *
 * OscBundle oscBundleToAdd;
 * OscBundleInitialise(&oscBundleToAdd, oscTimeTagZero);
 *
 * OscBundle oscBundle;
 * OscBundleInitialise(&oscBundle, oscTimeTagZero);
 * OscBundleAddContents(&oscBundle, &oscMessageToAdd);
 * OscBundleAddContents(&oscBundle, &oscBundleToAdd);
 * @endcode
 *
 * @param oscBundle OSC bundle that will contain the OSC message or OSC bundle
 * to be added.
 * @param oscContents OSC message or OSC bundle to be added to the OSC bundle.
 * @return Error code (0 if successful).
 */
OscError OscBundle::OscBundleAddContents(const char * const oscContents) {
    if ((oscBundleElementsSize + sizeof (OscArgument32)) > MAX_OSC_BUNDLE_ELEMENTS_SIZE) {
        return OscErrorBundleFull; // error: bundle full
    }
    OscBundleElement oscBundleElement;
    oscBundleElement.contents = &oscBundleElements[oscBundleElementsSize + sizeof (OscArgument32)];
    OscError oscError = OscErrorInvalidContents; // error: invalid or uninitialised OSC contents
    if (OscContentsIsMessage(oscContents) == true) {
        size_t oscBundleElementSize;
        OscMessage* oscMessage = (OscMessage *) oscContents;
        oscError = oscMessage->OscMessageToCharArray(&oscBundleElementSize, oscBundleElement.contents, this->OscBundleGetRemainingCapacity());
        oscBundleElement.size.int32 = static_cast<int32_t>(oscBundleElementSize);
    }
    if (OscContentsIsBundle(oscContents) == true) {
        size_t oscBundleElementSize;
        OscBundle* oscBundle = (OscBundle *) oscContents;
        oscError = oscBundle->OscBundleToCharArray(&oscBundleElementSize, oscBundleElement.contents, this->OscBundleGetRemainingCapacity());
        oscBundleElement.size.int32 = static_cast<int32_t>(oscBundleElementSize);
    }
    if (oscError != 0) {
        return oscError; // error: ???
    }
    oscBundleElements[oscBundleElementsSize++] = oscBundleElement.size.byteStruct.byte3;
    oscBundleElements[oscBundleElementsSize++] = oscBundleElement.size.byteStruct.byte2;
    oscBundleElements[oscBundleElementsSize++] = oscBundleElement.size.byteStruct.byte1;
    oscBundleElements[oscBundleElementsSize++] = oscBundleElement.size.byteStruct.byte0;
    oscBundleElementsSize += static_cast<unsigned int>(oscBundleElement.size.int32);
    return OscErrorNone;
}

/**
 * @brief Empties an OSC bundle.
 *
 * All OSC bundle elements contained within the OSC bundle are discarded.  The
 * OSC bundle's OSC time tag is not modified.
 *
 * Example use:
 * @code
 * OscBundleEmpty(&oscBundle);
 * @endcode
 *
 * @param oscBundle OSC bundle to be emptied.
 */
void OscBundle::OscBundleEmpty() {
    oscBundleElementsSize = 0;
}

/**
 * @brief Returns true if the OSC bundle is empty.
 *
 * An empty OSC bundle contains no OSC bundle elements (OSC messages or OSC
 * bundles) but does retain an OSC time tag.
 *
 * Example use:
 * @code
 * if(OscBundleIsEmpty(&oscBundle)) {
 *     printf("oscBundle is empty.");
 * }
 * @endcode
 *
 * @param oscBundle OSC bundle.
 * @return True if the OSC bundle is empty.
 */
bool OscBundle::OscBundleIsEmpty() {
    return oscBundleElementsSize == 0;
}

/**
 * @brief Returns the remaining capacity (number of bytes) of an OSC bundle.
 *
 * The remaining capacity of an OSC bundle is the number of bytes available to
 * contain an OSC message or OSC bundle.
 *
 * Example use:
 * @code
 * const size_t remainingCapacity = OscBundleGetRemainingCapacity(&oscBundle);
 * @endcode
 *
 * @param oscBundle OSC bundle.
 * @return Remaining capacity (number of bytes) of an OSC bundle.
 */
size_t OscBundle::OscBundleGetRemainingCapacity() {
    const size_t remainingCapacity = MAX_OSC_BUNDLE_ELEMENTS_SIZE - oscBundleElementsSize - sizeof (OscArgument32); // account for int32 size required by OSC bundle element
    if (static_cast<int>(remainingCapacity) < 0) {
        return 0; // avoid negative result of capacity calculation
    }
    return remainingCapacity;
}

/**
 * @brief Returns the size (number of bytes) of an OSC bundle.
 *
 * An example use of this function would be to check whether the OSC bundle size
 * exceeds the remaining capacity of a containing OSC bundle.
 *
 * Example use:
 * @code
 * if(OscBundleGetSize(&oscBundleChild) > OscBundleGetRemainingCapacity(&oscBundleParent)) {
 *     printf("oscBundleChild is too large to be contained within oscBundleParent");
 * }
 * @endcode
 *
 * @param oscBundle OSC bundle.
 * @return Size (number of bytes) of the OSC bundle.
 */
size_t OscBundle::OscBundleGetSize() {
    return sizeof (OSC_BUNDLE_HEADER) + sizeof (OscTimeTag) + oscBundleElementsSize;
}

/**
 * @brief Converts an OSC bundle into a byte array to be contained within an OSC
 * packet or containing OSC bundle.
 *
 * This function is used internally and should not be used by the user
 * application.
 *
 * @param oscBundle OSC bundle.
 * @param oscBundleSize OSC bundle size.
 * @param destination Destination byte array.
 * @param destinationSize Destination size that cannot exceed.
 * @return Error code (0 if successful).
 */
OscError OscBundle::OscBundleToCharArray(size_t * const oscBundleSize, char * const destination, const size_t destinationSize) {
    *oscBundleSize = 0; // size will be 0 if function unsuccessful
    if ((sizeof (OSC_BUNDLE_HEADER) + sizeof (OscTimeTag) + oscBundleElementsSize) > destinationSize) {
        return OscErrorDestinationTooSmall; // error: destination too small
    }
    size_t destinationIndex = 0;
    unsigned int index;
    for (index = 0; index < sizeof (OSC_BUNDLE_HEADER); index++) {
        destination[destinationIndex++] = header[index];
    }
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte7;
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte6;
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte5;
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte4;
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte3;
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte2;
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte1;
    destination[destinationIndex++] = oscTimeTag.byteStruct.byte0;
    for (index = 0; index < oscBundleElementsSize; index++) {
        destination[destinationIndex++] = oscBundleElements[index];
    }
    *oscBundleSize = destinationIndex;
    return OscErrorNone;
}

/**
 * @brief Initialises an OSC bundle from a byte array contained within an OSC
 * packet or containing OSC bundle.
 *
 * This function is used internally and should not be used by the user
 * application.
 *
 * @param oscBundle OSC bundle.
 * @param source Byte array.
 * @param numberOfBytes Number of bytes in byte array.
 * @return Error code (0 if successful).
 */
OscError OscBundle::OscBundleInitialiseFromCharArray(const char * const source, const size_t numberOfBytes) {
    unsigned int sourceIndex = 0;

    // Return error if not valid bundle
    if (numberOfBytes % 4 != 0) {
        return OscErrorSizeIsNotMultipleOfFour; // error: size not multiple of 4
    }
    if (numberOfBytes < MIN_OSC_BUNDLE_SIZE) {
        return OscErrorBundleSizeTooSmall; // error: too few bytes to contain bundle
    }
    if (numberOfBytes > MAX_OSC_BUNDLE_SIZE) {
        return OscErrorBundleSizeTooLarge; // error: size exceeds maximum bundle size
    }
    if (source[sourceIndex] != '#') {
        return OscErrorNoHashAtStartOfBundle; // error: first byte is not '#'
    }

    // Header
    header[0] = source[sourceIndex++];
    header[1] = source[sourceIndex++];
    header[2] = source[sourceIndex++];
    header[3] = source[sourceIndex++];
    header[4] = source[sourceIndex++];
    header[5] = source[sourceIndex++];
    header[6] = source[sourceIndex++];
    header[7] = source[sourceIndex++];

    // OSC time tag
    oscTimeTag.byteStruct.byte7 = source[sourceIndex++];
    oscTimeTag.byteStruct.byte6 = source[sourceIndex++];
    oscTimeTag.byteStruct.byte5 = source[sourceIndex++];
    oscTimeTag.byteStruct.byte4 = source[sourceIndex++];
    oscTimeTag.byteStruct.byte3 = source[sourceIndex++];
    oscTimeTag.byteStruct.byte2 = source[sourceIndex++];
    oscTimeTag.byteStruct.byte1 = source[sourceIndex++];
    oscTimeTag.byteStruct.byte0 = source[sourceIndex++];

    // Osc bundle elements
    oscBundleElementsSize = 0;
    oscBundleElementsIndex = 0;
    do {
        oscBundleElements[oscBundleElementsSize++] = source[sourceIndex++];
    } while (sourceIndex < numberOfBytes);

    return OscErrorNone;
}

/**
 * @brief Returns true if an OSC bundle element is available based on the
 * current oscBundleElementsIndex value.
 *
 * This function is used internally and should not be used by the user
 * application.
 *
 * @param oscBundle OSC bundle.
 * @return True if a bundle element is available.
 */
bool OscBundle::OscBundleIsBundleElementAvailable() {
    return (oscBundleElementsIndex + sizeof (OscArgument32)) < oscBundleElementsSize;
}

/**
 * @brief Gets the next OSC bundle element available within the OSC bundle based
 * on the current oscBundleElementsIndex.
 *
 * oscBundleElementsIndex will be incremented to the next OSC bundle element if
 * this function is successful.  Otherwise, the oscBundleElementsIndex will
 * remain unmodified.
 *
 * This function is used internally and should not be used by the user
 * application.
 *
 * @param oscBundle OSC bundle.
 * @param oscBundleElement OSC bundle element.
 * @return Error code (0 if successful).
 */
OscError OscBundle::OscBundleGetBundleElement(OscBundleElement * const oscBundleElement) {
    if ((oscBundleElementsIndex + sizeof (OscArgument32)) >= oscBundleElementsSize) {
        return OscErrorBundleElementNotAvailable; // error: too few bytes to contain bundle element
    }
    oscBundleElement->size.byteStruct.byte3 = oscBundleElements[oscBundleElementsIndex++];
    oscBundleElement->size.byteStruct.byte2 = oscBundleElements[oscBundleElementsIndex++];
    oscBundleElement->size.byteStruct.byte1 = oscBundleElements[oscBundleElementsIndex++];
    oscBundleElement->size.byteStruct.byte0 = oscBundleElements[oscBundleElementsIndex++];
    if (oscBundleElement->size.int32 < 0) {
        return OscErrorNegativeBundleElementSize; // error: size cannot be negative
    }
    if ((oscBundleElement->size.int32 % 4) != 0) {
        return OscErrorSizeIsNotMultipleOfFour; // error: size not multiple of 4
    }
    if ((oscBundleElementsIndex + static_cast<unsigned int>(oscBundleElement->size.int32)) > oscBundleElementsSize) {
        return OscErrorInvalidElementSize; // error: too few bytes for indicated size
    }
    oscBundleElement->contents = &oscBundleElements[oscBundleElementsIndex];
    oscBundleElementsIndex += static_cast<unsigned int>(oscBundleElement->size.int32);
    return OscErrorNone;
}

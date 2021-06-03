/**
 * @file OscMessage.h
 * @author Seb Madgwick
 * @brief Functions and structures for constructing and deconstructing OSC
 * messages.
 *
 * MAX_OSC_ADDRESS_PATTERN_LENGTH and MAX_NUMBER_OF_ARGUMENTS may be modified as
 * required by the user application.
 *
 * See http://opensoundcontrol.org/spec-1_0
 */

#ifndef OSC_MESSAGE_H
#define OSC_MESSAGE_H

//------------------------------------------------------------------------------
// Includes

#include "OscCommon.h"
#include "OscError.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Minimum size (number of bytes) of an OSC message as per the OSC
 * specification.  The size is 8 bytes which includes the terminating null
 * character.
 */
#define MIN_OSC_MESSAGE_SIZE (sizeof("/\0\0\0,\0\0"))

/**
 * @brief Maximum size (number of bytes) of an OSC message equal to the maximum
 * packet size permitted by the transport layer.
 */
#define MAX_OSC_MESSAGE_SIZE (MAX_TRANSPORT_SIZE)

/**
 * @brief Maximum string length (excludes terminating null characters) of an OSC
 * address pattern.  This value may be modified as required by the user
 * application.
 */
#define MAX_OSC_ADDRESS_PATTERN_LENGTH (64)

/**
 * @brief Maximum number of arguments that may be contained within an OSC
 * message.  This value may be modified as required by the user application.
 */
#define MAX_NUMBER_OF_ARGUMENTS (16)

/**
 * @brief Maximum length of an OSC type tag string (includes comma but not
 * terminating null characters).
 */
#define MAX_OSC_TYPE_TAG_STRING_LENGTH (1 + MAX_NUMBER_OF_ARGUMENTS)

/**
 * @brief Maximum combined size (number of bytes) of all arguments that may be
 * contained within an OSC message.  The calculation assumes the worst case of
 * and extra 4 null characters for both the OSC address pattern and the OSC type
 * tag string.
 */
#define MAX_ARGUMENTS_SIZE (MAX_OSC_MESSAGE_SIZE - (MAX_OSC_ADDRESS_PATTERN_LENGTH + 4) - (MAX_OSC_TYPE_TAG_STRING_LENGTH + 4))

typedef enum {
    OscTypeTagInt32 = 'i',
    OscTypeTagFloat32 = 'f',
    OscTypeTagString = 's',
    OscTypeTagBlob = 'b',
    OscTypeTagInt64 = 'h',
    OscTypeTagTimeTag = 't',
    OscTypeTagDouble = 'd',
    OscTypeTagAlternateString = 'S',
    OscTypeTagCharacter = 'c',
    OscTypeTagRgbaColour = 'r',
    OscTypeTagMidiMessage = 'm',
    OscTypeTagTrue = 'T',
    OscTypeTagFalse = 'F',
    OscTypeTagNil = 'N',
    OscTypeTagInfinitum = 'I',
    OscTypeTagBeginArray = '[',
    OscTypeTagEndArray = ']',
} OscTypeTag;

class OscMessage {
public:
    OscMessage();
    ~OscMessage();
    void OscMessageGetAddressPattern(char* address);

    // Message construction
    OscError OscMessageInitialise(const char * oscAddressPattern);
    OscError OscMessageSetAddressPattern(const char * oscAddressPattern);
    OscError OscMessageAppendAddressPattern(const char * appendedParts);
    OscError OscMessageAddInt32(const int32_t int32);
    OscError OscMessageAddFloat32(const float float32);
    OscError OscMessageAddString(const char * string);
    OscError OscMessageAddBlob(const char * const source, const size_t numberOfBytes);
    OscError OscMessageAddInt64(const uint64_t int64);
    OscError OscMessageAddTimeTag(const OscTimeTag oscTimeTag);
    OscError OscMessageAddDouble(const Double64 double64);
    OscError OscMessageAddAlternateString(const char * string);
    OscError OscMessageAddCharacter(const char asciiChar);
    OscError OscMessageAddRgbaColour(const RgbaColour rgbaColour);
    OscError OscMessageAddMidiMessage(const MidiMessage midiMessage);
    OscError OscMessageAddBool(const bool boolean);
    OscError OscMessageAddNil();
    OscError OscMessageAddInfinitum();
    OscError OscMessageAddBeginArray();
    OscError OscMessageAddEndArray();
    size_t OscMessageGetSize();
    OscError OscMessageToCharArray(size_t * const oscMessageSize, char * const destination, const size_t destinationSize);

    // Message deconstruction
    OscError OscMessageInitialiseFromCharArray(const char * const source, const size_t size);
    bool OscMessageIsArgumentAvailable();
    OscTypeTag OscMessageGetArgumentType();
    OscError OscMessageSkipArgument();
    OscError OscMessageGetInt32(int32_t * const int32);
    OscError OscMessageGetFloat32(float * const float32);
    OscError OscMessageGetString(char * const destination, const size_t destinationSize);
    OscError OscMessageGetBlob(size_t * const blobSize, char * const destination, const size_t destinationSize);
    OscError OscMessageGetInt64(int64_t * const int64);
    OscError OscMessageGetTimeTag(OscTimeTag * const oscTimeTag);
    OscError OscMessageGetDouble(Double64 * const double64);
    OscError OscMessageGetCharacter(char * const character);
    OscError OscMessageGetRgbaColour(RgbaColour * const rgbaColour);
    OscError OscMessageGetMidiMessage(MidiMessage * const midiMessage);
    OscError OscMessageGetArgumentAsInt32(int32_t * const int32);
    OscError OscMessageGetArgumentAsFloat32(float * const float32);
    OscError OscMessageGetArgumentAsString(char * const destination, const size_t destinationSize);
    OscError OscMessageGetArgumentAsBlob(size_t * const blobSize, char * const destination, const size_t destinationSize);
    OscError OscMessageGetArgumentAsInt64(int64_t * const int64);
    OscError OscMessageGetArgumentAsTimeTag(OscTimeTag * const oscTimeTag);
    OscError OscMessageGetArgumentAsDouble(Double64 * const double64);
    OscError OscMessageGetArgumentAsCharacter(char * const character);
    OscError OscMessageGetArgumentAsRgbaColour(RgbaColour * const rgbaColour);
    OscError OscMessageGetArgumentAsMidiMessage(MidiMessage * const midiMessage);
    OscError OscMessageGetArgumentAsBool(bool * const boolean);

private:
    char oscAddressPattern[MAX_OSC_ADDRESS_PATTERN_LENGTH + 1]; // must be first member so that first byte of structure is equal to '/'.  Null terminated.
    char oscTypeTagString[MAX_OSC_TYPE_TAG_STRING_LENGTH + 1]; // includes comma.  Null terminated
    char arguments[MAX_ARGUMENTS_SIZE];
    size_t oscAddressPatternLength; // does not include null characters
    size_t oscTypeTagStringLength; // includes comma but not null characters
    size_t argumentsSize;
    unsigned int oscTypeTagStringIndex;
    unsigned int argumentsIndex;
};

#endif //OSC_MESSAGE_H

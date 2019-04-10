#ifndef MYOCONTROL_H
#define MYOCONTROL_H

#include <algorithm>
#include <stdio.h>

/**
 * \brief The MyoControl class allows to use state machine based algorithms to achieve EMGs control.
 * It has to be used following those steps :
 *  - set the control type
 *  - init this control
 *  - call the getJointAction method
 */
class MyoControl {

public:
    /**
     * \brief The MODE enum are the different available modes on the prosthesis
     */
    enum MODE {
        MYO_MODE_NONE,
        MYO_MODE_ELBOW,
        MYO_MODE_WRIST,
        MYO_MODE_HAND,
        MYO_MODE_PINCH,
        MYO_MODE_WRIST_FORWARDING,
        MYO_MODE_WRIST_BACKWARDING,
        MYO_MODE_FORARM,
        MYO_MODE_HAND_OPENING,
        MYO_MODE_HAND_CLOSING,
        MYO_MODE_THUMB,
        MYO_MODE_FOREFINGER,
        MYO_MODE_MIDDLEFINGER,
        MYO_MODE_RINGFINGER,
        MYO_MODE_LITTLEFINGER
    };

    /**
     * \brief The JOINT_ACTION enum are the differents joint's action on the prosthesis
     */
    enum JOINT_ACTION {
        NONE,
        ELBOW_UP,
        ELBOW_DOWN,
        ELBOW_STOP,
        WRIST_FORWARD,
        WRIST_BACKWARD,
        WRIST_STOP,
        HAND_OPEN,
        HAND_CLOSE,
        HAND_STOP,
        PINCH_OPEN,
        PINCH_CLOSE,
        PINCH_STOP,
        FORARM_STOP,
        THUMB_CLOSE,
        THUMB_OPEN,
        THUMB_STOP,
        FOREFINGER_CLOSE,
        FOREFINGER_OPEN,
        FOREFINGER_STOP,
        MIDDLEFINGER_CLOSE,
        MIDDLEFINGER_OPEN,
        MIDDLEFINGER_STOP,
        RINGFINGER_CLOSE,
        RINGFINGER_OPEN,
        RINGFINGER_STOP,
        LITTLEFINGER_CLOSE,
        LITTLEFINGER_OPEN,
        LITTLEFINGER_STOP,
    };

    enum BUBBLE_MODE {
        HAS_TO_CHECK_COCO,
        IS_ACTIVATED
    };

    /**
     * \brief The CONTROL_TYPE enum are the different type of control coded here
     */
    enum CONTROL_TYPE {
        NO_CONTROL,
        COCO_CONTROL, // Classic cocontraction control
        BUBBLE_COCO_CONTROL, // Classic cocontraction control with intermediate thresholds
        PROP_CONTROL, // Classic proportionnal control
        BUBBLE_PROP_CONTROL // Classic proportionnal control with intermediate thresholds
    };

    MyoControl();
    void init();
    JOINT_ACTION getJointAction(int emg1, int emg2);
    void jumpThisMode();

    void initCocontractionControl(MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int threshold_emg1, int threshold_emg2, int cocontraction_threshold_emg1, int cocontraction_threshold_emg2);
    void initPropControl(MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int threshold_high_forarm_emg1, int threshold_low_forarm_emg1, int threshold_high_forarm_emg2, int threshold_low_forarm_emg2, int cocontraction_threshold_emg1, int cocontraction_threshold_emg2, int deltaTInCount, int threshold_elbow_emg1 = 0, int threshold_elbow_emg2 = 0);
    void initBubbleCocontractionControl(const MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int counts_before_bubble, int counts_after_bubble, int threshold_high_emg1, int threshold_low_emg1, int threshold_high_emg2, int threshold_low_emg2, int cocontraction_threshold_emg1, int cocontraction_threshold_emg2);
    void initBubblePropControl(MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int counts_before_bubble, int counts_after_bubble, int threshold_high_forarm_emg1, int threshold_low_forarm_emg1, int threshold_high_forarm_emg2, int threshold_low_forarm_emg2, int cocontraction_threshold_emg1, int cocontraction_threshold_emg2, int deltaTInCount, int threshold_high_elbow_emg1, int threshold_low_elbow_emg1, int threshold_high_elbow_emg2, int threshold_low_elbow_emg2);

    int getCurrentMode() { return _current_mode; }
    int get_current_index() { return _sequence_modes_index; }
    int getOldMode() { return _old_mode; }
    bool hasChangedMode() { return _has_changed_mode; }

    void setControlType(CONTROL_TYPE type);

private:
    JOINT_ACTION _cocontractionStateMachine(int emg1, int emg2);
    JOINT_ACTION _propStateMachine(int emg1, int emg2);
    JOINT_ACTION _bubbleCocontractionStateMachine(int emg1, int emg2);
    JOINT_ACTION _bubblePropStateMachine(int emg1, int emg2);

    /// Specific to all controls
    int _emg1;
    int _emg2;
    CONTROL_TYPE _control_type;

    MODE _current_mode;
    MODE _old_mode;
    bool _has_changed_mode;

    int _counts_after_modes;
    int _counter_after_modes;
    int _counts_cocontraction;
    int _counter_cocontraction;
    bool _smoothly_changed_mode;

    /// Specific to coco control
    bool _has_init_cocontraction;
    MODE* _sequence_modes;
    int _sequence_modes_index;
    int _sequence_modes_size;
    int _cocontraction_threshold_emg1;
    int _cocontraction_threshold_emg2;
    int _threshold_emg1;
    int _threshold_emg2;

    /// Specific to bubble coco control
    bool _has_init_bubble_cocontraction;
    int _threshold_high_emg1;
    int _threshold_low_emg1;
    int _threshold_high_emg2;
    int _threshold_low_emg2;
    int _activated_emg;

    /// Specific to bubble modes
    int _counter_before_bubble;
    int _counts_before_bubble;
    int _counter_after_bubble;
    int _counts_after_bubble;
    int _current_bubble_mode;

    /// Specific to prop control
    int* _old_emg1;
    int* _old_emg2;
    bool _has_init_prop;
    bool _prop_control_with_elbow;
    int _deltaT_in_count;
    int _threshold_high_forarm_emg1;
    int _threshold_low_forarm_emg1;
    int _threshold_elbow_emg1;
    int _threshold_high_forarm_emg2;
    int _threshold_low_forarm_emg2;
    int _threshold_elbow_emg2;

    /// Specific to bubble prop control
    int _threshold_high_elbow_emg1;
    int _threshold_low_elbow_emg1;
    int _threshold_high_elbow_emg2;
    int _threshold_low_elbow_emg2;
};

#endif // MYOCONTROL_H

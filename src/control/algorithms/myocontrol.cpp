// Error / debug handling
#ifndef DEBUG
#define QT_NO_DEBUG_OUTPUT
#endif
#include <QCoreApplication>

#include "myocontrol.h"

/**
 * \brief MyoControl::MyoControl Constructor
 */
MyoControl::MyoControl()
{
    _has_init_cocontraction = false;
    _has_init_prop = false;
    _sequence_modes = new MODE[0];
    _sequence_modes_index = 0;
    _old_emg1 = new int[0];
    _old_emg2 = new int[0];
}

/**
 * \brief MyoControl::initCocontractionControl Init the cocontraction control
 * \param sequence The joint's sequence the user have access
 * \param sizeOfSequence
 * \param counts_after_modes Number of cycles after a cocontraction, ie after a mode's change
 * \param counts_cocontraction Number of cycles needed to detect a cocontraction
 * \param threshold_emg1
 * \param threshold_emg2
 * \param cocontraction_threshold_emg1
 * \param cocontraction_threshold_emg2
 */
void MyoControl::initCocontractionControl(MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int threshold_emg1, int threshold_emg2,
    int cocontraction_threshold_emg1, int cocontraction_threshold_emg2)
{

    delete _sequence_modes;
    _sequence_modes_size = sizeOfSequence;
    qDebug("### MYOCONTROL : Size of cocontraction's sequence : %d", _sequence_modes_size);
    _sequence_modes = new MODE[_sequence_modes_size];

    for (int i = 0; i < _sequence_modes_size; i++)
        _sequence_modes[i] = sequence[i];

    _sequence_modes_index = 0;
    _current_mode = _sequence_modes[0];
    _old_mode = _current_mode;
    _counts_after_modes = counts_after_modes;
    _counts_cocontraction = counts_cocontraction;
    _threshold_emg1 = threshold_emg1;
    _threshold_emg2 = threshold_emg2;
    _cocontraction_threshold_emg1 = cocontraction_threshold_emg1;
    _cocontraction_threshold_emg2 = cocontraction_threshold_emg2;

    _counter_after_modes = 0;
    _counter_cocontraction = 0;
    _smoothly_changed_mode = false;

    _emg1 = 0;
    _emg2 = 0;

    _has_init_cocontraction = true;
}

/**
 * \brief MyoControl::_cocontractionStateMachine
 * This control changes joint using cocontraction and activates this joint using one or the other EMG.
 * \param emg1
 * \param emg2
 * \return joint to activate
 */
MyoControl::JOINT_ACTION MyoControl::_cocontractionStateMachine(int emg1, int emg2)
{
    JOINT_ACTION jointAction = NONE;

    if (_has_init_cocontraction) {
        _emg1 = emg1;
        _emg2 = emg2;
        _old_mode = _current_mode;

        switch (_current_mode) {
        case MYO_MODE_ELBOW:
            jointAction = ELBOW_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _counter_after_modes = 0;
                            _smoothly_changed_mode = false;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        _counter_cocontraction = 0;
                        if (_emg1 > _threshold_emg1) {
                            jointAction = ELBOW_DOWN;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = ELBOW_UP;
                        } else {
                            jointAction = ELBOW_STOP;
                        }
                    }
                }
            } else
                _counter_after_modes++;

            break;

        case MYO_MODE_WRIST:
            jointAction = WRIST_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = WRIST_STOP;
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _counter_after_modes = 0;
                            _smoothly_changed_mode = false;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        if (_emg1 > _threshold_emg1) {
                            jointAction = WRIST_BACKWARD;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = WRIST_FORWARD;
                        } else {
                            jointAction = WRIST_STOP;
                        }
                        _counter_cocontraction = 0;
                    }
                }
            } else
                _counter_after_modes++;
            break;
        case MYO_MODE_HAND:
            jointAction = HAND_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = HAND_STOP;
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _smoothly_changed_mode = false;
                            _counter_after_modes = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        if (_emg1 > _threshold_emg1) {
                            jointAction = HAND_OPEN;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = HAND_CLOSE;
                        } else {
                            jointAction = HAND_STOP;
                        }
                        _counter_cocontraction = 0;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_THUMB:
            jointAction = THUMB_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = THUMB_STOP;
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _smoothly_changed_mode = false;
                            _counter_after_modes = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        if (_emg1 > _threshold_emg1) {
                            jointAction = THUMB_OPEN;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = THUMB_CLOSE;
                        } else {
                            jointAction = THUMB_STOP;
                        }
                        _counter_cocontraction = 0;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_FOREFINGER:
            jointAction = FOREFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = FOREFINGER_STOP;
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _smoothly_changed_mode = false;
                            _counter_after_modes = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        if (_emg1 > _threshold_emg1) {
                            jointAction = FOREFINGER_OPEN;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = FOREFINGER_CLOSE;
                        } else {
                            jointAction = FOREFINGER_STOP;
                        }
                        _counter_cocontraction = 0;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_MIDDLEFINGER:
            jointAction = MIDDLEFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = MIDDLEFINGER_STOP;
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _smoothly_changed_mode = false;
                            _counter_after_modes = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        if (_emg1 > _threshold_emg1) {
                            jointAction = MIDDLEFINGER_OPEN;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = MIDDLEFINGER_CLOSE;
                        } else {
                            jointAction = MIDDLEFINGER_STOP;
                        }
                        _counter_cocontraction = 0;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_RINGFINGER:
            jointAction = RINGFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = RINGFINGER_STOP;
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _smoothly_changed_mode = false;
                            _counter_after_modes = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        if (_emg1 > _threshold_emg1) {
                            jointAction = RINGFINGER_OPEN;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = RINGFINGER_CLOSE;
                        } else {
                            jointAction = RINGFINGER_STOP;
                        }
                        _counter_cocontraction = 0;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_LITTLEFINGER:
            jointAction = LITTLEFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_emg1 && _emg2 < _threshold_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check coco
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = LITTLEFINGER_STOP;
                        if (_counter_cocontraction > _counts_cocontraction) {
                            _counter_cocontraction = 0;
                            _smoothly_changed_mode = false;
                            _counter_after_modes = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                        } else {
                            _counter_cocontraction++;
                        }
                    }
                    // Potentielly active function
                    else {
                        if (_emg1 > _threshold_emg1) {
                            jointAction = LITTLEFINGER_OPEN;
                        } else if (_emg2 > _threshold_emg2) {
                            jointAction = LITTLEFINGER_CLOSE;
                        } else {
                            jointAction = LITTLEFINGER_STOP;
                        }
                        _counter_cocontraction = 0;
                    }
                }
            } else
                _counter_after_modes++;

            break;

        default:
            qDebug("### MYOCONTROL WARNING : Default in state machine... Forcinf wrist mode");
            _current_mode = MYO_MODE_WRIST;
            break;
        }

        if (_old_mode != _current_mode)
            _has_changed_mode = true;
        else
            _has_changed_mode = false;

    } else {
        printf("MYOCONTROL ERROR : cocontraction has to be initialized !\n");
    }
    return jointAction;
}

/**
 * \brief MyoControl::initPropControl Init proportionnal control
 * \param sequence The joint's sequence the user have access
 * \param sizeOfSequence
 * \param counts_after_modes Number of cycles after a cocontraction, ie after a mode's change
 * \param counts_cocontraction Number of cycles needed to detect a cocontraction
 * \param threshold_high_forarm_emg1 Threshold to reach in (less than) deltaTInCount after reaching threshold_low_forarm_emg1 to access wrist forward mode
 * \param threshold_low_forarm_emg1 Threshold not to exceed during deltaTInCount to access hand opening mode
 * \param threshold_high_forarm_emg2 Threshold to reach in (less than) deltaTInCount after reaching threshold_low_forarm_emg2 to access wrist backward mode
 * \param threshold_low_forarm_emg2 Threshold not to exceed during deltaTInCount to access hand closing mode
 * \param cocontraction_threshold_emg1
 * \param cocontraction_threshold_emg2
 * \param deltaTInCount Number of cycles checked to detect hand modes (wrist could actually be reached in a few cycles)
 * \param threshold_elbow_emg1
 * \param threshold_elbow_emg2
 */
void MyoControl::initPropControl(MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int threshold_high_forarm_emg1, int threshold_low_forarm_emg1,
    int threshold_high_forarm_emg2, int threshold_low_forarm_emg2, int cocontraction_threshold_emg1, int cocontraction_threshold_emg2, int deltaTInCount,
    int threshold_elbow_emg1, int threshold_elbow_emg2)
{
    delete _sequence_modes;
    _sequence_modes_size = sizeOfSequence;
    qDebug("### MYOCONTROL : Size of prop's sequence : %d", _sequence_modes_size);
    _sequence_modes = new MODE[_sequence_modes_size];

    for (int i = 0; i < _sequence_modes_size; i++)
        _sequence_modes[i] = sequence[i];

    _sequence_modes_index = 0;
    _current_mode = _sequence_modes[0];
    _old_mode = _current_mode;

    delete _old_emg1;
    delete _old_emg2;
    _deltaT_in_count = deltaTInCount;
    _old_emg1 = new int[_deltaT_in_count];
    _old_emg2 = new int[_deltaT_in_count];

    for (int i = 0; i < _deltaT_in_count; i++) {
        _old_emg1[i] = 0;
        _old_emg2[i] = 0;
    }
    _emg1 = 0;
    _emg2 = 0;

    _counts_after_modes = counts_after_modes;
    _counts_cocontraction = counts_cocontraction;
    _smoothly_changed_mode = false;
    _cocontraction_threshold_emg1 = cocontraction_threshold_emg1;
    _cocontraction_threshold_emg2 = cocontraction_threshold_emg2;
    _threshold_high_forarm_emg1 = threshold_high_forarm_emg1;
    _threshold_low_forarm_emg1 = threshold_low_forarm_emg1;
    _threshold_high_forarm_emg2 = threshold_high_forarm_emg2;
    _threshold_low_forarm_emg2 = threshold_low_forarm_emg2;
    _threshold_elbow_emg1 = threshold_elbow_emg1;
    _threshold_elbow_emg2 = threshold_elbow_emg2;

    _has_init_prop = true;
}

/**
 * \brief MyoControl::_propStateMachine
 * This control changes between forearm and elbow using cocontraction. In elbow mode, it uses contraction to activate flexion/extension.
 * In forearm mode, a quick contraction will activate wrist movement, a slow one the hand.
 * \param emg1
 * \param emg2
 * \return
 */
MyoControl::JOINT_ACTION MyoControl::_propStateMachine(int emg1, int emg2)
{
    JOINT_ACTION jointAction = NONE;

    if (_has_init_prop) {
        for (int i = _deltaT_in_count - 1; i > 0; i--) {
            _old_emg1[i] = _old_emg1[i - 1];
            _old_emg2[i] = _old_emg2[i - 1];
        }
        _old_emg1[0] = _emg1;
        _old_emg2[0] = _emg2;
        _emg1 = emg1;
        _emg2 = emg2;
        _old_mode = _current_mode;

        switch (_current_mode) {
        case MYO_MODE_ELBOW:
            jointAction = ELBOW_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_elbow_emg1 && _emg2 < _threshold_elbow_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check for cocontraction
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        jointAction = ELBOW_STOP;
                        if (_counter_cocontraction < _counts_cocontraction)
                            _counter_cocontraction++;
                        else {
                            // STOP ELBOW / GO TO FORARM
                            _counter_after_modes = 0;
                            _counter_cocontraction = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                            _smoothly_changed_mode = false;
                        }
                    } else {
                        _counter_cocontraction = 0;
                        // Check for contraction
                        if (_emg1 > _threshold_elbow_emg1) {
                            jointAction = ELBOW_DOWN;
                        } else if (_emg2 > _threshold_elbow_emg2) {
                            jointAction = ELBOW_UP;
                        } else {
                            jointAction = ELBOW_STOP;
                        }
                    }
                }
            } else
                _counter_after_modes++;
            break;

        case MYO_MODE_FORARM:
            jointAction = FORARM_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_forarm_emg1 && _emg2 < _threshold_low_forarm_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check for cocontraction if elbow is used
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        if (_counter_cocontraction < _counts_cocontraction)
                            _counter_cocontraction++;
                        else {
                            // STOP FOREARM / GO TO ELBOW
                            _counter_after_modes = 0;
                            _counter_cocontraction = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                            _smoothly_changed_mode = false;
                        }
                    } else {
                        _counter_cocontraction = 0;
                        // Check for contraction
                        if (_emg1 > _threshold_low_forarm_emg1) {
                            // Check in which mode to go
                            bool shouldPassInHandMode = true;
                            bool shouldPassInWristMode = false;

                            if (_emg1 > _threshold_high_forarm_emg1) {
                                shouldPassInWristMode = true;
                                shouldPassInHandMode = false;
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg1[i] < _threshold_high_forarm_emg1) {
                                        shouldPassInWristMode = false;
                                        break;
                                    }
                                }
                            } else {
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg1[i] < _threshold_low_forarm_emg1) {
                                        shouldPassInHandMode = false;
                                        break;
                                    }
                                }
                            }

                            if (shouldPassInWristMode)
                                _current_mode = MYO_MODE_WRIST_FORWARDING;
                            else if (shouldPassInHandMode)
                                _current_mode = MYO_MODE_HAND_OPENING;
                        }

                        if (_emg2 > _threshold_low_forarm_emg2) {
                            bool shouldPassInHandMode = true;
                            bool shouldPassInWristMode = false;

                            if (_emg2 > _threshold_high_forarm_emg2) {
                                shouldPassInWristMode = true;
                                shouldPassInHandMode = false;
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg2[i] < _threshold_high_forarm_emg2) {
                                        shouldPassInWristMode = false;
                                        break;
                                    }
                                }
                            } else {
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg2[i] < _threshold_low_forarm_emg2) {
                                        shouldPassInHandMode = false;
                                        break;
                                    }
                                }
                            }

                            if (shouldPassInWristMode)
                                _current_mode = MYO_MODE_WRIST_BACKWARDING;
                            else if (shouldPassInHandMode)
                                _current_mode = MYO_MODE_HAND_CLOSING;
                        }
                    }
                }
            } else
                _counter_after_modes++;
            break;

        case MYO_MODE_HAND_CLOSING:
            if (_emg2 < _threshold_low_forarm_emg2) {
                jointAction = HAND_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = HAND_CLOSE;
            }
            break;

        case MYO_MODE_HAND_OPENING:
            if (_emg1 < _threshold_low_forarm_emg1) {
                jointAction = HAND_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = HAND_OPEN;
            }
            break;

        case MYO_MODE_WRIST_BACKWARDING:
            if (_emg2 < _threshold_low_forarm_emg2) {
                jointAction = WRIST_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = WRIST_BACKWARD;
            }
            break;

        case MYO_MODE_WRIST_FORWARDING:
            if (_emg1 < _threshold_low_forarm_emg1) {
                jointAction = WRIST_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = WRIST_FORWARD;
            }
            break;

        default:
            printf("MYOCONTROL WARNING : State machine is forcing forarm mode.\n");
            _current_mode = MYO_MODE_FORARM;
            break;
        }

        if (_old_mode != _current_mode)
            _has_changed_mode = true;
        else
            _has_changed_mode = false;

    } else {
        printf("MYOCONTROL ERROR : Init proportionnal control before using it...\n");
    }
    return jointAction;
}

/**
 * \brief MyoControl::initBubbleCocontractionControl Init cocontraction control with bubble smoothing
 * \param sequence The joint's sequence the user have access
 * \param sizeOfSequence
 * \param counts_after_modes Number of cycles after a cocontraction, ie after a mode's change
 * \param counts_cocontraction Number of cycles needed to detect a cocontraction
 * \param counts_before_bubble Number of cycles needed above high thresholds to stop checking cocontraction and start comparing the low thresholds
 * \param counts_after_bubble Number of cycles needed below low threshold to check again high thresholds and cococontraction
 * \param threshold_high_emg1 The high threshold to overtake to activate fn1
 * \param threshold_low_emg1 The low threshold to stay above to continue activating fn1
 * \param threshold_high_emg2 The high threshold to overtake to activate fn2
 * \param threshold_low_emg2 The low threshold to stay above to continue activating fn2
 * \param cocontraction_threshold_emg1
 * \param cocontraction_threshold_emg2
 */
void MyoControl::initBubbleCocontractionControl(const MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int counts_before_bubble, int counts_after_bubble,
    int threshold_high_emg1, int threshold_low_emg1, int threshold_high_emg2, int threshold_low_emg2, int cocontraction_threshold_emg1,
    int cocontraction_threshold_emg2)
{

    delete _sequence_modes;
    _sequence_modes_size = sizeOfSequence;
    qDebug("### MYOCONTROL : Size of cocontraction's sequence : %d", _sequence_modes_size);
    _sequence_modes = new MODE[_sequence_modes_size];

    for (int i = 0; i < _sequence_modes_size; i++)
        _sequence_modes[i] = sequence[i];

    _sequence_modes_index = 0;
    _current_mode = _sequence_modes[0];
    _old_mode = _current_mode;
    _counts_after_modes = counts_after_modes;
    _counts_cocontraction = counts_cocontraction;
    _counts_before_bubble = counts_before_bubble;
    _counts_after_bubble = counts_after_bubble;

    _threshold_high_emg1 = threshold_high_emg1;
    _threshold_high_emg2 = threshold_high_emg2;
    _threshold_low_emg1 = threshold_low_emg1;
    _threshold_low_emg2 = threshold_low_emg2;
    _cocontraction_threshold_emg1 = cocontraction_threshold_emg1;
    _cocontraction_threshold_emg2 = cocontraction_threshold_emg2;

    _counter_after_modes = 0;
    _counter_cocontraction = 0;
    _smoothly_changed_mode = false;
    _current_bubble_mode = HAS_TO_CHECK_COCO;

    _emg1 = 0;
    _emg2 = 0;
    _activated_emg = 0;

    _has_init_bubble_cocontraction = true;
}

/**
 * \brief MyoControl::_bubbleCocontractionStateMachine
 * This control changes joint using cocontraction and activates this joint using one or the other EMG.
 * In every modes, a low threshold is used once the high one is "really" overtaken to reduce muscular fatigue.
 * \param emg1
 * \param emg2
 * \return
 */
MyoControl::JOINT_ACTION MyoControl::_bubbleCocontractionStateMachine(int emg1, int emg2)
{
    JOINT_ACTION jointAction = NONE;

    if (_has_init_bubble_cocontraction) {
        _emg1 = emg1;
        _emg2 = emg2;
        _old_mode = _current_mode;

        switch (_current_mode) {
        case MYO_MODE_ELBOW:
            jointAction = ELBOW_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = ELBOW_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = ELBOW_DOWN;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = ELBOW_UP;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = ELBOW_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = ELBOW_DOWN;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = ELBOW_UP;
                        } else {
                            _counter_after_bubble++;
                            jointAction = ELBOW_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;

        case MYO_MODE_WRIST:
            jointAction = WRIST_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = WRIST_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = WRIST_BACKWARD;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = WRIST_FORWARD;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = WRIST_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = WRIST_BACKWARD;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = WRIST_FORWARD;
                        } else {
                            _counter_after_bubble++;
                            jointAction = WRIST_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;

        case MYO_MODE_HAND:
            jointAction = HAND_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = HAND_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = HAND_OPEN;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = HAND_CLOSE;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = HAND_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = HAND_OPEN;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = HAND_CLOSE;
                        } else {
                            _counter_after_bubble++;
                            jointAction = HAND_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;

        case MYO_MODE_THUMB:
            jointAction = THUMB_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = THUMB_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = THUMB_OPEN;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = THUMB_CLOSE;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = THUMB_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = THUMB_OPEN;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = THUMB_CLOSE;
                        } else {
                            _counter_after_bubble++;
                            jointAction = THUMB_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_FOREFINGER:
            jointAction = FOREFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = FOREFINGER_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = FOREFINGER_OPEN;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = FOREFINGER_CLOSE;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = FOREFINGER_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = FOREFINGER_OPEN;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = FOREFINGER_CLOSE;
                        } else {
                            _counter_after_bubble++;
                            jointAction = FOREFINGER_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_MIDDLEFINGER:
            jointAction = MIDDLEFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = MIDDLEFINGER_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = MIDDLEFINGER_OPEN;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = MIDDLEFINGER_CLOSE;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = MIDDLEFINGER_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = MIDDLEFINGER_OPEN;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = MIDDLEFINGER_CLOSE;
                        } else {
                            _counter_after_bubble++;
                            jointAction = MIDDLEFINGER_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_RINGFINGER:
            jointAction = RINGFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = RINGFINGER_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = RINGFINGER_OPEN;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = RINGFINGER_CLOSE;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = RINGFINGER_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = RINGFINGER_OPEN;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = RINGFINGER_CLOSE;
                        } else {
                            _counter_after_bubble++;
                            jointAction = RINGFINGER_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;
        case MYO_MODE_LITTLEFINGER:
            jointAction = LITTLEFINGER_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_emg1 && _emg2 < _threshold_low_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = LITTLEFINGER_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = LITTLEFINGER_OPEN;
                            } else if (_emg2 > _threshold_high_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = LITTLEFINGER_CLOSE;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = LITTLEFINGER_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = LITTLEFINGER_OPEN;
                        } else if (_emg2 > _threshold_low_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = LITTLEFINGER_CLOSE;
                        } else {
                            _counter_after_bubble++;
                            jointAction = LITTLEFINGER_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;

        default:
            qDebug("### MYOCONTROL WARNING : Default in state machine... Forcinf wrist mode");
            _current_mode = MYO_MODE_WRIST;
            break;
        }

        if (_old_mode != _current_mode)
            _has_changed_mode = true;
        else
            _has_changed_mode = false;

    } else {
        printf("MYOCONTROL ERROR : cocontraction has to be initialized !\n");
    }
    return jointAction;
}

/**
 * \brief MyoControl::initBubblePropControl Init proportionnal control with bubble smoothing
 * \param sequence The joint's sequence the user have access
 * \param sizeOfSequence
 * \param counts_after_modes Number of cycles after a cocontraction, ie after a mode's change
 * \param counts_cocontraction Number of cycles needed to detect a cocontraction
 * \param counts_before_bubble Number of cycles needed above high thresholds to stop checking cocontraction and start comparing the low thresholds
 * \param counts_after_bubble Number of cycles needed below low threshold to check again high thresholds and cococontraction
 * \param threshold_high_forarm_emg1 Threshold to reach in (less than) deltaTInCount after reaching threshold_low_forarm_emg1 to access wrist forward mode
 * \param threshold_low_forarm_emg1 Threshold not to exceed during deltaTInCount to access hand opening mode
 * \param threshold_high_forarm_emg2 Threshold to reach in (less than) deltaTInCount after reaching threshold_low_forarm_emg2 to access wrist backward mode
 * \param threshold_low_forarm_emg2 Threshold not to exceed during deltaTInCount to access hand closing mode
 * \param cocontraction_threshold_emg1
 * \param cocontraction_threshold_emg2
 * \param deltaTInCount Number of cycles checked to detect hand modes (wrist could actually be reached in a few cycles)
 * \param threshold_high_elbow_emg1 The high threshold to overtake to activate flexion
 * \param threshold_low_elbow_emg1 The low threshold to stay above to continue activating flexion
 * \param threshold_high_elbow_emg2 The high threshold to overtake to activate extension
 * \param threshold_low_elbow_emg2 The low threshold to stay above to continue activating extension
 */
void MyoControl::initBubblePropControl(MODE sequence[], int sizeOfSequence, int counts_after_modes, int counts_cocontraction, int counts_before_bubble, int counts_after_bubble,
    int threshold_high_forarm_emg1, int threshold_low_forarm_emg1, int threshold_high_forarm_emg2, int threshold_low_forarm_emg2, int cocontraction_threshold_emg1,
    int cocontraction_threshold_emg2, int deltaTInCount, int threshold_high_elbow_emg1, int threshold_low_elbow_emg1, int threshold_high_elbow_emg2,
    int threshold_low_elbow_emg2)
{
    delete _sequence_modes;
    _sequence_modes_size = sizeOfSequence;
    qDebug("### MYOCONTROL : Size of prop's sequence : %d", _sequence_modes_size);
    _sequence_modes = new MODE[_sequence_modes_size];

    for (int i = 0; i < _sequence_modes_size; i++)
        _sequence_modes[i] = sequence[i];

    _sequence_modes_index = 0;
    _current_mode = _sequence_modes[0];
    _old_mode = _current_mode;

    delete _old_emg1;
    delete _old_emg2;
    _deltaT_in_count = deltaTInCount;
    _old_emg1 = new int[_deltaT_in_count];
    _old_emg2 = new int[_deltaT_in_count];

    for (int i = 0; i < _deltaT_in_count; i++) {
        _old_emg1[i] = 0;
        _old_emg2[i] = 0;
    }
    _emg1 = 0;
    _emg2 = 0;

    if (sequence[0] == MYO_MODE_ELBOW)
        _current_mode = MYO_MODE_ELBOW;
    else
        _current_mode = MYO_MODE_FORARM;

    _old_mode = _current_mode;
    _counts_after_modes = counts_after_modes;
    _counts_cocontraction = counts_cocontraction;
    _smoothly_changed_mode = false;
    _counts_before_bubble = counts_before_bubble;
    _counts_after_bubble = counts_after_bubble;
    _cocontraction_threshold_emg1 = cocontraction_threshold_emg1;
    _cocontraction_threshold_emg2 = cocontraction_threshold_emg2;
    _threshold_high_forarm_emg1 = threshold_high_forarm_emg1;
    _threshold_low_forarm_emg1 = threshold_low_forarm_emg1;
    _threshold_high_forarm_emg2 = threshold_high_forarm_emg2;
    _threshold_low_forarm_emg2 = threshold_low_forarm_emg2;

    _threshold_high_elbow_emg1 = threshold_high_elbow_emg1;
    _threshold_low_elbow_emg1 = threshold_low_elbow_emg1;
    _threshold_high_elbow_emg2 = threshold_high_elbow_emg2;
    _threshold_low_elbow_emg2 = threshold_low_elbow_emg2;

    _has_init_prop = true;
}

/**
 * \brief MyoControl::_bubblePropStateMachine
 * This control changes between forearm and elbow using cocontraction.
 * In elbow mode, it uses high thresholds to activate flexion/extension and low thresholds to coninue the activation.
 * In forearm mode, a quick contraction will activate wrist movement, a slow one the hand.
 * \param emg1
 * \param emg2
 * \return
 */
MyoControl::JOINT_ACTION MyoControl::_bubblePropStateMachine(int emg1, int emg2)
{
    JOINT_ACTION jointAction = NONE;

    if (_has_init_prop) {
        for (int i = _deltaT_in_count - 1; i > 0; i--) {
            _old_emg1[i] = _old_emg1[i - 1];
            _old_emg2[i] = _old_emg2[i - 1];
        }
        _old_emg1[0] = _emg1;
        _old_emg2[0] = _emg2;
        _emg1 = emg1;
        _emg2 = emg2;
        _old_mode = _current_mode;

        switch (_current_mode) {
        case MYO_MODE_ELBOW:
            jointAction = ELBOW_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_elbow_emg1 && _emg2 < _threshold_low_elbow_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    switch (_current_bubble_mode) {
                    case HAS_TO_CHECK_COCO:
                        // Check coco
                        if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                            jointAction = ELBOW_STOP;
                            if (_counter_cocontraction > _counts_cocontraction) {
                                _counter_cocontraction = 0;
                                _counter_after_modes = 0;
                                _smoothly_changed_mode = false;
                                _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                                _current_mode = _sequence_modes[_sequence_modes_index];
                            } else {
                                _counter_cocontraction++;
                            }
                        }
                        // Potentielly active function
                        else {
                            _counter_cocontraction = 0;
                            if (_emg1 > _threshold_high_elbow_emg1) {
                                _counter_before_bubble++;
                                _activated_emg = 1;
                                jointAction = ELBOW_DOWN;
                            } else if (_emg2 > _threshold_high_elbow_emg2) {
                                _counter_before_bubble++;
                                _activated_emg = 2;
                                jointAction = ELBOW_UP;
                            } else {
                                _counter_before_bubble = 0;
                                jointAction = ELBOW_STOP;
                            }

                            if (_counter_before_bubble > _counts_before_bubble) {
                                _current_bubble_mode = IS_ACTIVATED;
                                _counter_after_bubble = 0;
                            }
                        }

                        break;

                    case IS_ACTIVATED:
                        if (_emg1 > _threshold_low_elbow_emg1 && _activated_emg == 1) {
                            _counter_after_bubble = 0;
                            jointAction = ELBOW_DOWN;
                        } else if (_emg2 > _threshold_low_elbow_emg2 && _activated_emg == 2) {
                            _counter_after_bubble = 0;
                            jointAction = ELBOW_UP;
                        } else {
                            _counter_after_bubble++;
                            jointAction = ELBOW_STOP;
                        }

                        if (_counter_after_bubble > _counts_after_bubble) {
                            _current_bubble_mode = HAS_TO_CHECK_COCO;
                        }
                        break;
                    }
                }
            } else
                _counter_after_modes++;

            break;

        case MYO_MODE_FORARM:
            jointAction = FORARM_STOP;
            if (_counter_after_modes > _counts_after_modes) {
                // Wait for signal to go down before starting again a new mode
                if (_emg1 < _threshold_low_forarm_emg1 && _emg2 < _threshold_low_forarm_emg2) {
                    _smoothly_changed_mode = true;
                }
                if (_smoothly_changed_mode) {
                    // Check for cocontraction if elbow is used
                    if (_emg1 > _cocontraction_threshold_emg1 && _emg2 > _cocontraction_threshold_emg2) {
                        if (_counter_cocontraction < _counts_cocontraction)
                            _counter_cocontraction++;
                        else {
                            // STOP FOREARM / GO TO ELBOW
                            _counter_after_modes = 0;
                            _counter_cocontraction = 0;
                            _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
                            _current_mode = _sequence_modes[_sequence_modes_index];
                            _smoothly_changed_mode = false;
                        }
                    } else {
                        _counter_cocontraction = 0;
                        // Check for forearm activation
                        if (_emg1 > _threshold_low_forarm_emg1) {
                            bool shouldPassInHandMode = true;
                            bool shouldPassInWristMode = false;

                            if (_emg1 > _threshold_high_forarm_emg1) {
                                shouldPassInWristMode = true;
                                shouldPassInHandMode = false;
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg1[i] < _threshold_high_forarm_emg1) {
                                        shouldPassInWristMode = false;
                                        break;
                                    }
                                }
                            } else {
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg1[i] < _threshold_low_forarm_emg1) {
                                        shouldPassInHandMode = false;
                                        break;
                                    }
                                }
                            }
                            //
                            if (shouldPassInWristMode)
                                _current_mode = MYO_MODE_WRIST_FORWARDING;
                            else if (shouldPassInHandMode)
                                _current_mode = MYO_MODE_HAND_OPENING;
                        }

                        if (_emg2 > _threshold_low_forarm_emg2) {
                            bool shouldPassInHandMode = true;
                            bool shouldPassInWristMode = false;

                            if (_emg2 > _threshold_high_forarm_emg2) {
                                shouldPassInWristMode = true;
                                shouldPassInHandMode = false;
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg2[i] < _threshold_high_forarm_emg2) {
                                        shouldPassInWristMode = false;
                                        break;
                                    }
                                }
                            } else {
                                for (int i = 0; i < _deltaT_in_count; i++) {
                                    if (_old_emg2[i] < _threshold_low_forarm_emg2) {
                                        shouldPassInHandMode = false;
                                        break;
                                    }
                                }
                            }

                            if (shouldPassInWristMode)
                                _current_mode = MYO_MODE_WRIST_BACKWARDING;
                            else if (shouldPassInHandMode)
                                _current_mode = MYO_MODE_HAND_CLOSING;
                        }
                    }
                }
            } else
                _counter_after_modes++;
            break;

        case MYO_MODE_HAND_CLOSING:
            if (_emg2 < _threshold_low_forarm_emg2) {
                jointAction = HAND_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = HAND_CLOSE;
            }
            break;

        case MYO_MODE_HAND_OPENING:
            if (_emg1 < _threshold_low_forarm_emg1) {
                jointAction = HAND_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = HAND_OPEN;
            }
            break;

        case MYO_MODE_WRIST_BACKWARDING:
            if (_emg2 < _threshold_low_forarm_emg2) {
                jointAction = WRIST_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = WRIST_BACKWARD;
            }
            break;

        case MYO_MODE_WRIST_FORWARDING:
            if (_emg1 < _threshold_low_forarm_emg1) {
                jointAction = WRIST_STOP;
                _current_mode = MYO_MODE_FORARM;
            } else {
                jointAction = WRIST_FORWARD;
            }
            break;

        default:
            printf("MYOCONTROL WARNING : State machine is forcing forarm mode.\n");
            _current_mode = MYO_MODE_FORARM;
            break;
        }

        if (_old_mode != _current_mode)
            _has_changed_mode = true;
        else
            _has_changed_mode = false;

    } else {
        printf("MYOCONTROL ERROR : Init proportionnal control before using it...\n");
    }
    return jointAction;
}

/**
 * \brief MyoControl::setControlType
 * \param type
 */
void MyoControl::setControlType(CONTROL_TYPE type)
{
    _control_type = type;
}

/**
 * \brief MyoControl::init
 * NB : Set control type before !
 */
void MyoControl::init()
{
    switch (_control_type) {
    case NO_CONTROL:
        printf("MYOCONTROL ERROR : Please set a control type !\n");
        break;

    case COCO_CONTROL:
        initCocontractionControl(_sequence_modes, _sequence_modes_size, _counter_after_modes, _counts_cocontraction, _threshold_emg1, _threshold_emg2, _cocontraction_threshold_emg1,
            _cocontraction_threshold_emg2);
        break;

    case BUBBLE_COCO_CONTROL:
        initBubbleCocontractionControl(_sequence_modes, _sequence_modes_size, _counter_after_modes, _counts_cocontraction, _counts_before_bubble, _counts_after_bubble, _threshold_high_emg1,
            _threshold_low_emg1, _threshold_high_emg2, _threshold_low_emg2, _cocontraction_threshold_emg1, _cocontraction_threshold_emg2);
        break;

    case PROP_CONTROL:
        initPropControl(_sequence_modes, _sequence_modes_size, _counter_after_modes, _counts_cocontraction, _threshold_high_forarm_emg1, _threshold_low_forarm_emg1,
            _threshold_high_forarm_emg2, _threshold_low_forarm_emg2, _cocontraction_threshold_emg1, _cocontraction_threshold_emg2,
            _deltaT_in_count, _threshold_elbow_emg1, _threshold_elbow_emg2);
        break;
    case BUBBLE_PROP_CONTROL:
        initBubblePropControl(_sequence_modes, _sequence_modes_size, _counter_after_modes, _counts_cocontraction, _counts_before_bubble, _counts_after_bubble,
            _threshold_high_forarm_emg1, _threshold_low_forarm_emg1, _threshold_high_forarm_emg2, _threshold_low_forarm_emg2, _cocontraction_threshold_emg1,
            _cocontraction_threshold_emg2, _deltaT_in_count, _threshold_high_elbow_emg1, _threshold_low_elbow_emg1, _threshold_high_elbow_emg2, _threshold_low_elbow_emg2);
        break;
    default:
        printf("MYOCONTROL ERROR : No reason to be here\n");
        break;
    }
}

/**
 * \brief MyoControl::getjointAction Calls the set control type to return the joint to ativate depending on the EMGs
 * \param emg1
 * \param emg2
 * \return The joint to activate
 */
MyoControl::JOINT_ACTION MyoControl::getJointAction(int emg1, int emg2)
{
    if (_control_type == NO_CONTROL) {
        printf("MYOCONTROL ERROR : Please choose a control type !\n");
    } else {
        switch (_control_type) {
        case COCO_CONTROL:
            return _cocontractionStateMachine(emg1, emg2);

        case BUBBLE_COCO_CONTROL:
            return _bubbleCocontractionStateMachine(emg1, emg2);

        case PROP_CONTROL:
            return _propStateMachine(emg1, emg2);

        case BUBBLE_PROP_CONTROL:
            return _bubblePropStateMachine(emg1, emg2);

        default:
            return NONE;
        }
    }
    return NONE;
}

/**
 * \brief MyoControl::jumpThisMode Allows to manually jump a mode
 */
void MyoControl::jumpThisMode()
{
    _sequence_modes_index = (_sequence_modes_index + 1) % _sequence_modes_size;
    _current_mode = _sequence_modes[_sequence_modes_index];
}

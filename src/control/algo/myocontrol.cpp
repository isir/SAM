#include "myocontrol.h"
#include <stdexcept>

namespace MyoControl {

Classifier::Classifier(std::vector<Action> actions, EMGThresholds thresholds, unsigned int counts_after_mode_change, unsigned int counts_cocontraction)
    : _actions(actions)
    , _it(_actions.begin())
    , _thresholds(thresholds)
    , _counts_after_mode_change(counts_after_mode_change)
    , _counts_cocontraction(counts_cocontraction)
    , _mode_changed(false)
{
    if (_actions.size() == 0) {
        throw std::runtime_error("Empty action list");
    }
}

Classifier::~Classifier()
{
    //_it->on_none();
}

void Classifier::previous()
{
    if (_it == _actions.begin()) {
        _it = _actions.end() - 1;
    } else {
        --_it;
    }
    _mode_changed = true;
}

void Classifier::next()
{
    if (_it == _actions.end() - 1) {
        _it = _actions.begin();
    } else {
        ++_it;
    }
    _mode_changed = true;
}

int Classifier::current_index()
{
    return _it - _actions.begin();
}

bool Classifier::has_changed_mode()
{
    bool ret = _mode_changed;
    _mode_changed = false;
    return ret;
}

BubbleCocoClassifier::BubbleCocoClassifier(std::vector<Action> actions, EMGThresholds thresholds, unsigned int counts_after_mode_change, unsigned int counts_cocontraction, unsigned int counts_before_bubble, unsigned int counts_after_bubble)
    : Classifier(actions, thresholds, counts_after_mode_change, counts_cocontraction)
    , _counts_before_bubble(counts_before_bubble)
    , _counts_after_bubble(counts_after_bubble)
{
}

void BubbleCocoClassifier::process(int forward_emg, int backward_emg, bool)
{
    static bool smoothly_changed_mode = false;
    static bool has_to_check_cocontraction = false;
    static unsigned int mode_changed_counter = 0;
    static unsigned int cocontraction_counter = 0;
    static unsigned int before_bubble_counter = 0;
    static unsigned int after_bubble_counter = 0;
    static unsigned int activated_emg = 1;

    if (mode_changed_counter > _counts_after_mode_change) {
        if (forward_emg < _thresholds.forward_lower && backward_emg < _thresholds.backward_lower) {
            smoothly_changed_mode = true;
        }
        if (smoothly_changed_mode) {
            if (has_to_check_cocontraction) {
                if (forward_emg > _thresholds.forward_cocontraction && backward_emg > _thresholds.backward_cocontraction) {
                    if (cocontraction_counter > _counts_cocontraction) {
                        cocontraction_counter = 0;
                        mode_changed_counter = 0;
                        smoothly_changed_mode = false;
                        _it->on_none();
                        next();
                    } else {
                        cocontraction_counter++;
                    }
                }
                // Potentielly active function
                else {
                    cocontraction_counter = 0;
                    if (forward_emg > _thresholds.forward_upper) {
                        ++before_bubble_counter;
                        activated_emg = 1;
                        _it->on_forward();
                    } else if (backward_emg > _thresholds.backward_upper) {
                        ++before_bubble_counter;
                        activated_emg = 2;
                        _it->on_backward();
                    } else {
                        before_bubble_counter = 0;
                        _it->on_none();
                    }

                    if (before_bubble_counter > _counts_before_bubble) {
                        has_to_check_cocontraction = false;
                        after_bubble_counter = 0;
                    }
                }
            } else {
                if (forward_emg > _thresholds.forward_lower && activated_emg == 1) {
                    after_bubble_counter = 0;
                    _it->on_forward();
                } else if (backward_emg > _thresholds.backward_lower && activated_emg == 2) {
                    after_bubble_counter = 0;
                    _it->on_backward();
                } else {
                    ++after_bubble_counter;
                    _it->on_none();
                }

                if (after_bubble_counter > _counts_after_bubble) {
                    has_to_check_cocontraction = true;
                }
            }
        }
    } else
        mode_changed_counter++;
}


QuantumClassifier::QuantumClassifier(std::vector<Action> actions, EMGThresholds thresholds, unsigned int counts_after_mode_change, unsigned int counts_btn, unsigned int counts_before_bubble, unsigned int counts_after_bubble)
    : Classifier(actions, thresholds, counts_after_mode_change, counts_btn)
    , _counts_before_bubble(counts_before_bubble)
    , _counts_after_bubble(counts_after_bubble)
{
}

void QuantumClassifier::process(int forward_emg, int backward_emg, bool btn)
{
    static bool smoothly_changed_mode = false;
    static bool has_to_check_btn = false;
    static unsigned int mode_changed_counter = 0;
    static unsigned int btn_counter = 0;
    static unsigned int before_bubble_counter = 0;
    static unsigned int after_bubble_counter = 0;
    static unsigned int activated_emg = 1;

    if (mode_changed_counter > _counts_after_mode_change) { //si on a laissé assez de temps après le changement de posture
        if (!btn) { //si on a bien relaché le bouton
            smoothly_changed_mode = true;
        }
        if (smoothly_changed_mode) { //si le bouton a bien été relaché depuis le dernier changement de mode
            if (has_to_check_btn) { //s'il s'est écoulé suffisamment de temps depuis la dernière contraction
                if (btn) { //si appui sur le bouton
                    if (btn_counter > _counts_cocontraction) { //si le bouton est maintenu assez longtemps
                        btn_counter = 0;
                        mode_changed_counter = 0;
                        smoothly_changed_mode = false;
                        _it->on_none(); //changement de posture (malgré le nom on_none)
                        next();
                    } else {
                        btn_counter++;
                    }
                }
                else {
                    btn_counter = 0;
                    if (forward_emg > _thresholds.forward_upper) {
                        ++before_bubble_counter;
                        activated_emg = 1;
                        //_it->on_forward(); //fermer main rapidement
                        _it->on_forward_slow(); //fermer main lentement
//                    } else if (forward_emg > _thresholds.backward_lower) {
//                        ++before_bubble_counter;
//                        activated_emg = 1;
//                        _it->on_forward_slow(); //fermer main lentement
                    } else if (backward_emg > _thresholds.backward_upper) {
                        ++before_bubble_counter;
                        activated_emg = 2;
                        _it->on_backward(); //ouvrir main rapidement
                    } else if (backward_emg > _thresholds.backward_lower) {
                        ++before_bubble_counter;
                        activated_emg = 2;
                        _it->on_backward_slow(); //ouvrir main lentement
                    } else {
                        before_bubble_counter = 0;
                    }

                    if (before_bubble_counter > _counts_before_bubble) {
                        has_to_check_btn = false;
                        after_bubble_counter = 0;
                    }
                }
            } else {
                if (forward_emg > _thresholds.forward_upper && activated_emg == 1) {
                    after_bubble_counter = 0;
                    //_it->on_forward(); //fermer main rapidement
                    _it->on_forward_slow(); //fermer main lentement
//                } else if (forward_emg > _thresholds.backward_lower && activated_emg == 1) {
//                    after_bubble_counter = 0;
//                    _it->on_forward_slow(); //fermer main lentement
                } else if (backward_emg > _thresholds.backward_upper && activated_emg == 2) {
                    after_bubble_counter = 0;
                    _it->on_backward(); //ouvrir main rapidement
                } else if (backward_emg > _thresholds.backward_lower && activated_emg == 2) {
                    after_bubble_counter = 0;
                    _it->on_backward_slow(); //ouvrir main lentement
                } else {
                    ++after_bubble_counter;
                }

                if (after_bubble_counter > _counts_after_bubble) {
                    has_to_check_btn = true;
                }
            }
        }
    } else
        mode_changed_counter++;
}

}

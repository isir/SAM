#ifndef MYOCONTROL_H
#define MYOCONTROL_H

#include <functional>
#include <string>
#include <vector>

namespace MyoControl {

class Action {
public:
    Action(std::string name, std::function<void(void)>&& on_forward, std::function<void(void)>&& on_backward, std::function<void(void)>&& on_none)
        : _name(name)
        , _on_forward(on_forward)
        , _on_forward_slow(on_forward)
        , _on_backward(on_backward)
        , _on_backward_slow(on_backward)
        , _on_none(on_none)
    {
    }

    Action(std::string name, std::function<void(void)>&& on_forward, std::function<void(void)>&& on_forward_slow, std::function<void(void)>&& on_backward, std::function<void(void)>&& on_backward_slow, std::function<void(void)>&& on_none)
        : _name(name)
        , _on_forward(on_forward)
        , _on_forward_slow(on_forward_slow)
        , _on_backward(on_backward)
        , _on_backward_slow(on_backward_slow)
        , _on_none(on_none)
    {
    }

    void on_forward() { _on_forward(); }
    void on_forward_slow() { _on_forward_slow(); }
    void on_backward() { _on_backward(); }
    void on_backward_slow() { _on_backward_slow(); }
    void on_none() { _on_none(); }

private:
    std::string _name;
    std::function<void(void)> _on_forward;
    std::function<void(void)> _on_forward_slow;
    std::function<void(void)> _on_backward;
    std::function<void(void)> _on_backward_slow;
    std::function<void(void)> _on_none;
};

class EMGThresholds {
public:
    EMGThresholds(int f_up, int f_low, int f_co, int b_up, int b_low, int b_co)
        : forward_upper(f_up)
        , forward_lower(f_low)
        , forward_cocontraction(f_co)
        , backward_upper(b_up)
        , backward_lower(b_low)
        , backward_cocontraction(b_co)
    {
    }

    int forward_upper;
    int forward_lower;
    int forward_cocontraction;

    int backward_upper;
    int backward_lower;
    int backward_cocontraction;
};

class Classifier {
public:
    Classifier(std::vector<Action> actions, EMGThresholds thresholds, unsigned int counts_after_mode_change, unsigned int counts_cocontraction);
    virtual ~Classifier();

    virtual void process(int forward_emg, int backward_emg, bool btn=0) = 0;

    void previous();
    void next();

    int current_index();

    bool has_changed_mode();

protected:
    std::vector<Action> _actions;
    std::vector<Action>::iterator _it;

    EMGThresholds _thresholds;

    unsigned int _counts_after_mode_change;
    unsigned int _counts_cocontraction;

    bool _mode_changed;
};

class BubbleCocoClassifier : public Classifier {
public:
    BubbleCocoClassifier(std::vector<Action> actions, EMGThresholds thresholds, unsigned int counts_after_mode_change, unsigned int counts_cocontraction, unsigned int counts_before_bubble, unsigned int counts_after_bubble);
    void process(int forward_emg, int backward_emg, bool btn=0);

private:
    unsigned int _counts_before_bubble;
    unsigned int _counts_after_bubble;
};

class QuantumClassifier : public Classifier {
public:
    QuantumClassifier(std::vector<Action> actions, EMGThresholds thresholds, unsigned int counts_after_mode_change, unsigned int counts_btn, unsigned int counts_before_bubble, unsigned int counts_after_bubble);
    void process(int forward_emg, int backward_emg, bool btn=0);

private:
    unsigned int _counts_before_bubble;
    unsigned int _counts_after_bubble;
};

}

#endif // MYOCONTROL_H

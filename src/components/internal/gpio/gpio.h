#ifndef GPIO_H
#define GPIO_H

class GPIO {
public:
    enum Direction {
        DIR_INPUT,
        DIR_OUTPUT
    };

    enum Pull {
        PULL_UP,
        PULL_DOWN,
        PULL_NONE
    };

    GPIO(int pin, Direction dir, Pull pull);

    GPIO& operator=(int v);
    GPIO& operator=(bool v);
    operator int();
    operator bool();

private:
    int _pin;
    Direction _dir;
    Pull _pull;
};

#endif // GPIO_H

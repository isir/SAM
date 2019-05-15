#ifndef CASTHELPER_H
#define CASTHELPER_H

#include <QByteArray>

namespace RC {
class CastHelper {
public:
    template <typename T>
    static T to(const QByteArray input)
    {
        T ret = 0;

        if (input.size() < static_cast<int>(sizeof(T)))
            return ret;

        for (unsigned int i = 0; i < sizeof(T); ++i) {
            ret = (ret << 8) | input[i];
        }
        return ret;
    }

    template <typename T>
    static QByteArray from(T input)
    {
        QByteArray ret;
        for (unsigned int i = 0; i < sizeof(T); ++i) {
            ret.push_front(input & 0xff);
            input >>= 8;
        }
        return ret;
    }
};
}

#endif // CASTHELPER_H

#ifndef CASTHELPER_H
#define CASTHELPER_H

#include <cstddef>
#include <vector>

namespace RC {
class CastHelper {
public:
    template <typename T>
    static T to(const std::vector<std::byte>& input)
    {
        T ret = 0;

        if (input.size() < static_cast<int>(sizeof(T)))
            return ret;

        for (unsigned int i = 0; i < sizeof(T); ++i) {
            ret = (ret << 8) | static_cast<T>(input[i]);
        }
        return ret;
    }

    template <typename T>
    static std::vector<std::byte> from(T input)
    {
        std::vector<std::byte> ret;
        for (unsigned int i = 0; i < sizeof(T); ++i) {
            ret.insert(ret.begin(), static_cast<std::byte>(input & 0xff));
            input >>= 8;
        }
        return ret;
    }
};
}

#endif // CASTHELPER_H

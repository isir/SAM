#ifndef ANSWER_H
#define ANSWER_H

#include <cstddef>
#include <vector>

namespace RC {
namespace Answer {
    class BaseAnswer {
    public:
        virtual ~BaseAnswer() = 0;

        virtual bool try_match(std::vector<std::byte> v, std::vector<std::byte> command = std::vector<std::byte>()) = 0;
        virtual std::vector<std::byte> format(std::vector<std::byte> v);
    };

    class ExactMatch : public BaseAnswer {
    public:
        ExactMatch(std::vector<std::byte> pattern);

        bool try_match(std::vector<std::byte> v, std::vector<std::byte> = std::vector<std::byte>());

    private:
        std::vector<std::byte> _pattern;
    };

    class Any : public BaseAnswer {
    public:
        bool try_match(std::vector<std::byte>, std::vector<std::byte> = std::vector<std::byte>()) { return true; }
    };

    class EndsWithCRC : public BaseAnswer {
    public:
        bool try_match(std::vector<std::byte> v, std::vector<std::byte> command);
        std::vector<std::byte> format(std::vector<std::byte> v);
    };
}
}

#endif // ANSWER_H

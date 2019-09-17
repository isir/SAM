#ifndef NAMED_OBJECT_H
#define NAMED_OBJECT_H

#include <string>
#include <vector>

class NamedObject {
public:
    NamedObject(std::string name, const NamedObject* parent = nullptr);
    virtual ~NamedObject() = 0;

    std::string full_name();

    inline std::string name()
    {
        return _name;
    }

    static const std::string base_name;

protected:
    std::string _name;
    std::vector<std::string> _parents;
};

#endif // NAMED_OBJECT_H

#include "named_object.h"
#include <algorithm>

NamedObject::NamedObject(std::string name, const NamedObject* parent)
    : _name(name)
{
    std::transform(_name.begin(), _name.end(), _name.begin(), [](unsigned char c) { return std::tolower(c); });

    if (parent) {
        _parents = parent->_parents;
        _parents.push_back(parent->_name);
    } else {
        _parents.push_back("sam");
    }
}

NamedObject::~NamedObject()
{
}

std::string NamedObject::full_name()
{
    std::string s;
    for (auto p : _parents) {
        s += p + "/";
    }
    s += _name;
    return s;
}

#include "named_object.h"

NamedObject::NamedObject(std::string name, const NamedObject* parent)
    : _name(name)
{
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

#include "named_object.h"

NamedObject::NamedObject(QString name, const NamedObject* parent)
    : _name(name)
{
    if (parent) {
        _parents.append(parent->_parents);
        _parents.append(parent->_name);
    } else {
        _parents.append("sam");
    }
}

NamedObject::~NamedObject()
{
}

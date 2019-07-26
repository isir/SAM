#ifndef NAMED_OBJECT_H
#define NAMED_OBJECT_H

#include <QList>
#include <QString>

class NamedObject {
public:
    NamedObject(QString name, const NamedObject* parent = nullptr);
    virtual ~NamedObject();

    inline QString name()
    {
        return _name;
    }

protected:
    QString _name;
    QList<QString> _parents;
};

#endif // NAMED_OBJECT_H

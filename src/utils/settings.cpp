#include "settings.h"

#include <QDebug>

Settings::Settings(const QString& group)
    : QSettings()
{
    if (!group.isEmpty()) {
        beginGroup(group);
    }
}

Settings::~Settings()
{
}

QVariant Settings::value(const QString& key, const QVariant& defaultValue)
{
    if (!contains(key)) {
        setValue(key, defaultValue);
    }
    return value(key, defaultValue);
}

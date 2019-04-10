#ifndef SETTINGS_H
#define SETTINGS_H

#include <QSettings>
#include <QString>

class Settings : public QSettings {
public:
    Settings(const QString& group = QString());
    virtual ~Settings();

    QVariant value(const QString& key, const QVariant& defaultValue = QVariant());
};

#endif // SETTINGS_H

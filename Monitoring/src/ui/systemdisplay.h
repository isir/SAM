#ifndef SYSTEMDISPLAY_H
#define SYSTEMDISPLAY_H

#include <QList>
#include <QProgressBar>
#include <QWidget>

namespace Ui {
class SystemDisplay;
}

class SystemDisplay : public QWidget {
    Q_OBJECT

public:
    explicit SystemDisplay(QWidget* parent = nullptr);
    ~SystemDisplay();

public slots:
    void setup();

private:
    Ui::SystemDisplay* ui;
    QList<QProgressBar*> _pbs;
};

#endif // SYSTEMDISPLAY_H

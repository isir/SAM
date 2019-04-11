#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "logdisplay.h"
#include "menudisplay.h"
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow* ui;
    LogDisplay _ld;
    MenuDisplay _md;

private slots:
    void setup();
};

#endif // MAINWINDOW_H

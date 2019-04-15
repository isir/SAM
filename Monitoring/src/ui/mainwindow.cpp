#include "mainwindow.h"
#include "systemdisplay.h"
#include "topicplotter.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _tp("system/cpu_load")
{
    ui->setupUi(this);
    ui->verticalLayout->addWidget(&_ld);
    ui->horizontalLayout->addWidget(&_md);
    ui->horizontalLayout->addWidget(&_tp);

    ui->statusBar->addPermanentWidget(new SystemDisplay());
    setWindowTitle("SAM Monitoring");

    QObject::connect(&MqttClient::instance(), &QMqttClient::connected, this, &MainWindow::setup);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setup()
{
    _tp.enable();
}

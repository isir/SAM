#include "mainwindow.h"
#include "systemdisplay.h"
#include "topicplotter.h"
#include "ui/mqttconnect.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _tp("sam/myoband/acc")
{
    ui->setupUi(this);
    ui->verticalLayout->insertWidget(0, new MqttConnect());
    ui->verticalLayout->addWidget(&_ld);
    ui->horizontalLayout->addWidget(&_md);
    ui->horizontalLayout->addWidget(&_tp);

    ui->verticalLayout->setStretch(0, 3);
    ui->verticalLayout->setStretch(1, 1);

    ui->horizontalLayout->setStretch(0, 2);
    ui->horizontalLayout->setStretch(1, 5);

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

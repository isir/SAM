#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QLayout>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->verticalLayout->addWidget(&_ld);
    ui->horizontalLayout->addWidget(&_md);

    QObject::connect(&MqttClient::instance(), &QMqttClient::connected, this, &MainWindow::setup);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setup()
{
    QMqttSubscription* sub = MqttClient::instance().subscribe(QString("system/cpu_load"));
    QObject::connect(sub, &QMqttSubscription::messageReceived, [this](QMqttMessage msg) { ui->statusBar->showMessage("CPU load : " + msg.payload()); });
}

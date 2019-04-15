#include "systemdisplay.h"
#include "../src/utils/mqttclient.h"
#include "ui_systemdisplay.h"

SystemDisplay::SystemDisplay(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::SystemDisplay)
{
    ui->setupUi(this);
    _pbs.push_back(ui->progressBar_cpu0);
    _pbs.push_back(ui->progressBar_cpu1);
    _pbs.push_back(ui->progressBar_cpu2);
    _pbs.push_back(ui->progressBar_cpu3);

    QObject::connect(&MqttClient::instance(), &QMqttClient::connected, this, &SystemDisplay::setup);
}

SystemDisplay::~SystemDisplay()
{
    delete ui;
}

void SystemDisplay::setup()
{
    QMqttSubscription* sub = MqttClient::instance().subscribe(QString("system/cpu_load"));
    for (unsigned int i = 0; i < 4; ++i) {
        QObject::connect(sub, &QMqttSubscription::messageReceived, [this, i](QMqttMessage msg) { _pbs[i]->setValue(100 * (QString(msg.payload()).split(' ', QString::SkipEmptyParts)).at(i + 1).toDouble()); });
    }
    sub = MqttClient::instance().subscribe(QString("system/cpu_temp"));
    QObject::connect(sub, &QMqttSubscription::messageReceived, [this](QMqttMessage msg) { ui->label_cpu_temp->setText(QString::number(msg.payload().toInt() / 1000) + "°C"); });
}

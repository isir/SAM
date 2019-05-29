#include "mqttconnect.h"
#include "ui_mqttconnect.h"

MqttConnect::MqttConnect(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::MqttConnect)
{
    ui->setupUi(this);
    QObject::connect(ui->pushButton_connect, &QPushButton::clicked, this, &MqttConnect::button_callback);
    QObject::connect(&MqttClient::instance(), &QMqttClient::stateChanged, this, &MqttConnect::mqtt_state_callback);
}

MqttConnect::~MqttConnect()
{
    delete ui;
}

void MqttConnect::button_callback()
{
    MqttClient::instance().connect_to_host(ui->lineEdit_hostname->text(), 1883);
}

void MqttConnect::mqtt_state_callback(QMqttClient::ClientState state)
{
    switch (state) {
    case QMqttClient::Connected:
    case QMqttClient::Connecting:
        ui->pushButton_connect->setEnabled(false);
        break;
    default:
        ui->pushButton_connect->setEnabled(true);
        break;
    }
}

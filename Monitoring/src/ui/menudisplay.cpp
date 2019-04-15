#include "menudisplay.h"
#include "ui_menudisplay.h"

MenuDisplay::MenuDisplay(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::MenuDisplay)
    , _mqtt(MqttClient::instance())
{
    ui->setupUi(this);
    QObject::connect(&_mqtt, &QMqttClient::connected, this, &MenuDisplay::setup);
    QObject::connect(ui->pushButton_send, &QPushButton::pressed, this, &MenuDisplay::send_callback);
    QObject::connect(ui->lineEdit_command, &QLineEdit::returnPressed, this, &MenuDisplay::send_callback);
}

MenuDisplay::~MenuDisplay()
{
    delete ui;
}

void MenuDisplay::setup()
{
    QMqttSubscription* sub = _mqtt.subscribe(QString("sam/menu/output"));
    QObject::connect(sub, &QMqttSubscription::messageReceived, this, &MenuDisplay::mqtt_message_callback);
}

void MenuDisplay::send_callback()
{
    _mqtt.publish(QString("sam/menu/input"), ui->lineEdit_command->text().toLatin1());
    ui->lineEdit_command->clear();
}

void MenuDisplay::mqtt_message_callback(QMqttMessage msg)
{
    ui->textEdit_menu->clear();
    ui->textEdit_menu->append(msg.payload());
}

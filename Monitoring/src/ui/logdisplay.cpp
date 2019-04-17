#include "logdisplay.h"
#include "ui_logdisplay.h"
#include <QTime>

LogDisplay::LogDisplay(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::LogDisplay)
    , _mqtt(MqttClient::instance())
{
    ui->setupUi(this);
    QObject::connect(&_mqtt, &QMqttClient::connected, this, &LogDisplay::setup);
    QObject::connect(ui->pushButton_clear, &QPushButton::pressed, [this]() { ui->textEdit->clear(); _messages.clear(); });
    QObject::connect(ui->checkBox_debug, &QCheckBox::clicked, this, &LogDisplay::refresh);
    QObject::connect(ui->checkBox_info, &QCheckBox::clicked, this, &LogDisplay::refresh);
    QObject::connect(ui->checkBox_warning, &QCheckBox::clicked, this, &LogDisplay::refresh);
    QObject::connect(ui->checkBox_critical, &QCheckBox::clicked, this, &LogDisplay::refresh);
    QObject::connect(ui->checkBox_fatal, &QCheckBox::clicked, this, &LogDisplay::refresh);
}

LogDisplay::~LogDisplay()
{
    delete ui;
}

void LogDisplay::setup()
{
    QMqttSubscription* sub = _mqtt.subscribe(QString("sam/log/#"));
    QObject::connect(sub, &QMqttSubscription::messageReceived, this, &LogDisplay::mqtt_message_callback);
}

void LogDisplay::display_message(QMqttMessage msg)
{
    QString payload = QTime::currentTime().toString("[hh:mm:ss] ") + msg.payload();
    if (payload.endsWith('\n')) {
        payload.chop(1);
    }
    if (payload.endsWith('\r')) {
        payload.chop(1);
    }
    if ((msg.topic().name().endsWith("debug") && ui->checkBox_debug->isChecked()) || (msg.topic().name().endsWith("info") && ui->checkBox_info->isChecked())) {
        ui->textEdit->setTextColor(QColor::fromRgb(0, 0, 0));
        ui->textEdit->setTextBackgroundColor(QColor::fromRgb(255, 255, 255));
    } else if (msg.topic().name().endsWith("warning") && ui->checkBox_warning->isChecked()) {
        ui->textEdit->setTextColor(QColor::fromRgb(253, 106, 2));
        ui->textEdit->setTextBackgroundColor(QColor::fromRgb(255, 255, 255));
    } else if (msg.topic().name().endsWith("critical") && ui->checkBox_critical->isChecked()) {
        ui->textEdit->setTextColor(QColor::fromRgb(255, 0, 0));
        ui->textEdit->setTextBackgroundColor(QColor::fromRgb(255, 255, 255));
    } else if (msg.topic().name().endsWith("fatal") && ui->checkBox_fatal->isChecked()) {
        ui->textEdit->setTextColor(QColor::fromRgb(255, 255, 255));
        ui->textEdit->setTextBackgroundColor(QColor::fromRgb(255, 0, 0));
    } else {
        return;
    }
    ui->textEdit->append(payload);
}

void LogDisplay::refresh()
{
    ui->textEdit->clear();
    foreach (QMqttMessage msg, _messages) {
        display_message(msg);
    }
}

void LogDisplay::mqtt_message_callback(QMqttMessage msg)
{
    display_message(msg);
    _messages.push_back(msg);
    if (_messages.size() > 1000) {
        _messages.removeFirst();
    }
}

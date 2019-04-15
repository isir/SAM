#ifndef TOPICPLOTTER_H
#define TOPICPLOTTER_H

#include "../src/utils/mqttclient.h"
#include <QWidget>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

namespace Ui {
class TopicPlotter;
}

class TopicPlotter : public QWidget {
    Q_OBJECT

public:
    explicit TopicPlotter(QString topic_name, QWidget* parent = nullptr);
    ~TopicPlotter();

    void setup();

public slots:
    void enable();
    void disable();

private:
    Ui::TopicPlotter* ui;

    QString _topic_name;
    QMqttSubscription* _sub;

    QtCharts::QChart* _chart;
    QtCharts::QChartView _chartview;

    QVector<QtCharts::QLineSeries*> _series;
    QVector<QVector<QPointF>*> _buffers;
    QtCharts::QValueAxis _x_axis;
    QtCharts::QValueAxis _y_axis;

    const int _max_size;

private slots:
    void mqtt_callback(QMqttMessage msg);
};

#endif // TOPICPLOTTER_H

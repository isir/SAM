#ifndef CONSOLEINPUT_H
#define CONSOLEINPUT_H

#include <QObject>

class ConsoleInput : public QObject {
    Q_OBJECT
public:
    ConsoleInput(QObject* parent = nullptr);
    ~ConsoleInput();

public slots:
    void read_line();

private:
    static const unsigned int _buffer_size = 128;
    char _buffer[_buffer_size];

signals:
    void new_line(QString data);
};

#endif // CONSOLEINPUT_H

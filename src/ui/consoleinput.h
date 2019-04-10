#ifndef CONSOLEINPUT_H
#define CONSOLEINPUT_H

#include <QObject>
#include <QThread>

class ConsoleInput : public QObject {
    Q_OBJECT
public:
    ~ConsoleInput();

    static ConsoleInput& instance()
    {
        static ConsoleInput _instance;
        return _instance;
    }
    ConsoleInput(ConsoleInput const&) = delete;
    void operator=(ConsoleInput const&) = delete;

public slots:
    void read_line();

private:
    ConsoleInput(QObject* parent = nullptr);

    QThread _thread;
    static const unsigned int _buffer_size = 128;
    char _buffer[_buffer_size];

signals:
    void new_line(QString data);
};

#endif // CONSOLEINPUT_H

#ifndef TYPES_H
#define TYPES_H

#include <QtGlobal>

namespace RC {
typedef struct {
    float p;
    float i;
    float d;
    quint32 i_max;
    quint32 deadzone;
    qint32 min_pos;
    qint32 max_pos;
} position_pid_params_t;

typedef struct {
    float p;
    float i;
    float d;
    quint32 qpps;
} velocity_pid_params_t;
}

#endif // TYPES_H

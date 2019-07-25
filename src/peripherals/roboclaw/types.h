#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

namespace RC {
typedef struct {
    float p;
    float i;
    float d;
    uint32_t i_max;
    uint32_t deadzone;
    int32_t min_pos;
    int32_t max_pos;
} position_pid_params_t;

typedef struct {
    float p;
    float i;
    float d;
    uint32_t qpps;
} velocity_pid_params_t;
}

#endif // TYPES_H

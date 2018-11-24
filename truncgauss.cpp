#include "truncgauss.h"
#include <math.h>

uint16_t truncgauss(uint16_t time_elapsed, uint16_t total_duration) {
    uint16_t value = (uint16_t) 4096 * exp(-0.5 * pow((6.0 * time_elapsed/((double) total_duration) - 3), 2));
    return (value);
    return ledGamma(value);
}

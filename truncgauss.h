#ifndef TRUNCGAUSS_H
#define TRUNCGAUSS_H

#include <Arduino.h> //needed for Serial.println
#include <string.h> //needed for memcpy

/* Truncated gaussian function */
/* Gaussian with mean 0, peak height 1, std.dev 1 is truncated at x=-3 and x=3 and then scaled to the appropriate duration and peak
 *  
 */

// Gamma function handles inverting the PWM from source to sink

/* f(x, u, var) = 1/sqrt(2*pi*var) * exp(-(x-u)^2/(2*var))) 
 * u: mean i.e X value of peak
 * var: variance, stddev^2
 * 
 * Simplify: u = 0, var = 1
 * 
 * f(x) = exp(-0.5x^2)/sqrt(2*pi)
 * 
 * okay lets define the function
 * 99.7% of function mass is within 3 sigma so that's a nice starting range
 * lets try defining:
 * 
 * total_duration is the time in milliseconds from start to stop; peak is halfway
 * time_elapsed is the time since function start, in milliseconds
 * peak value is 4096
 * 
 */

uint16_t truncgauss(uint16_t time_elapsed, uint16_t total_duration);

inline unsigned short int ledGamma(unsigned short int linear) {
    signed short int gamma_out;
    float gamma = 2.8;
    signed short int max_in = 4096;
    signed short int max_out = 4096;

    if (linear > (unsigned) max_in) {
        linear = max_in;
    }
    
    gamma_out =  max_out - (signed short int) (pow( (((float) linear) / ((float) max_in)), gamma) * max_out + 0.5);
    // Truncate if over/underflow
    if (gamma_out < 0) {
        gamma_out = 0;
    }
    if (gamma_out > max_out) {
        gamma_out = max_out;
    }
    return (unsigned short int) gamma_out;
}

#endif

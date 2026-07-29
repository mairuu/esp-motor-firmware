/* Quadrature encoders, counted by the ESP32's PCNT hardware peripheral. */

#pragma once

/* Returns false if the PCNT units could not be configured. */
bool initEncoders();

/* Accumulated tick count for LEFT or RIGHT. 4 counts per quadrature cycle. */
long readEncoder(int i);

void resetEncoders();

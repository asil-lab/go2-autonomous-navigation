/*
 * This structure is used to store the data received from the slave device.
 * Author: Alexander James Becoy
 * Date: 2024-11-04
 */

#ifndef AUXILIARY_SENSING__MISO_H
#define AUXILIARY_SENSING__MISO_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define FLOAT_SIZE 32
#define MISO_BUFFER_SIZE FLOAT_SIZE * 3

// Function prototypes
void float_to_uint32(float f, uint32_t *i);
char * mosi_to_string(struct MOSI *mosi);

struct MOSI {
    float temperature;
    float humidity;
    float lux;
};

// Function to convert float to uint32_t for transmission.
void float_to_uint32(float f, uint32_t *i) {
    memcpy(i, &f, sizeof(f));
}

// Function to convert uint32_t to binary string for transmission.
char * uint32_to_string(uint32_t i) {
    static char buffer[FLOAT_SIZE + 1];
    buffer[FLOAT_SIZE] = '\0';

    // Order the bits from MSB to LSB.
    for (int j = FLOAT_SIZE - 1; j >= 0; j--) {
        buffer[j] = (i & 1) + '0';
        i >>= 1;
    }

    return buffer;
}

// Function to output the data as string of bits received from the slave device.
char * mosi_to_string(struct MOSI *mosi) {
    static char buffer[MISO_BUFFER_SIZE];

    // Convert the float values to uint32_t.
    uint32_t temperature, humidity, lux;
    float_to_uint32(mosi->temperature, &temperature);
    float_to_uint32(mosi->humidity, &humidity);
    float_to_uint32(mosi->lux, &lux);

    // Convert the uint32_t values to string of bits.
    char *temperature_str = uint32_to_string(temperature);
    char *humidity_str = uint32_to_string(humidity);
    char *lux_str = uint32_to_string(lux);

    // Concatenate the strings.
    snprintf(buffer, MISO_BUFFER_SIZE, "%s %s %s", temperature_str, humidity_str, lux_str);

    return buffer;
}

#endif //AUXILIARY_SENSING__MISO_H
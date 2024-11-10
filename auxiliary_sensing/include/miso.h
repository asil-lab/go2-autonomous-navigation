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

#define FLOAT_SIZE                          32
#define MISO_BUFFER_TEMPERATURE_OFFSET      (FLOAT_SIZE * 0)
#define MISO_BUFFER_HUMIDITY_OFFSET         (FLOAT_SIZE * 1)
#define MISO_BUFFER_LIGHT_OFFSET            (FLOAT_SIZE * 2)
#define MISO_BUFFER_SIZE                    (FLOAT_SIZE * 3) + 1

// Function prototypes
void float_to_uint32(float f, uint32_t *i);
char * mosi_to_string(struct MOSI *mosi);

struct MOSI {
    float temperature;
    float humidity;
    float light;
};

// Function to convert float to uint32_t according to IEEE 754 standard.
void float_to_uint32(float f, uint32_t *ui) {
    memcpy(ui, &f, sizeof(f));
}

// Function to convert uint32_t to binary string for transmission.
void uint32_to_string(uint32_t i, char *buffer) {
    buffer[FLOAT_SIZE] = '\0';
    for (int j = 0; j < FLOAT_SIZE; j++) {
        buffer[j] = (i & 1) + '0';
        i >>= 1;
    }
}

// Function to convert float to binary string for transmission.
void float_to_string(float f, char *buffer) {
    uint32_t i;
    float_to_uint32(f, &i);
    uint32_to_string(i, buffer);
}

// Function to output the data as string of bits received from the slave device.
char * mosi_to_string(struct MOSI *mosi) {
    static char buffer[MISO_BUFFER_SIZE];

    // Convert the float values to binary strings.
    float_to_string(mosi->temperature, buffer + MISO_BUFFER_TEMPERATURE_OFFSET);
    float_to_string(mosi->humidity, buffer + MISO_BUFFER_HUMIDITY_OFFSET);
    float_to_string(mosi->light, buffer + MISO_BUFFER_LIGHT_OFFSET);

    // Concatenate the strings.
    snprintf(buffer, MISO_BUFFER_SIZE, "%s", buffer);

    return buffer;
}

#endif //AUXILIARY_SENSING__MISO_H
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
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

// Function prototypes
void float_to_uint32(float f, uint32_t *i);
char * mosi_to_string(struct MOSI *mosi);

struct MOSI {
    float temperature;
    float humidity;
    float light;
};

// Function to convert float to uint32_t according to IEEE 754 standard.
void float_to_uint32(float f, uint32_t *i) {
    memcpy(i, &f, sizeof(f));
}

// Function to convert uint32_t to binary string for transmission.
char * uint32_to_string(uint32_t i) {
    static char buffer[FLOAT_SIZE + 1];
    buffer[FLOAT_SIZE] = '\0';

    // Order the bits according to the IEEE 754 standard.
    for (int j = 0; j < FLOAT_SIZE; j++) {
        buffer[FLOAT_SIZE - j - 1] = 0;
    //     buffer[FLOAT_SIZE - j - 1] = (i & (1 << j)) ? '1' : '0';
    }

    return buffer;
}

// Function to output the data as string of bits received from the slave device.
char * mosi_to_string(struct MOSI *mosi) {
    static char buffer[MISO_BUFFER_SIZE];

    // Convert the float values to uint32_t.
    uint32_t temperature, humidity, light;
    float_to_uint32(mosi->temperature, &temperature);
    float_to_uint32(mosi->humidity, &humidity);
    float_to_uint32(mosi->light, &light);

    // Convert the uint32_t values to string of bits.
    char *temperature_str = uint32_to_string(temperature);
    char *humidity_str = uint32_to_string(humidity);
    char *light_str = uint32_to_string(light);

    // Concatenate the strings.
    snprintf(buffer, MISO_BUFFER_SIZE, "%s %s %s", temperature, humidity, light);

    return buffer;
}

#endif //AUXILIARY_SENSING__MISO_H
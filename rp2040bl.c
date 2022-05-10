#include "rp2040.h"
#include "rp2040bl.h"

void flush_stdin() {
    uint8_t data[256];
    uart_read_bytes(0, data, sizeof(data), 20 / portTICK_PERIOD_MS);
}

bool read_stdin(uint8_t* buffer, uint32_t len, uint32_t timeout) {
    int read = uart_read_bytes(0, buffer, len, timeout / portTICK_PERIOD_MS);
    return (read == len);
}

bool rp2040_bl_sync() {
    flush_stdin();
    char command[] = "SYNC";
    uart_write_bytes(0, command, 4);
    uint8_t rx_buffer[4 * 6];
    read_stdin(rx_buffer, sizeof(rx_buffer), 1000);
    if (memcmp(rx_buffer, "PICO", 4) != 0) return false;
    return true;
}

bool rp2040_bl_get_info(uint32_t* flash_start, uint32_t* flash_size, uint32_t* erase_size, uint32_t* write_size, uint32_t* max_data_len) {
    flush_stdin();
    char command[] = "INFO";
    uart_write_bytes(0, command, 4);
    uint8_t rx_buffer[4 * 6];
    read_stdin(rx_buffer, sizeof(rx_buffer), 1000);
    if (memcmp(rx_buffer, "OKOK", 4) != 0) return false;
    memcpy((uint8_t*) flash_start,  &rx_buffer[4 * 1], 4);
    memcpy((uint8_t*) flash_size,   &rx_buffer[4 * 2], 4);
    memcpy((uint8_t*) erase_size,   &rx_buffer[4 * 3], 4);
    memcpy((uint8_t*) write_size,   &rx_buffer[4 * 4], 4);
    memcpy((uint8_t*) max_data_len, &rx_buffer[4 * 5], 4);
    return true;
}

bool rp2040_bl_erase(uint32_t address, uint32_t length) {
    flush_stdin();
    char command[12];
    snprintf(command, 5, "ERAS");
    memcpy(command + 4, (char*) &address, 4);
    memcpy(command + 8, (char*) &length, 4);
    uart_write_bytes(0, command, sizeof(command));
    uint8_t rx_buffer[4];
    read_stdin(rx_buffer, sizeof(rx_buffer), 10000);
    if (memcmp(rx_buffer, "OKOK", 4) != 0) return false;
    return true;
}

bool rp2040_bl_write(uint32_t address, uint32_t length, uint8_t* data, uint32_t* crc) {
    flush_stdin();
    char command[12];
    snprintf(command, 5, "WRIT");
    memcpy(command + 4, (char*) &address, 4);
    memcpy(command + 8, (char*) &length, 4);
    uart_write_bytes(0, command, sizeof(command));
    uart_write_bytes(0, data, length);
    uint8_t rx_buffer[8];
    read_stdin(rx_buffer, sizeof(rx_buffer), 10000);
    if (memcmp(rx_buffer, "OKOK", 4) != 0) return false;
    memcpy((uint8_t*) crc, &rx_buffer[4 * 1], 4);
    return true;
}

bool rp2040_bl_seal(uint32_t addr, uint32_t vtor, uint32_t length, uint32_t crc) {
    flush_stdin();
    char command[20];
    snprintf(command, 5, "SEAL");
    memcpy(command + 4, (char*) &addr, 4);
    memcpy(command + 8, (char*) &vtor, 4);
    memcpy(command + 12, (char*) &length, 4);
    memcpy(command + 16, (char*) &crc, 4);
    uart_write_bytes(0, command, sizeof(command));
    uint8_t rx_buffer[4];
    read_stdin(rx_buffer, sizeof(rx_buffer), 10000);
    if (memcmp(rx_buffer, "OKOK", 4) != 0) return false;
    return true;
}

bool rp2040_bl_go(uint32_t vtor) {
    flush_stdin();
    char command[8];
    snprintf(command, 5, "GOGO");
    memcpy(command + 4, (char*) &vtor, 4);
    uart_write_bytes(0, command, sizeof(command));
    return true;
}

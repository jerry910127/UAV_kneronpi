#include "miramar_ctrl.h"
#include <stdio.h>
#include <stdint.h>
#include <pthread.h>

// The following symbols come from uart.cpp (copied from peripheral.zip).
extern int g_fd;
extern int file_open_and_get_descriptor(const char *fname);
extern void configure_serial_port(void);
extern void close_serial_port(void);
extern uint16_t readRegister(uint32_t address, uint16_t length, uint8_t *data);
extern uint16_t writeRegister(uint32_t address, uint16_t length, uint8_t *data);

static const uint32_t TM_CONTROL = 0x2C000000;

static pthread_mutex_t g_miramar_mutex = PTHREAD_MUTEX_INITIALIZER;

int miramar_open(const char* dev)
{
    if (!dev) return -1;

    pthread_mutex_lock(&g_miramar_mutex);

    // If already open, keep it open (caller can close/reopen if needed).
    if (g_fd >= 0) {
        pthread_mutex_unlock(&g_miramar_mutex);
        return 0;
    }

    g_fd = file_open_and_get_descriptor(dev);
    if (g_fd < 0) {
        printf("[miramar] open %s failed\n", dev);
        pthread_mutex_unlock(&g_miramar_mutex);
        return -1;
    }

    // uart.cpp config: 115200 + raw
    configure_serial_port();
    pthread_mutex_unlock(&g_miramar_mutex);
    return 0;
}


void miramar_close()
{
    pthread_mutex_lock(&g_miramar_mutex);
    if (g_fd >= 0) {
        close_serial_port();
        g_fd = -1;
    }
    pthread_mutex_unlock(&g_miramar_mutex);
}


int miramar_set_agc_clahe(int agc_on, int clahe_on)
{
    pthread_mutex_lock(&g_miramar_mutex);

    if (g_fd < 0) {
        printf("[miramar] control port not open\n");
        pthread_mutex_unlock(&g_miramar_mutex);
        return -1;
    }

    uint32_t cur = 0;
    uint16_t rlen = readRegister(TM_CONTROL, 4, (uint8_t*)&cur);
    if (rlen != 4) {
        printf("[miramar] read TM_CONTROL failed, rlen=%u\n", rlen);
        pthread_mutex_unlock(&g_miramar_mutex);
        return -1;
    }

    // Only update bit0/bit1; keep other bits unchanged.
    uint32_t newv = (cur & ~0x3u) | (agc_on ? 1u : 0u) | (clahe_on ? 2u : 0u);

    uint16_t w = writeRegister(TM_CONTROL, 4, (uint8_t*)&newv);
    if (w == 0) {
        printf("[miramar] write TM_CONTROL failed\n");
        pthread_mutex_unlock(&g_miramar_mutex);
        return -1;
    }

    pthread_mutex_unlock(&g_miramar_mutex);
    return 0;
}


int miramar_get_agc_clahe(int *agc_on, int *clahe_on)
{
    if (!agc_on || !clahe_on) return -1;

    pthread_mutex_lock(&g_miramar_mutex);

    if (g_fd < 0) {
        pthread_mutex_unlock(&g_miramar_mutex);
        return -1;
    }

    uint32_t cur = 0;
    uint16_t rlen = readRegister(TM_CONTROL, 4, (uint8_t*)&cur);
    pthread_mutex_unlock(&g_miramar_mutex);

    if (rlen != 4) return -1;

    *agc_on = (cur >> 0) & 0x1;
    *clahe_on = (cur >> 1) & 0x1;
    return 0;
}

int miramar_is_open(void)
{
    pthread_mutex_lock(&g_miramar_mutex);
    int ok = (g_fd >= 0) ? 1 : 0;
    pthread_mutex_unlock(&g_miramar_mutex);
    return ok;
}

#include "include/i2c_bus.h"

static SemaphoreHandle_t g_i2c_bus_mutex = NULL;

void i2c_bus_init(void)
{
    if (g_i2c_bus_mutex == NULL) {
        g_i2c_bus_mutex = xSemaphoreCreateMutex();
    }
}

void i2c_bus_lock(void)
{
    if (g_i2c_bus_mutex) {
        xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY);
    }
}

void i2c_bus_unlock(void)
{
    if (g_i2c_bus_mutex) {
        xSemaphoreGive(g_i2c_bus_mutex);
    }
}


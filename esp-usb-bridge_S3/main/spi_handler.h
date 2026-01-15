#pragma once

#include "esp_err.h"

#define GPIO_MOSI 11
#define GPIO_MISO 9  
#define GPIO_SCLK 12
#define GPIO_CS   10

#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    SPI_DMA_CH_AUTO

// Структура нашего сообщения (для очереди)
typedef struct {
    uint8_t data[128]; // Буфер данных
    size_t len;        // Длина
} spi_message_t;

// Объявление функции инициализации
void spi_slave_init(void);

// Объявление задачи обработки (если вы запускаете её через xTaskCreate в main)
void spi_processing_task(void *pvParameters);
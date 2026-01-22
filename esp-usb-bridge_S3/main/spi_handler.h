#pragma once

#include "esp_err.h"

#define GPIO_MOSI 11
#define GPIO_MISO 9  
#define GPIO_SCLK 12
#define GPIO_CS   10

#define GPIO_HANDSHAKE 13

#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    SPI_DMA_CH_AUTO

#define BUFFER_SIZE  65535 

#define GEMINI_CODE 1


// Структура нашего сообщения (для очереди)
typedef struct {
    uint8_t *data; // Буфер данных
    uint32_t len;        // Длина
} spi_message_t;

#if GEMINI_CODE
// Объявление функции инициализации
void spi_slave_init(void);

// Объявление задачи обработки (если вы запускаете её через xTaskCreate в main)
void spi_processing_task(void *pvParameters);
#else

void spi_slave_esp_idf(void *pvParameters);

#endif
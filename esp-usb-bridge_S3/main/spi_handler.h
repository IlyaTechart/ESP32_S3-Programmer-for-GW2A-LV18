#pragma once

#include "esp_err.h"

#define GEMINI_CODE 0

#define GPIO_MOSI 11
#define GPIO_MISO 9  
#define GPIO_SCLK 12
#define GPIO_CS   10

#define GPIO_HANDSHAKE 13

#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    SPI_DMA_CH_AUTO

#define BUFFER_SIZE  1

#define KILOBYTES_IN_MEGABYTE 1024
#define BYTES_IN_KILOBYTE     1024

#define CONVERT_TO_BYTES_FORM_KILOBYTE(x) ((x) * BYTES_IN_KILOBYTE)

#define CURRENT_SIZE    CONVERT_TO_BYTES_FORM_KILOBYTE(BUFFER_SIZE)    
     
typedef struct{
    uint8_t *pssram_tx_buffer;
    uint8_t *pssram_rx_buffer;
    uint8_t *innternal_tx_buffer;
    uint8_t *innternal_rx_buffer;
} spi_buffers_t;

// Структура нашего сообщения (для очереди)
typedef struct {
    uint8_t data[CURRENT_SIZE]; // Буфер данных
    uint32_t len;        // Длина
} spi_message_t;

#if GEMINI_CODE
// Объявление функции инициализации
void spi_slave_init(void);

// Объявление задачи обработки (если вы запускаете её через xTaskCreate в main)
void spi_processing_task(void *pvParameters);
#else

void spi_slave_init(void);

void spi_slave_esp_idf(void *pvParameters);

#endif
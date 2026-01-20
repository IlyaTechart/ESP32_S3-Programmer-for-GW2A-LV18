#include "spi_handler.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"

static const char *TAG = "SPI_HANDLER";

// Очередь должна быть определена в .c файле
static QueueHandle_t spi_evt_queue;

// Буферы для драйвера: HEAP_CAPS_DMA обязателен для DMA-capable памяти на ESP32S3
// Выравнивание 16 байт требуется для L1CACHE на ESP32S3
char *sendbuf = NULL;
char *recvbuf = NULL;

// Прототипы функций (чтобы init их видел)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans);
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans);
static void spi_driver_task(void *pvParameters); // Внутренняя задача драйвера

void spi_slave_init(void) {

    // 0. Выделяем DMA-capable память с правильным выравниванием для ESP32S3 (16 байт)
    sendbuf = heap_caps_aligned_alloc(16, BUFFER_SIZE, MALLOC_CAP_DMA);
    recvbuf = heap_caps_aligned_alloc(16, BUFFER_SIZE, MALLOC_CAP_DMA);
    
    if (!sendbuf || !recvbuf) {
        ESP_LOGE(TAG, "Failed to allocate DMA-capable buffers");
        return;
    }
    
    memset(sendbuf, 0, BUFFER_SIZE);
    memset(recvbuf, 0, BUFFER_SIZE);
    
    ESP_LOGI(TAG, "RX buffer @ %p, TX buffer @ %p", recvbuf, sendbuf);

    // 1. Создаем очередь
    spi_evt_queue = xQueueCreate(10, sizeof(spi_message_t));

    if (spi_evt_queue == NULL) {
        ESP_LOGE(TAG, "Error creating queue");
        return;
    }
    
    // 2. Настраиваем шину SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO // Лучше AUTO
    };

    // 3. Настраиваем интерфейс слейва
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 1,
        .post_setup_cb = my_post_setup_cb, 
        .post_trans_cb = my_post_trans_cb
    };

    // ВАЖНО: Используем SPI2_HOST (RCV_HOST определен в хедере)
    esp_err_t ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SPI: %s", esp_err_to_name(ret));
        return;
    }

    // 5. ЗАПУСК ДРАЙВЕРА (Обязательно!)
    // Без этой задачи контроллер SPI не будет знать, куда принимать данные
    xTaskCreate(spi_driver_task, "spi_driver", (4096 + BUFFER_SIZE / 4), NULL, 10, NULL);

    ESP_LOGI(TAG, "SPI Init done.");
}

// Задача, которая постоянно "кормит" драйвер буферами
static void spi_driver_task(void *pvParameters) {
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while (1) {
        // Очищаем буфер приема
        memset(recvbuf, 0, BUFFER_SIZE);

        // Настраиваем транзакцию
        t.length = BUFFER_SIZE * 8; // Размер в битах!
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        t.flags = SPI_SLAVE_TRANS_DMA_BUFFER_ALIGN_AUTO; // Разрешаем автопереаллокацию если нужна

        // Ждем данные от мастера
        esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI trans failed: %s", esp_err_to_name(ret));
        }
    }
}

// Обработчик прерывания (вызывается ПОСЛЕ приема данных)
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    spi_message_t msg;
    
    // Вычисляем сколько байт реально пришло
    volatile size_t bytes_rcv = trans->length / 8;
    if (bytes_rcv > sizeof(msg.data)) bytes_rcv = sizeof(msg.data);
    
    // ВАЖНО: На ESP32S3 нужно инвалидировать кеш перед чтением DMA данных!
    // Иначе CPU будет читать старые значения из L1 кеша
    esp_cache_msync((void *)trans->rx_buffer, (bytes_rcv + 15) & ~15, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    
    // Копируем данные из буфера драйвера в сообщение очереди
    memcpy(msg.data, trans->rx_buffer, bytes_rcv);
    msg.len = bytes_rcv;

    // Отправляем в очередь
    xQueueSendFromISR(spi_evt_queue, &msg, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Пустышка для setup (вызывается ПЕРЕД транзакцией)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans) {
    // Здесь можно выставить GPIO в 1, чтобы сигнализировать Мастеру "Я готов"
}

// Задача обработки (Пользовательская логика)
void spi_processing_task(void *pvParameters) {
    spi_message_t msg;
    ESP_LOGI(TAG, "SPI Processing task started");

    while(1) {
        // Ждем сообщения из очереди
        if((xQueueReceive(spi_evt_queue, &msg, portMAX_DELAY)) == pdPASS) {
            ESP_LOGI(TAG, "Received %d bytes", msg.len);
            ESP_LOG_BUFFER_HEX(TAG, msg.data, msg.len);
            
            // Здесь ваша логика обработки полученных команд
        }
    }
}


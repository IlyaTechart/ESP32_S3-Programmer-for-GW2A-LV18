#include "spi_handler.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"



#if GEMINI_CODE
static const char *TAG = "SPI_HANDLER";

// Очередь должна быть определена в .c файле
static QueueHandle_t spi_evt_queue;

SemaphoreHandle_t sema_for_driverTask;

// Буферы для драйвера: HEAP_CAPS_DMA обязателен для DMA-capable памяти на ESP32S3
// Выравнивание 16 байт требуется для L1CACHE на ESP32S3

volatile spi_buffers_t spi_buffers;
EXT_RAM_BSS_ATTR volatile spi_message_t msg;

//Переменные для логов 
uint8_t LogBuffer[11] = {0};
uint8_t LogFromDriverTask = 0;
uint8_t LogFromISR = 0;
uint8_t LogFromProcessingTask = 0;

extern void assert_failed(const char *file, int line, const char *func, const char *expr);

//Функции для логов 

// Прототипы функций (чтобы init их видел)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans);
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans);
static void spi_driver_task(void *pvParameters); // Внутренняя задача драйвера
void memory_allocate(void);

void spi_slave_init(void) {

     memory_allocate();

    // 1. Создаем очередь
    spi_evt_queue = xQueueCreate(1, 10);

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
        .max_transfer_sz = CURRENT_SIZE + 1,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO // Лучше AUTO
    };

    // 3. Настраиваем интерфейс слейва
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 2,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb, 
        .post_trans_cb = my_post_trans_cb
    };

    // ВАЖНО: Используем SPI2_HOST (RCV_HOST определен в хедере)
    esp_err_t ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SPI: %s", esp_err_to_name(ret));
        return;
    }

    sema_for_driverTask = xSemaphoreCreateBinary();
    if (sema_for_driverTask == NULL) {
        ESP_LOGE(TAG, "Error creating semaphore");
        return;
    }
    
    // Выдаём семафор один раз, чтобы задача могла начать работу
    xSemaphoreGive(sema_for_driverTask);

    // 5. ЗАПУСК ДРАЙВЕРА (Обязательно!)
    // Без этой задачи контроллер SPI не будет знать, куда принимать данные
    BaseType_t TaskReturned = xTaskCreate(spi_driver_task, "spi_driver", (4096 + BUFFER_SIZE / 4), NULL, 10, NULL);
    if(TaskReturned != pdPASS)
    {
        assert_failed("spi_driver_task: FAILED", 285, "TaskCreate", NULL);
    }


    ESP_LOGI(TAG, "SPI Init done.");
}

// Задача, которая постоянно "кормит" драйвер буферами
static void spi_driver_task(void *pvParameters) {
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while (1) {
        // Очищаем буфер приема
        memset(spi_buffers.pssram_rx_buffer, 0, BUFFER_SIZE);

        // Настраиваем транзакцию
        t.length =  CURRENT_SIZE * 8; // Размер в битах!
        t.tx_buffer = spi_buffers.pssram_tx_buffer;
        t.rx_buffer = spi_buffers.pssram_rx_buffer;
//        t.flags = SPI_SLAVE_TRANS_DMA_BUFFER_ALIGN_AUTO; // Разрешаем автопереаллокацию если нужна
        LogFromDriverTask++;

        // Ждем данные от мастера
        volatile esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI trans failed: %s", esp_err_to_name(ret));
        }

        xSemaphoreTake(sema_for_driverTask, portMAX_DELAY);
    }
}

// Обработчик прерывания (вызывается ПОСЛЕ приема данных)
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 1);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Вычисляем сколько байт реально пришло
    volatile uint32_t bytes_rcv = trans->trans_len / 8;
    if (bytes_rcv > sizeof(msg.data)) bytes_rcv = sizeof(msg.data);
    
    // Копируем данные из буфера драйвера в сообщение очереди
    memcpy(msg.data, trans->rx_buffer, bytes_rcv);
    LogFromISR++;
 //   msg.data = trans->rx_buffer;
    msg.len = bytes_rcv;
 //   ESP_LOGE(TAG, "FORM ISR SPI LEN %lu", msg.len);

    // Отправляем в очередь
    xQueueSendFromISR(spi_evt_queue, &msg, &xHigherPriorityTaskWoken);
    gpio_set_level(GPIO_HANDSHAKE, 0);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Пустышка для setup (вызывается ПЕРЕД транзакцией)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans) {
//    gpio_set_level(GPIO_HANDSHAKE, 0);
    // Здесь можно выставить GPIO в 1, чтобы сигнализировать Мастеру "Я готов"
}

// Задача обработки (Пользовательская логика)
void spi_processing_task(void *pvParameters) {
    ESP_LOGI(TAG, "SPI Processing task started");
    uint32_t cnt_msg;

    while(1) {
        // Ждем сообщения из очереди
        LogFromProcessingTask++;
        if((xQueueReceive(spi_evt_queue, &msg, portMAX_DELAY)) == pdPASS) {

            for(uint32_t i = 0; i < msg.len; i++)
            {
                if(msg.data[i] == 0xAA)
                {
                    cnt_msg++;
                }
            }
            ESP_LOGI(TAG, "Received %lu bytes", msg.len);
            ESP_LOGI(TAG,"Total num 0xAA: %lu", cnt_msg);
           //TODO:
            memset(&msg, 0x00, sizeof(msg));
            cnt_msg = 0;
            xSemaphoreGive(sema_for_driverTask);
 //           ESP_LOG_BUFFER_HEX(TAG, msg.data, msg.len);
            
            // Здесь ваша логика обработки полученных команд
        }
    }
}

void memory_allocate(void)
{
    // Выделяем DMA-буферы из внутренней DRAM
    spi_buffers.pssram_rx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, BUFFER_SIZE,  MALLOC_CAP_DMA);
    spi_buffers.pssram_tx_buffer = (uint8_t*)heap_caps_aligned_alloc(32, BUFFER_SIZE,  MALLOC_CAP_DMA);

    if ((spi_buffers.pssram_rx_buffer == NULL) || (spi_buffers.pssram_tx_buffer == NULL)) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers");
        return;
    }
    
    memset(spi_buffers.pssram_rx_buffer, 0, BUFFER_SIZE);
    memset(spi_buffers.pssram_tx_buffer, 0, BUFFER_SIZE);

    ESP_LOGI(TAG, "RX buffer @ 0x%p, TX buffer @ 0x%p", spi_buffers.pssram_rx_buffer, spi_buffers.pssram_tx_buffer);
    
    // Проверяем, что буферы действительно в DMA-capable памяти
    multi_heap_info_t info = {0};
    heap_caps_get_info(&info, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    ESP_LOGI(TAG, "DMA-capable SPIRAM: total=%zu, free=%zu", info.total_allocated_bytes + info.total_free_bytes, info.total_free_bytes);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////



#if !GEMINI_CODE

static const char *TAG = "SPI_FAST";

// --- НАСТРОЙКИ ---
#define CHUNK_SIZE      4096       // Размер одного DMA буфера (Internal RAM)
#define CHUNK_COUNT     8          // Количество буферов в кольце (запас прочности)
#define TOTAL_FW_SIZE   (1024*1024) // 1 МБ итоговых данных

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ---

// 1. Итоговое хранилище в PSRAM
uint8_t *psram_firmware_buffer = NULL;
volatile uint32_t fw_write_offset = 0;

// 2. Буферы для DMA (Internal RAM)
uint8_t *dma_chunks[CHUNK_COUNT];
spi_slave_transaction_t trans_desc[CHUNK_COUNT]; // Дескрипторы транзакций

// 3. Очередь для передачи индексов заполненных буферов от ISR к Задаче
QueueHandle_t rcv_queue;

// --- CALLBACK ПРЕРЫВАНИЯ ---
// Вызывается, когда транзакция завершена (FPGA закончила передачу куска)
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Получаем индекс буфера, который только что заполнился (мы сохранили его в trans->user)
    int finished_buf_index = (int)trans->user;

    // Отправляем индекс в очередь, чтобы задача забрала данные
    xQueueSendFromISR(rcv_queue, &finished_buf_index, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans) {
    // Если нужен GPIO handshake, то тут поднимаем пин.
    // Но для "слепого" приема от FPGA тут обычно ничего не делают.
}

// --- ЗАДАЧА ОБРАБОТКИ ---
void spi_processing_task(void *pvParameters) {
    int buf_idx;
    esp_err_t ret;

    ESP_LOGI(TAG, "Processing task started. Waiting for data...");

    while(1) {
        // Ждем, пока ISR сообщит, что какой-то буфер заполнился
        if (xQueueReceive(rcv_queue, &buf_idx, portMAX_DELAY)) {
            
            // 1. Узнаем, сколько байт реально пришло (обычно равно CHUNK_SIZE, если FPGA шлет потоком)
            size_t bytes_received = trans_desc[buf_idx].trans_len / 8;
            
            // 2. Проверяем, влезет ли в PSRAM
            if (fw_write_offset + bytes_received <= TOTAL_FW_SIZE) {
                // КОПИРУЕМ из быстрого внутреннего буфера в медленный PSRAM
                // memcpy здесь безопасен, так как мы не в прерывании
                memcpy(&psram_firmware_buffer[fw_write_offset], dma_chunks[buf_idx], bytes_received);
                fw_write_offset += bytes_received;
                
                // Лог каждые 4КБ, чтобы не спамить
                if (fw_write_offset % (4*1024) == 0) {
                    ESP_LOGI(TAG, "Received: %lu / %d bytes", fw_write_offset, TOTAL_FW_SIZE);
                }
            } else {
                // Буфер переполнен, игнорируем или ставим флаг "Готово"
                if (fw_write_offset < TOTAL_FW_SIZE + 1) { // Логируем только один раз
                     ESP_LOGW(TAG, "Firmware buffer FULL!");
                     fw_write_offset = TOTAL_FW_SIZE + 10; // Блокировка
                }
            }

            // 3. САМОЕ ВАЖНОЕ: Вернуть буфер в работу!
            // Мы очищаем длину (на всякий случай) и снова ставим транзакцию в очередь драйвера.
            // Она встанет в "хвост" очереди аппаратного контроллера.
            trans_desc[buf_idx].length = CHUNK_SIZE * 8; 
            trans_desc[buf_idx].trans_len = 0; // Сброс принятой длины
            
            ret = spi_slave_queue_trans(RCV_HOST, &trans_desc[buf_idx], portMAX_DELAY);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to re-queue buffer %d", buf_idx);
            }
        }
    }
}

// --- ИНИЦИАЛИЗАЦИЯ ---
void spi_slave_init(void) {
    esp_err_t ret;

    // 1. Выделяем ОГРОМНЫЙ буфер в PSRAM
    ESP_LOGI(TAG, "Allocating 1MB in PSRAM...");
    psram_firmware_buffer = (uint8_t*)heap_caps_malloc(TOTAL_FW_SIZE, MALLOC_CAP_SPIRAM);
    if (psram_firmware_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM buffer!");
        return; // Или abort()
    }
    memset(psram_firmware_buffer, 0, TOTAL_FW_SIZE);

    // 2. Инициализация шины
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CHUNK_SIZE * 2, // Должен быть >= размера транзакции
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = CHUNK_COUNT, // ВАЖНО: Размер очереди драйвера равен числу наших чанков
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    // ВАЖНО: Используем DMA_CHAN (SPI_DMA_CH_AUTO)
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    assert(ret == ESP_OK);

    // 3. Создаем очередь сообщений для Task
    rcv_queue = xQueueCreate(CHUNK_COUNT, sizeof(int));

    // 4. Аллоцируем DMA буферы и ПРЕДЗАГРУЖАЕМ их в драйвер
    ESP_LOGI(TAG, "Allocating DMA chunks and queuing...");
    
    for (int i = 0; i < CHUNK_COUNT; i++) {
        // Выделяем память во ВНУТРЕННЕЙ (быстрой) RAM с поддержкой DMA
        dma_chunks[i] = heap_caps_aligned_alloc(32, CHUNK_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        assert(dma_chunks[i] != NULL);
        memset(dma_chunks[i], 0, CHUNK_SIZE);

        // Настраиваем дескриптор
        memset(&trans_desc[i], 0, sizeof(spi_slave_transaction_t));
        trans_desc[i].length = CHUNK_SIZE * 8; // Размер в битах
        trans_desc[i].tx_buffer = NULL; // Мы только принимаем (или поставьте буфер если надо слать мусор)
        trans_desc[i].rx_buffer = dma_chunks[i];
        trans_desc[i].user = (void*)i; // Сохраняем индекс, чтобы узнать его в ISR

        // Ставим в очередь драйвера СРАЗУ
        ret = spi_slave_queue_trans(RCV_HOST, &trans_desc[i], portMAX_DELAY);
        assert(ret == ESP_OK);
    }

    // 5. Запускаем задачу, которая будет разгребать данные
    // Приоритет должен быть достаточно высоким, но ниже ISR
    xTaskCreate(spi_processing_task, "spi_proc", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "SPI Slave Ready. Waiting for FPGA stream...");
}



#endif
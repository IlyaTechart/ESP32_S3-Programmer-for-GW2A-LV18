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
char *sendbuf = NULL;
char *recvbuf = NULL;

spi_message_t msg;

//Переменные для логов 
uint8_t LogBuffer[11] = {0};

//Функции для логов 

// Прототипы функций (чтобы init их видел)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans);
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans);
static void spi_driver_task(void *pvParameters); // Внутренняя задача драйвера

void spi_slave_init(void) {

    // 0. Выделяем DMA-capable память с правильным выравниванием для ESP32S3 (16 байт)
    sendbuf = heap_caps_aligned_alloc(32, BUFFER_SIZE, MALLOC_CAP_DMA);
    recvbuf = heap_caps_aligned_alloc(32, BUFFER_SIZE, MALLOC_CAP_DMA);
    
    if (!sendbuf || !recvbuf) {
        ESP_LOGE(TAG, "Failed to allocate DMA-capable buffers");
        return;
    }
    
    memset(sendbuf, 0, BUFFER_SIZE);
    memset(recvbuf, 0, BUFFER_SIZE);
    
    ESP_LOGI(TAG, "RX buffer @ %p, TX buffer @ %p", recvbuf, sendbuf);

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
        .max_transfer_sz = 65535,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO // Лучше AUTO
    };

    // 3. Настраиваем интерфейс слейва
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 10,
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

    // 5. ЗАПУСК ДРАЙВЕРА (Обязательно!)
    // Без этой задачи контроллер SPI не будет знать, куда принимать данные
    xTaskCreate(spi_driver_task, "spi_driver", (4096 + BUFFER_SIZE / 4), NULL, 5, NULL);


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
        volatile esp_err_t ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI trans failed: %s", esp_err_to_name(ret));
        }

        xSemaphoreTake(sema_for_driverTask, portMAX_DELAY);
    }
}

// Обработчик прерывания (вызывается ПОСЛЕ приема данных)
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Вычисляем сколько байт реально пришло
    volatile uint32_t bytes_rcv = trans->trans_len / 8;
    if (bytes_rcv > sizeof(msg.data)) bytes_rcv = sizeof(msg.data);
    
    // Копируем данные из буфера драйвера в сообщение очереди
//    memcpy(msg.data, trans->rx_buffer, bytes_rcv);
    msg.data = trans->rx_buffer;
    msg.len = bytes_rcv;

    // Отправляем в очередь
    xQueueSendFromISR(spi_evt_queue, &msg, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Пустышка для setup (вызывается ПЕРЕД транзакцией)
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(GPIO_HANDSHAKE, 0);
    // Здесь можно выставить GPIO в 1, чтобы сигнализировать Мастеру "Я готов"
}

// Задача обработки (Пользовательская логика)
void spi_processing_task(void *pvParameters) {
    ESP_LOGI(TAG, "SPI Processing task started");
    uint32_t cnt_msg;

    while(1) {
        // Ждем сообщения из очереди
        
         gpio_set_level(GPIO_HANDSHAKE, 1);
        if((xQueueReceive(spi_evt_queue, &msg, portMAX_DELAY)) == pdPASS) {
            for(uint32_t i = 0; i < BUFFER_SIZE; i++)
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
#endif

///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////



#if !GEMINI_CODE

static const char *TAG = "SPI_HANDLER";

// Магическая функция из ROM для принудительной записи кэша в RAM
extern void Cache_WriteBack_Addr(uint32_t addr, uint32_t size);


//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 0);
}

char *sendbuf = NULL;
char *recvbuf = NULL;

void spi_slave_esp_idf(void *pvParameters) {
        int n = 0;
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    // char *sendbuf = spi_bus_dma_memory_alloc(RCV_HOST, 129, 0);
    // char *recvbuf = spi_bus_dma_memory_alloc(RCV_HOST, 129, 0);
    
    //sendbuf = heap_caps_aligned_alloc(32, BUFFER_SIZE, MALLOC_CAP_DMA);
    // recvbuf = heap_caps_aligned_alloc(32, BUFFER_SIZE, MALLOC_CAP_DMA);
    //recvbuf = malloc(4);
    sendbuf = heap_caps_aligned_alloc(32, BUFFER_SIZE, MALLOC_CAP_DMA);
    recvbuf = heap_caps_aligned_alloc(32, BUFFER_SIZE, MALLOC_CAP_DMA);

    memset(sendbuf, 0x00, 4);
    memset(recvbuf, 0x00, 4);

    assert(sendbuf && recvbuf);
    spi_slave_transaction_t t = {0};

    while (1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0xA5, 4);
        sprintf(sendbuf, "This is the receiver, sending data for transmission number %04d.", n);

        //Cache_WriteBack_Addr((uint32_t)recvbuf, 4);

        //Set up a transaction of 128 bytes to send/receive
        t.length = 4 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.
        ESP_LOGI(TAG, "RX_Bufer_ISR: %d %d %d %d", recvbuf[0], recvbuf[1], recvbuf[2], recvbuf[3]);
        n++;
    }
}


#endif
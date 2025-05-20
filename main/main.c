#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "mqtt_client.h"
static const char *TAG= "APP";

static const char *TAG_UART_MQTT = "UART_MQTT";
static const char *TAG_MQTT = "CLIENT_MQTT";
static const char *TAG_GPRS = "GPRS";
static const char *TAG_UART = "UART";
static const char *TAG_UART_MODEM = "UART_MODEM";
static const char *TAG_UART_RS232 = "UART_RS232";


#define UART_MODEM_NUM     UART_NUM_1
#define UART_RS232_NUM     UART_NUM_0

#define UART_MODEM_TX_PIN 4
#define UART_MODEM_RX_PIN 5

#define UART_RS232_TX_PIN  10   // UART0 TX
#define UART_RS232_RX_PIN  9    // UART0 RX       

#define INTENTOSCONEXION 3

#define BUF_SIZE 1024
#define UART_BUF_SIZE 256
#define UART_TIMEOUT_MS 1000
#define COLA_TAMANO 10

// APN 
const char *apn = "wap.gprs.unifon.com.ar";
const char *gprsUser = "wap";
const char *gprsPass = "wap";
//MQTT
const char *mqttUri =       "mqtt://broker.hivemq.com";
const char *mqttTopicData = "dnv/contador1/transito";
const char *mqttTopicCmd  = "dnv/contador1/cmd";
const char *mqttTopicResp = "dnv/contador1/response";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static QueueHandle_t dataQueue;
//estado mqtt ready
static bool mqtt_ready = false;


// Prototipo del handler MQTT
static void mqtt_event_manejador(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

static bool uart_instalado_modem = false;

static bool uart_modem_init()
{



    if (uart_instalado_modem) {
        uart_driver_delete(UART_MODEM_NUM);
        uart_instalado_modem= false;
    }

    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t err = uart_param_config(UART_MODEM_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_MODEM, "Error en uart_param_config: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG_UART_MODEM, "paso uart_param_config");
    err = uart_set_pin(UART_MODEM_NUM, UART_MODEM_TX_PIN, UART_MODEM_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_MODEM, "Error en uart_set_pin: %s", esp_err_to_name(err));
        return false;
    }
     ESP_LOGI(TAG_UART_MODEM, "paso uart_set_pin");
    err = uart_driver_install(UART_MODEM_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_MODEM, "Error en uart_driver_install: %s", esp_err_to_name(err));
        return false;
    }

    uart_instalado_modem= true;

    ESP_LOGI(TAG_UART_MODEM, "UART MODEM inicializada");
    return true;
}


static bool uart_rs232_init()
{
 uart_config_t uart_config = {
       .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //IMPORTANTE
    .source_clk = UART_SCLK_APB  


    };
    esp_err_t err;
     ESP_LOGI(TAG_UART_RS232, "Iniciacizacion UART rs232");
     uart_driver_delete(UART_RS232_NUM);
    // UART para RS232
    err = uart_param_config(UART_RS232_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_RS232  , "Error en uart_param_config RS232: %s", esp_err_to_name(err));
         return false;
    }

    err = uart_set_pin(UART_RS232_NUM,UART_RS232_TX_PIN , UART_RS232_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_RS232  , "Error en uart_set_pin RS232: %s", esp_err_to_name(err));
         return false;
    }

    err = uart_driver_install(UART_RS232_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_RS232  , "Error en uart_driver_install RS232: %s", esp_err_to_name(err));
         return false;
    }
   
    ESP_LOGI(TAG_UART_RS232 , "Uart RS232 inicializada");

    return true;
}

static bool uart_init()
{
    
    if( !uart_modem_init() || !uart_rs232_init()){
        ESP_LOGE(TAG_UART, "No se pudo inicializar uarts.");
        return false;
    }
      
    ESP_LOGI(TAG_UART , "UARTs inicializadas correctamente.");
    return true;
}

static bool read_response_ok() {
    uint8_t buf[UART_BUF_SIZE] = {0};
    int len = uart_read_bytes(UART_RS232_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(UART_TIMEOUT_MS));

    if (len > 0) {
        buf[len] = '\0';  
        ESP_LOGI(TAG_UART_RS232, "Respuesta UART: %s", (char *)buf);

        if (strstr((char *)buf, "OK") != NULL) {
            return true;
        } else {
            ESP_LOGW(TAG_UART_RS232 , "No se encontró OK en la respuesta");
            return false;
        }
    } else {
        ESP_LOGE(TAG_UART_RS232, "No se recibió respuesta por UART");
        return false;
    }
}
    
// Enviar comando AT al módem
static void send_at_command(const char *cmd)
{
    uart_write_bytes(UART_MODEM_NUM, cmd, strlen(cmd));
    uart_write_bytes(UART_MODEM_NUM, "\r\n", 2);
    ESP_LOGI(TAG_UART_MODEM, "mandó el comando AT : %s", cmd);
}


static bool send_at_command_and_wait_ok(const char *cmd) {
    send_at_command(cmd);  
    if (read_response_ok()) {
        ESP_LOGI(TAG_UART_MODEM , "Comando AT exitoso");
        return true;
    } else {
        ESP_LOGE(TAG_UART_MODEM , "Comando AT falló");
        return false;
}
}

static void encolar(char* msg)
{
    if (xQueueSend(dataQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Error encolar mensaje RS232");
          if(msg != NULL){ 
            free(msg);
            msg = NULL;
            }
    }
}

// Tarea para leer respuestas del RS232 y ponerlas en cola
static void rs232_lectura_tarea(void *arg)
{
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_RS232_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = 0;

            char *msg = malloc(len + 1);
            if (msg != NULL) {
                memcpy(msg, data, len + 1);
                msg[len] = '\0';
                encolar(msg);
              
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//Crear cola para mensajes
static bool crear_cola(void)
{
    dataQueue = xQueueCreate(COLA_TAMANO, sizeof(char*));
    if (dataQueue == NULL) {
        ESP_LOGE(TAG, "No se pudo crear la cola");
        return false;
    }
    ESP_LOGI(TAG, "Se creo la cola");
    return true;
}

// Procesar mensajes de la cola para publicar en MQTT
static void enviar_datos_cola_mqtt()
{
    char *msg = NULL;
    if (xQueueReceive(dataQueue, &msg, pdMS_TO_TICKS(100))) {
        if (msg != NULL) {

           const char *contenido;
           ESP_LOGI(TAG_UART_MQTT, "Envio mensaje de la cola a mqtt: %s", msg);
           if (strncmp(msg, "RES:", 4)==0 ) {
              contenido = msg + 4;  // apunta al texto después de "TRN:"
              ESP_LOGI(TAG_UART_MQTT, "Publicando respuesta: %s", contenido);
              esp_mqtt_client_publish(mqtt_client, mqttTopicResp, contenido, 0, 1, 0);
           } else if (strncmp(msg, "TRN:", 4)== 0) {
              contenido = msg + 4;  // apunta al texto después de "TRN:"
              ESP_LOGI(TAG_UART_MQTT, "Publicando dato de tránsito: %s", contenido);
              esp_mqtt_client_publish(mqtt_client, mqttTopicData, contenido, 0, 1, 0); 
            
           } else {
             ESP_LOGW(TAG_UART_MQTT, "Mensaje RS232 desconocido o no clasificado: %s", msg);
            }
            if(msg != NULL){ 
            free(msg);
            msg = NULL;
            }
        }
    }
}

//manejador de eventos MQTT
static void mqtt_event_manejador(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(mqtt_client, mqttTopicCmd, 1);
            mqtt_ready = true;
            break;
        
            case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "MQTT desconectado");
            mqtt_ready = false;
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA received");
            if (event->data_len > 0 && event->data != NULL) {
                // Enviar el mensaje MQTT recibido por RS232
                uart_write_bytes(UART_RS232_NUM, event->data, event->data_len);
                uart_write_bytes(UART_RS232_NUM, "\r\n", 2);  // agregar salto de línea si tu dispositivo lo espera
                ESP_LOGI(TAG_UART_MQTT, "Enviado por RS232 lo que llego por mqtt: %.*s", event->data_len, event->data);
            }
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG_MQTT, "MQTT_EVENT_ERROR");
            break;

        default:
            ESP_LOGI(TAG_MQTT, "Otro evento MQTT: id=%li", event_id);
            break;
    }
}
// Inicializar MQTT y registrar evento
static bool mqtt_app_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqttUri,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG_MQTT, "Error al inicializar el cliente MQTT");
        return false;
    }

    esp_err_t err;

    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MQTT, "Error al iniciar el cliente MQTT: 0x%x", err);
        return false;
    }

    err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_manejador, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MQTT, "Error al registrar el manejador de eventos MQTT: 0x%x", err);
        return false;
    }
return true;
}

static bool gprs_is_connected()
{
    ESP_LOGI(TAG_GPRS, "Enviando comando AT+CIPSTATUS para verificar estado GPRS...");
    send_at_command("AT+CIPSTATUS");
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t buf[128] = {0};
    int len = uart_read_bytes(UART_MODEM_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(1000));
    if (len > 0) {
        buf[len] = '\0';
        ESP_LOGI(TAG_GPRS, "Respuesta AT+CIPSTATUS: %s", (char *)buf);

        if (strstr((char *)buf, "CONNECT OK")) {
            ESP_LOGI(TAG_GPRS, "Estado GPRS: CONECTADO (CONNECT OK)");
            return true;
        } else if (strstr((char *)buf, "IP STATUS")) {
            ESP_LOGI(TAG_GPRS, "Estado GPRS: IP asignada (IP STATUS)");
            return true;
        } else if (strstr((char *)buf, "ERROR")) {
            ESP_LOGE(TAG_GPRS, "Error reportado por el módem al consultar CIPSTATUS");
        } else {
            ESP_LOGW(TAG_GPRS, "Estado inesperado o desconectado: %s", (char *)buf);
        }
    } else {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta del módem tras enviar AT+CIPSTATUS");
    }

    return false;
}

static bool gprs_connect()
{
    ESP_LOGI(TAG_GPRS, "Chequeando registro en red con AT+CREG?...");
    send_at_command("AT+CREG?");
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t buf[64] = {0};
    int len = uart_read_bytes(UART_MODEM_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(1000));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta al consultar registro de red");
        return false;
    }
    buf[len] = '\0';
    ESP_LOGI(TAG_GPRS, "Respuesta AT+CREG?: %s", (char*)buf);

    if (!(strstr((char*)buf, "+CREG: 0,1") || strstr((char*)buf, "+CREG: 0,5"))) {
        ESP_LOGW(TAG_GPRS, "No registrado en red móvil");
        return false;
    }
    ESP_LOGI(TAG_GPRS, "Registrado en red móvil");

    ESP_LOGI(TAG_GPRS, "Enviando AT para probar comunicación...");
    if (!send_at_command_and_wait_ok("AT")) {
        ESP_LOGE(TAG_GPRS, "Error: no responde comando AT");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG_GPRS, "Activando GPRS con AT+CGATT=1...");
    if (!send_at_command_and_wait_ok("AT+CGATT=1")) {
        ESP_LOGE(TAG_GPRS, "Error: no se pudo activar GPRS (AT+CGATT=1)");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    char apn_cmd[64];
    snprintf(apn_cmd, sizeof(apn_cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, gprsUser, gprsPass);
    ESP_LOGI(TAG_GPRS, "Configurando APN con: %s", apn_cmd);
    if (!send_at_command_and_wait_ok(apn_cmd)) {
        ESP_LOGE(TAG_GPRS, "Error: no se pudo configurar APN");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG_GPRS, "Iniciando conexión con AT+CIICR...");
    if (!send_at_command_and_wait_ok("AT+CIICR")) {
        ESP_LOGE(TAG_GPRS, "Error: no se pudo iniciar conexión GPRS (AT+CIICR)");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG_GPRS, "Obteniendo IP con AT+CIFSR...");
    send_at_command("AT+CIFSR");
    vTaskDelay(pdMS_TO_TICKS(1000));
    memset(buf, 0, sizeof(buf));
    len = uart_read_bytes(UART_MODEM_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(1000));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se pudo obtener IP (AT+CIFSR)");
        return false;
    }
    buf[len] = '\0';
    ESP_LOGI(TAG_GPRS, "IP obtenida: %s", (char*)buf);

    if (strchr((char*)buf, '.') == NULL) {  // no hay punto, no es IP válida
        ESP_LOGE(TAG_GPRS, "IP obtenida inválida");
        return false;
    }
    ESP_LOGI(TAG_GPRS, "Conexión GPRS establecida correctamente");
    return true;
}

static void gprs_mqtt_task(void *pvParameters)
{
    bool mqtt_iniciado = false;
    bool gprs_conectado = false;

    while (1) {
        // Verificar estado de conexión GPRS
        gprs_conectado = gprs_is_connected();

        if (!gprs_conectado) {
            int intento = 0;
            mqtt_iniciado = false;

            // Intentar conectar GPRS
            gprs_connect();

            // Reintentos con espera
            while (!gprs_is_connected() && intento < INTENTOSCONEXION) {
                intento++;
                ESP_LOGW(TAG_GPRS, "Intento %d fallido en conexión GPRS. Esperando 5 segundos...", intento);
                vTaskDelay(pdMS_TO_TICKS(5000));
            }

            // Si se agotaron intentos
            if (intento == INTENTOSCONEXION) {
                ESP_LOGE(TAG_GPRS, "No se pudo reconectar GPRS después de %d intentos. Reintentando en 30 segundos...", INTENTOSCONEXION);
                vTaskDelay(pdMS_TO_TICKS(30000));
                continue; // Saltar esta iteración y volver a intentar después del delay
            }
        }

        // Si hay conexión GPRS y MQTT no iniciado, iniciar MQTT
        if (gprs_conectado && !mqtt_iniciado) {
            ESP_LOGI(TAG_MQTT, "Inicializando MQTT...");
            if (mqtt_app_init() == 0) {
                mqtt_iniciado = true;
                ESP_LOGI(TAG_MQTT, "MQTT inicializado correctamente.");
            } else {
                ESP_LOGE(TAG_MQTT, "Error al inicializar el cliente MQTT.");
                mqtt_iniciado = false;
            }
        }

        // Si MQTT está listo y la cola de datos no está vacía, enviar datos
        if (mqtt_iniciado && mqtt_ready && dataQueue != NULL) {
            enviar_datos_cola_mqtt();
        }

        // Esperar un poco para no saturar la CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}




void app_main(void)
{
    ESP_LOGI(TAG, "Inicio Aplicación...");

    if (!uart_init())
      {
        ESP_LOGE(TAG, "No se pudo inicializar uarts. Saliendo...");
        return;
      }
   
    
    if (!crear_cola())
      {
        ESP_LOGE(TAG, "No se pudo crear la cola. Saliendo...");
        return;
      }
    
   
      //Iniciar tarea para leer RS232
     if (xTaskCreate(rs232_lectura_tarea, "rs232_lectura_tarea", 4096, NULL, 10, NULL)!= pdPASS)
      {  
       ESP_LOGE(TAG, "No se pudo crear la tarea rs232_lectura_tarea");
       return;
      }
      //Iniciar tarea para comunicar gprs y mqtt
      if (xTaskCreate(gprs_mqtt_task, "gprs_mqtt_task", 8192, NULL, 9, NULL)!= pdPASS)
      {  
       ESP_LOGE(TAG, "No se pudo crear la tarea gprs_mqtt_task");
       return;
      }
    }
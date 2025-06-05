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
#include "esp_http_client.h"
#include "esp_netif.h"


#include "driver/uart.h"
//#include "mqtt_client.h"
static const char *TAG= "APP";

static const char *TAG_UART_MQTT = "UART_MQTT";
static const char *TAG_MQTT = "CLIENT_MQTT";
static const char *TAG_GPRS = "GPRS";
static const char *TAG_UART = "UART";
static const char *TAG_UART_MODEM = "UART_MODEM";
static const char *TAG_UART_RS232 = "UART_RS232";


#define UART_MODEM_NUM     UART_NUM_1
#define UART_RS232_NUM     UART_NUM_0

#define UART_MODEM_TX_PIN 5
#define UART_MODEM_RX_PIN 6


#define UART_RS232_TX_PIN  1    // UART0 TX
#define UART_RS232_RX_PIN  4   // UART0 RX       

#define INTENTOSCONEXION 3

#define BUF_SIZE 1024
#define UART_BUF_SIZE 256
#define UART_TIMEOUT_MS 1000
#define COLA_TAMANO 10

// APN 
const char *apn = "igprs.claro.com.ar";
const char *gprsUser = "";
const char *gprsPass = "";
//MQTT
const char *mqttUri =       "mqtt://broker.hivemq.com";
const char *mqttTopicData = "dnv/contador1/transito";
const char *mqttTopicCmd  = "dnv/contador1/cmd";
const char *mqttTopicResp = "dnv/contador1/response";

//static esp_mqtt_client_handle_t mqtt_client = NULL;
static QueueHandle_t dataQueue;
//estado mqtt ready
static bool mqtt_ready = false;


// Prototipo del handler MQTT
//static void mqtt_event_manejador(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

static bool uart_instalado_modem = false;
static bool uart_instalado_r232 = false;
static bool gprs_conectado=false;



static bool uart_modem_init()
{

    if (uart_instalado_modem) {
        uart_driver_delete(UART_MODEM_NUM);
        uart_instalado_modem= false;
    }

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };


    esp_err_t err; 
    err = uart_driver_install(UART_MODEM_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_MODEM, "Error en uart_driver_install: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG_UART_MODEM, "uart_driver_install OK");

     err = uart_param_config(UART_MODEM_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_MODEM, "Error en uart_param_config: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG_UART_MODEM, "uart_param_config OK");
    err = uart_set_pin(UART_MODEM_NUM, UART_MODEM_TX_PIN, UART_MODEM_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_MODEM, "Error en uart_set_pin: %s", esp_err_to_name(err));
        return false;
    }
     
     ESP_LOGI(TAG_UART_MODEM, "uart_set_pin OK");   
    
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

    if (uart_instalado_r232) {
        uart_driver_delete(UART_RS232_NUM);
        ESP_LOGI(TAG_UART_RS232, "Iniciacizacion UART rs232");
        uart_instalado_r232= false;
    }
     
    err = uart_driver_install(UART_RS232_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_RS232  , "Error en uart_driver_install RS232: %s", esp_err_to_name(err));
         return false;
    }
   
    ESP_LOGI(TAG_UART_RS232, "uart_driver_install OK");
    // UART para RS232
    err = uart_param_config(UART_RS232_NUM, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_RS232  , "Error en uart_param_config RS232: %s", esp_err_to_name(err));
         return false;
    }

    ESP_LOGI(TAG_UART_RS232, "uart_driver_install OK");

    err = uart_set_pin(UART_RS232_NUM,UART_RS232_TX_PIN , UART_RS232_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_UART_RS232  , "Error en uart_set_pin RS232: %s", esp_err_to_name(err));
         return false;
    }
    
    ESP_LOGI(TAG_UART_RS232, "uart_set_pin OK");
  
    ESP_LOGI(TAG_UART_RS232 , "Uart RS232 inicializada");
    uart_instalado_r232 = true;
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


// Enviar comando 

static void send_at_command(const char* t,uart_port_t un, const char *cmd) {
    // Añadir terminación CR+LF si no está
    char cmd_with_crlf[128];
    snprintf(cmd_with_crlf, sizeof(cmd_with_crlf), "%s\r\n", cmd);
    uart_write_bytes(un, cmd_with_crlf, strlen(cmd_with_crlf));
    ESP_LOGI(t, "Comando enviado: %s", cmd_with_crlf);
}




static bool send_at_command_and_wait_ok(const char *cmd, uint32_t timeout_ms ,const char* t,uart_port_t un) {
    // Limpiar buffer UART antes de enviar comando
    uart_flush(un);

  
    send_at_command(t,un,cmd);

    char resp[128] = {0};
    size_t len = 0;
    uint32_t start_tick = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
        int r = uart_read_bytes(un, (uint8_t*)(resp + len), sizeof(resp) - len - 1, pdMS_TO_TICKS(100));
        if (r > 0) {
            len += r;
            resp[len] = '\0';

            if (strstr(resp, "OK") != NULL) {
                ESP_LOGI(t, "Respuesta OK recibida: %s", resp);
                return true;
            }
            
            if (strstr(resp,"CONNECT OK") != NULL) {
                ESP_LOGI(t, "Respuesta OK recibida: %s", resp);
                return true;
            }

             if (strstr(resp,">") != NULL) {
                ESP_LOGI(t, "Respuesta OK recibida: %s", resp);
                return true;
            }
           


            if (strstr(resp, "ERROR") != NULL) {
                ESP_LOGE(t, "Respuesta ERROR recibida: %s", resp);
                return false;
            }
        }
    }

    ESP_LOGE(t, "Timeout esperando OK o ERROR: %s", resp);
    return false;
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
/*
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
    */
/*
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

    // Registrar manejador antes de iniciar
    err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_manejador, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MQTT, "Error al registrar el manejador de eventos MQTT: 0x%x", err);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return false;
    }

    // Iniciar cliente MQTT
    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MQTT, "Error al iniciar el cliente MQTT: 0x%x", err);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return false;
    }

    ESP_LOGI(TAG_MQTT, "Cliente MQTT iniciado correctamente");
    return true;
}
    */
static bool gprs_is_connected() {
    char buf[64];
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CGATT?");
    vTaskDelay(pdMS_TO_TICKS(500));
    int len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) return false;
    buf[len] = 0;
    return (strstr(buf, "+CGATT: 1") != NULL);
}

static bool tcp_is_connected() {
    char buf[128];
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CIPSTATUS");
    vTaskDelay(pdMS_TO_TICKS(500));
    int len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) return false;
    buf[len] = 0;
    // Estado típico "STATE: CONNECT OK" si está conectado
    return (strstr(buf, "STATE: CONNECT OK") != NULL);
}


static bool gprs_connect()
{   uart_flush(UART_MODEM_NUM);
    ESP_LOGI(TAG_GPRS, "Verificando comunicación con AT...");
    if (!send_at_command_and_wait_ok("AT",1000,TAG_GPRS,UART_MODEM_NUM)) {
        ESP_LOGE(TAG_GPRS, "Error: el módulo no responde a AT");
        return false;
    }

   
    
    ESP_LOGI(TAG_GPRS, "Verificando estado de SIM con AT+CPIN?...");
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS,UART_MODEM_NUM,"AT+CPIN?");
    vTaskDelay(pdMS_TO_TICKS(500));
    char buf[64] = {0};
    int len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CPIN?");
        return false;
    }
    buf[len] = 0;
    if (strstr(buf, "READY") == NULL) {
        ESP_LOGE(TAG_GPRS, "SIM no lista o no detectada: %s", buf);
        return false;
    }

    ESP_LOGI(TAG_GPRS, "Chequeando calidad de señal con AT+CSQ...");
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS,UART_MODEM_NUM,"AT+CSQ");
    vTaskDelay(pdMS_TO_TICKS(500));
    memset(buf, 0, sizeof(buf));
    len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CSQ");
        return false;
    }
    buf[len] = 0;
    if (strstr(buf, "+CSQ") == NULL) {
        ESP_LOGE(TAG_GPRS, "No se pudo obtener calidad de señal: %s", buf);
        return false;
    }
    ESP_LOGI(TAG_GPRS, "Calidad de señal: %s", buf);

    ESP_LOGI(TAG_GPRS, "Verificando registro en red con AT+CREG?...");
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS,UART_MODEM_NUM,"AT+CREG?");
    vTaskDelay(pdMS_TO_TICKS(500));
    memset(buf, 0, sizeof(buf));
    len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CREG?");
        return false;
    }
    buf[len] = 0;
    if (!(strstr(buf, "+CGREG: 0,1") || strstr(buf, "+CGREG: 0,5"))) {
         ESP_LOGW(TAG_GPRS, "No registrado en red");
    } 
    else 
    {
        send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CIPGSMLOC=1,1");
        vTaskDelay(pdMS_TO_TICKS(500)); 
        len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
        buf[len] = 0;
        ESP_LOGI(TAG, "Ubicación: %s", buf);
    }
    ESP_LOGI(TAG_GPRS, "Activando GAT+CGREG?...");
    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CGREG?");
    vTaskDelay(pdMS_TO_TICKS(500));
    memset(buf, 0, sizeof(buf));
    len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) {
       ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CGREG?");
       }
    buf[len] = 0;




    ESP_LOGI(TAG_GPRS, "Activando GPRS con AT+CGATT=1...");
    if (!send_at_command_and_wait_ok("AT+CGATT=1", 2000,TAG_GPRS,UART_MODEM_NUM)) {
        ESP_LOGE(TAG_GPRS, "Error: no se pudo activar GPRS (AT+CGATT=1)");
        return false;
    }

    char apn_cmd[64];
    snprintf(apn_cmd, sizeof(apn_cmd), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, gprsUser, gprsPass);
    ESP_LOGI(TAG_GPRS, "Configurando APN con: %s", apn_cmd);
    if (!send_at_command_and_wait_ok(apn_cmd, 2000,TAG_GPRS,UART_MODEM_NUM)) {
        ESP_LOGE(TAG_GPRS, "Error: no se pudo configurar APN");
        return false;
    }

    ESP_LOGI(TAG_GPRS, "Iniciando conexión con AT+CIICR...");
    if (!send_at_command_and_wait_ok("AT+CIICR", 5000,TAG_GPRS,UART_MODEM_NUM)) {
        ESP_LOGE(TAG_GPRS, "Error: no se pudo iniciar conexión GPRS (AT+CIICR)");
        return false;
    }

    ESP_LOGI(TAG_GPRS, "Obteniendo IP con AT+CIFSR...");
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS,UART_MODEM_NUM,"AT+CIFSR");
    vTaskDelay(pdMS_TO_TICKS(1000));
    memset(buf, 0, sizeof(buf));
    len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1000));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CIFSR");
        return false;
    }
    buf[len] = 0;
    if (strchr(buf, '.') == NULL) {
        ESP_LOGE(TAG_GPRS, "IP obtenida inválida o no disponible: %s", buf);
        return false;
    }

    ESP_LOGI(TAG_GPRS, "IP obtenida: %s", buf);
    ESP_LOGI(TAG_GPRS, "Conexión GPRS establecida correctamente");
    // 2. Configuración inicial crítica (¡NUEVO!)
    ESP_LOGI(TAG_GPRS, "Configurando módem...");
    if (!send_at_command_and_wait_ok("ATE0",1000,TAG_GPRS,UART_MODEM_NUM)) {
        ESP_LOGE(TAG_GPRS, "Error: modem init");
        return false;
    }    // Desactiva el eco
  if (!send_at_command_and_wait_ok("AT+CMEE=1",1000,TAG_GPRS,UART_MODEM_NUM)) { 
  ESP_LOGE(TAG_GPRS, "Error: modem init");
        return false;
    } // Habilita códigos de error detallados


    if (!send_at_command_and_wait_ok("AT+CIPHEAD=0",1000,TAG_GPRS,UART_MODEM_NUM)) { 
          ESP_LOGE(TAG_GPRS, "Error: modem init");
        return false;
    }    // Sin cabeceras extra

   

    return true;
}

static bool tcp_connect(const char* host, int port) {
    
    char buf[64] = {0};
    char cmd[128];
    uart_flush(UART_MODEM_NUM);
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d", host, port);
    send_at_command(TAG_GPRS,UART_MODEM_NUM,cmd);   
    vTaskDelay(pdMS_TO_TICKS(5000));
    int len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(5000));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CIPSTART");
        return false;
    }
    buf[len] = 0;
    if (!strstr(buf, "OK")|| !strstr(buf, "CONNECT OK") ) {
        ESP_LOGW(TAG_GPRS, "No funciono AT+CIPSTART: %s", buf);
        return false;
    }

    ESP_LOGI(TAG_GPRS, "Conexión TCP realizada");
    return true;



}

static bool tcp_send(char* data,unsigned int len) {

char buf[128] = {0};

uart_flush(UART_MODEM_NUM);
// 2. Enviar comando para enviar datos
send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CIPSEND");



// Esperar el símbolo '>' dinámicamente
bool prompt_found = false;
int total = 0;
TickType_t start = xTaskGetTickCount();

while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(5000)) {
    int len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf + total, sizeof(buf) - total, pdMS_TO_TICKS(500));
    if (len > 0) {
        total += len;
        for (int i = 0; i < total; i++) {
            if (buf[i] == '>') {
                prompt_found = true;
                break;
            }
  

            // Verificar si ya está cerrado antes de enviar
            if (strstr(buf, "CLOSED")) {
                ESP_LOGW(TAG_GPRS, "Conexión cerrada antes de enviar");
                return false;
            }

        }
        if (prompt_found) break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

if (!prompt_found) {
    ESP_LOGE(TAG_GPRS, "No se recibió el símbolo '>' después de AT+CIPSEND");
    return false;
}


uart_flush(UART_MODEM_NUM);
ESP_LOGI("HTTP", "Enviando solicitud:\n%s", data);
ESP_LOGI("HTTP", "Longitud: %u", len);
uart_write_bytes(UART_MODEM_NUM, data, len);

//vTaskDelay(pdMS_TO_TICKS(100));  // Esperar un poco


const char end_char = 0x1A;  // Ctrl+Z
uart_write_bytes(UART_MODEM_NUM, &end_char, 1);
uart_flush(UART_MODEM_NUM);
vTaskDelay(pdMS_TO_TICKS(100));

len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(5000));
if (len <= 0) {
    ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CIPSEND");
    return false;
}
 buf[len] = 0;
 if ( !strstr(buf,"SEND OK") ) {
         ESP_LOGW(TAG_GPRS, "No funciono AT+CIPSEND: %s", buf);
        return false;
    }
ESP_LOGI(TAG_GPRS, "Se envió correctamente");
return true;

}

static bool gprs_disconnect() {

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (!send_at_command_and_wait_ok("AT+CIPSHUT", 5000, TAG_GPRS, UART_MODEM_NUM)) {
        ESP_LOGE(TAG_GPRS, "No se pudo cerrar conexión GPRS");
        return false;
    }
    ESP_LOGI(TAG_GPRS, "Conexión GPRS cerrada");
    return true;
}

static void gprs_mqtt_task(void *pvParameters)
{
//const char *host = "httpbin.org";
//const char *http_request = "GET /get HTTP/1.1\r\nHost: httpbin.org\r\n\r\n";

//const char *host = "api.ipify.org";
//const char *http_request =     "GET /?format=json HTTP/1.1\r\nHost: api.ipify.org\r\n\r\n";
//const char *host = "worldtimeapi.org";
//const char *http_request = "GET / HTTP/1.1\r\nHost: worldtimeapi.org\r\n\r\n";
//unsigned int len=  strlen(http_request);
//int port = 80;

const char* host = "io.adafruit.com";
const int port = 80;

const char *body = "{\"value\":\"{\\\"id\\\":\\\"1\\\",\\\"carril\\\":\\\"1\\\",\\\"liv\\\":\\\"64\\\"}\"}";



char http_request[512]; // suficiente para todo
sprintf(http_request,
  "POST /api/v2/km100fuegos/feeds/transito/data HTTP/1.1\r\n"
  "Host: io.adafruit.com\r\n"

  "Content-Type: application/json\r\n"
  "Content-Length: %d\r\n"
  "Connection: close\r\n"
  "\r\n"
  "%s", strlen(body), body);

 unsigned int len=  strlen(http_request);

/*const char *host="test.mosquitto.org";
const uint8_t connect_packet[] = {
    0x10, 0x12, 0x00, 0x04, 'M','Q','T','T', 0x04, 0x02, 0x00, 0x3C, 0x00, 0x04, 't','e','s','t'
};
*/
//signed int len = sizeof(connect_packet);
//int port =  1883;
    
    bool mqtt_iniciado = false;
    bool gprs_conectado = false;


    
    //gprs_connect();

    while (1) {
        // Verificar estado de conexión GPRS
        //gprs_conectado = gprs_is_connected();

         gprs_connect();
        //tcp_connect(host, port);
        //tcp_send(http_request);
        //*/
        // . Leer respuesta
       /* char response[512];
        int l  = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)response, sizeof(response)-1, 5000);
        if (l > 0) {
         response[l] = '\0';
         ESP_LOGI(TAG_GPRS,"Respuesta HTTP: %s\n", response);
         } else {
            ESP_LOGE(TAG_GPRS,"No se recibió respuesta del servidor\n");
        }
        tcp_disconnect();
        gprs_disconnect();
       */
    
      // if (!gprs_is_connected()) {
            ESP_LOGW(TAG_GPRS, "GPRS desconectado, reconectando...");
            /*if (!gprs_connect()) {
                ESP_LOGE(TAG_GPRS, "No se pudo conectar GPRS, reintentando en 5s...");
                vTaskDelay(pdMS_TO_TICKS(5000));
                gprs_disconnect();
                continue;
            }
            ESP_LOGI(TAG_GPRS, "GPRS conectado");
      //  }
*/
        if (!tcp_is_connected()) {
            ESP_LOGW(TAG_GPRS, "TCP desconectado, conectando...");
            if (!tcp_connect(host, port)) {
                ESP_LOGE(TAG_GPRS, "No se pudo conectar TCP, reintentando en 5s...");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            ESP_LOGI(TAG_GPRS, "TCP conectado");
        }

         // Enviar datos

    
        if (!tcp_send(http_request,len)) {
            ESP_LOGE(TAG_GPRS, "Error enviando datos, cerrando TCP");
            //tcp_disconnect();
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        
        char response[1272];
        int l  = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)response, sizeof(response)-1, 5000);
        if (l > 0) {
         response[l] = '\0';
         ESP_LOGI(TAG_GPRS,"Respuesta: %s\n", response);
         } else {
            ESP_LOGE(TAG_GPRS,"No se recibió respuesta del servidor\n");
        }
        
        //tcp_disconnect();
        gprs_disconnect();
          
        vTaskDelay(pdMS_TO_TICKS(2000));
        // Si hay conexión GPRS y MQTT no iniciado, iniciar MQTT
      /* if (gprs_conectado && !mqtt_iniciado) {
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
     */ 
        // Esperar un poco para no saturar la CPU
       
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
     /* if (xTaskCreate(rs232_lectura_tarea, "rs232_lectura_tarea", 4096, NULL, 10, NULL)!= pdPASS)
      {  
       ESP_LOGE(TAG, "No se pudo crear la tarea rs232_lectura_tarea");
       return;
      }
    */
      //Iniciar tarea para comunicar gprs y mqtt
      if (xTaskCreate(gprs_mqtt_task, "gprs_mqtt_task", 8192, NULL, 9, NULL)!= pdPASS)
      {  
       ESP_LOGE(TAG, "No se pudo crear la tarea gprs_mqtt_task");
       return;
      }
    }
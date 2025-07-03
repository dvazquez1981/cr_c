#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "esp_http_client.h"
#include "esp_netif.h"

#include "esp_sntp.h"
#include <time.h>

#include "esp_log.h"

#include "mbedtls/aes.h"
#include "mbedtls/base64.h"

#include "driver/uart.h"
//#include "mqtt_client.h"
static const char *TAG= "APP_ESP32C3_TST";


static const char *TAG_MQTT = "CLIENT_MQTT";
static const char *TAG_GPRS = "GPRS";
static const char *TAG_UART = "UART";
static const char *TAG_UART_MODEM = "UART_MODEM";
static const char *TAG_UART_RS232 = "UART_RS232";

static TickType_t last_activity = 0;

//uarts
#define UART_MODEM_NUM     UART_NUM_1
#define UART_RS232_NUM     UART_NUM_0
//pines modem
#define UART_MODEM_TX_PIN 5
#define UART_MODEM_RX_PIN 6

//pines rs232 del contador.
#define UART_RS232_TX_PIN  1   // UART0 TX
#define UART_RS232_RX_PIN  4   // UART0 RX       



#define BUF_SIZE 512
#define UART_BUF_SIZE 256
#define UART_TIMEOUT_MS 1000
#define COLA_TAMANO 10
#define MENSAJE_TAMANO 128


static volatile bool encriptado=true;  

// Clave de 16 bytes
const uint8_t aes_key[16] = "mikeysecreto1234";  
// APN 
const char *apn = "igprs.claro.com.ar";
const char *gprsUser = "";
const char *gprsPass = "";

//MQTT
//señal de seguridad
static volatile bool mqtt_can_publish = true;  

static volatile SemaphoreHandle_t uart_mutex;
static volatile bool received_pingresp = false;

static struct    tm timeFromSIM800l; 

//const char* host = "broker.hivemq.com";
static const char* host = "test.mosquitto.org";
static int port =  1883;



const char *mqttTopicData = "dispositivo/123/datos";
const char *mqttTopicCmd  = "dispositivo/123/comando";
//const char *mqttTopicResp = "dispositivo/123/respuesta ";


static QueueHandle_t dataQueue;



static   bool uart_instalado_modem = false;
static   bool uart_instalado_r232  = false;
static   bool gprs_conected=false;
volatile bool tcp_conected=false;


#define MAX_MQTT_PACKET_SIZE 512  // ajustable si necesitás más

//UART
// semaforo de UART
static bool init_uart_mutex() {
    uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL) {
        ESP_LOGE(TAG_UART, "Error creando mutex UART");
        return false;
    }
    return true;
}
//inicio de UART MODEM
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

//UART para RS232
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
//Inicio de ambas UARTS
static bool uart_init()
{
    
    if( !uart_modem_init() || !uart_rs232_init()){
        ESP_LOGE(TAG_UART, "No se pudo inicializar uarts.");
        return false;
    }
      
    ESP_LOGI(TAG_UART , "UARTs inicializadas correctamente.");
    return true;
}

//Manejo con el modem
// Enviar comando 
static void send_at_command(const char* t,uart_port_t un, const char *cmd) {
    // Añadir terminación CR+LF si no está
    char cmd_with_crlf[128];
    snprintf(cmd_with_crlf, sizeof(cmd_with_crlf), "%s\r\n", cmd);
    uart_write_bytes(un, cmd_with_crlf, strlen(cmd_with_crlf));
    ESP_LOGI(t, "Comando enviado: %s", cmd_with_crlf);
}


// Enviar comando con wait ok
static bool send_at_command_and_wait_ok(const char *cmd, uint32_t timeout_ms ,const char* t,uart_port_t un) {
    //Limpiar buffer UART antes de enviar comando
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


//manejo de la cola
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

static char* desencolar()
{
    char* msg_recibido = NULL;

    if (xQueueReceive(dataQueue, &msg_recibido, pdMS_TO_TICKS(100)) == pdPASS) {
        return msg_recibido;
    }
    return NULL;
}

//AES-128 en modo ECB con padding PKCS#7 
static char* encrypt_payload_base64(const char* input,const uint8_t * aeskey) {
    size_t input_len = strlen(input);
    size_t padded_len = ((input_len / 16) + 1) * 16;  // PKCS#7 padding
    uint8_t* padded_input = calloc(1, padded_len);
    if (!padded_input) return NULL;

    memcpy(padded_input, input, input_len);

    uint8_t pad = padded_len - input_len;
    for (int i = input_len; i < padded_len; i++) {
        padded_input[i] = pad;
    }

    uint8_t* encrypted = calloc(1, padded_len);
    if (!encrypted) {
        free(padded_input);
        return NULL;
    }

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, aeskey, 128);

    for (int i = 0; i < padded_len; i += 16) {
        mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, padded_input + i, encrypted + i);
    }

    mbedtls_aes_free(&aes);
    free(padded_input);

    // Base64
    size_t b64_len = 0;
    char* b64_output = calloc(1, (padded_len * 4 / 3) + 5);  // base64 puede crecer hasta ~1.37x
    if (!b64_output) {
        free(encrypted);
        return NULL;
    }

    mbedtls_base64_encode((unsigned char*)b64_output, (padded_len * 4 / 3) + 5, &b64_len, encrypted, padded_len);

    free(encrypted);
    return b64_output;  
}



struct tm parse_cclk(const char* cclk_str) {
    struct tm t = {0};
    int year, month, day, hour, min, sec, tz_quarters = 0;

    const char* ptr = strstr(cclk_str, "+CCLK:");
    if (!ptr) {
        ESP_LOGW(TAG_GPRS, "No se encontró +CCLK en la respuesta");
        return t;
    }

    if (sscanf(ptr, "+CCLK: \"%2d/%2d/%2d,%2d:%2d:%2d%3d\"",
               &year, &month, &day, &hour, &min, &sec, &tz_quarters) == 7  &&  (year + 2000)>2020 ) {

        
        t.tm_year = (year < 70 ? 2000 + year : 1900 + year) - 1900;
     
        t.tm_mon  = month - 1;
        t.tm_mday = day;
        t.tm_hour = hour;
        t.tm_min  = min;
        t.tm_sec  = sec;
     // No tocar más la hora acá. El ajuste vendrá después con TZ.
    
    } else if ((year + 2000)<2020)
    {
      ESP_LOGW(TAG_GPRS, "Fecha inválida detectada (%d), esperando hora válida", (year + 2000));

    }
    else {
        ESP_LOGW(TAG_GPRS, "Error al parsear +CCLK");
    }

    return t;
}


//Tarea para leer respuestas del RS232 y ponerlas en cola
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

static bool tcp_disconnect() {
    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CIPCLOSE");

    char buf[64];
    TickType_t start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(3000)) {
        int n = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf)-1, pdMS_TO_TICKS(100));
        if (n > 0) {
            buf[n] = '\0';
            if (strstr(buf, "CLOSE OK") || strstr(buf, "CLOSED")) {
                ESP_LOGI(TAG_GPRS, "TCP desconectado exitosamente");
                return true;
            }
        }
    }

    ESP_LOGW(TAG_GPRS, "Timeout esperando respuesta a CIPCLOSE");
    return false;
}



static bool tcp_is_connected() {
    char buf[128];
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CIPSTATUS");
    vTaskDelay(pdMS_TO_TICKS(500));
    int len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) return false;
    buf[len] = 0;
    
    if (strstr((char*)buf, "STATE: CONNECT OK")) {
      ESP_LOGI(TAG_GPRS, "STATE: CONNECT OK");
      
      return true; 
    }

    // Estado típico "STATE: CONNECT OK" si está conectado
    ESP_LOGE(TAG_GPRS, "Error: STATE: CONNECT OK");

    return false;
}


static bool gprs_connect()
{   
    
    uart_flush(UART_MODEM_NUM);
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


    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CLTS=1");
    vTaskDelay(pdMS_TO_TICKS(500));
    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT&W"); // guarda config
    vTaskDelay(pdMS_TO_TICKS(500));


    ESP_LOGI(TAG_GPRS, "Verificando registro en red con AT+CREG?...");
    uart_flush(UART_MODEM_NUM);
    send_at_command(TAG_GPRS,UART_MODEM_NUM,"AT+CREG?");
    vTaskDelay(pdMS_TO_TICKS(500));
    memset(buf, 0, sizeof(buf));
    len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(2000));
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

    send_at_command(TAG_GPRS, UART_MODEM_NUM, "AT+CCLK?");
    vTaskDelay(pdMS_TO_TICKS(500));
    memset(buf, 0, sizeof(buf));
    len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(1500));
    if (len <= 0) {
       ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CCLK?");
       }
    else
    { buf[len] = 0;
      timeFromSIM800l=parse_cclk(buf);
    }
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
    }   
    // Desactiva el eco
  if (!send_at_command_and_wait_ok("AT+CMEE=1",1000,TAG_GPRS,UART_MODEM_NUM)) { 
  ESP_LOGE(TAG_GPRS, "Error: modem init");
        return false;
    } 

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

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d", host,port);
   // snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d,%d", host, port, CONNECTION_TIMEOUT);
    send_at_command(TAG_GPRS,UART_MODEM_NUM,cmd);   
    vTaskDelay(pdMS_TO_TICKS(3000));
    int len = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf) - 1, pdMS_TO_TICKS(5000));
    if (len <= 0) {
        ESP_LOGE(TAG_GPRS, "No se recibió respuesta a AT+CIPSTART");
        return false;
    }
    buf[len] = 0;
   if (!(strstr(buf, "OK") && strstr(buf, "CONNECT OK"))) {
        ESP_LOGW(TAG_GPRS, "No funciono AT+CIPSTART: %s", buf);
        return false;
    }

    ESP_LOGI(TAG_GPRS, "Conexión TCP realizada");
    return true;



}

static bool wait_for_mqtt_connack(TickType_t timeout_ms) {
    uint8_t buf[64];
    size_t idx = 0;
    TickType_t start = xTaskGetTickCount();

    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(timeout_ms)) {
        int n = uart_read_bytes(UART_MODEM_NUM, buf + idx, sizeof(buf) - idx, pdMS_TO_TICKS(100));
        if (n > 0) {
            idx += n;
            ESP_LOG_BUFFER_HEXDUMP(TAG_MQTT, buf, idx, ESP_LOG_INFO);
            for (size_t i = 0; i + 3 < idx; ++i) {
                if (buf[i]   == 0x20 &&
                    buf[i+1] == 0x02 &&
                    (buf[i+2] == 0x00 || buf[i+2] == 0x01) && // Session Present: 0 o 1
                    buf[i+3] == 0x00) {
                    ESP_LOGI(TAG_GPRS, "CONNACK recibido, MQTT conectado");
                 
                    return true;
                }
            }
            if (strstr((char*)buf, "CLOSED")) {
                ESP_LOGW(TAG_MQTT, "Broker cerró antes de enviar CONNACK");
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGW(TAG_MQTT , "No se recibió CONNACK en %lu ms", timeout_ms);
    return false;
}

static bool tcp_send(const uint8_t* data, unsigned int len) {
    char buf[128];
    char cmd[32];
    
    // 1) Solicitar envío de “len” bytes
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", len);
    send_at_command(TAG_GPRS, UART_MODEM_NUM, cmd);

    // 2) Esperar prompt ‘>’
    TickType_t start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(7000)) {
        int n = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf)-1, pdMS_TO_TICKS(100));
        if (n > 0 && strchr(buf, '>')) {
            break;
        }
    }
    if (!strchr(buf, '>')) {
        ESP_LOGE(TAG_GPRS, "No vino '>' tras CIPSEND");
        return false;
    }

    //Enviar dato binario
    ESP_LOG_BUFFER_HEXDUMP(TAG_GPRS, data, len, ESP_LOG_INFO);
    uart_write_bytes(UART_MODEM_NUM, (const char*)data, len);

    //Confirmar SEND OK
    start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(7000)) {
        int n = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf)-1, pdMS_TO_TICKS(25));
        if (n > 0) {
            buf[n] = '\0';
            char* pos_ok = strstr(buf, "SEND OK");
            if (pos_ok)  {
                ESP_LOGI(TAG_GPRS, "CIPSEND OK: %s", buf);
                return true;
            }
            if (strstr(buf, "ERROR") || strstr(buf, "CLOSED")) {
                ESP_LOGW(TAG_GPRS, "Fallo tras CIPSEND: %s", buf);
                return false;
            }
        }
    }
    ESP_LOGE(TAG_GPRS, "Timeout esperando SEND OK");
    return false;
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


static bool mqtt_publish(const char* topic, const char* payload) {

if (!mqtt_can_publish) {
        ESP_LOGW(TAG_MQTT , "Publicacion bloqueada temporalmente (esperando PINGRESP)");
        return false;
    }
 if(uart_mutex == NULL) {
    ESP_LOGE(TAG_MQTT, "uart_mutex es NULL");
    return false;
 }

 if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {

    uint16_t topic_len   = strlen(topic);
    uint16_t payload_len = strlen(payload);
    uint16_t rem_len     = 2 + topic_len + payload_len;

    if (rem_len + 2 > MAX_MQTT_PACKET_SIZE) {
        ESP_LOGE(TAG_MQTT, "MQTT publish demasiado largo: %u bytes", rem_len + 2);
        xSemaphoreGive(uart_mutex); 
        return false;
    }

    uint8_t publish_packet[MAX_MQTT_PACKET_SIZE];
    size_t  publish_len = 2 + rem_len;

    publish_packet[0] = 0x30;         // PUBLISH, QoS 0
    publish_packet[1] = rem_len;
    publish_packet[2] = (topic_len >> 8) & 0xFF;
    publish_packet[3] = (topic_len     ) & 0xFF;

    memcpy(&publish_packet[4], topic, topic_len);
    memcpy(&publish_packet[4 + topic_len], payload, payload_len);

    if (!tcp_send(publish_packet, publish_len)) {
        ESP_LOGE(TAG_MQTT, "Error enviando PUBLISH: %s -> %s", topic, payload);
        xSemaphoreGive(uart_mutex); 
        return false;
    }

    ESP_LOGI(TAG_MQTT, "PUBLISH enviado: %s -> %s", topic, payload);
    xSemaphoreGive(uart_mutex);
    return true;
    } else {
        ESP_LOGW(TAG, "No se pudo tomar mutex UART para publicar");
        return false;
    }
}

static bool mqtt_subscribe(const char* topic) {

      if (!mqtt_can_publish) {
        ESP_LOGW(TAG_MQTT, "subscripcion bloqueada temporalmente (esperando PINGRESP)");
        return false;
    }
    
    if (uart_mutex == NULL) {
    ESP_LOGE(TAG_MQTT, "uart_mutex es NULL");
    return false;
    }

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    uint16_t topic_len = strlen(topic);
    uint16_t rem_len = 2 + 2 + topic_len + 1; // MsgID (2) + TopicLen (2) + topic + QoS (1)
    
    if (rem_len + 2 > MAX_MQTT_PACKET_SIZE) {
        ESP_LOGE(TAG_GPRS, "MQTT subscribe demasiado largo");
        xSemaphoreGive(uart_mutex); 
        return false;
    }

    uint8_t subscribe_packet[MAX_MQTT_PACKET_SIZE];
    size_t  packet_len = 2 + rem_len;

    subscribe_packet[0] = 0x82; // SUBSCRIBE packet
    subscribe_packet[1] = rem_len;

    // Message ID = 0x0001 (puede cambiarse si necesitás distinguir respuestas)
    subscribe_packet[2] = 0x00;
    subscribe_packet[3] = 0x01;

    // Topic
    subscribe_packet[4] = (topic_len >> 8) & 0xFF;
    subscribe_packet[5] = (topic_len     ) & 0xFF;
    memcpy(&subscribe_packet[6], topic, topic_len);

    // QoS
    subscribe_packet[6 + topic_len] = 0x00;

    if (!tcp_send(subscribe_packet, packet_len)) {
        ESP_LOGE(TAG_MQTT, "Error enviando SUBSCRIBE a %s", topic);
        xSemaphoreGive(uart_mutex); 
        return false;
    }

    ESP_LOGI(TAG_MQTT, "SUBSCRIBE enviado: %s", topic);
    xSemaphoreGive(uart_mutex);
    return true;
     } else {
        ESP_LOGW(TAG_MQTT, "No se pudo tomar mutex UART para subscribir");
        return false;
    }
}

static int tcp_receive(uint8_t* buffer, size_t max_len, TickType_t timeout_ms) {
    size_t total = 0;
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        int n = uart_read_bytes(UART_MODEM_NUM, buffer + total, max_len - total, pdMS_TO_TICKS(100));
        if (n > 0) {
            total += n;
            // si recibimos al menos 2 bytes, podríamos saber el largo MQTT
            if (total >= 2) {
                uint8_t rem_len = buffer[1];
                size_t packet_len = 2 + rem_len;

                if (total >= packet_len) {
                    // Ya recibimos todo el paquete
                    return packet_len;
                }
            }
        }
    }

    // Timeout o paquete incompleto
    return total > 0 ? total : -1;
}


int uart_read_exact(uint8_t *buf, int len, int timeout_ms) {
    int total = 0;
    TickType_t start = xTaskGetTickCount();

    while (total < len) {
        int r = uart_read_bytes(UART_MODEM_NUM, buf + total, len - total, pdMS_TO_TICKS(100));
        if (r > 0) total += r;
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) break;
    }

    return total;
}


int read_remaining_length(uint32_t* length, int* consumed_bytes) {
    *length = 0;
    *consumed_bytes = 0;
    int multiplier = 1;
    uint8_t byte;

    do {
        if (uart_read_exact(&byte, 1, 1000) != 1) return -1;
        *length += (byte & 0x7F) * multiplier;
        multiplier *= 128;
        (*consumed_bytes)++;

        if (*consumed_bytes > 4) return -2; // Error: too long
    } while (byte & 0x80);

    return 0;
}


void mqtt_handle_incoming(void) {
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG_MQTT, "Timeout esperando mutex UART");
        return;
    }

    uint8_t fixed_header;
    if (uart_read_exact(&fixed_header, 1, 1000) != 1) {
        xSemaphoreGive(uart_mutex);
        return;
    }

    uint8_t packet_type = fixed_header & 0xF0;
    uint8_t flags       = fixed_header & 0x0F;

    // Leer Remaining Length (variable)
    uint32_t remaining_length = 0;
    int consumed_rl_bytes = 0;
    if (read_remaining_length(&remaining_length, &consumed_rl_bytes) != 0) {
        ESP_LOGW(TAG_MQTT, "Fallo leyendo Remaining Length");
        xSemaphoreGive(uart_mutex);
        return;
    }

    if (remaining_length > 1024) {
        ESP_LOGW(TAG_MQTT, "Remaining Length demasiado grande: %lu", remaining_length);
        xSemaphoreGive(uart_mutex);
        return;
    }

    uint8_t body[1024];
    int read_body = uart_read_exact(body, remaining_length, 2000);
    if (read_body != remaining_length) {
        ESP_LOGW(TAG_MQTT, "No se recibió el paquete completo (%d/%lu)", read_body, remaining_length);
        xSemaphoreGive(uart_mutex);
        return;
    }

    last_activity = xTaskGetTickCount();

    switch (packet_type) {
        case 0xD0: ESP_LOGI(TAG_MQTT, "PINGRESP recibido");
        received_pingresp = true;
         break;
        case 0x90: ESP_LOGI(TAG_MQTT, "SUBACK recibido"); break;
        case 0x20: ESP_LOGI(TAG_MQTT, "CONNACK recibido"); break;
        case 0x40: ESP_LOGI(TAG_MQTT, "PUBACK recibido"); break;
        case 0x60: ESP_LOGI(TAG_MQTT, "PUBREL recibido"); break;
        case 0x70: ESP_LOGI(TAG_MQTT, "PUBCOMP recibido"); break;
        case 0x10: ESP_LOGW(TAG_MQTT, "CONNECT recibido inesperadamente"); break;

        case 0x30: {  // PUBLISH
            bool retained = (flags & 0x01);
            ESP_LOGI(TAG_MQTT, "PUBLISH recibido. Retained: %s", retained ? "Sí" : "No");

            if (remaining_length < 2) break;
            uint16_t topic_len = (body[0] << 8) | body[1];
            if (topic_len + 2 > remaining_length || topic_len >= 128) {
                ESP_LOGW(TAG_MQTT, "Longitud de tópico inválida: %d", topic_len);
                break;
            }

            char topic[128];
            memcpy(topic, &body[2], topic_len);
            topic[topic_len] = '\0';

            int payload_offset = 2 + topic_len;
            int payload_len = remaining_length - payload_offset;
            if (payload_len >= 256) payload_len = 255;

            char payload[256];
            memcpy(payload, &body[payload_offset], payload_len);
            payload[payload_len] = '\0';

            ESP_LOGI(TAG_MQTT, "Tópico: %s", topic);
            ESP_LOGI(TAG_MQTT, "Payload: %s", payload);
            break;
        }

        default:
            ESP_LOGW(TAG_MQTT, "Paquete MQTT no manejado: tipo 0x%02X", packet_type);
            break;
    }

    xSemaphoreGive(uart_mutex);
}


static void gprs_mqtt_task(void *pvParameters)
{

//paquete para realizar la conexion
static const uint8_t mqtt_connect_packet[26] = {
  0x10, 0x18,             // CONNECT, Remaining Length = 24
  0x00, 0x04,             // Len("MQTT") = 4
   'M', 'Q', 'T', 'T',    // Protocol Name
  0x04,                   // Protocol Level = 4 (3.1.1)
  0x02,                   // Connect Flags: Clean Session
  0x00, 0x3C,             // Keep Alive = 60 s
  0x00, 0x0C,             // Client ID length = 12
 'e','s','p','3','2','_','C','3','_','D','A','V'
};

signed int len = sizeof(mqtt_connect_packet);

gprs_conected = false;
   
while (1) {

         mqtt_can_publish = true;
    
     
        ESP_LOGW(TAG_GPRS, "GPRS desconectado, reconectando...");

       
            if (!(gprs_conected=gprs_connect()))
               {
                ESP_LOGE(TAG_GPRS, "GPRS no se pudo conectar");
                gprs_disconnect();
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
               }
                ESP_LOGI(TAG_GPRS, "GPRS conectado");
            

                //Establecer zona horaria local ANTES del mktime
                setenv("TZ", "ART3", 1);  // Argentina Time UTC-3
                tzset();
                time_t utc;
                //Convertir a timestamp
                if( (utc = mktime(&timeFromSIM800l))!= -1  && timeFromSIM800l.tm_year>=2020 )
                {

               //Aplicar la hora al sistema
               struct timeval now = {
               .tv_sec = utc,
               .tv_usec = 0
                };
               settimeofday(&now, NULL);

                //Mostrar hora local
                char hora_str[32];
                strftime(hora_str, sizeof(hora_str), "%Y-%m-%d %H:%M:%S", localtime(&utc));
                ESP_LOGI(TAG_GPRS, "Hora local ajustada: %s", hora_str);
               }

         if (!(tcp_conected=tcp_is_connected())) {
            ESP_LOGW(TAG_GPRS, "TCP desconectado, conectando...");
            if (!(tcp_conected=tcp_connect(host, port))) {
                ESP_LOGE(TAG_GPRS, "No se pudo conectar TCP, reintentando en 5s...");
                tcp_disconnect();
                gprs_disconnect();
                gprs_conected=false;
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            ESP_LOGI(TAG_GPRS, "TCP conectado");
        }

        // Enviar datos
       if (!tcp_send(mqtt_connect_packet, len)) {
       ESP_LOGE(TAG_GPRS, "Error enviando CONNECT, reconectando...");
       tcp_disconnect();
       tcp_conected=false;
       gprs_disconnect();
       gprs_conected=false;
    
       vTaskDelay(pdMS_TO_TICKS(2000));
       continue;
}

// Esperar CONNACK
if (!wait_for_mqtt_connack(40000) ){
    ESP_LOGE(TAG_GPRS, "No CONNACK, reconectando...");
    tcp_disconnect();
    tcp_conected=false;
    gprs_disconnect();
    gprs_conected=false;
    vTaskDelay(pdMS_TO_TICKS(2000));
    continue;
}

   ESP_LOGI(TAG_GPRS, "MQTT conectado");
   uart_flush_input(UART_MODEM_NUM);  

      vTaskDelay(pdMS_TO_TICKS(400));  // opcional  
     if (!mqtt_subscribe(mqttTopicCmd)) {
           tcp_disconnect();
           tcp_conected=false;
           gprs_disconnect();
           gprs_conected=false;
           vTaskDelay(pdMS_TO_TICKS(2000));
           continue;
   
      }
     
      //Esperar posibles mensajes retenidos inmediatamente después de suscribirse
      for (int i = 0; i < 15; ++i) {
                   mqtt_handle_incoming();
                   vTaskDelay(pdMS_TO_TICKS(200));
           }


TickType_t last_activity = xTaskGetTickCount();
bool conectado_a_mqtt = true;

TickType_t last_publish = 0;
const TickType_t publish_interval = pdMS_TO_TICKS(10000); // ejemplo: publicar cada 10 s


while (conectado_a_mqtt) {
    //Leer mensajes entrantes (PUBLISH, SUBACK, etc.)
    mqtt_handle_incoming();
    TickType_t now = xTaskGetTickCount();
     // Publicar solo si se puede y si pasó tiempo desde última publicación
    if (mqtt_can_publish && (now - last_publish) > publish_interval) {
        char* msg = NULL;
        msg = desencolar();
        
        if (msg != NULL) {

               if (mqtt_publish(mqttTopicData, msg)) {
               last_publish = now;
                    }
                 else
               {
                encolar(msg);
               }
             if(msg != NULL) {  free(msg);}
        }


    }


    if ((now - last_activity) > pdMS_TO_TICKS(40000)) {
        const uint8_t pingreq[2] = {0xC0, 0x00};
        ESP_LOGI(TAG_GPRS, "PINGREQ enviado");
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(500))) {
              if (!tcp_send(pingreq, 2)) {
                 ESP_LOGW(TAG_GPRS, "Fallo PINGREQ, saliendo del loop");
                 xSemaphoreGive(uart_mutex);
                 break;
                 }
           xSemaphoreGive(uart_mutex);
        
        }
        else {
            ESP_LOGW(TAG_GPRS, "No pudo tomar mutex UART para PINGREQ");
            break;
        }
        
        //durante los próximos 7s, NO publicar
        mqtt_can_publish = false;
        last_activity = now;
        received_pingresp = false;

        TickType_t wait_start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - wait_start) < pdMS_TO_TICKS(7000)) {
            mqtt_handle_incoming();
            if (received_pingresp)
                break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!received_pingresp) {
            ESP_LOGW(TAG_GPRS, "No se recibió PINGRESP a tiempo");
         
            break;
        }
        mqtt_can_publish = true;
        
    }
     // para no quemar CPU
    vTaskDelay(pdMS_TO_TICKS(5)); 
}

        // reconectar todo 
        ESP_LOGW(TAG_GPRS, "Conexión MQTT caída, reiniciando...");
        tcp_disconnect();
        tcp_conected=false;
        gprs_disconnect();
        gprs_conected=false;
        vTaskDelay(pdMS_TO_TICKS(2000));
        
          
       
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
    if (!init_uart_mutex())
      {
        ESP_LOGE(TAG, "No se pudo inicializar uart mutex. Saliendo...");
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



       char hora_str[32];
      // Obtener la hora local actual (después de settimeofday)
       time_t now = time(NULL);  // segundos desde 1970 (UTC)
       struct tm local_tm;
       localtime_r(&now, &local_tm);  // convierte a hora local según TZ

       
      strftime(hora_str, sizeof(hora_str), "%Y-%m-%d %H:%M:%S", &local_tm);

   
      char* msg = malloc(MENSAJE_TAMANO);
  
      /*snprintf(msg, strlen(msg),
       "{\"hora\":\"%s\",\"l1\":\"2321\",\"p1\":\"65\",\"l2\":\"23\",\"p2\":\"63\"}",
         hora_str);*/ 
      strcpy(msg,"{\"hora\":\"12:03\",\"l1\":\"2300\",\"p1\":\"63\",\"l2\":\"21\",\"p2\":\"60\"}");

      
      if(encriptado)
      {
        char* cifrado_b64 = encrypt_payload_base64(msg,aes_key);
        if(cifrado_b64!=NULL)
                 {
                  encolar(cifrado_b64);
                  if(msg!=NULL)
                  {
                  free(msg);
                  }
                 }
      }
      else   
         encolar(msg);
      
      
      msg = malloc(MENSAJE_TAMANO);
      strcpy(msg,"{\"hora\":\"12:03\",\"l1\":\"2300\",\"p1\":\"63\",\"l2\":\"21\",\"p2\":\"60\"}");
      encolar(msg);
      msg = malloc(MENSAJE_TAMANO);
      strcpy(msg,"{\"hora\":\"12:05\",\"l1\":\"23\",\"p1\":\"3\",\"l2\":\"1\",\"p2\":\"10\"}");
      encolar(msg);
      msg = malloc(MENSAJE_TAMANO);
      strcpy(msg,"{\"hora\":\"12:06\",\"l1\":\"200\",\"p1\":\"6\",\"l2\":\"1\",\"p2\":\"6\"}");
      encolar(msg);


    }
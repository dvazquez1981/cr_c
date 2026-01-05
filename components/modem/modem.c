
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

static bool tcp_send(const uint8_t* data, unsigned int len) {
    char buf[128];
    char cmd[32];

    //1) Solicitar envío de “len” bytes
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", len);
    send_at_command(TAG_GPRS, UART_MODEM_NUM, cmd);

    //2) Esperar prompt ‘>’
    TickType_t start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(5000)) {
        int n = uart_read_bytes(UART_MODEM_NUM, (uint8_t*)buf, sizeof(buf)-1, pdMS_TO_TICKS(100));
        if (n > 0 && strchr(buf, '>')) {
            break;
        }
    }
    if (!strchr(buf, '>')) {
        ESP_LOGE(TAG_GPRS, "No vino '>' tras CIPSEND");
        return false;
    }

    // 3) Enviar dato binario
    ESP_LOG_BUFFER_HEXDUMP(TAG_GPRS, data, len, ESP_LOG_INFO);
    uart_write_bytes(UART_MODEM_NUM, (const char*)data, len);

    // 4) Confirmar SEND OK
    start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(5000)) {
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

    vTaskDelay(pdMS_TO_TICKS(2000));
    if (!send_at_command_and_wait_ok("AT+CIPSHUT", 5000, TAG_GPRS, UART_MODEM_NUM)) {
        ESP_LOGE(TAG_GPRS, "No se pudo cerrar conexión GPRS");
        return false;
    }
    ESP_LOGI(TAG_GPRS, "Conexión GPRS cerrada");
    return true;
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
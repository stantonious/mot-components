
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "cJSON.h"
#include <sys/param.h>

#include "core2forAWS.h"
#include "float_buffer.h"
#include "mot_mqtt_client.h"

#define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
#define SUBSCRIBE_TOPIC_LEN (CLIENT_ID_LEN + 3)
#define JSON_BUFSIZE 1024 * 3

int8_t op_x= -1;
int8_t op_y= -1;
int8_t op_t= -1;
char op_id[OP_ID_LEN];

static const char *TAG = "MOT_MQTT_CLIENT";
static const char *CONFIG_BROKER_URI = "mqtts://ahdizksxaeeun-ats.iot.us-east-2.amazonaws.com:8883";

extern const uint8_t root_CA_crt_start[] asm("_binary_root_CA_crt_start");
extern const uint8_t root_CA_crt_end[] asm("_binary_root_CA_crt_end");
extern const uint32_t root_CA_crt_length;
extern const uint8_t mot0_device_pem_start[] asm("_binary_mot0_device_pem_start");
extern const uint8_t mot0_device_pem_end[] asm("_binary_mot0_device_pem_end");
extern const uint32_t mot0_device_pem_length;

extern const uint8_t mot0_private_pem_start[] asm("_binary_mot0_private_pem_start");
extern const uint8_t mot0_private_pem_end[] asm("_binary_mot0_private_pem_end");
extern const uint32_t mot0_private_pem_length;

extern const uint8_t mot0_public_pem_start[] asm("_binary_mot0_public_pem_start");
extern const uint8_t mot0_public_pem_end[] asm("_binary_mot0_public_pem_end");
extern const uint32_t mot0_public_pem_length;


static esp_mqtt_client_handle_t glb_client;

static bool is_inited = false;
static char *json_buf = NULL;
static char game_topic[64];
static char stats_topic[64];
static const char *train_topic = "motivate/train";

static const char *mot_client_id = CONFIG_MOT_CLIENT_ID;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, game_topic, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGD(TAG, "MQTT_EVENT_DATA");
        //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        //printf("DATA=%.*s\r\n", event->data_len, event->data);

        cJSON *message_json = cJSON_Parse(event->data);
        if (message_json == NULL)
        {
            fprintf(stderr, "Error with json");
            const char *error_ptr = cJSON_GetErrorPtr();
            if (error_ptr != NULL)
            {
                fprintf(stderr, "Error before: %s\n", error_ptr);
            }
        }
        else
        {
            cJSON *id = cJSON_GetObjectItemCaseSensitive(message_json, "id");
            cJSON *time = cJSON_GetObjectItemCaseSensitive(message_json, "time");
            cJSON *x = cJSON_GetObjectItemCaseSensitive(message_json, "x");
            cJSON *y = cJSON_GetObjectItemCaseSensitive(message_json, "y");
            cJSON *t = cJSON_GetObjectItemCaseSensitive(message_json, "t");

            if (strcmp(id->valuestring, mot_client_id) != 0)
            {
                if (x == NULL || y == NULL)
                {
                    printf("x and y must be provided!");
                }
                else
                {
                    op_x = x->valueint;
                    op_y = y->valueint;
                    op_t = t->valueint;
                    strncpy(op_id, id->valuestring, OP_ID_LEN);
                }
            }
            else
            {
                ESP_LOGI(TAG, "Ignoring self position");
            }
            cJSON_Delete(message_json);
        }

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        }
        else
        {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mot_mqtt_client_init(int game_id, char *player_id)
{
    sprintf(game_topic, "motivate/game/%d", game_id);
    sprintf(stats_topic, "motivate/stats/%s", player_id);
    ESP_LOGI(TAG, "Client ID:%s", mot_client_id);
    json_buf = malloc(JSON_BUFSIZE);
    esp_err_t esp_ret = ESP_FAIL;
    bzero(op_id, OP_ID_LEN);

    esp_ret = esp_tls_set_global_ca_store(root_CA_crt_start, root_CA_crt_length + 1);
    if (esp_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in setting the global ca store: [%02X] (%s),could not complete the https_request using global_ca_store", esp_ret, esp_err_to_name(esp_ret));
        return;
    }

    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URI,
        .client_cert_pem = (const char *)mot0_device_pem_start,
        .client_key_pem = (const char *)mot0_private_pem_start,
        .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .use_global_ca_store = true,
        .client_id = mot_client_id};

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    glb_client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(glb_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    esp_mqtt_client_start(glb_client);
    is_inited = true;
}

void check_null(cJSON *p, int l)
{
    if (p == NULL)
        ESP_LOGI(TAG, "!!!!!!!!!!!!!!!!  cJSON NULL  !!!!!!!!!!!!!!!!!!! %d", l);
}

void fdump(float **buf, int m)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            ESP_LOGI("FLOATBUF", "i:%d,j:%d,v:%f\n", i, j, buf[i][j]);
        }
    }
}

void send_position(int x, int y, int t, unsigned time)
{
    if (!is_inited)
    {
        ESP_LOGW(TAG, "MQTT NOT INITED!!!");
        return;
    }
    cJSON *pos = cJSON_CreateObject();
    cJSON *ltime = cJSON_CreateNumber(time);
    cJSON *id = cJSON_CreateString(mot_client_id);
    cJSON *pos_x = cJSON_CreateNumber(x);
    cJSON *pos_y = cJSON_CreateNumber(y);
    cJSON *pos_t = cJSON_CreateNumber(t);

    cJSON_AddItemToObject(pos, "time", ltime);
    cJSON_AddItemToObject(pos, "id", id);
    cJSON_AddItemToObject(pos, "x", pos_x);
    cJSON_AddItemToObject(pos, "y", pos_y);
    cJSON_AddItemToObject(pos, "t", pos_t);

    cJSON_PrintPreallocated(pos, json_buf, JSON_BUFSIZE, false);
    int pub_ret = esp_mqtt_client_publish(glb_client, game_topic, json_buf, 0, 1, 0);
    //ESP_LOGI(TAG, "Publishing of =%s returned=%d", out, pub_ret);

    cJSON_Delete(pos);
}

void send_stats(unsigned maze_id, unsigned session, unsigned step_count, unsigned capture_count, unsigned caught_count, unsigned time)
{
    if (!is_inited)
    {
        ESP_LOGW(TAG, "MQTT NOT INITED!!!");
        return;
    }
    cJSON *stats = cJSON_CreateObject();
    cJSON *m_id = cJSON_CreateNumber(maze_id);
    cJSON *s = cJSON_CreateNumber(session);
    cJSON *s_cnt = cJSON_CreateNumber(step_count);
    cJSON *cap_cnt = cJSON_CreateNumber(capture_count);
    cJSON *caught_cnt = cJSON_CreateNumber(caught_count);
    cJSON *t = cJSON_CreateNumber(time);

    cJSON_AddItemToObject(stats, "time", t);
    cJSON_AddItemToObject(stats, "maze_id", m_id);
    cJSON_AddItemToObject(stats, "session", s);
    cJSON_AddItemToObject(stats, "step_count", s_cnt);
    cJSON_AddItemToObject(stats, "capture_count", cap_cnt);
    cJSON_AddItemToObject(stats, "caught_count", caught_cnt);

    cJSON_PrintPreallocated(stats, json_buf, JSON_BUFSIZE, false);
    int pub_ret = esp_mqtt_client_publish(glb_client, stats_topic, json_buf, 0, 1, 0);
    cJSON_Delete(stats);
}

void send_sample(float **a_samples, float **g_samples, int a_size, int g_size, int type, unsigned time)
{
    if (!is_inited)
    {
        ESP_LOGW(TAG, "MQTT NOT INITED!!!");
        return;
    }
    //fdump(a_samples, 3);
    //fdump(g_samples, 3);
    //ESP_LOGI(TAG, "S1");
    cJSON *sample = cJSON_CreateObject();
    check_null(sample, __LINE__);
    cJSON *ax_samp = cJSON_CreateFloatArray(a_samples[0], BUFSIZE);
    check_null(ax_samp, __LINE__);
    cJSON *ay_samp = cJSON_CreateFloatArray(a_samples[1], BUFSIZE);
    check_null(ay_samp, __LINE__);
    cJSON *az_samp = cJSON_CreateFloatArray(a_samples[2], BUFSIZE);
    check_null(az_samp, __LINE__);
    cJSON *gx_samp = cJSON_CreateFloatArray(g_samples[0], BUFSIZE);
    check_null(gx_samp, __LINE__);
    cJSON *gy_samp = cJSON_CreateFloatArray(g_samples[1], BUFSIZE);
    check_null(gy_samp, __LINE__);
    cJSON *gz_samp = cJSON_CreateFloatArray(g_samples[2], BUFSIZE);
    check_null(gz_samp, __LINE__);
    cJSON *g_samp = cJSON_CreateObject();
    check_null(g_samp, __LINE__);
    cJSON *a_samp = cJSON_CreateObject();
    check_null(a_samp, __LINE__);
    cJSON *ltime = cJSON_CreateNumber(time);
    check_null(ltime, __LINE__);
    cJSON *ltype = cJSON_CreateNumber(type);
    check_null(ltype, __LINE__);

    cJSON_AddItemToObject(sample, "time", ltime);
    cJSON_AddItemToObject(sample, "type", ltype);
    cJSON_AddItemToObject(sample, "acc_samples", a_samp);
    cJSON_AddItemToObject(sample, "gyro_samples", g_samp);
    cJSON_AddItemToObject(a_samp, "x", ax_samp);
    cJSON_AddItemToObject(a_samp, "y", ay_samp);
    cJSON_AddItemToObject(a_samp, "z", az_samp);

    cJSON_AddItemToObject(g_samp, "x", gx_samp);
    cJSON_AddItemToObject(g_samp, "y", gy_samp);
    cJSON_AddItemToObject(g_samp, "z", gz_samp);

    cJSON_PrintPreallocated(sample, json_buf, JSON_BUFSIZE, false);
    int pub_ret = esp_mqtt_client_publish(glb_client, train_topic, json_buf, 0, 1, 0);
    //ESP_LOGI(TAG, "Publishing of =%s returned=%d", out, pub_ret);

    cJSON_Delete(sample);
}

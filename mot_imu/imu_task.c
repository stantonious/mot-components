#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "imu_task.h"
#include "esp_dsp.h"
#include "mot_math.h"
#include "float_buffer.h"

static const char *TAG = "IMU_TASK";

SemaphoreHandle_t xImuSemaphore;

void *ax_buf;
void *ay_buf;
void *az_buf;

void *gz_buf;
void *gx_buf;
void *gy_buf;

void *dot_buf;
float dot_avg = 0.;

static const float down_uv[3] = {-.007, .48, .87}; ///~ 15 deg.

void init_imu(void)
{
    xImuSemaphore = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(imu_handler_task, "ImuTask", 2048, NULL, 1, &IMU_handle, 0);
    xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
    ax_buf = get_buffer();
    ay_buf = get_buffer();
    az_buf = get_buffer();
    gx_buf = get_buffer();
    gy_buf = get_buffer();
    gz_buf = get_buffer();
    xSemaphoreGive(xImuSemaphore);

    dot_buf = get_buffer();
}

void imu_handler_task(void *pvParameters)
{
    ESP_LOGI(TAG, "recording IMU");
    float gx, gy, gz;
    float ax, ay, az;
    for (;;)
    {
        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        float res;
        float current_down[3] = {ax, ay, az};
        float current_down_uv[3] = {0};
        unit_vect(current_down, current_down_uv, 3);
        esp_err_t ret = dsps_dotprod_f32(down_uv, current_down_uv, &res, 3);

        if (ret != ESP_OK)
            printf("dotprod Operation error = %i\n", ret);
        else
            push(dot_buf, res);
        dot_avg = avg(dot_buf);

        xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
        push(ax_buf, ax);
        push(ay_buf, ay);
        push(az_buf, az);

        push(gx_buf, gx);
        push(gy_buf, gy);
        push(gz_buf, gz);
        xSemaphoreGive(xImuSemaphore);
        //vTaskDelay(pdMS_TO_TICKS(66)); // 15Hz
        vTaskDelay(pdMS_TO_TICKS(33)); // 30Hz
    }
    vTaskDelete(NULL); // Should never get to here...
}

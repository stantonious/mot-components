
#include "mot-imu-tf.h"

#include "esp_log.h"
#include <stdio.h>
#include "freertos/semphr.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
//#include "output_handler.h"
#include "tensorflow/lite/kernels/internal/tensor_ctypes.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include "float_buffer.h"
#include "CircularBuffer.h"
#include "mot-imu-model.h"
#include "imu_task.h"

static const char *TAG = "MOT_IMU_TFL_HANDLER";

static TfLiteTensor *mot_input = nullptr;
static TfLiteTensor *mot_output = nullptr;
//must match model training hyperparameters (don't change)
static const int g_min = -90;
static const int g_max = 90;
static const int a_min = -1;
static const int a_max = 1;

#define INF_THRESH .60
#define INF_SIZE 10

static SemaphoreHandle_t xInfSemaphore;
static CircularBuffer<float, INF_SIZE> conf_cbs[NUM_CLASSES];

// Globals, used for compatibility with Arduino-style sketches.
namespace
{
  tflite::ErrorReporter *error_reporter = nullptr;
  const tflite::Model *model = nullptr;
  tflite::MicroInterpreter *mot_interpreter = nullptr;

  // Create an area of memory to use for input, output, and intermediate arrays.
  // Minimum arena size, at the time of writing. After allocating tensors
  // you can retrieve this value by invoking interpreter.arena_used_bytes().
  const int kModelArenaSize = 4156;
  // Extra headroom for model + alignment + future interpreter changes.
  const int kExtraArenaSize = 560 + 16 + 100;
  const int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
  static uint8_t mot_tensor_arena[kTensorArenaSize];
} // namespace

void init_mot_imu(void)
{
  ESP_LOGI(TAG, "INITING Model");
  xInfSemaphore = xSemaphoreCreateMutex();
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION)
  {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // This pulls in all the operation implementations we need.
  // NOLINTNEXTLINE(runtime-global-variables)
  //static tflite::AllOpsResolver resolver;
  static tflite::MicroMutableOpResolver<5> resolver;
  resolver.AddConv2D();
  resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                      tflite::ops::micro::Register_MAX_POOL_2D());
  resolver.AddFullyConnected();
  resolver.AddSoftmax();
  resolver.AddReshape();

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter mot_static_interpreter(
      model, resolver, mot_tensor_arena, kTensorArenaSize, error_reporter);
  mot_interpreter = &mot_static_interpreter;

  // Allocate memory from the mot_tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = mot_interpreter->AllocateTensors();
  ESP_LOGI(TAG, "Arena size %d", mot_static_interpreter.arena_used_bytes());
  if (allocate_status != kTfLiteOk)
  {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }
  mot_input = mot_interpreter->input(0);
  mot_output = mot_interpreter->output(0);

  xTaskCreatePinnedToCore(mot_imu_task, "MotImuTfTask", 2096, NULL, 1, &mot_imu_handle, 0);
}

float get_max_idx_cb(CircularBuffer<float, INF_SIZE> &buf, int lastn)
{
  int max_idx = 0;
  float max = buf[buf.size() - 1];
  for (int i = buf.size() - 2; i >= buf.size() - lastn && i >= 0; i--)
  {
    if (buf[i] > max)
    {
      max_idx = i;
      max = buf[i];
    }
  }
  return max_idx;
}
float get_max(float *f, int n)
{
  float max = f[0];
  for (int i = 1; i < n; i++)
  {
    if (f[i] > max)
      max = f[i];
  }
  return max;
}
int get_max_idx(float *f, int n, float thresh)
{
  int max_idx = 0;
  float max = f[0];
  for (int i = 1; i < n; i++)
  {
    if (f[i] > max)
    {
      max_idx = i;
      max = f[i];
    }
  }
  if (max > thresh)
    return max_idx;
  else
  {
    printf("max too low :%f :%d\n", max, max_idx);
    return UNCERTAIN_LABEL;
  }
}

int get_max_from_confs( int last_n, float conf)
{
  float max = 0.;
  int max_idx;
  for (int j = 0; j < NUM_CLASSES; j++)
  {
    for (int i = INF_SIZE - 1; i >= INF_SIZE - last_n; i--)
    {
      if (conf_cbs[j][i] > max)
      {
        max = conf_cbs[j][i];
        max_idx = j;
      }
    }
  }
  if (max >= conf) return max_idx;
  
  return UNCERTAIN_LABEL;
}

int get_max_avg_idx(CircularBuffer<float, INF_SIZE> conf_cbs[NUM_CLASSES], int last_n, float conf)
{
  float avgs[NUM_CLASSES] = {0.};
  for (int j = 0; j < NUM_CLASSES; j++)
  {
    for (int i = INF_SIZE - 1; i >= INF_SIZE - last_n; i--)
    {
      avgs[j] += conf_cbs[j][i] / last_n;
    }
  }
  return get_max_idx(avgs, NUM_CLASSES, conf);
}

int buffer_confs(
    CircularBuffer<float, BUFSIZE> *ax_cb,
    CircularBuffer<float, BUFSIZE> *ay_cb,
    CircularBuffer<float, BUFSIZE> *az_cb,
    CircularBuffer<float, BUFSIZE> *gx_cb,
    CircularBuffer<float, BUFSIZE> *gy_cb,
    CircularBuffer<float, BUFSIZE> *gz_cb,
    CircularBuffer<float, INF_SIZE> conf_cbs[NUM_CLASSES])
{

  int8_t *d_i = tflite::GetTensorData<int8>(mot_input);

  // Place our calculated x value in the model's input tensor
  for (int i = 0; i < BUFSIZE; i++)
  {
    int8_t quant_val = ((*ax_cb)[i] - a_min) / (a_max - a_min) * 255 / mot_input->params.scale + mot_input->params.zero_point;
    d_i[i * 6 + 0] = quant_val;

    quant_val = ((*ay_cb)[i] - a_min) / (a_max - a_min) * 255 / mot_input->params.scale + mot_input->params.zero_point;
    d_i[i * 6 + 1] = quant_val;

    quant_val = ((*az_cb)[i] - a_min) / (a_max - a_min) * 255 / mot_input->params.scale + mot_input->params.zero_point;
    d_i[i * 6 + 2] = quant_val;

    quant_val = ((*gx_cb)[i] - g_min) / (g_max - g_min) * 255 / mot_input->params.scale + mot_input->params.zero_point;
    d_i[i * 6 + 3] = quant_val;

    quant_val = ((*gy_cb)[i] - g_min) / (g_max - g_min) * 255 / mot_input->params.scale + mot_input->params.zero_point;
    d_i[i * 6 + 4] = quant_val;

    quant_val = ((*gz_cb)[i] - g_min) / (g_max - g_min) * 255 / mot_input->params.scale + mot_input->params.zero_point;
    d_i[i * 6 + 5] = quant_val;
  }

  // Run inference, and report any error
  TfLiteStatus invoke_status = mot_interpreter->Invoke();
  if (invoke_status != kTfLiteOk)
  {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed on x_val: %f\n", 0.);
    return -1;
  }

  // Read the predicted y value from the model's output tensor

  conf_cbs[0].push((mot_output->data.int8[0] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[1].push((mot_output->data.int8[1] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[2].push((mot_output->data.int8[2] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[3].push((mot_output->data.int8[3] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[4].push((mot_output->data.int8[4] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[5].push((mot_output->data.int8[5] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[6].push((mot_output->data.int8[6] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[7].push((mot_output->data.int8[7] - mot_output->params.zero_point) * mot_output->params.scale);
  conf_cbs[8].push((mot_output->data.int8[8] - mot_output->params.zero_point) * mot_output->params.scale);

  return 0;
}

void mot_imu_task(void *pvParameters)
{

  for (;;)
  {

    xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
    xSemaphoreTake(xInfSemaphore, portMAX_DELAY);
    buffer_confs(
        (CircularBuffer<float, BUFSIZE> *)ax_buf,
        (CircularBuffer<float, BUFSIZE> *)ay_buf,
        (CircularBuffer<float, BUFSIZE> *)az_buf,
        (CircularBuffer<float, BUFSIZE> *)gx_buf,
        (CircularBuffer<float, BUFSIZE> *)gy_buf,
        (CircularBuffer<float, BUFSIZE> *)gz_buf,
        conf_cbs);
    xSemaphoreGive(xInfSemaphore);
    xSemaphoreGive(xImuSemaphore);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

int get_latest_inf(int n_last, float conf)
{
  xSemaphoreTake(xInfSemaphore, portMAX_DELAY);
  int res = get_max_avg_idx(conf_cbs, n_last, conf);
  xSemaphoreGive(xInfSemaphore);
  return res;
}

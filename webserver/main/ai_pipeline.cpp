#include "ai_pipeline.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "esp_log.h"

#include "esp32s3/rom/tjpgd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "person_detect_model_data.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define TAG "AIPIPE"

#define TFLITE_TASK_STACK_SIZE 4096
#define TFLITE_TASK_PRIORITY   10

static constexpr int kTensorArenaSize = 128 * 1024;
static const tflite::Model* model;
static tflite::MicroInterpreter* interpreter;
static uint8_t *tensor_arena;

static TaskHandle_t tflite_task_handle;
static SemaphoreHandle_t tflite_done_sem;
static SemaphoreHandle_t decode_done_sem;
static const uint8_t *p_input;
static size_t input_size;

static cJSON *p_results_root;
static cJSON *p_results_person_score;
static cJSON *p_results_no_person_score;

// Keeping these as constant expressions allow us to allocate fixed-sized arrays
// on the stack for our working memory.

// All of these values are derived from the values used during model training,
// if you change your model you'll need to update these constants.
constexpr int kNumCols = 96;
constexpr int kNumRows = 96;
constexpr int kNumChannels = 1;

constexpr int kMaxImageSize = kNumCols * kNumRows * kNumChannels;

constexpr int kCategoryCount = 2;
constexpr int kPersonIndex = 1;
constexpr int kNotAPersonIndex = 0;

typedef struct decode_ctx_t {
    const uint8_t *in_buf;
    size_t in_size;
    size_t in_offset;
    uint8_t *out_buf;
    uint32_t out_width;
} decode_ctx_t;

static bool jpeg_decode(const uint8_t *p_in_data, size_t len, uint8_t *p_out_data);
static size_t jpg_get_data_cb(JDEC* decoder, uint8_t* buff, size_t ndata);
static size_t jpg_write_data_cb(JDEC *decoder, void *data, JRECT *rect);
static void tflite_task_entry(void *);

#ifdef __cplusplus
extern "C" {
#endif

void ai_pipeline_init(void)
{
    tensor_arena = (uint8_t *) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (tensor_arena == NULL) {
        ESP_LOGE(TAG, "Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
        return;
    }

    model = tflite::GetModel(g_person_detect_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model provided is schema version %lu not equal to supported "
                    "version %d", model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // Pull in only the operation implementations we need.
    // This relies on a complete list of all the ops needed by this graph.
    // An easier approach is to just use the AllOpsResolver, but this will
    // incur some penalty in code space for op implementations that are not
    // needed by this graph.
    //
    // tflite::AllOpsResolver resolver;
    // NOLINTNEXTLINE(runtime-global-variables)
    static tflite::MicroMutableOpResolver<5> micro_op_resolver;
    micro_op_resolver.AddAveragePool2D();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddDepthwiseConv2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddSoftmax();

    // Build an interpreter to run the model with.
    // NOLINTNEXTLINE(runtime-global-variables)
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return;
    }

    p_results_root = cJSON_CreateObject();
    p_results_person_score = cJSON_AddNumberToObject(p_results_root, "person_score", 0);
    p_results_no_person_score = cJSON_AddNumberToObject(p_results_root, "no_person_score", 1);

    tflite_done_sem = xSemaphoreCreateBinary();
    decode_done_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(tflite_task_entry, "TFLITE", TFLITE_TASK_STACK_SIZE,
                            NULL, TFLITE_TASK_PRIORITY, &tflite_task_handle, 1);
    configASSERT(&tflite_task_handle);
}

void ai_pipeline_start(const uint8_t *p_frame_data, uint32_t size)
{
    // Run the model on this input and make sure it succeeds.
    p_input = p_frame_data;
    input_size = size;
    xTaskNotify(tflite_task_handle, 0, eNoAction);
}

void ai_pipeline_wait_decode_finish(TickType_t timeous_ticks)
{
    xSemaphoreTake(decode_done_sem, portMAX_DELAY);
}

void ai_pipeline_wait(TickType_t timeous_ticks)
{
    xSemaphoreTake(tflite_done_sem, portMAX_DELAY);
}

cJSON *ai_pipeline_get_results(void)
{
    return p_results_root;
}

void ai_pipeline_deinit(void)
{
    // TODO
}

#ifdef __cplusplus
}
#endif

static bool jpeg_decode(const uint8_t *p_in_data, size_t len, uint8_t *p_out_data)
{
    static uint8_t buf[3100];
    JDEC decoder;
    decode_ctx_t ctx = {
        .in_buf = p_in_data,
        .in_size = len,
        .in_offset = 0,
        .out_buf = p_out_data,
        .out_width = 0,
    };

    JRESULT jres = jd_prepare(&decoder, jpg_get_data_cb, buf, sizeof(buf), &ctx);
    if(jres != JDR_OK){
        ESP_LOGE(TAG, "JPG Header Parse Failed: %d", jres);
        return ESP_FAIL;
    }

    ctx.out_width = decoder.width / 8;

    jres = jd_decomp(&decoder, jpg_write_data_cb, 3);

    if (jres != JDR_OK) {
        ESP_LOGE(TAG, "JPG Decompression Failed: %d", jres);
        return false;
    }

    return true;
}

static size_t jpg_get_data_cb(JDEC* decoder, uint8_t* buff, size_t ndata)
{
    decode_ctx_t *p_ctx = (decode_ctx_t *)decoder->device;
    const size_t to_read = MIN(p_ctx->in_size - p_ctx->in_offset, ndata);
    if (NULL != buff) {
        memcpy(buff, p_ctx->in_buf + p_ctx->in_offset, to_read);
    }
    p_ctx->in_offset += to_read;
    return to_read;
}

static size_t jpg_write_data_cb(JDEC *decoder, void *data, JRECT *rect)
{
    decode_ctx_t *p_ctx = (decode_ctx_t *)decoder->device;
    for (int i = rect->top; i < rect->bottom + 1; i++) {
        for (int j = rect->left; j < rect->right + 1; j++) {
            const uint8_t r = ((uint8_t *)data)[3 * ((i - rect->top)* p_ctx->out_width + j - rect->left) + 0];
            const uint8_t g = ((uint8_t *)data)[3 * ((i - rect->top) * p_ctx->out_width + j - rect->left) + 1];
            const uint8_t b = ((uint8_t *)data)[3 * ((i - rect->top) * p_ctx->out_width + j - rect->left) + 2];
            const int8_t bw = ((r + g + b) / 3) ^ 0x80;
            p_ctx->out_buf[i * p_ctx->out_width + j] = bw;
        }
    }
    return 1;
}

static void tflite_task_entry(void *)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Get information about the memory area to use for the model's input.
        TfLiteTensor *input = interpreter->input(0);

        jpeg_decode(p_input, input_size, input->data.uint8);
        xSemaphoreGive(decode_done_sem);

        if (kTfLiteOk != interpreter->Invoke()) {
            ESP_LOGE(TAG, "Invoke failed");
        }
        TfLiteTensor *output = interpreter->output(0);
        int8_t person_score = output->data.uint8[kPersonIndex];
        int8_t no_person_score = output->data.uint8[kNotAPersonIndex];

        const float person_score_f =
            (person_score - output->params.zero_point) * output->params.scale;
        const float no_person_score_f =
            (no_person_score - output->params.zero_point) * output->params.scale;
        p_results_person_score->valuedouble = person_score_f;
        p_results_no_person_score->valuedouble = no_person_score_f;
        xSemaphoreGive(tflite_done_sem);
    }
}

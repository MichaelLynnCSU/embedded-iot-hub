/******************************************************************************
 * \file inference_worker.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Shared TFLite inference worker — used by inference_daemon (PIR)
 *        and doorbell_daemon (doorbell).
 *
 * \details Owns the TFLite model, interpreter, and label table.
 *          All perception logic lives here; transport parsing stays in
 *          each daemon. Both daemons link this file.
 *
 * \note    Timestamp out-param (2026-06-14):
 *          inference_worker_save() copies its internal timestamp string
 *          into out_ts (if non-NULL) before returning, so callers can
 *          publish an asset identifier guaranteed to match the saved
 *          filename. See inference_worker.h for the contract.
 ******************************************************************************/

#include "inference_worker.h"
#include "inference_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include <stdarg.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"
#include "tensorflow/lite/c/c_api.h"

/*---------------------------------------------------------------------------*/
/* Module globals                                                              */
/*---------------------------------------------------------------------------*/

static TfLiteModel       *g_model       = NULL; /**< loaded TFLite model      */
static TfLiteInterpreter *g_interp      = NULL; /**< TFLite interpreter        */
static char               g_labels[MAX_LABELS][LABEL_LEN]; /**< label table   */
static int                g_label_count = 0;    /**< number of loaded labels   */

/*---------------------------------------------------------------------------*/
/* Internal logging — stderr only, callers own their log files                */
/*---------------------------------------------------------------------------*/

static void worker_log(const char *fmt, ...)
{
   va_list    ap;
   time_t     now = time(NULL);
   char       ts[32];
   struct tm  tm_buf;

   localtime_r(&now, &tm_buf);
   strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_buf);
   fprintf(stderr, "[%s] inference_worker: ", ts);
   va_start(ap, fmt);
   vfprintf(stderr, fmt, ap);
   va_end(ap);
   fprintf(stderr, "\n");
}

/*---------------------------------------------------------------------------*/
/* Public API                                                                  */
/*---------------------------------------------------------------------------*/

int inference_worker_init(const char *model_path, const char *label_path)
{
   FILE *f = NULL; /**< label file handle */

   /* Load labels */
   f = fopen(label_path, "r");
   if (NULL == f)
   {
      worker_log("ERROR: cannot open labels: %s", label_path);
      return -1;
   }
   while (g_label_count < MAX_LABELS &&
          fgets(g_labels[g_label_count], LABEL_LEN, f))
   {
      g_labels[g_label_count][strcspn(g_labels[g_label_count], "\r\n")] = 0;
      g_label_count++;
   }
   fclose(f);
   worker_log("Loaded %d labels from %s", g_label_count, label_path);

   /* Load model */
   g_model = TfLiteModelCreateFromFile(model_path);
   if (NULL == g_model)
   {
      worker_log("ERROR: cannot load model: %s", model_path);
      return -1;
   }

   /* Create interpreter */
   TfLiteInterpreterOptions *opts = TfLiteInterpreterOptionsCreate();
   TfLiteInterpreterOptionsSetNumThreads(opts, 1);
   g_interp = TfLiteInterpreterCreate(g_model, opts);
   TfLiteInterpreterOptionsDelete(opts);

   if (NULL == g_interp)
   {
      worker_log("ERROR: cannot create interpreter");
      TfLiteModelDelete(g_model);
      g_model = NULL;
      return -1;
   }

   if (TfLiteInterpreterAllocateTensors(g_interp) != kTfLiteOk)
   {
      worker_log("ERROR: AllocateTensors failed");
      TfLiteInterpreterDelete(g_interp);
      TfLiteModelDelete(g_model);
      g_interp = NULL;
      g_model  = NULL;
      return -1;
   }

   worker_log("Model loaded: %s", model_path);
   return 0;
}

/*---------------------------------------------------------------------------*/

void inference_worker_shutdown(void)
{
   if (NULL != g_interp) { TfLiteInterpreterDelete(g_interp); g_interp = NULL; }
   if (NULL != g_model)  { TfLiteModelDelete(g_model);        g_model  = NULL; }
}

/*---------------------------------------------------------------------------*/

int inference_worker_run(const uint8_t *jpeg, size_t jpeg_len,
                         int *person_detected, float *confidence)
{
   TfLiteTensor *input_tensor = NULL; /**< model input tensor  */
   int           input_h      = 0;    /**< model input height  */
   int           input_w      = 0;    /**< model input width   */
   int           img_w        = 0;    /**< decoded image width */
   int           img_h        = 0;    /**< decoded image height */
   int           img_ch       = 0;    /**< decoded image channels */
   uint8_t      *img          = NULL; /**< decoded image buffer */
   uint8_t      *input_buf    = NULL; /**< resized input buffer */
   size_t        input_size   = 0;    /**< input buffer size    */
   int           num_det      = 0;    /**< number of detections */
   int           i            = 0;    /**< loop index           */

   *person_detected = 0;
   *confidence      = 0.0f;

   input_tensor = TfLiteInterpreterGetInputTensor(g_interp, 0);
   if (NULL == input_tensor)
   {
      worker_log("ERROR: no input tensor");
      return -1;
   }

   input_h = TfLiteTensorDim(input_tensor, 1);
   input_w = TfLiteTensorDim(input_tensor, 2);

   img = stbi_load_from_memory(jpeg, (int)jpeg_len,
                               &img_w, &img_h, &img_ch, 3);
   if (NULL == img)
   {
      worker_log("ERROR: stbi_load failed: %s", stbi_failure_reason());
      return -1;
   }
   worker_log("Decoded JPEG %dx%d -> model %dx%d", img_w, img_h, input_w, input_h);

   input_size = (size_t)(input_h * input_w * 3);
   input_buf  = malloc(input_size);
   if (NULL == input_buf)
   {
      stbi_image_free(img);
      return -1;
   }

   stbir_resize_uint8_linear(img, img_w, img_h, 0,
                              input_buf, input_w, input_h, 0, STBIR_RGB);
   stbi_image_free(img);

   if (TfLiteTensorCopyFromBuffer(input_tensor,
                                   input_buf, input_size) != kTfLiteOk)
   {
      worker_log("ERROR: CopyFromBuffer");
      free(input_buf);
      return -1;
   }
   free(input_buf);

   if (TfLiteInterpreterInvoke(g_interp) != kTfLiteOk)
   {
      worker_log("ERROR: Invoke failed");
      return -1;
   }

   const TfLiteTensor *scores_tensor  =
       TfLiteInterpreterGetOutputTensor(g_interp, 2);
   const TfLiteTensor *classes_tensor =
       TfLiteInterpreterGetOutputTensor(g_interp, 1);

   if (NULL == scores_tensor || NULL == classes_tensor)
   {
      worker_log("ERROR: missing output tensors");
      return -1;
   }

   num_det         = TfLiteTensorDim(scores_tensor, 1);
   const float *scores  = (const float *)TfLiteTensorData(scores_tensor);
   const float *classes = (const float *)TfLiteTensorData(classes_tensor);

   for (i = 0; i < num_det; i++)
   {
      if (!infer_above_threshold(scores[i])) { break; }

      int         class_idx = (int)classes[i] + 1;
      const char *label     = infer_label_for(g_labels, g_label_count,
                                               class_idx);
      worker_log("Detection %d: label=%s score=%.2f", i, label, scores[i]);

      if (infer_is_person(label))
      {
         *person_detected = 1;
         *confidence      = scores[i];
         break;
      }
   }
   return 0;
}

/*---------------------------------------------------------------------------*/

void inference_worker_save(const uint8_t *jpeg, size_t len,
                           const char *dir,
                           int detected, float confidence,
                           const char *tag,
                           char *out_ts, size_t out_ts_len)
{
   char      path[320];  /**< output file path          */
   char      ts[32];     /**< timestamp string          */
   time_t    now  = time(NULL); /**< current time        */
   struct tm tm_buf;     /**< broken-down time           */
   FILE     *f    = NULL; /**< output file handle        */

   mkdir(dir, 0755);

   localtime_r(&now, &tm_buf);
   strftime(ts, sizeof(ts), "%Y%m%dT%H%M%SZ", &tm_buf);

   if (NULL != tag && '\0' != tag[0])
   {
      snprintf(path, sizeof(path), "%s/%s_%s_%s_%03d.jpg",
               dir, ts,
               infer_detection_token(detected),
               tag,
               (int)(confidence * 100));
   }
   else
   {
      snprintf(path, sizeof(path), "%s/%s_%s_%03d.jpg",
               dir, ts,
               infer_detection_token(detected),
               (int)(confidence * 100));
   }

   f = fopen(path, "wb");
   if (NULL == f)
   {
      worker_log("ERROR: save JPEG: %s", strerror(errno));
      return;
   }
   fwrite(jpeg, 1, len, f);
   fclose(f);
   worker_log("Saved %s", path);

   /* Return the exact timestamp string used in the filename above, so
    * callers can publish an asset identifier that is guaranteed to match
    * (no independent time(NULL) call, no drift). Left untouched on error
    * paths above (caller's buffer keeps whatever it had, e.g. zeroed). */
   if (NULL != out_ts && out_ts_len > 0)
   {
      strncpy(out_ts, ts, out_ts_len - 1);
      out_ts[out_ts_len - 1] = '\0';
   }
}

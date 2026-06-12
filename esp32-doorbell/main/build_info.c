#include "esp_log.h"

static const char *TAG = "TRINITY";

#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
static const char s_build_mode[] = "TRINITY_BUILD_MODE=BENCH";
#else
static const char s_build_mode[] = "TRINITY_BUILD_MODE=FIELD";
#endif

static const char s_build_date[] = "TRINITY_BUILD_DATE=" __DATE__ " " __TIME__;

void trinity_build_info_print(void)
{
    ESP_LOGI(TAG, "%s", s_build_mode);
    ESP_LOGI(TAG, "%s", s_build_date);
}

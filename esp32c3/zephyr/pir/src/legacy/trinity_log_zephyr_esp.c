/******************************************************************************
 * \file trinity_log_zephyr_esp.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Trinity crash logger -- ESP32-C3 Zephyr (pir board).
 *
 * \details Platform: ESP32-C3, Zephyr RTOS, RISC-V.
 *          Storage:  Flash partition (log_partition in DTS overlay).
 *          Console:  USB CDC (bench/USB) / UART (field).
 *          Fault:    k_sys_fatal_error_handler + RISC-V mepc/ra.
 *          Canary:   .noinit ELF section + SYS_INIT(PRE_KERNEL_1).
 *          WDT:      Zephyr wdt driver (bench: skipped entirely).
 *
 *          Build modes (set via west build -D):
 *            TRINITY_MODE_BENCH -- halt (OpenOCD/USB-JTAG)
 *            TRINITY_MODE_USB   -- CDC ACM 500ms flush + sys_reboot()
 *            TRINITY_MODE_FIELD -- UART + sys_reboot() (default)
 *
 *          prj.conf requirements:
 *            CONFIG_WATCHDOG=y
 *            CONFIG_THREAD_ANALYZER=y
 *            CONFIG_THREAD_ANALYZER_USE_PRINTK=n
 *            CONFIG_SYS_HEAP_RUNTIME_STATS=y
 *            CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=3072
 *            CONFIG_MAIN_STACK_SIZE=2048
 *            CONFIG_HEAP_MEM_POOL_SIZE=4096
 *
 * \note    Architecture differences from nRF52840 version:
 *          - RISC-V fault frame: mepc/ra instead of pc/lr
 *          - irq_lock() instead of __disable_irq()
 *          - sys_reboot(SYS_REBOOT_COLD) instead of NVIC_SystemReset()
 *          - No #include <nrf.h>, no RADIO_SYNC concern
 *          - No flash mutex needed (ESP32-C3 flash driver serializes internally)
 *          - Bench WDT skipped entirely (OpenOCD is safety net)
 *
 * \note    Sched-lock violation fix (2026-03-27):
 *          Same as nRF52840 siblings. collect-then-write pattern applied.
 *          Heap-allocated stat buffer to avoid .noinit overlap (same fix).
 *
 * \note    Canary snapshot fix (2026-03-26):
 *          g_canary_snapshot saves previous session canary before overwrite.
 *          Same fix as nRF52840. No g_init_stage on this board (pir does
 *          not use boot stage cookies yet -- add if needed).
 ******************************************************************************/

#include "trinity_log.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/sys/mem_stats.h>
#include <zephyr/init.h>
#include <string.h>
#include <stdio.h>

/******************************** CONSTANTS ***********************************/

#define LOG_PARTITION_ID    FIXED_PARTITION_ID(log_partition)
#define LOG_MAGIC           0xA55A
#define LOG_ENTRY_SIZE      128
#define LOG_PAGE_SIZE       4096
#define LOG_MSG_MAX         (LOG_ENTRY_SIZE - 8)
#define CRASH_BUF_SIZE      80u
#define STAT_BUF_SIZE       80u
#define MAX_STAT_THREADS    12
#define WDT_TIMEOUT_MS      3000u
#define TRINITY_INIT_MAGIC  0xBEEFCAFEu

/************************** STRUCTURE DATA TYPES ******************************/

typedef struct
{
   uint16_t magic;
   uint16_t len;
   uint32_t cycles;
   char     msg[LOG_MSG_MAX];
} LOG_ENTRY_T;

#define ENTRY_TOTAL sizeof(LOG_ENTRY_T)

typedef struct
{
   char msg[STAT_BUF_SIZE];
} STAT_ENTRY_T;

/************************** NOINIT VARIABLES **********************************/

static volatile uint32_t g_noinit_guard
    __attribute__((section(".noinit")));

volatile uint32_t g_init_stage
    __attribute__((section(".noinit")));

static volatile uint32_t g_pre_init_canary
    __attribute__((section(".noinit")));

static volatile uint32_t g_canary_snapshot
    __attribute__((section(".noinit")));

static int canary_pre_init(void)
{
    g_canary_snapshot = g_pre_init_canary;
    g_pre_init_canary = TRINITY_CANARY_ALIVE;
    return 0;
}
SYS_INIT(canary_pre_init, PRE_KERNEL_1, 0);

/************************** STATIC (PRIVATE) DATA *****************************/

static const struct flash_area *g_log_fa           = NULL;
static uint32_t                 g_write_offset      = 0;
static bool                     g_initialized       = false;
static uint32_t                 g_initialized_magic = 0u;
static bool                     g_part_open         = false;

static const struct device     *g_wdt               = NULL;
static int                      g_wdt_channel       = -1;

/* Heap-allocated stat buffer -- same fix as nRF52840 to avoid .noinit overlap */
static STAT_ENTRY_T *s_stat_entries = NULL;
static int           s_stat_count   = 0;

/************************** STATIC (PRIVATE) FUNCTIONS ************************/

static const char *reason_to_str(unsigned int reason)
{
   switch (reason)
   {
      case K_ERR_CPU_EXCEPTION:  { return "CPU_EXCEPTION";  }
      case K_ERR_SPURIOUS_IRQ:   { return "SPURIOUS_IRQ";   }
      case K_ERR_STACK_CHK_FAIL: { return "STACK_OVERFLOW"; }
      case K_ERR_KERNEL_OOPS:    { return "KERNEL_OOPS";    }
      case K_ERR_KERNEL_PANIC:   { return "KERNEL_PANIC";   }
      default:                   { return "UNKNOWN";        }
   }
}

static const char *stage_to_str(uint32_t stage)
{
   switch (stage)
   {
      case TRINITY_STAGE_RESET:     { return "RESET (cookie not yet written)";        }
      case TRINITY_STAGE_GPIO:      { return "GPIO      -- about to init GPIO";       }
      case TRINITY_STAGE_BATTERY:   { return "BATTERY   -- about to call battery_init()"; }
      case TRINITY_STAGE_NVS_INIT:  { return "NVS_INIT  -- about to settings_subsys_init()"; }
      case TRINITY_STAGE_NVS_LOAD:  { return "NVS_LOAD  -- about to load settings";  }
      case TRINITY_STAGE_LOG_INIT:  { return "LOG_INIT  -- about to trinity_log_init()"; }
      case TRINITY_STAGE_WDT_INIT:  { return "WDT_INIT  -- about to trinity_wdt_init()"; }
      case TRINITY_STAGE_BT_ENABLE: { return "BT_ENABLE -- about to bt_enable()";    }
      case TRINITY_STAGE_BT_ADV:    { return "BT_ADV    -- about to bt_le_adv_start()"; }
      case TRINITY_STAGE_MAIN_LOOP: { return "MAIN_LOOP -- entered main while(1)";   }
      default:                      { return "UNKNOWN stage value";                  }
   }
}

static int open_partition(void)
{
   int rc = 0;
   if (g_part_open) { return 0; }
   rc = flash_area_open(LOG_PARTITION_ID, &g_log_fa);
   if (0 == rc) { g_part_open = true; }
   return rc;
}

static void close_partition(void)
{
   if (g_part_open)
   {
      flash_area_close(g_log_fa);
      g_part_open = false;
      g_log_fa    = NULL;
   }
}

static uint32_t find_write_offset(void)
{
   uint32_t    total_entries = 0;
   uint32_t    offset        = 0;
   uint32_t    i             = 0;
   LOG_ENTRY_T entry;

   if (!g_part_open) { return 0; }

   total_entries = g_log_fa->fa_size / ENTRY_TOTAL;
   for (i = 0; i < total_entries; i++)
   {
      offset = i * ENTRY_TOTAL;
      (void)flash_area_read(g_log_fa, offset, &entry, sizeof(entry));
      if (LOG_MAGIC != entry.magic) { return offset; }
   }
   return 0;
}

static void write_internal(const char *p_msg, uint16_t len)
{
   uint32_t    page_start = 0;
   LOG_ENTRY_T entry;

   if (!g_part_open || 0u == len) { return; }

   (void)memset(&entry, 0xFF, sizeof(entry));
   entry.magic  = LOG_MAGIC;
   entry.cycles = k_cycle_get_32();
   entry.len    = MIN(len, (uint16_t)(sizeof(entry.msg) - 1u));
   (void)memcpy(entry.msg, p_msg, entry.len);
   entry.msg[entry.len] = '\0';

   page_start = (g_write_offset / LOG_PAGE_SIZE) * LOG_PAGE_SIZE;
   if (g_write_offset == page_start)
   {
      (void)flash_area_flatten(g_log_fa, page_start, LOG_PAGE_SIZE);
   }
   (void)flash_area_write(g_log_fa, g_write_offset, &entry, sizeof(entry));

   g_write_offset += ENTRY_TOTAL;
   if ((g_write_offset + ENTRY_TOTAL) > g_log_fa->fa_size)
   {
      g_write_offset = 0u;
   }
}

/* No mutex on ESP32-C3 Zephyr -- flash driver serializes internally */
static void write_entry(const char *p_msg, uint16_t len)
{
   if (!g_initialized || !g_part_open || 0u == len) { return; }
   write_internal(p_msg, len);
}

static void write_panic(const char *p_msg, uint16_t len)
{
   if (TRINITY_INIT_MAGIC != g_initialized_magic) { return; }
   if (!g_part_open || 0u == len)                 { return; }
   write_internal(p_msg, len);
}

/************************** PUBLIC FUNCTIONS -- FLASH LOG *********************/

void trinity_log_dump_previous(void)
{
   uint32_t    total_entries = 0;
   uint32_t    i             = 0;
   bool        found_any     = false;
   LOG_ENTRY_T entry;

   if (g_noinit_guard != 0xDEADBEEF)
   {
      printk("[TRINITY] NOINIT_GUARD: 0x%08X (first boot or BSS bleed)\n",
             g_noinit_guard);
   }
   g_noinit_guard = 0xDEADBEEF;

   uint32_t canary = g_canary_snapshot;
   uint32_t stage  = g_init_stage;
   g_canary_snapshot = 0U;
   g_init_stage      = TRINITY_STAGE_RESET;

   if (TRINITY_CANARY_ALIVE == canary)
   {
      printk("[TRINITY] EVENT: BOOT | REASON: PRE_INIT_CRASH\n");
      printk("[TRINITY] NOTE: Previous boot crashed before Trinity init.\n");
      printk("[TRINITY] STAGE: 0x%04X -- %s\n", stage, stage_to_str(stage));
   }
   else if (TRINITY_CANARY_BOOTED == canary)
   {
      printk("[TRINITY] EVENT: BOOT | REASON: POST_INIT_CRASH\n");
      printk("[TRINITY] NOTE: Flash log should contain fault details.\n");
      printk("[TRINITY] STAGE: 0x%04X -- %s\n", stage, stage_to_str(stage));
   }
   else
   {
      printk("[TRINITY] NOTE: Cold boot or first power-on detected.\n");
   }

   if (0 != open_partition())
   {
      printk("[TRINITY] Cannot open log partition\n");
      return;
   }

   total_entries = g_log_fa->fa_size / ENTRY_TOTAL;
   printk("=== Previous Session Logs ===\n");

   for (i = 0; i < total_entries; i++)
   {
      (void)flash_area_read(g_log_fa, i * ENTRY_TOTAL, &entry, sizeof(entry));
      if ((LOG_MAGIC == entry.magic) &&
          (0 < entry.len) &&
          (entry.len < sizeof(entry.msg)))
      {
         entry.msg[entry.len] = '\0';
         printk("[%u cyc] %s", entry.cycles, entry.msg);
         found_any = true;
      }
   }

   if (!found_any) { printk("  (no logs found)\n"); }
   printk("=== End of Previous Logs ===\n");
   close_partition();
}

int trinity_log_erase(void)
{
   int rc = 0;
   if (!g_part_open)
   {
      rc = open_partition();
      if (0 != rc) { return -EIO; }
   }
   rc             = flash_area_flatten(g_log_fa, 0, g_log_fa->fa_size);
   g_write_offset = 0;
   return rc;
}

void trinity_log_event(const char *p_msg)
{
   if (!g_initialized) { return; }
   write_entry(p_msg, (uint16_t)strlen(p_msg));
   printk("[TRINITY] %s\n", p_msg);
}

int trinity_log_init(void)
{
   int rc = 0;
   rc = open_partition();
   if (0 != rc) { return -EIO; }

   g_write_offset      = find_write_offset();
   g_initialized       = true;
   g_initialized_magic = TRINITY_INIT_MAGIC;
   g_pre_init_canary   = TRINITY_CANARY_BOOTED;

   s_stat_entries = k_malloc(sizeof(STAT_ENTRY_T) * MAX_STAT_THREADS);
   if (NULL == s_stat_entries)
   {
      printk("[TRINITY] FATAL: stat buffer alloc failed -- thread stats disabled\n");
      return -ENOMEM;
   }

   return 0;
}

/************************** PUBLIC FUNCTIONS -- WATCHDOG **********************/

void trinity_wdt_init(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH)
   /* Bench: skip WDT entirely. OpenOCD/USB-JTAG is the safety net.
    * wdt_setup(WDT_OPT_PAUSE_HALTED_BY_DBG) resets the chip under the
    * debugger before console output flushes on ESP32-C3. */
   printk("[TRINITY] WDT skipped (bench mode)\n");
   return;
#endif

   struct wdt_timeout_cfg wdt_cfg = {0};
   int                    rc      = 0;

   g_wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));

   if (!device_is_ready(g_wdt))
   {
      printk("[TRINITY] WDT device not ready\n");
      trinity_log_event("EVENT: WDT_INIT_FAIL\n");
      return;
   }

   wdt_cfg.window.min = 0u;
   wdt_cfg.window.max = WDT_TIMEOUT_MS;
   wdt_cfg.callback   = NULL;

   g_wdt_channel = wdt_install_timeout(g_wdt, &wdt_cfg);
   if (0 > g_wdt_channel)
   {
      printk("[TRINITY] WDT install timeout failed: %d\n", g_wdt_channel);
      trinity_log_event("EVENT: WDT_TIMEOUT_FAIL\n");
      return;
   }

   rc = wdt_setup(g_wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
   if (0 != rc)
   {
      printk("[TRINITY] WDT setup failed: %d\n", rc);
      trinity_log_event("EVENT: WDT_SETUP_FAIL\n");
      return;
   }

   printk("[TRINITY] WDT armed (%u ms timeout)\n", WDT_TIMEOUT_MS);
   trinity_log_event("EVENT: WDT_ARMED\n");
}

void trinity_wdt_kick(void)
{
   if ((NULL != g_wdt) && (0 <= g_wdt_channel))
   {
      (void)wdt_feed(g_wdt, g_wdt_channel);
   }
}

/************************** PUBLIC FUNCTIONS -- HEAP STATS ********************/

void trinity_log_heap_stats(void)
{
#if defined(CONFIG_SYS_HEAP_RUNTIME_STATS)
   extern struct k_heap _system_heap;
   struct sys_memory_stats stats = {0};
   char                    msg[STAT_BUF_SIZE];
   int                     rc    = 0;

   rc = sys_heap_runtime_stats_get(&_system_heap.heap, &stats);
   if (0 != rc)
   {
      printk("[TRINITY] Heap stats unavailable\n");
      return;
   }

   (void)snprintf(msg, sizeof(msg),
                  "Heap: free=%zu allocated=%zu max_allocated=%zu\n",
                  stats.free_bytes,
                  stats.allocated_bytes,
                  stats.max_allocated_bytes);
   trinity_log_event(msg);
#else
   trinity_log_event("Heap stats: CONFIG_SYS_HEAP_RUNTIME_STATS not set\n");
#endif
}

/************************** PUBLIC FUNCTIONS -- TASK STATS ********************/

static void trinity_thread_cb_collect(struct thread_analyzer_info *p_info)
{
   if (NULL == s_stat_entries)            { return; }
   if (s_stat_count >= MAX_STAT_THREADS)  { return; }

   (void)snprintf(s_stat_entries[s_stat_count].msg, STAT_BUF_SIZE,
                  "[TRINITY] Thread: %-16s cpu=%3u%%  stack=%u/%u\n",
                  p_info->name,
                  p_info->utilization,
                  p_info->stack_used,
                  p_info->stack_size);
   s_stat_count++;
}

void trinity_log_task_stats(void)
{
   int i;

   if (NULL == s_stat_entries) { return; }

   trinity_wdt_kick();
   s_stat_count = 0;
   thread_analyzer_run(trinity_thread_cb_collect, 0);

   trinity_wdt_kick();
   write_entry("[TRINITY] Thread stats:\n",
               (uint16_t)strlen("[TRINITY] Thread stats:\n"));
   printk("[TRINITY] Thread stats:\n");

   for (i = 0; i < s_stat_count; i++)
   {
      write_entry(s_stat_entries[i].msg,
                  (uint16_t)strlen(s_stat_entries[i].msg));
      printk("%s", s_stat_entries[i].msg);
   }
   trinity_wdt_kick();
}

/************************** ZEPHYR FAULT HOOKS ********************************/

void k_sys_fatal_error_handler(unsigned int reason,
                                const struct arch_esf *p_esf)
{
   char msg[CRASH_BUF_SIZE] = {0};

   if (NULL != p_esf)
   {
      /* RISC-V: mepc = machine exception PC, ra = return address */
      (void)snprintf(msg, sizeof(msg),
                     "EVENT: CRASH | TYPE: %s | PC: 0x%08X | RA: 0x%08X"
                     " | STG: 0x%04X\n",
                     reason_to_str(reason),
                     (unsigned int)p_esf->mepc,
                     (unsigned int)p_esf->ra,
                     (unsigned int)g_init_stage);
   }
   else
   {
      (void)snprintf(msg, sizeof(msg),
                     "EVENT: CRASH | TYPE: %s | PC: unknown | STG: 0x%04X\n",
                     reason_to_str(reason),
                     (unsigned int)g_init_stage);
   }

   write_panic(msg, (uint16_t)strlen(msg));

#if defined(CONFIG_TRINITY_MODE_BENCH)
   printk("\n[TRINITY] === FAULT HALT ===\n");
   printk("[TRINITY] %s", msg);
   printk("[TRINITY] === HALTED -- attach OpenOCD ===\n");
   irq_lock();
   while (1) {}

#elif defined(CONFIG_TRINITY_MODE_USB)
   printk("[TRINITY] %s", msg);
   k_sleep(K_MSEC(500));
   irq_lock();
   sys_reboot(SYS_REBOOT_COLD);

#else
   irq_lock();
   sys_reboot(SYS_REBOOT_COLD);
#endif
}

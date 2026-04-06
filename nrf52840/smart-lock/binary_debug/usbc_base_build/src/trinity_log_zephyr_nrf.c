/******************************************************************************
 * \file trinity_log_zephyr_nrf.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Trinity crash logger -- nRF52840 Zephyr (reed-sensor, smart-lock,
 *        smart-light).
 *
 * \details Platform: nRF52840, Zephyr RTOS, Cortex-M4.
 *          Storage:  Flash partition (log_partition in DTS overlay).
 *          Console:  RTT (bench) / UART (field) / USB CDC (USB).
 *          Fault:    k_sys_fatal_error_handler + CFSR/HFSR/MMFAR/BFAR.
 *          Canary:   .noinit ELF section + SYS_INIT(PRE_KERNEL_1).
 *          WDT:      Zephyr wdt driver; bench feeds bootloader WDT ch0.
 *
 *          Build modes (set via Kconfig / boards/bench.conf):
 *            TRINITY_MODE_BENCH -- RTT + SWD halt (J-Link)
 *            TRINITY_MODE_USB   -- CDC ACM 500ms flush + NVIC_SystemReset()
 *            TRINITY_MODE_FIELD -- UART + NVIC_SystemReset() (default)
 *
 *          prj.conf requirements:
 *            CONFIG_WATCHDOG=y
 *            CONFIG_THREAD_ANALYZER=y
 *            CONFIG_THREAD_ANALYZER_USE_PRINTK=n
 *            CONFIG_SYS_HEAP_RUNTIME_STATS=y
 *            CONFIG_THREAD_RUNTIME_STATS=y
 *            CONFIG_INIT_STACKS=y
 *            CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=3072
 *            CONFIG_MAIN_STACK_SIZE=2048
 *            CONFIG_HEAP_MEM_POOL_SIZE=4096
 *
 * \note    WDT fix (2026-03-21):
 *          nRF52840 WDT cannot be stopped after first boot.
 *          wdt_install_timeout() returns -ENOTSUP on subsequent resets.
 *          Treat -EALREADY/-EBUSY/-ENOTSUP as "already running, use ch0".
 *
 * \note    WDT bench fix (2026-03-22):
 *          wdt_setup(WDT_OPT_PAUSE_HALTED_BY_DBG) resets the chip under
 *          J-Link before RTT can output anything on a fresh board.
 *          Bench mode grabs wdt0, assumes channel 0 (bootloader), and feeds
 *          immediately without calling wdt_install_timeout()/wdt_setup().
 *
 * \note    Pre-init canary (2026-03-23):
 *          g_pre_init_canary written at PRE_KERNEL_1,0. Canary snapshot
 *          saves the previous session value before overwriting, so
 *          dump_previous() can classify PRE vs POST init crash correctly.
 *
 * \note    Init stage cookie (2026-03-24):
 *          g_init_stage in .noinit. Written by main() before each risky
 *          call. 0x0000 = crashed before main() wrote its first cookie.
 *
 * \note    Flash concurrency fix (2026-03-24):
 *          g_flash_mutex serializes all NVMC writes. nRF52840 NVMC is
 *          single-writer. Callers outside Trinity must use trinity_flash_lock/
 *          unlock() around settings_save_one() or equivalent.
 *
 * \note    Static buffer fix (2026-03-24):
 *          trinity_thread_cb() previously used static char msg[]. Concurrent
 *          calls corrupted return addresses.
 *          Confirmed crash: PC=0x5954494E ("NITY"), SP=0x003BA538.
 *
 * \note    Panic bypass + write_internal refactor (2026-03-25):
 *          write_internal() -- raw flash write, no locks, no OS calls.
 *          write_entry()    -- mutex wrapper for normal thread context.
 *          write_panic()    -- lock-free path for fault handler only.
 *
 * \note    k_cycle_get_32() in fault handler (2026-03-25):
 *          k_uptime_get_32() depends on SysTick ISR which may be dead at
 *          fault time. k_cycle_get_32() reads DWT hardware counter directly.
 *
 * \note    g_initialized_magic sentinel (2026-03-25):
 *          write_panic() validates TRINITY_INIT_MAGIC before touching flash
 *          to avoid nested faults if Trinity never completed init.
 *
 * \note    Canary snapshot fix (2026-03-26):
 *          canary_pre_init() copies g_pre_init_canary into g_canary_snapshot
 *          before overwriting with CANARY_ALIVE. dump_previous() reads
 *          g_canary_snapshot so it sees the previous session's value, not
 *          the freshly-written CANARY_ALIVE from this boot.
 *
 * \note    Sched-lock violation fix (2026-03-27):
 *          k_thread_foreach() holds _sched_spinlock for entire iteration.
 *          Calling write_entry() -> k_mutex_lock() inside that region is
 *          illegal and corrupts kernel state under contention.
 *          Confirmed crash: PC=0x5954494E ("NITY") every ~360s.
 *          Fix: collect-then-write. trinity_thread_cb_collect() does only
 *          snprintf under sched lock. write_entry() called after lock released.
 *
 * \note    Stat buffer heap fix (2026-03-28):
 *          s_stat_entries[12] as a static BSS array placed at 0x200079C1.
 *          960-byte span crossed .noinit boundary at 0x20007A80, trampling
 *          g_init_stage, g_pre_init_canary, g_canary_snapshot, g_noinit_guard,
 *          and logging_stack on every stats cycle.
 *          Confirmed via arm-none-eabi-nm: s_stat_entries at 0x200079C1,
 *          g_init_stage at 0x20007A80 -- 191 bytes gap, 960-byte array.
 *          Fix: heap-allocated in trinity_log_init() via k_malloc().
 *          Heap base (0x2000B8B0) is above all .noinit and stack regions.
 *
 * \note    RESETREAS normalize fix (2026-03-27):
 *          RESETREAS is a latched OR-history register. trinity_log_boot_reason()
 *          centralizes decode so all nRF52840 boards get identical logging.
 *          main() must read and clear RESETREAS before calling anything else.
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
#include <nrf.h>

/******************************** CONSTANTS ***********************************/

#define LOG_PARTITION_ID    FIXED_PARTITION_ID(log_partition)
#define LOG_MAGIC           0xA55A
#define LOG_ENTRY_SIZE      128
#define LOG_PAGE_SIZE       4096
#define LOG_MSG_MAX         (LOG_ENTRY_SIZE - 8)
#define CRASH_BUF_SIZE      112u
#define STAT_BUF_SIZE       80u
#define MAX_STAT_THREADS    12

#define WDT_TIMEOUT_MS      3000u
#define TRINITY_INIT_MAGIC  0xBEEFCAFEu

/* ARM Cortex-M4 SCB fault register addresses */
#define SCB_CFSR   (*((volatile uint32_t *)0xE000ED28))
#define SCB_HFSR   (*((volatile uint32_t *)0xE000ED2C))
#define SCB_MMFAR  (*((volatile uint32_t *)0xE000ED34))
#define SCB_BFAR   (*((volatile uint32_t *)0xE000ED38))
#define CFSR_MMARVALID  (1u << 7)
#define CFSR_BFARVALID  (1u << 15)

/* nRF52840 RESETREAS bit definitions -- private, not exposed in header */
#define TRINITY_RESET_BROWNOUT  (1u << 0)
#define TRINITY_RESET_PIN       (1u << 1)
#define TRINITY_RESET_DOG       (1u << 2)
#define TRINITY_RESET_SREQ      (1u << 3)
#define TRINITY_RESET_REPOR     (1u << 16)

/************************** STRUCTURE DATA TYPES ******************************/

typedef struct
{
   uint16_t magic;
   uint16_t len;
   uint32_t cycles;   /* k_cycle_get_32() -- immune to interrupt state */
   char     msg[LOG_MSG_MAX];
} LOG_ENTRY_T;

#define ENTRY_TOTAL sizeof(LOG_ENTRY_T)

typedef struct
{
   char msg[STAT_BUF_SIZE];
} STAT_ENTRY_T;

/************************** NOINIT VARIABLES **********************************/

/* g_noinit_guard: sentinel to detect BSS bleed into .noinit.
 * Set to 0xDEADBEEF on first valid boot. If read as 0 on a non-cold boot,
 * BSS zeroing has overrun the section boundary. */
static volatile uint32_t g_noinit_guard
    __attribute__((section(".noinit")));

/* g_init_stage: declared in header as extern, defined here. */
volatile uint32_t g_init_stage
    __attribute__((section(".noinit")));

/* g_pre_init_canary: written CANARY_ALIVE at PRE_KERNEL_1 and upgraded
 * to CANARY_BOOTED inside trinity_log_init(). Reflects CURRENT boot. */
static volatile uint32_t g_pre_init_canary
    __attribute__((section(".noinit")));

/* g_canary_snapshot: copy of g_pre_init_canary saved by canary_pre_init()
 * BEFORE overwriting with CANARY_ALIVE. Holds PREVIOUS session's value. */
static volatile uint32_t g_canary_snapshot
    __attribute__((section(".noinit")));

static int canary_pre_init(void)
{
    g_canary_snapshot = g_pre_init_canary;
    g_pre_init_canary = TRINITY_CANARY_ALIVE;
    return 0;
}
SYS_INIT(canary_pre_init, PRE_KERNEL_1, 0);

/************************** FLASH MUTEX ***************************************/

static struct k_mutex g_flash_mutex;
static bool           g_mutex_ready = false;

/************************** STATIC (PRIVATE) DATA *****************************/

static const struct flash_area *g_log_fa           = NULL;
static uint32_t                 g_write_offset      = 0;
static bool                     g_initialized       = false;
static uint32_t                 g_initialized_magic = 0u;
static bool                     g_part_open         = false;

static const struct device     *g_wdt               = NULL;
static int                      g_wdt_channel       = -1;

/*
 * Thread stats collection buffer -- heap allocated in trinity_log_init().
 *
 * MUST NOT be a static BSS array. The linker may place a 960-byte BSS
 * array at an address that crosses the .noinit boundary (confirmed at
 * 0x200079C1 on smart-lock, crossing .noinit at 0x20007A80), trampling
 * g_init_stage, g_pre_init_canary, g_canary_snapshot, g_noinit_guard,
 * and the logging thread stack. Every stats cycle overwrote these with
 * thread name strings, causing PRE_INIT_CRASH misclassification and
 * PC=0x5954494E HardFaults. Heap base (0x2000B8B0) is safely above all
 * .noinit and stack regions.
 */
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
      case TRINITY_STAGE_GPIO:      { return "GPIO      -- about to init GPIO/PWM/LED"; }
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

/**
 * \brief  Raw flash write core -- no locks, no OS calls.
 *         Called by write_entry() (holds mutex) and write_panic() (no mutex).
 *         Uses k_cycle_get_32() -- immune to SysTick state at crash time.
 */
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

/** Thread-safe flash write -- acquires g_flash_mutex. Normal thread context. */
static void write_entry(const char *p_msg, uint16_t len)
{
   if (!g_initialized || !g_part_open || 0u == len) { return; }
   k_mutex_lock(&g_flash_mutex, K_FOREVER);
   write_internal(p_msg, len);
   k_mutex_unlock(&g_flash_mutex);
}

/** Lock-free flash write -- fault handler only. Validates init magic first. */
static void write_panic(const char *p_msg, uint16_t len)
{
   if (TRINITY_INIT_MAGIC != g_initialized_magic) { return; }
   if (!g_part_open || 0u == len)                 { return; }
   write_internal(p_msg, len);
}

/************************** PUBLIC FUNCTIONS -- FLASH MUTEX *******************/

void trinity_flash_lock(void)
{
   if (g_mutex_ready) { k_mutex_lock(&g_flash_mutex, K_FOREVER); }
}

void trinity_flash_unlock(void)
{
   if (g_mutex_ready) { k_mutex_unlock(&g_flash_mutex); }
}

/************************** PUBLIC FUNCTIONS -- FLASH LOG *********************/

void trinity_log_dump_previous(void)
{
   uint32_t    total_entries = 0;
   uint32_t    i             = 0;
   bool        found_any     = false;
   LOG_ENTRY_T entry;

   printk("[TRINITY] BUILD: %s | MODE: %s\n",
          CONFIG_TRINITY_MODE_STR,
          IS_ENABLED(CONFIG_TRINITY_MODE_BENCH) ? "BENCH" :
          IS_ENABLED(CONFIG_TRINITY_MODE_USB)   ? "USB"   : "FIELD");

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
      printk("[TRINITY] NOTE: ISR stack overflow or early SYS_INIT fault.\n");
      printk("[TRINITY] NOTE: No flash log available for that crash.\n");
      printk("[TRINITY] STAGE: 0x%04X -- %s\n", stage, stage_to_str(stage));
   }
   else if (TRINITY_CANARY_BOOTED == canary)
   {
      printk("[TRINITY] EVENT: BOOT | REASON: POST_INIT_CRASH\n");
      printk("[TRINITY] NOTE: Previous boot crashed after Trinity init.\n");
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

void trinity_log_boot_reason(uint32_t resetreas)
{
   const char *label   = "NONE";
   char        buf[80] = {0};

   if      (resetreas & TRINITY_RESET_DOG)      { label = "WATCHDOG";   }
   else if (resetreas & TRINITY_RESET_BROWNOUT) { label = "BROWNOUT";   }
   else if (resetreas & TRINITY_RESET_SREQ)     { label = "SOFT_RESET"; }
   else if (resetreas & TRINITY_RESET_PIN)      { label = "RESET_PIN";  }
   else if (resetreas & TRINITY_RESET_REPOR)    { label = "COLD_POR";   }

   (void)snprintf(buf, sizeof(buf),
                  "EVENT: BOOT | RESETREAS: 0x%08X | REASON: %s\n",
                  resetreas, label);
   trinity_log_event(buf);
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

   k_mutex_init(&g_flash_mutex);
   g_mutex_ready = true;

   rc = open_partition();
   if (0 != rc) { return -EIO; }

   g_write_offset      = find_write_offset();
   g_initialized       = true;
   g_initialized_magic = TRINITY_INIT_MAGIC;
   g_pre_init_canary   = TRINITY_CANARY_BOOTED;

   /* Heap-allocate stat buffer. Must not be BSS -- linker may place 960-byte
    * array crossing .noinit boundary, confirmed via nm on smart-lock build. */
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
   /* Bench: bootloader WDT cannot be stopped (-EPERM). Grab wdt0, assume
    * channel 0 (bootloader always uses 0), feed immediately. Do NOT call
    * wdt_install_timeout()/wdt_setup() -- bootloader already configured it. */
   g_wdt         = DEVICE_DT_GET(DT_NODELABEL(wdt0));
   g_wdt_channel = 0;
   if (device_is_ready(g_wdt))
   {
      wdt_feed(g_wdt, 0);
      printk("[TRINITY] WDT bench-kick (bootloader WDT cannot be stopped on nRF52840)\n");
   }
   else
   {
      printk("[TRINITY] WDT device not ready in bench mode\n");
   }
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

   if ((-EALREADY == g_wdt_channel) ||
       (-EBUSY    == g_wdt_channel) ||
       (-ENOTSUP  == g_wdt_channel))
   {
      g_wdt_channel = 0;
      printk("[TRINITY] WDT already running, continuing\n");
   }
   else if (0 > g_wdt_channel)
   {
      printk("[TRINITY] WDT install timeout failed: %d\n", g_wdt_channel);
      trinity_log_event("EVENT: WDT_TIMEOUT_FAIL\n");
      return;
   }

   rc = wdt_setup(g_wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
   if (0 != rc)
   {
      if (-EBUSY != rc)
      {
         printk("[TRINITY] WDT setup failed: %d\n", rc);
         trinity_log_event("EVENT: WDT_SETUP_FAIL\n");
         return;
      }
      printk("[TRINITY] WDT setup skipped (already running)\n");
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

/**
 * \brief  Thread analyzer callback -- runs under sched lock (k_thread_foreach).
 *
 * \warning MUST NOT call write_entry() or k_mutex_lock() here.
 *          Doing so is illegal under sched lock and corrupts kernel state.
 *          Only snprintf into the heap-allocated buffer is permitted.
 */
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

/**
 * \brief  Log per-thread CPU% and stack HWM -- collect-then-write pattern.
 *
 * \details Phase 1: thread_analyzer_run() collects under sched lock.
 *          Only snprintf into heap buffer -- no mutex, no blocking.
 *          Phase 2: sched lock released. write_entry() called safely.
 *          WDT kicked before Phase 1 and after Phase 2.
 *
 * \warning Call only from sysworkq. s_stat_entries has no extra locking.
 */
void trinity_log_task_stats(void)
{
   int i;

   if (NULL == s_stat_entries) { return; }

   /* Phase 1: collect under sched lock */
   trinity_wdt_kick();
   s_stat_count = 0;
   thread_analyzer_run(trinity_thread_cb_collect, 0);

   /* Phase 2: write after sched lock released */
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
   uint32_t cfsr   = SCB_CFSR;
   uint32_t hfsr   = SCB_HFSR;
   uint32_t mmfar  = (cfsr & CFSR_MMARVALID) ? SCB_MMFAR : 0u;
   uint32_t bfar   = (cfsr & CFSR_BFARVALID) ? SCB_BFAR  : 0u;
   uint32_t cycles = k_cycle_get_32();

   char line1[CRASH_BUF_SIZE] = {0};
   char line2[CRASH_BUF_SIZE] = {0};

   if (NULL != p_esf)
   {
      (void)snprintf(line1, sizeof(line1),
                     "EVENT: CRASH | TYPE: %s(%u)"
                     " | PC: 0x%08X | LR: 0x%08X | SP: 0x%08X"
                     " | CYC: %u | STG: 0x%04X\n",
                     reason_to_str(reason), reason,
                     (unsigned int)p_esf->basic.pc,
                     (unsigned int)p_esf->basic.lr,
                     (unsigned int)p_esf,
                     cycles,
                     (unsigned int)g_init_stage);
   }
   else
   {
      (void)snprintf(line1, sizeof(line1),
                     "EVENT: CRASH | TYPE: %s(%u) | PC: unknown"
                     " | CYC: %u | STG: 0x%04X\n",
                     reason_to_str(reason), reason,
                     cycles,
                     (unsigned int)g_init_stage);
   }

   (void)snprintf(line2, sizeof(line2),
                  "EVENT: FAULT | CFSR: 0x%08X | HFSR: 0x%08X"
                  " | MMFAR: 0x%08X | BFAR: 0x%08X\n",
                  cfsr, hfsr, mmfar, bfar);

   write_panic(line1, (uint16_t)strlen(line1));
   write_panic(line2, (uint16_t)strlen(line2));

#if defined(CONFIG_TRINITY_MODE_BENCH)
   printk("\n[TRINITY] === FAULT HALT ===\n");
   printk("[TRINITY] %s", line1);
   printk("[TRINITY] %s", line2);
   printk("[TRINITY] STG  : 0x%04X -- %s\n",
          (unsigned int)g_init_stage, stage_to_str(g_init_stage));
   printk("[TRINITY] === HALTED -- attach JLink or read RTT ===\n");
   __disable_irq();
   while (1) {}

#elif defined(CONFIG_TRINITY_MODE_USB)
   k_sleep(K_MSEC(500));
   __disable_irq();
   NVIC_SystemReset();

#else
   __disable_irq();
   NVIC_SystemReset();
#endif
}

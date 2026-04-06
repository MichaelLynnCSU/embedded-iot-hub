/******************************************************************************
 * \file trinity_flash_esp.c
 * \brief Trinity flash log -- ESP32-C3 Zephyr.
 *
 * \details No flash mutex -- ESP32-C3 flash driver serializes internally.
 *          Depends on trinity_canary_esp.c for noinit variables.
 *          Does NOT depend on trinity_wdt_esp.c -- no WDT kick during
 *          partition scan because ESP32-C3 bootloader WDT is not an issue.
 ******************************************************************************/

#include "trinity_log.h"
#include "trinity_private_esp.h"
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <string.h>
#include <stdio.h>

/******************************** CONSTANTS ***********************************/

#define LOG_PARTITION_ID    FIXED_PARTITION_ID(log_partition)
#define LOG_MAGIC           0xA55A
#define LOG_ENTRY_SIZE      128
#define LOG_PAGE_SIZE       4096
#define LOG_MSG_MAX         (LOG_ENTRY_SIZE - 8)
#define STAT_BUF_SIZE       80u
#define MAX_STAT_THREADS    12
#define TRINITY_INIT_MAGIC  0xBEEFCAFEu

typedef struct
{
   uint16_t magic;
   uint16_t len;
   uint32_t cycles;
   char     msg[LOG_MSG_MAX];
} LOG_ENTRY_T;

#define ENTRY_TOTAL sizeof(LOG_ENTRY_T)

/************************** PRIVATE DATA **************************************/

static const struct flash_area *g_log_fa           = NULL;
static uint32_t                 g_write_offset      = 0;
static bool                     g_initialized       = false;
static uint32_t                 g_initialized_magic = 0u;
static bool                     g_part_open         = false;

/************************** PRIVATE FUNCTIONS *********************************/

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

/* No mutex -- ESP32-C3 flash driver serializes internally */
static void write_entry(const char *p_msg, uint16_t len)
{
   if (!g_initialized || !g_part_open || 0u == len) { return; }
   write_internal(p_msg, len);
}

/* Exported via trinity_private_esp.h for trinity_fault_esp.c only */
void write_panic(const char *p_msg, uint16_t len)
{
   if (TRINITY_INIT_MAGIC != g_initialized_magic) { return; }
   if (!g_part_open || 0u == len)                 { return; }
   write_internal(p_msg, len);
}

/************************** DUMP CORE *****************************************/

static const char *stage_to_str(uint32_t stage)
{
   switch (stage)
   {
      case TRINITY_STAGE_RESET:     { return "RESET (cookie not yet written)";          }
      case TRINITY_STAGE_GPIO:      { return "GPIO      -- about to init GPIO";         }
      case TRINITY_STAGE_BATTERY:   { return "BATTERY   -- about to call battery_init()"; }
      case TRINITY_STAGE_NVS_INIT:  { return "NVS_INIT  -- about to settings_subsys_init()"; }
      case TRINITY_STAGE_NVS_LOAD:  { return "NVS_LOAD  -- about to load settings";    }
      case TRINITY_STAGE_LOG_INIT:  { return "LOG_INIT  -- about to trinity_log_init()"; }
      case TRINITY_STAGE_WDT_INIT:  { return "WDT_INIT  -- about to trinity_wdt_init()"; }
      case TRINITY_STAGE_BT_ENABLE: { return "BT_ENABLE -- about to bt_enable()";      }
      case TRINITY_STAGE_BT_ADV:    { return "BT_ADV    -- about to bt_le_adv_start()"; }
      case TRINITY_STAGE_MAIN_LOOP: { return "MAIN_LOOP -- entered main while(1)";     }
      default:                      { return "UNKNOWN stage value";                    }
   }
}

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

/************************** PUBLIC API ****************************************/

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

   trinity_canary_set_booted();
   trinity_log_stats_init();

   return 0;
}

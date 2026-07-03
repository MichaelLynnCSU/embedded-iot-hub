/******************************************************************************
 * \file    cam_trigger_status_reader.c
 * \brief   Listens for CamTriggerStatus drop notifications from
 *          camera_manager and surfaces them into data_controller.log,
 *          closing the Finding 7 visibility gap.
 *
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "cam_trigger_ipc.h"
#include "log.h"
#include "cam_trigger_status_reader.h"

#include "globals.h"

static const char *drop_reason_str(uint8_t reason)
{
   switch (reason)
   {
      case CAM_DROP_ZONE_OUT_OF_RANGE: return "zone_out_of_range";
      case CAM_DROP_ZONE_OFFLINE:      return "camera_offline";
      case CAM_DROP_NO_IP:             return "no_ip_known";
      default:                         return "unknown";
   }
}

void *cam_trigger_status_thread(void *p_arg)
{
   struct sockaddr_un       bind_addr = {0};
   struct CamTriggerStatus  msg       = {0};
   int                      sock      = -1;
   ssize_t                  n         = 0;
   (void)p_arg;

   unlink(CAM_TRIGGER_STATUS_SOCK);
   sock = socket(AF_UNIX, SOCK_DGRAM, 0);
   if (sock < 0)
   {
      LOG_ERR("cam_trigger_status_thread: socket() failed errno=%d", errno);
      return NULL;
   }

   bind_addr.sun_family = AF_UNIX;
   strncpy(bind_addr.sun_path, CAM_TRIGGER_STATUS_SOCK,
           sizeof(bind_addr.sun_path) - 1);

   if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
   {
      LOG_ERR("cam_trigger_status_thread: bind() failed on %s errno=%d",
              CAM_TRIGGER_STATUS_SOCK, errno);
      close(sock);
      return NULL;
   }

   LOG_INF("cam_trigger_status_thread: listening on %s", CAM_TRIGGER_STATUS_SOCK);

   while (running)
   {
      n = recv(sock, &msg, sizeof(msg), 0);
      if (n != (ssize_t)sizeof(msg))
      {
         if (running) { LOG_ERR("cam_trigger_status_thread: bad recv n=%zd", n); }
         continue;
      }

      LOG_INF("[DISPATCH] cam_trigger_dropped event_id=%llu zone=%u reason=%s",
              (unsigned long long)msg.event_id,
              (unsigned)msg.zone,
              drop_reason_str(msg.reason));
   }

   close(sock);
   unlink(CAM_TRIGGER_STATUS_SOCK);
   return NULL;
}

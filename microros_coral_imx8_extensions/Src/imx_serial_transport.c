#include <uxr/client/profile/transport/serial/serial_transport_external.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "fsl_debug_console.h"
#include "fsl_uart_freertos.h"
#include "fsl_uart.h"
#include "board.h"


#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"

#define RPMSG_LITE_SHMEM_BASE         (VDEV0_VRING_BASE)
#define RPMSG_LITE_LINK_ID            (RL_PLATFORM_IMX8MQ_M4_USER_LINK_ID)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel-1"
#define APP_TASK_STACK_SIZE (256)
#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif


volatile uint32_t remote_addr_rpmsg;
struct rpmsg_lite_endpoint *volatile my_ept;
volatile rpmsg_queue_handle my_queue;
struct rpmsg_lite_instance *volatile my_rpmsg;

#define BUFSIZE 1024
char rx_buffer[BUFSIZE];
uint32_t rx_buffer_pos = 0;

uint32_t packetCount = 0;

void *rx_buf;


bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{

    int32_t result;
    uint32_t len;

    #ifdef MCMGR_USED
    uint32_t startupData;

    /* Get the startup data */
    (void)MCMGR_GetStartupData(kMCMGR_Core1, &startupData);

    my_rpmsg = rpmsg_lite_remote_init((void *)startupData, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);

    /* Signal the other core we are ready */
    (void)MCMGR_SignalReady(kMCMGR_Core1);
    #else
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    #endif /* MCMGR_USED */

    while (0 == rpmsg_lite_is_link_up(my_rpmsg)) {
      vTaskDelay(10);
    }

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept   = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    (void)rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);

    printf("\r\nNameservice sent, ready for incoming messages...\r\n");

    /* Wait for initial hello messge of the imx_rpmsg_tty driver */
    result =
        rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr_rpmsg, (char **)&rx_buf, &len, RL_BLOCK);

    result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);


  return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{
 // HAL_UART_DMAStop(platform->uart);
  return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
  void *tx_buf;
  int32_t result;
  if(len >= 496) {
    printf("PACKET TOO LARGE!\r\n");
    return 0;
  }

  // get a tx buffer and copy buf contents
  tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, 496, RL_BLOCK);
  memcpy(tx_buf,buf,len);
  result = rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr_rpmsg,tx_buf, len );

  if(result == RL_SUCCESS){
    return len;
  }
  else {
    printf("Write Failed. %i \r\n", result);
    return 0;
  }
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
  int32_t result;
 	uint32_t len_message = 0;

  // if buffer is empty, get new data
  if(rx_buffer_pos == 0){
      result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr_rpmsg, (char **)&rx_buf, &len_message, timeout);
      if(result == RL_SUCCESS) {
        // printf("%i \r\n", len_message);
        memcpy(rx_buffer, rx_buf, len_message);
        rx_buffer_pos+= len_message;
        result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
      }
  }

  // if more data is requested than present
  if(len > rx_buffer_pos) {
    len = rx_buffer_pos;
  }
  if(len > 0){
    memcpy(buf, rx_buffer, len);

    if((rx_buffer_pos-len) > 0) {
      memcpy(rx_buffer, rx_buffer+len, rx_buffer_pos-len);
    }

    rx_buffer_pos-= len;
  }

  return len;
}

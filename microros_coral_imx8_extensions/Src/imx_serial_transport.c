#include <uxr/client/profile/transport/serial/serial_transport_external.h>
//#include "stm32f4xx_hal_dma.h"

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

#define BOARD_DEBUG_UART_CLK_FREQ                                                           \
    CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootUart3)) / \
        (CLOCK_GetRootPostDivider(kCLOCK_RootUart3)) / 10


#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

uart_rtos_handle_t handle;
struct _uart_handle t_handle;

const char *to_send               = "here\r\n";

volatile uint32_t remote_addr_rpmsg;
struct rpmsg_lite_endpoint *volatile my_ept;
volatile rpmsg_queue_handle my_queue;
struct rpmsg_lite_instance *volatile my_rpmsg;

void *rx_buf;
void *tx_buf;

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{

PRINTF("Calling init serial from correct place\r\n");
// uint8_t background_buffer[32];
//
// uart_rtos_config_t uart_config = {
//
//     .baudrate    = 230400,
//
//     .parity      = kUART_ParityDisabled,
//
//     .stopbits    = kUART_OneStopBit,
//
//     .buffer      = dma_buffer,
//
//     .buffer_size = sizeof(dma_buffer),
//
// };
//
//     int ret;
//     uart_config.srcclk = BOARD_DEBUG_UART_CLK_FREQ;
//     uart_config.base   = UART3;
//
//     ret = UART_RTOS_Init(&handle, &t_handle, &uart_config);
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

    PRINTF("\r\nNameservice sent, ready for incoming messages...\r\n");

    result =
        rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr_rpmsg, (char **)&rx_buf, &len, RL_BLOCK);

    result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);

    PRINTF("Ret: %i\r\n", result);
    len = 496;
    tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &len, RL_BLOCK);

  return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{
 // HAL_UART_DMAStop(platform->uart);
  return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
int32_t result;
// PRINTF("Writing Serial: %i\r\n", len);
memcpy(tx_buf,buf,len);
result = rpmsg_lite_send(my_rpmsg, my_ept, remote_addr_rpmsg, tx_buf, len, 100);
// PRINTF("Done writing Serial: %i\r\n", result);
// UART_RTOS_Send(&handle, buf, len);

  return len;
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{

  // PRINTF("Reading Serial: %i\r\n", len);

  int32_t result;

  result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (uint32_t *)&remote_addr_rpmsg, (char **)&rx_buf, &len, 1000);
  // PRINTF("Serial read: %i %i\r\n", len, result);

  if(result == 0) {
    // PRINTF("Received: %i\r\n", len);
    memcpy(buf, rx_buf, len);
    result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
  }
  else {
    len = 0;
  }

  // size_t n = 0;
  // int error;
  //
  // error = UART_RTOS_Receive(&handle, buf, len, &n, timeout);

  return len;
}

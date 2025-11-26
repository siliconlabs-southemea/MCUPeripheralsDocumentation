#include "em_chip.h"
#include "app.h"
#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_prs.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_eusart.h"
#include "dmadrv.h"
#include "sl_clock_manager.h"
#include "sl_power_manager.h"
#include "sl_device_peripheral.h"


#define MATCH_VALUE         0x01
#define TIMER0_PRS_CHANNEL    0
#define DELAY  1UL  //1 ms

// SPI transfer buffer size
#define BUFFER_SIZE         4

// Descriptor linked lists for LDMA transfer
LDMA_Descriptor_t descLink0[8];
LDMA_Descriptor_t descLink1[3];


uint8_t dstBuffer[2][BUFFER_SIZE];
uint8_t srcBuffer[BUFFER_SIZE] = {0x11, 0x22, 0x44, 0x88};

unsigned int rx_channel, tx_channel;
#define SL_USART_SPI EUSART1
LDMA_TransferCfg_t ldmaTXConfig;
LDMA_TransferCfg_t ldmaRXConfig;
uint8_t buffer_idx = 0;

/***************************************************************************//**
 * @brief
 *   LDMA IRQ handler.
 ******************************************************************************/
void ldma_cb(void)
{
  uint8_t idx = buffer_idx % 2;

/* This should only be enabled in SPI loopback test */
#ifdef DEBUG_DRIVER
  uint8_t i = buffer_idx % BUFFER_SIZE;
  /* A RX operation has just terminated */
  if (dstBuffer[idx][i] != ((1 << i) | (1 << (i + 4))))
    printf(".");
  //printf( exp %x, got %x, idx: %d\n", (1 << i) | (1 << (i + 4)), dstBuffer[idx][i], buffer_idx);

  for (uint8_t j = 0; j < BUFFER_SIZE; j++)
    dstBuffer[idx][j] = 0;
#endif

  sensor_driver_callback(dstBuffer[idx]);
  buffer_idx++;
}

#define EUS1SPI_PORT gpioPortC
#define CS_PIN 3

void initEUSART1(void)
{
  CMU_ClockEnable(cmuClock_EUSART1, true);
  // Configure CS pin as an output
  GPIO_PinModeSet(EUS1SPI_PORT, CS_PIN, gpioModePushPull, 1);
  // Configure MOSI (TX) pin as an output
  GPIO_PinModeSet(EUS1SPI_PORT, 0, gpioModePushPull, 0);
  // Configure MISO (RX) pin as an input
  GPIO_PinModeSet(EUS1SPI_PORT, 1, gpioModeInput, 0);
  // Configure SCLK pin as an output low (CPOL = 0)
  GPIO_PinModeSet(EUS1SPI_PORT, 2, gpioModePushPull, 0);

  // SPI advanced configuration (part of the initializer)
  EUSART_SpiAdvancedInit_TypeDef adv = EUSART_SPI_ADVANCED_INIT_DEFAULT;

  adv.msbFirst = true;        // SPI standard MSB first
  adv.autoCsEnable = false;
  adv.autoInterFrameTime = 7; // 7 bit times of delay between frames
                              // to accommodate non-DMA secondaries

  // Default asynchronous initializer (main/master mode and 8-bit data)
  EUSART_SpiInit_TypeDef init = EUSART_SPI_MASTER_INIT_DEFAULT_HF;

  init.bitRate = 4000000;         // 4 MHz shift clock
  init.advancedSettings = &adv;   // Advanced settings structure

  /*
   * Route EUSART1 MOSI, MISO, and SCLK to the specified pins.  CS is
   * not controlled by EUSART1 so there is no write to the corresponding
   * EUSARTROUTE register to do this.
   */
  GPIO->EUSARTROUTE[1].TXROUTE = (EUS1SPI_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
      | (0 << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[1].RXROUTE = (EUS1SPI_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
      | (1 << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[1].SCLKROUTE = (EUS1SPI_PORT << _GPIO_EUSART_SCLKROUTE_PORT_SHIFT)
      | (2 << _GPIO_EUSART_SCLKROUTE_PIN_SHIFT);
  // Enable EUSART interface pins
  GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN |    // MISO
                                 GPIO_EUSART_ROUTEEN_TXPEN |    // MOSI
                                 GPIO_EUSART_ROUTEEN_SCLKPEN;

  // Configure and enable EUSART1
  EUSART_SpiInit(EUSART1, &init);
}

void initTIMER(void)
{
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  // Do not start counter upon initialization
  timerInit.enable = false;

  TIMER_Init(TIMER0, &timerInit);

  uint32_t tfreq = 0;

  sl_clock_branch_t  clock_branch = sl_device_peripheral_get_clock_branch(SL_PERIPHERAL_TIMER0);
  sl_clock_manager_get_clock_branch_frequency(clock_branch, &tfreq);

  uint32_t top_value = (tfreq * DELAY)/1000;
#ifdef DEBUG_DRIVER
  printf("freq: %d Hz\ntop value: %d\n", tfreq, top_value);
#endif
  /* Set the top value for 1 ms tick to LDMA */
  TIMER_TopSet(TIMER0, top_value);

  // Now start the TIMER
  TIMER_Enable(TIMER0, true);
}

static void initPrs()
{
  PRS_SourceAsyncSignalSet(
      TIMER0_PRS_CHANNEL,
      PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER0,
      _PRS_ASYNC_CH_CTRL_SIGSEL_TIMER0OF);

#ifdef DEBUG_DRIVER
  // Route output to pin
  GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 0);
  PRS_PinOutput(TIMER0_PRS_CHANNEL, prsTypeAsync, gpioPortA , 6);
#endif
}

/***************************************************************************//**
 * @brief
 *   Initialize the LDMA controller for inter-channel synchronization
 ******************************************************************************/

void initLdma(void)
{
  DMADRV_Init();

  // Allocate channels for transmission and reception
  uint32_t stx = DMADRV_AllocateChannel(&tx_channel, NULL);
  uint32_t srx = DMADRV_AllocateChannel(&rx_channel, NULL);

  if (stx || srx)
  {
    printf("Allocation error");
    return;
  }

  // Initialize buffers for memory transfer
  for (uint8_t i = 0; i < BUFFER_SIZE; i++)
  {
    dstBuffer[0][i] = 0;
    dstBuffer[1][i] = 0;
  }

  LDMA->SYNCSWCLR = 0xFF;

  /* SPI RX transfer */

  // Wait for MATCH_VALUE to be set
  descLink0[0] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_SYNC(0, 0, MATCH_VALUE, MATCH_VALUE, 1);

  // Assert CS
  descLink0[1] = (LDMA_Descriptor_t)
      LDMA_DESCRIPTOR_LINKREL_WRITE(1 << (CS_PIN), &(GPIO->P_CLR[gpioPortC].DOUT), 1);

  // Transfer RX PING
  descLink0[2] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_P2M_BYTE ((void*)&(SL_USART_SPI->RXDATA), dstBuffer[0], BUFFER_SIZE, 1);

  // De-assert CS
  descLink0[3] = (LDMA_Descriptor_t)
      LDMA_DESCRIPTOR_LINKREL_WRITE(1 << (CS_PIN), &(GPIO->P_SET[gpioPortC].DOUT), 1);

  // Wait for MATCH_VALUE to be set
  descLink0[4] = (LDMA_Descriptor_t)
      LDMA_DESCRIPTOR_LINKREL_SYNC(0, 0, MATCH_VALUE, MATCH_VALUE, 1);

  // Assert CS
  descLink0[5] = (LDMA_Descriptor_t)
        LDMA_DESCRIPTOR_LINKREL_WRITE(1 << (CS_PIN), &(GPIO->P_CLR[gpioPortC].DOUT), 1);

  // Transfer RX PONG
  descLink0[6] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_P2M_BYTE ((void*)&(SL_USART_SPI->RXDATA), dstBuffer[1], BUFFER_SIZE, 1);

  // De-assert CS
  descLink0[7] = (LDMA_Descriptor_t)
       LDMA_DESCRIPTOR_LINKREL_WRITE(1 << (CS_PIN), &(GPIO->P_SET[gpioPortC].DOUT), -7);


  // Transfer a byte on receive FIFO level event
  ldmaRXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_EUSART1_RXFL);

  // Enable interrupts
  descLink0[2].xfer.doneIfs = true;
  descLink0[6].xfer.doneIfs = true;

  /* SPI TX transfer */

  // Wait for MATCH_VALUE to be set
  descLink1[0] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_SYNC(0, 0, MATCH_VALUE, MATCH_VALUE, 1);

  //Clear SYNCTRIG previously set by PRS channel
  descLink1[1] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_WRITE(1 << (TIMER0_PRS_CHANNEL), &(LDMA->SYNCSWCLR), 1);

  // Transfer TX PING
  descLink1[2] = (LDMA_Descriptor_t)
    LDMA_DESCRIPTOR_LINKREL_M2P_BYTE (srcBuffer, (void*)&(SL_USART_SPI->TXDATA), BUFFER_SIZE, -2);

  ldmaTXConfig = (LDMA_TransferCfg_t)LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_EUSART1_TXFL);

  // First transfer in each set waits for request
  descLink0[0].xfer.structReq = false;
  descLink1[0].xfer.structReq = false;

  /* PRS channel will unblock SYNC desciptor */
  LDMA->SYNCHWEN = (1 << TIMER0_PRS_CHANNEL);

  // Start both channels
  DMADRV_LdmaStartTransfer(rx_channel, (void*)&ldmaRXConfig, (void*)&descLink0, (DMADRV_Callback_t)&ldma_cb, NULL);
  DMADRV_LdmaStartTransfer(tx_channel, (void*)&ldmaTXConfig, (void*)&descLink1, NULL, NULL);
}



void sensor_driver_init(void)
{
  CMU_ClockEnable(cmuClock_LDMA, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_PRS, true);

  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);

  /* Initialisation order is important */
  initEUSART1();
  initPrs();
  initLdma();
  initTIMER();

  printf("Sensor Driver Initialisation done");

  /* At this point, SPI transfers to read the external sensor run in
   * background at 1 ms period with no CPU intervention. The CPU will get an
   * interrupt once SPI data are collected and available in ping-pong buffer
   * */
}

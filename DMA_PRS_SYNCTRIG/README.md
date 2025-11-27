---
sort: 1
---

# Sensor Driver using LDMA SYNC descriptor

{% include list.liquid all=true %}

## Introduction

This project aims to offload EFR32 CPU from any SPI transaction management to read an external SPI sensor. All the SPI transfers are running in loop every 1 ms in background without CPU intervention but involving TIMER0, LDMA, PRS and USART1. A callback is called (from LDMA interrupt handler) once a new sensor value is available. This gives the impression that the sensor is internal to EFR32 while it is on a SPI bus. This sensor driver can be used with or without FreeRTOS. 
This driver ensures that your SPI sensor is read every 1 ms with a very high accuracy so that it is hard real time.
 
LDMA is configured in PING-PONG mode. Thus, it uses 2 differents buffers in reception to store incomming sensor data. This mode lets more time to main CPU to process each sensor data.

In this example, LDMA uses a SYNC descriptor to wait for a event from PRS. This event is trigged by TIMER0 every 1 ms.

The format of the SPI transactions used in this example will need to be modified to fit your use case.
 
## The Sensor driver

### LDMA initialisation

The most difficult part of the example is the DMA descriptor initialisation. Since the sensor communicates over SPI, we use two lists of descriptor. One for RX and another list for TX

#### RX descriptor list

The RX descriptor list does the following:

0. Wait for a stimulus from PRS before continuing
1. Assert the CS signal
2. Wait data from SPI controller and store to dstBuffer[0]
3. De-assert the CS signal
4. Wait for a stimulus from PRS
5. Assert the CS signal
6. Wait data from SPI controller and store to dstBuffer[1]
7. De-assert the CS signal and return to 0

DMA interrupt (callback) is raised at end of operation 2 and 6.

```c
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
```

#### TX descriptor list

The TX descriptor list does the following:

0. Wait for a stimulus from PRS before continuing
1. Clear SYNCTRIG previously set by PRS channel 
2. Write data to the SPI controller and return to 0

```c
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

```

LDMA information can be found in EFR32 reference manual and in section 5 of the following [link](https://www.silabs.com/documents/public/application-notes/AN1029-efm32-ldma.pdf)

### Sensor Driver integration in a BLE project

An integration of the sensor driver to a BLE empty example for BRD4316 is done in the following .sls project:

* [bt_soc_empty](src/bt_soc_empty_zign_dma-brd4316.sls)

The code of the sensor and other useful files of the above project are located here:

* [sensor_driver.c](src/sensor_driver.c)
* [app.c](src/app.c)
* [gatt.btconf](src/gatt_configuration.btconf)


The pinout used on BRD4316 is the following :

|  SPI     |  EFR32  |   BRD4002    |
| -------- | ------- | -------------|
| MOSI     | PC00    | EXP_HEADER4  |
| MISO     | PC01    | EXP_HEADER6  |
| CLK      | PC02    | EXP_HEADER8  |
| CS       | PC03    | EXP_HEADER10 |


## Disclaimer

The Gecko SDK suite supports development with Silicon Labs IoT SoC and module devices. Unless otherwise specified in the specific directory, all examples are considered to be EXPERIMENTAL QUALITY which implies that the code provided in the repos has not been formally tested and is provided as-is. It is not suitable for production environments. In addition, this code will not be maintained and there may be no bug maintenance planned for these resources. Silicon Labs may update projects from time to time.

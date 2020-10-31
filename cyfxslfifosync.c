/*
 * This example shows the both DMA and GPIO mode of SPI communication
 * It also use I2C communication. A UART communication is
 *
 *
 *
 *
 *
 * */
#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyfxslfifosync.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "cyu3utils.h"
#include "pib_regs.h"
#include <cyu3gpio.h>
#include "cyfxgpif_syncsf.h"

CyU3PThread slFifoAppThread;	        /* Slave FIFO application thread structure */
CyU3PDmaChannel glChHandleSlFifoUtoP;   /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel glChHandleSlFifoPtoU;   /* DMA Channel handle for P2U transfer. */
CyU3PDmaChannel glSpiTxHandle, glSpiRxHandle;   // SPI channel handles
uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received from USB. */
uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers sent to USB. */
CyBool_t glIsApplnActive = CyFalse;      /* Whether the loopback application is active or not. */
uint8_t glEp0Buffer[16384] __attribute__ ((aligned (32))); /* Local buffer used for vendor command handling. */
uint16_t glSpiPageSize = 0x100;  /* SPI Page size to be used for transfers. */
uint16_t glI2cPageSize = 0x100;   /* I2C Page size to be used for transfers. */
//CyU3PDmaChannel glI2cTxHandle;   /* I2C Tx channel handle */
//CyU3PDmaChannel glI2cRxHandle;   /* I2C Rx channel handle */
const uint8_t glFirmwareID[32]  __attribute__ ((aligned (32))) = { 'F', 'X', '3', ' ', '1', '0', '1', '\0' };// firmware Revision
const uint32_t maxStartByteAddress = 0x7FFF00;
const uint16_t maxPageAddress = 0x7FF;

/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}

/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void
CyFxSlFifoApplnDebugInit (void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set UART configuration */
    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set the UART transfer to a really large value. */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Initialize the debug module. */
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }
}

/***************************EEPROM programming operation *****************************************************/
/* I2c initialization for EEPROM programming. */
CyU3PReturnStatus_t
CyFxI2cInit (uint16_t pageLen)
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module. */
    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the I2C master block. The bit rate is set at 1000KHz.
     * The data transfer is done via DMA. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate    = CY_FX_USBI2C_I2C_BITRATE;
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyFalse;

    status = CyU3PI2cSetConfig (&i2cConfig, NULL);
    if (status == CY_U3P_SUCCESS)
    {
        glI2cPageSize = pageLen;
    }

    return status;
}


/* I2C read / write for programmer application. */
CyU3PReturnStatus_t
CyFxUsbI2cTransfer (
        uint16_t  pageAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    CyU3PI2cPreamble_t preamble;
    uint16_t pageCount = (byteCount / glI2cPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint16_t resCount = glI2cPageSize;
    uint8_t devAddr = 0xA0 | (uint8_t)(((pageAddress >> 8) & 0x07) << 1);
    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % glI2cPageSize) != 0)
    {
        pageCount ++;
        resCount = byteCount % glI2cPageSize;
    }

    CyU3PDebugPrint (2, "I2C access - dev: 0x%x, address: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            devAddr, pageAddress, byteCount, pageCount);

    while (pageCount != 0)
    {
        if (isRead)
        {
            /* Update the preamble information. */
            preamble.length    = 4;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(pageAddress & 0xFF);
            preamble.buffer[2] = 0x00;
            preamble.buffer[3] = (devAddr | 0x01);
            preamble.ctrlMask  = 0x0004;

            status = CyU3PI2cReceiveBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }
        else /* Write */
        {
            /* Update the preamble information. */
            preamble.length    = 3;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(pageAddress & 0xFF);
            preamble.buffer[2] = 0x00;
            preamble.ctrlMask  = 0x0000;

            status = CyU3PI2cTransmitBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }


            /* Wait for the write to complete. */
            preamble.length = 1;
            status = CyU3PI2cWaitForAck(&preamble, 200);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }

        /* An additional delay seems to be required after receiving an ACK. */
        CyU3PThreadSleep (1);

        /* Update the parameters */
        pageAddress ++;
        buffer += glI2cPageSize;
        pageCount --;
    }

    return CY_U3P_SUCCESS;
}

/***************************FPGA configuration flash operation functions*************************************/
CyU3PReturnStatus_t
CyFPGASpiSetClockValue (
        CyBool_t isHigh        /* Cyfalse: Pull down the Clock line,
                                  CyTrue: Pull up the Clock line */
        )
{
    CyU3PReturnStatus_t status;
    status = CyU3PGpioSetValue (FX3_SPI_CLK, isHigh);
    return status;
}

/* This function pulls up/down the slave select line. */
CyU3PReturnStatus_t
CyFPGASpiSetSsnLine (
        CyBool_t isHigh        /* Cyfalse: Pull down the SSN line,
                                  CyTrue: Pull up the SSN line */
        )
{
#ifndef FX3_USE_GPIO_REGS
    CyU3PReturnStatus_t status;

    status = CyU3PGpioSetValue (FX3_SPI_SS, isHigh);

    return status;
#else
    uvint32_t *regPtrSS;
    regPtrSS = &GPIO->lpp_gpio_simple[FX3_SPI_SS];
    if(isHigh)
    {
        *regPtrSS |=CYFX_GPIO_HIGH;
    }
    else
    {
        *regPtrSS&=~CYFX_GPIO_HIGH;
    }

    return CY_U3P_SUCCESS;
#endif
}

/* This function transmits the byte to the SPI slave device one bit a time.
   Most Significant Bit is transmitted first.
 */
CyU3PReturnStatus_t
CyFPGASpiWriteByte (
        uint8_t data)
{
    uint8_t i = 0;
#ifdef FX3_USE_GPIO_REGS
    CyBool_t value;
    uvint32_t *regPtrMOSI, *regPtrClock;
	regPtrMOSI = &GPIO->lpp_gpio_simple[FX3_SPI_MOSI];
	regPtrClock = &GPIO->lpp_gpio_simple[FX3_SPI_CLK];
#endif
    for (i = 0; i < 8; i++)
    {
#ifndef FX3_USE_GPIO_REGS
        /* Most significant bit is transferred first. */
        CyU3PGpioSetValue (FX3_SPI_MOSI, ((data >> (7 - i)) & 0x01));

        CyFPGASpiSetClockValue (CyTrue);
        CyU3PBusyWait (1);
        CyFPGASpiSetClockValue (CyFalse);
        CyU3PBusyWait (1);
#else
        /* Most significant bit is transferred first. */
		value =((data >> (7 - i)) & 0x01);
		if(value)
		{
			*regPtrMOSI |=	CYFX_GPIO_HIGH;
		}
		else
		{
			*regPtrMOSI &=~CYFX_GPIO_HIGH;
		}
		*regPtrClock|=CYFX_GPIO_HIGH;
        CyU3PBusyWait (1);
        *regPtrClock&=~CYFX_GPIO_HIGH;
        CyU3PBusyWait (1);
#endif
    }

    return CY_U3P_SUCCESS;
}
/* This function transmits the byte to the SPI slave device one bit a time.
   LEAST Significant Bit is transmitted first. .rpd file need to be shifted so that LSB is loaded first
 */
CyU3PReturnStatus_t
CyFPGASpiWriteByteLSB (
        uint8_t data)
{
    uint8_t i = 0;
#ifdef FX3_USE_GPIO_REGS
    CyBool_t value;
    uvint32_t *regPtrMOSI, *regPtrClock;
	regPtrMOSI = &GPIO->lpp_gpio_simple[FX3_SPI_MOSI];
	regPtrClock = &GPIO->lpp_gpio_simple[FX3_SPI_CLK];
#endif
    for (i = 0; i < 8; i++)
    {
#ifndef FX3_USE_GPIO_REGS
        /* Most significant bit is transferred first. */
        CyU3PGpioSetValue (FX3_SPI_MOSI, ((data >> i) & 0x01));

        CyFPGASpiSetClockValue (CyTrue);
        CyU3PBusyWait (1);
        CyFPGASpiSetClockValue (CyFalse);
        CyU3PBusyWait (1);
#else
        /* Most significant bit is transferred first. */
		value =((data >> i) & 0x01);
		if(value)
		{
			*regPtrMOSI |=	CYFX_GPIO_HIGH;
		}
		else
		{
			*regPtrMOSI &=~CYFX_GPIO_HIGH;
		}
		*regPtrClock|=CYFX_GPIO_HIGH;
        CyU3PBusyWait (1);
        *regPtrClock&=~CYFX_GPIO_HIGH;
        CyU3PBusyWait (1);
#endif
    }

    return CY_U3P_SUCCESS;
}


/* This function receives the byte from the SPI slave device one bit at a time.
   Most Significant Bit is received first.
 */
CyU3PReturnStatus_t
CyFPGASpiReadByte (
        uint8_t *data)
{
    uint8_t i = 0;
    CyBool_t temp = CyFalse;

#ifdef FX3_USE_GPIO_REGS
    uvint32_t *regPtrClock;
	regPtrClock = &GPIO->lpp_gpio_simple[FX3_SPI_CLK];
#endif
    *data = 0;

    for (i = 0; i < 8; i++)
    {
#ifndef FX3_USE_GPIO_REGS
        CyFPGASpiSetClockValue (CyTrue);

        CyU3PGpioGetValue (FX3_SPI_MISO, &temp);
        *data |= (temp << (7 - i));

        CyU3PBusyWait (1);

        CyFPGASpiSetClockValue (CyFalse);
        CyU3PBusyWait (1);
#else
        *regPtrClock|=CYFX_GPIO_HIGH;
		temp = (GPIO->lpp_gpio_simple[FX3_SPI_MISO] & CY_U3P_LPP_GPIO_IN_VALUE)>>1;
        *data |= (temp << (7 - i));
        CyU3PBusyWait (1);
        *regPtrClock&=~CYFX_GPIO_HIGH;
        CyU3PBusyWait (1);
#endif
    }

    return CY_U3P_SUCCESS;
}

/* This function receives the byte from the SPI slave device one bit at a time.
   LEAST Significant Bit is received first. .rpd file need to be shifted so that LSB is loaded first
 */
CyU3PReturnStatus_t
CyFPGASpiReadByteLSB (
        uint8_t *data)
{
    uint8_t i = 0;
    CyBool_t temp = CyFalse;

#ifdef FX3_USE_GPIO_REGS
    uvint32_t *regPtrClock;
	regPtrClock = &GPIO->lpp_gpio_simple[FX3_SPI_CLK];
#endif
    *data = 0;

    for (i = 0; i < 8; i++)
    {
#ifndef FX3_USE_GPIO_REGS
        CyFPGASpiSetClockValue (CyTrue);

        CyU3PGpioGetValue (FX3_SPI_MISO, &temp);
        *data |= (temp << i);

        CyU3PBusyWait (1);

        CyFPGASpiSetClockValue (CyFalse);
        CyU3PBusyWait (1);
#else
        *regPtrClock|=CYFX_GPIO_HIGH;
		temp = (GPIO->lpp_gpio_simple[FX3_SPI_MISO] & CY_U3P_LPP_GPIO_IN_VALUE)>>1;
        *data |= (temp << i);
        CyU3PBusyWait (1);
        *regPtrClock&=~CYFX_GPIO_HIGH;
        CyU3PBusyWait (1);
#endif
    }

    return CY_U3P_SUCCESS;
}

/* This function is used to transmit data to the SPI slave device. The function internally
   calls the CyFPGASpiWriteByte function to write to the slave device.
 */
CyU3PReturnStatus_t
CyFPGASpiTransmitWords (
        uint8_t *data,
        uint32_t byteCount,
        CyBool_t LSB )
{
    uint32_t i = 0;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if ((!byteCount) || (!data))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    for (i = 0; i < byteCount; i++)
    {
        status = LSB? CyFPGASpiWriteByteLSB (data[i]) : CyFPGASpiWriteByte (data[i]);

        if (status != CY_U3P_SUCCESS)
        {
            break;
        }
    }

    return status;
}

/* This function is used receive data from the SPI slave device. The function internally
   calls the CyFPGASpiReadByte function to read data from the slave device.
 */
CyU3PReturnStatus_t
CyFPGASpiReceiveWords (
        uint8_t *data,
        uint32_t byteCount,
        CyBool_t LSB)
{
    uint32_t i = 0;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if ((!byteCount) || (!data))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    for (i = 0; i < byteCount; i++)
    {
        status = LSB? CyFPGASpiReadByteLSB (&data[i]): CyFPGASpiReadByte (&data[i]);

        if (status != CY_U3P_SUCCESS)
        {
            break;
        }
    }

    return status;
}

/* Wait for the status response from the SPI flash. */
CyU3PReturnStatus_t
CyFPGASpiWaitForStatus (
        void)
{
    uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Wait for status response from SPI flash device. */
    do
    {
        buf[0] = 0x06;  // Write enable command.

        CyFPGASpiSetSsnLine (CyFalse);
        status = CyFPGASpiTransmitWords (buf, 1,CyFalse);
        CyFPGASpiSetSsnLine (CyTrue);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI WR_ENABLE command failed\n\r");
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

        CyFPGASpiSetSsnLine (CyFalse);
        status = CyFPGASpiTransmitWords (buf, 1,CyFalse);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI READ_STATUS command failed\n\r");
            CyFPGASpiSetSsnLine (CyTrue);
            return status;
        }

        status = CyFPGASpiReceiveWords (rd_buf, 2, CyFalse);
        CyFPGASpiSetSsnLine (CyTrue);
        if(status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI status read failed\n\r");
            return status;
        }

    } while ((rd_buf[0] & 1)|| (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}

/* SPI read / write for programmer application. */
CyU3PReturnStatus_t
CyFPGASpiTransfer (
        uint16_t  pageAddress, // page starting address
        uint16_t  byteCount, // data length to be read/write
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / glSpiPageSize); // get the page size if data length >= 256
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }
    if ((byteCount % glSpiPageSize) != 0)
    {
        pageCount ++;                         // get the page size if data length < 256
    }

    byteAddress  = pageAddress * glSpiPageSize; // get the byte starting address
    CyU3PDebugPrint (2, "FPGA flash access - addr: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            byteAddress, byteCount, pageCount);

    while (pageCount != 0)
    {
        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead)
        {
            location[0] = 0x03; /* Read command. */

           status = CyFPGASpiWaitForStatus ();
           if (status != CY_U3P_SUCCESS)
               return status;

            CyFPGASpiSetSsnLine (CyFalse);
            status = CyFPGASpiTransmitWords (location, 4,CyFalse);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "FPGA READ command failed\r\n");
                CyFPGASpiSetSsnLine (CyTrue);
                return status;
            }

            status = CyFPGASpiReceiveWords (buffer, glSpiPageSize, CyTrue);
            if (status != CY_U3P_SUCCESS)
            {
                CyFPGASpiSetSsnLine (CyTrue);
                return status;
            }

            CyFPGASpiSetSsnLine (CyTrue);
        }
        else /* Write */
        {
            location[0] = 0x02; /* Write command */

            status = CyFPGASpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
                return status;

            CyFPGASpiSetSsnLine (CyFalse);
            status = CyFPGASpiTransmitWords (location, 4, CyFalse);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "FPGA WRITE command failed\r\n");
                CyFPGASpiSetSsnLine (CyTrue);
                return status;
            }

            status = CyFPGASpiTransmitWords (buffer, glSpiPageSize, CyTrue);
            if (status != CY_U3P_SUCCESS)
            {
                CyFPGASpiSetSsnLine (CyTrue);
                return status;
            }
            CyFPGASpiSetSsnLine (CyTrue);

            // turn off FPGA progam signal if reaching the end of the flash memory
            if (byteAddress == maxStartByteAddress)
            {
            	status = CyFPGASpiWaitForStatus ();
            	CyU3PGpioSetValue (FPGA_PROGRAM, CyFalse); // end programming
            }
        }

        /* Update the parameters */
        byteAddress  += glSpiPageSize;
        buffer += glSpiPageSize;
        pageCount --;
        CyU3PThreadSleep (10);
    }
    return CY_U3P_SUCCESS;
}

/* Function to erase SPI flash sectors. */
//static CyU3PReturnStatus_t
CyU3PReturnStatus_t
CyFPGASpiEraseSector (
     uint8_t   sector
     )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint8_t  location[4];
    location[0] = 0xD8; /* Sector erase. */
    location[1] = sector;
    location[2] = 0x00;
    location[3] = 0x00;
    status = CyFPGASpiWaitForStatus ();
    if (status != CY_U3P_SUCCESS)
        return status;
    CyFPGASpiSetSsnLine (CyFalse);
    status = CyFPGASpiTransmitWords (location, 4, CyFalse);
    CyFPGASpiSetSsnLine (CyTrue);
    return status;
}

/**************************registers read & write operation functions**********************************/

CyU3PReturnStatus_t CySpiInit (uint16_t pageLen){

    CyU3PReturnStatus_t Status = CY_U3P_SUCCESS;
	Status = CyU3PSpiInit();
	//CheckStatus("initialize SPI module", Status);

    /* Start the SPI master block. Run the SPI clock at 33MHz
     * and configure the word length to 32 bits. Also configure
     * the slave select using FW. */
    CyU3PSpiConfig_t spiConfig;
    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyTrue;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyTrue;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;  /**< SSN is controlled by API and is not at clock boundaries. */
    spiConfig.clock      = 25000000;
    spiConfig.wordLen    = 8;
    Status = CyU3PSpiSetConfig (&spiConfig, NULL);
    //CheckStatus ("configure SPI module", Status);

    CyU3PDmaChannelConfig_t dmaConfig;
    CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig)); // No buffers need to be allocated as this channel will be used only in override mode.
    dmaConfig.size           = pageLen;
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;
    dmaConfig.notification   = 0;
    dmaConfig.cb             = NULL;

    /* Channel to write to SPI flash. */
    dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig.consSckId = CY_U3P_LPP_SOCKET_SPI_CONS;
	Status = CyU3PDmaChannelCreate (&glSpiTxHandle, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);

    /* Channel to read from SPI flash. */
    dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_SPI_PROD;
    dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
    Status = CyU3PDmaChannelCreate (&glSpiRxHandle, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

	return Status;
}

CyU3PReturnStatus_t CyFxSpiWaitForStatus (void)
{
	uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
   // do
   // {
        buf[0] = 0x06;  /* Write enable command. */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        CyU3PSpiSetSsnLine (CyTrue);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI WR_ENABLE command failed\n\r");
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI READ_STATUS command failed\n\r");
            CyU3PSpiSetSsnLine (CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords (rd_buf, 2);
        CyU3PSpiSetSsnLine (CyTrue);
        if(status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI status read failed\n\r");
            return status;
        }

   // } while ((rd_buf[0] & 1)|| (!(rd_buf[0] & 0x2)));
   return status;
}

CyU3PReturnStatus_t CyFxSpiTransfer (
        uint16_t  registerAddress,
        uint8_t  *buffer,
        CyBool_t  isRead,
        uint8_t   cmd)
{
    CyU3PDmaBuffer_t buf_p;
    buf_p.buffer = buffer;
    buf_p.status = 0;
    buf_p.count = 0x10;
    buf_p.size  = 0x10;
    uint8_t location[4];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    //location[1] = (registerAddress >> 16) & 0xFF;      /* MS byte */
    location[1] = (registerAddress >> 8) & 0xFF;
    location[2] = (registerAddress) & 0xFF;            /* LS byte */
    location[3] = 0x00;
    if (isRead)
        {
            // initiate transaction
    	    status = CyFxSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
                return status;

            // start to transmit words
            location[0] = cmd; /* Read command */
            CyU3PSpiSetSsnLine (CyFalse);
            status = CyU3PSpiTransmitWords (location, 4);
            if (status != CY_U3P_SUCCESS)
            {
                //DebugPrint (2, "SPI READ command failed\r\n");
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            CyU3PSpiSetBlockXfer (0, 0x10);
            status = CyU3PDmaChannelSetupRecvBuffer (&glSpiRxHandle, &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion (&glSpiRxHandle,2000);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            CyU3PSpiSetSsnLine (CyTrue);
            CyU3PSpiDisableBlockXfer (CyFalse, CyTrue);
        }
    else /* Write */
        {
    	    // initiate transaction
            status = CyFxSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
                return status;

            location[0] = cmd; /* Write command */
            CyU3PSpiSetSsnLine (CyFalse);
            status = CyU3PSpiTransmitWords (location, 4);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "SPI WRITE command failed\r\n");
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            CyU3PSpiSetBlockXfer (0x10, 0);
            status = CyU3PDmaChannelSetupSendBuffer (&glSpiTxHandle, &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glSpiTxHandle, 2000);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetSsnLine (CyTrue);
            CyU3PSpiDisableBlockXfer (CyTrue, CyFalse);

        CyU3PThreadSleep (10);
    }
    return CY_U3P_SUCCESS;
}

/***************************Bulk IN & out app definition**********************************************/
/* DMA callback function to handle the produce events for U to P transfers. */
void
CyFxSlFifoUtoPDmaCallback (
        CyU3PDmaChannel   *chHandle,
        CyU3PDmaCbType_t  type,
        CyU3PDmaCBInput_t *input
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* This is a produce event notification to the CPU. This notification is 
         * received upon reception of every buffer. The buffer will not be sent
         * out unless it is explicitly committed. The call shall fail if there
         * is a bus reset / usb disconnect or if there is any application error. */
     	CyU3PDmaBuffer_t dmaBuffer;
     	CyU3PDmaChannelGetBuffer(chHandle, &dmaBuffer, CYU3P_NO_WAIT);
// modifying entire buffer will time out the call back function
     	/*     	for (uint8_t i =0; i< dmaBuffer.count; i++ )
     	{
     		switch (i%0x04)
     		{
     		case 0x00: dmaBuffer.buffer[i] = 0x78; break;
     		case 0x01: dmaBuffer.buffer[i] = 0x56; break;
     		case 0x02: dmaBuffer.buffer[i] = 0x34; break;
     		case 0x03: dmaBuffer.buffer[i] = 0x12; break;
     		}
     	}

*/
     	CyU3PDmaChannelSetupSendBuffer (chHandle, &dmaBuffer);
     	status = CyU3PDmaChannelCommitBuffer (chHandle, dmaBuffer.count, 0);
    	if (status != CY_U3P_SUCCESS)
    	   {
    	    CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
    	   }
    }
}

/* DMA callback function to handle the produce events for P to U transfers. */
void
CyFxSlFifoPtoUDmaCallback (
        CyU3PDmaChannel   *chHandle,
        CyU3PDmaCbType_t  type,
        CyU3PDmaCBInput_t *input
        )
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* This is a produce event notification to the CPU. This notification is 
         * received upon reception of every buffer. The buffer will not be sent
         * out unless it is explicitly committed. The call shall fail if there
         * is a bus reset / usb disconnect or if there is any application error. */

    	status = CyU3PDmaChannelCommitBuffer (chHandle, input->buffer_p.count, 0);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
        }

        /* Increment the counter. */
      //  CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer success, buffer receieved = %d\n", glDMATxCount++);
    }
}

/* This function starts the slave FIFO loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
CyFxSlFifoApplnStart (
        void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            break;

        case  CY_U3P_SUPER_SPEED:
            size = 1024;
            break;

        default:
            CyU3PDebugPrint (4, "Error! Invalid USB speed.\n");
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Create a DMA MANUAL channel for U2P transfer.
     * DMA size is set based on the USB speed. */
    dmaCfg.size  = 16384;
    dmaCfg.count = 4;
    dmaCfg.prodSckId = CY_FX_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId = CY_FX_CONSUMER_PPORT_SOCKET;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    /* Enabling the callback for produce event. */
    dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    //dmaCfg.cb = CyFxSlFifoUtoPDmaCallback;
    dmaCfg.cb = NULL; // automode
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoUtoP,
    		CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Create a DMA MANUAL channel for P2U transfer. */
    dmaCfg.prodSckId = CY_FX_PRODUCER_PPORT_SOCKET;
    dmaCfg.consSckId = CY_FX_CONSUMER_USB_SOCKET;
    dmaCfg.cb = CyFxSlFifoPtoUDmaCallback;
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoPtoU,
    		CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Set DMA channel transfer size. */
    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
    // initialize the SPI transfer (DMA mode) for register read & write
    apiRetStatus = CySpiInit (0x100);
     if (apiRetStatus)
     	CyU3PDebugPrint (4, "SPI initialized");

     // initialize the IIC transfer (DMA mode) for register read & write
    apiRetStatus = CyFxI2cInit (0x100);
      if (apiRetStatus)
      	CyU3PDebugPrint (4, "IIC initialized");

    // initialize other functions



    /* Update the status flag. */
    glIsApplnActive = CyTrue;
//	CyU3PGpioSetValue (59, CyFalse);
	CyU3PGpioSetValue (FPGA_PROGRAM, CyFalse);
}

/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void
CyFxSlFifoApplnStop (
        void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag. */
    glIsApplnActive = CyFalse;

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Destroy the channel */
    CyU3PDmaChannelDestroy (&glChHandleSlFifoUtoP);
    CyU3PDmaChannelDestroy (&glChHandleSlFifoPtoU);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t
CyFxSlFifoApplnUSBSetupCB (
        uint32_t setupdat0,
        uint32_t setupdat1
    )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */

    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyFalse;
    CyU3PReturnStatus_t status;
    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength   = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);

    // Leave the standard request alone
    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
                CyU3PUsbAckSetup ();
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a loopback operation, there is no higher
         * level protocol. So flush the EP memory and reset the DMA channel associated
         * with it. If there are more than one EP associated with the channel reset both
         * the EPs. The endpoint stall and toggle / sequence number is also expected to be
         * reset. Return CyFalse to make the library clear the stall and reset the endpoint
         * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
         * Here we are clearing the stall. */
        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (glIsApplnActive)
            {
                if (wIndex == CY_FX_EP_PRODUCER)
                {
                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyTrue);
                    CyU3PBusyWait (125);

                    CyU3PDmaChannelReset (&glChHandleSlFifoUtoP);
                    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
                    CyU3PUsbResetEp (CY_FX_EP_PRODUCER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);

                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyFalse);
                }

                if (wIndex == CY_FX_EP_CONSUMER)
                {
                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyTrue);
                    CyU3PBusyWait (125);

                    CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);

                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyFalse);
                }

                CyU3PUsbStall (wIndex, CyFalse, CyTrue);

                CyU3PUsbAckSetup ();
                isHandled = CyTrue;
            }
        }
    }
    // customized for me
    CyBool_t gpioValue = CyFalse;
    CyU3PGpioGetValue (FPGA_PROGRAM, &gpioValue); // read FPGA program signal
    if (bType == CY_U3P_USB_VENDOR_RQT)
        {
            isHandled = CyTrue;
         switch (bRequest)
         {
            case 0xC5:
            	 CyU3PGpioSetValue (FPGA_PROGRAM, CyFalse);
            	 status = CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
            	break;
            case 0xC0:
            	if (!gpioValue)
            	{
            		CyU3PGpioSetValue (FPGA_PROGRAM, CyTrue); // start programming
            	}
            	// program the FPGA configuration flash memory
            	// if (wIndex < (maxPageAddress + 0x01))
            	{
                   status = CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
                   if (status == CY_U3P_SUCCESS)
                   {
                       status = CyFPGASpiTransfer (wIndex, wLength,
                                glEp0Buffer, CyFalse);
                   }
            	}
                break;
            case 0xC1:
            	// read the FPGA configuration flash memory
            	//if (wIndex < (maxPageAddress + 0x01))
            	{
                    CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                    status = CyFPGASpiTransfer (wIndex, wLength,
                             glEp0Buffer, CyTrue);
                    if (status == CY_U3P_SUCCESS)
                    {
                        status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                    }
            	}
            	break;
             case 0xC4:
            	// erase the FPGA configuration flash memory
            	 if (!gpioValue)
            	 {
            		 CyU3PGpioSetValue (FPGA_PROGRAM, CyTrue); // start programming
            	 }
            	// if ((wIndex & 0xFF) < 0x20)
            	 {
                   status = CyFPGASpiEraseSector (wIndex & 0xFF);
                   if (status == CY_U3P_SUCCESS)
                   {
                    CyFPGASpiWaitForStatus ();
                    CyU3PUsbAckSetup ();
                   }
            	 }
            	break;
              case 0xC3:
            	  CyU3PMemSet (glEp0Buffer, 0, 4);
                  status = CyFxSpiTransfer (wIndex, glEp0Buffer, CyTrue,0x03);
                  if (status == CY_U3P_SUCCESS)
                  {
                      status = CyU3PUsbSendEP0Data (4, glEp0Buffer);
                       if (status)
                    	   CyU3PDebugPrint (4,"SPI read %d success", wIndex);
                  }
            	  break;
              case 0xC2:
            	  status = CyU3PUsbGetEP0Data (0x0004, glEp0Buffer, NULL);
            	//  CheckStatus ("SPI write ready",status);
                  if (status == CY_U3P_SUCCESS)
    		      {
    		    	  status = CyFxSpiTransfer (wIndex, glEp0Buffer, CyFalse,0x02);
    		    	  CyU3PDebugPrint (4,"SPI write %d success",wIndex);
    		      }
            	  break;
              case 0xC8:
            	  status = CyU3PUsbGetEP0Data (0x0004, glEp0Buffer, NULL);
            	 // CheckStatus ("SPI write ready",status);
                  if (status == CY_U3P_SUCCESS)
    		      {
                	  status = CyFxSpiTransfer (wIndex, glEp0Buffer, CyFalse,0x08);
    		      }
            	  break;
              case 0xC9:
            	  status = CyU3PUsbGetEP0Data (0x0004, glEp0Buffer, NULL);
            	//  CheckStatus ("SPI write ready",status);
                  if (status == CY_U3P_SUCCESS)
    		      {
                	  status = CyFxSpiTransfer (wIndex, glEp0Buffer, CyFalse,0x09);
    		      }
            	  break;

              case CY_FX_RQT_ID_CHECK: // firmware revision check
                  CyU3PUsbSendEP0Data (8, (uint8_t *)glFirmwareID);
                  break;

              case CY_FX_RQT_I2C_EEPROM_WRITE:
                 //i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
                  status  = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
                  if (status == CY_U3P_SUCCESS)
                  {
                      CyFxUsbI2cTransfer (wIndex, wLength,glEp0Buffer, CyFalse);
                  }
                  break;

              case CY_FX_RQT_I2C_EEPROM_READ:
                  //i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
                  CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                  status = CyFxUsbI2cTransfer (wIndex, wLength, glEp0Buffer, CyTrue);
                  if (status == CY_U3P_SUCCESS)
                  {
                      status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
                  }
                  break;
              default:
            	  isHandled = CyFalse; // unknown request
                  break;
            }
        }

    return isHandled;
}

/* This is the callback function to handle the USB events. */
void
CyFxSlFifoApplnUSBEventCB (
    CyU3PUsbEventType_t evtype,
    uint16_t            evdata
    )
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:
            /* Disable the low power entry to optimize USB throughput */
            CyU3PUsbLPMDisable();
            /* Stop the application before re-starting. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();
            }
            CyFxSlFifoApplnStart ();
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            /* Stop the loop back function. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();
            }
            break;

        default:
            break;
    }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
CyFxApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

/* This function initializes the GPIF interface and initializes
 * the USB interface. */
void
CyFxSlFifoApplnInit (void)
{
    CyU3PPibClock_t pibClock;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;

    /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "P-port Initialization failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Load the GPIF configuration for Slave FIFO sync mode. */
    apiRetStatus = CyU3PGpifLoad (&CyFxGpifConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PGpifLoad failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 1)	
    CyU3PGpifSocketConfigure (0,CY_FX_PRODUCER_PPORT_SOCKET,6,CyFalse,1);
    CyU3PGpifSocketConfigure (3,CY_FX_CONSUMER_PPORT_SOCKET,6,CyFalse,1);
#else
	CyU3PGpifSocketConfigure (0,CY_FX_PRODUCER_PPORT_SOCKET,3,CyFalse,1);
    CyU3PGpifSocketConfigure (3,CY_FX_CONSUMER_PPORT_SOCKET,3,CyFalse,1);
#endif
	
    /* Start the state machine. */
    apiRetStatus = CyU3PGpifSMStart (RESET, ALPHA_RESET);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PGpifSMStart failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
	
	/* Init the GPIO module */
	gpioClock.fastClkDiv = 2;
	gpioClock.slowClkDiv = 0;
	gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
	gpioClock.clkSrc = CY_U3P_SYS_CLK;
	gpioClock.halfDiv = 0;

	apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
	if (apiRetStatus != 0)
	{
		/* Error Handling */
		CyU3PDebugPrint (4, "CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}

    // Configure GPIO 38 as output(SPI_CLOCK).
    gpioConfig.outValue    = CyFalse;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FX3_SPI_CLK, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        // Error handling
        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig for GPIO Id %d failed, error code = %d\n",
                FX3_SPI_CLK, apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    // Configure GPIO 35 as output(SPI_SSN)
    gpioConfig.outValue    = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FX3_SPI_SS, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        // Error handling
        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig for GPIO Id %d failed, error code = %d\n",
                FX3_SPI_SS, apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    // Configure GPIO 33 as input(MISO)
    gpioConfig.outValue    = CyFalse;
    gpioConfig.inputEn     = CyTrue;
    gpioConfig.driveLowEn  = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FX3_SPI_MISO, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        // Error handling
        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig for GPIO Id %d failed, error code = %d\n",
                FX3_SPI_MISO, apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    // Configure GPIO 36 as output(MOSI)
    gpioConfig.outValue    = CyFalse;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FX3_SPI_MOSI, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        // Error handling
        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig for GPIO Id %d failed, error code = %d\n",
                FX3_SPI_MOSI, apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    // Configure GPIO 39 as FPGA program start
    gpioConfig.outValue    = CyFalse;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FPGA_PROGRAM, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        // Error handling
        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig for GPIO Id %d failed, error code = %d\n",
        		FPGA_PROGRAM, apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PUsbStart failed to Start, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxSlFifoApplnUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxSlFifoApplnUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);    

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device qualifier descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Connect failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
}

/* Entry function for the slFifoAppThread. */
void
SlFifoAppThread_Entry (
        uint32_t input)
{
    /* Initialize the debug module */
    CyFxSlFifoApplnDebugInit();

    /* Initialize the slave FIFO application */
    CyFxSlFifoApplnInit();

    for (;;)
    {
        CyU3PThreadSleep (1000);
        if (glIsApplnActive)
        {
            /* Print the number of buffers received so far from the USB host. */
          //  CyU3PDebugPrint (6, "Data tracker: buffers received: %d, buffers sent: %d.\n",
          //          glDMARxCount, glDMATxCount);
        }
    }
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;

    /* Allocate the memory for the thread */
    ptr = CyU3PMemAlloc (CY_FX_SLFIFO_THREAD_STACK);

    /* Create the thread for the application */
    retThrdCreate = CyU3PThreadCreate (&slFifoAppThread,           /* Slave FIFO app thread structure */
                          "21:Slave_FIFO_sync",                    /* Thread ID and thread name */
                          SlFifoAppThread_Entry,                   /* Slave FIFO app thread entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          CY_FX_SLFIFO_THREAD_STACK,               /* App Thread stack size */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread priority */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread pre-emption threshold */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the thread immediately */
                          );

    /* Check the return code */
    if (retThrdCreate != 0)
    {
        /* Thread Creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
        while(1);
    }
}

/*
 * Main function
 */
int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clockConfig;

    /* When the GPIF data bus is configured as 32-bits wide and running at 100 MHz (synchronous),
       the FX3 system clock has to be set to a frequency greater than 400 MHz. */
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    clockConfig.setSysClk400  = CyFalse;
#else
    clockConfig.setSysClk400  = CyTrue;
#endif
    clockConfig.cpuClkDiv     = 2;
    clockConfig.dmaClkDiv     = 2;
    clockConfig.mmioClkDiv    = 2;
    clockConfig.useStandbyClk = CyFalse;
    clockConfig.clkSrc        = CY_U3P_SYS_CLK;
    status = CyU3PDeviceInit (&clockConfig);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable both Instruction and Data Caches. */
    status = CyU3PDeviceCacheControl (CyTrue, CyTrue, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration for 16 bit slave FIFO configuration and setting
     * isDQ32Bit for 32-bit slave FIFO configuration. */
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyTrue;
    io_cfg.useSpi    = CyTrue;
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
#else
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
#endif
    /* No GPIOs are enabled. */
    // NEED TO INITILIZE SOME GPIOS FOR SPI TALK WITH FPGA CONFIGURATION FLASH
    io_cfg.gpioSimpleEn[0]  = 0x00000000;
    io_cfg.gpioSimpleEn[1]  = (0x00000001 << 1)| (0x00000001 << 3)| (0x00000001 << 4)| (0x00000001 << 6)| (0x00000001 << 7);
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */


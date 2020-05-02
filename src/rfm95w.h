#ifndef _VVC_RFM95W
#define _VVC_RFM95W

/*
 * LoRa register address and bitfield definitions for
 * RFM95W radio modules, silicon revision 0x12. Datasheet:
 * https://www.hoperf.com/data/upload/portal/20190801/RFM95W-V2.0.pdf
 *
 * This header file only describes a minimal subset of the available
 * registers and bitfields, and it does not include any definitions
 * for FSK/OOK modulation. It is meant for trivial LoRa applications.
 */

/**********************/
/* Register addresses */
/**********************/
// FIFO access.
#define RF_FIFO        ( 0x00 )
// Operation mode.
#define RF_OPMODE      ( 0x01 )
// Carrier frequency bytes. With the default 32MHz crystal, each bit
// is worth 61.035Hz. The default setting is for 434MHz.
// If you bought an assembled module, it probably has RF filtering
// passives which tune it to a narrow frequency range, i.e. 915MHz.
#define RF_FREQ_MSB    ( 0x06 )
#define RF_FREQ_MID    ( 0x07 )
#define RF_FREQ_LSB    ( 0x08 )
// FIFO address pointer.
#define RF_FIFO_ADDR   ( 0x0D )
// Base address for the 'transmit' and 'receive' FIFOs.
// The FIFO holds 256 bytes, which you can split between TX and RX
// however you want. But there is no overrun protection, so if you
// use both in your application, they may cross-contaminate.
// See section 4.1.2.3 of the datasheet for more information.
#define RF_FIFO_TXBASE ( 0x0E )
#define RF_FIFO_RXBASE ( 0x0F )
// Starting address in the FIFO of the last packet received.
#define RF_FIFO_RXADDR ( 0x10 )
// Current interrupt request flags status.
#define RF_IRQ_FLAGS   ( 0x12 )
// Number of bytes in the last packet received.
#define RF_RX_LEN      ( 0x13 )
// Current LoRa modem status.
#define RF_LORA_STAT   ( 0x18 )
// Payload length in bytes, for transmitting.
#define RF_TX_LEN      ( 0x22 )
// Hardware revision ID.
#define RF_ID          ( 0x42 )

/**********************/
/* Register bitfields */
/**********************/

/* Operation mode register */
// Modulation mode.
#define RF_OP_MOD      ( 0x80 )
#define RF_OP_MOD_OOK  ( 0x00 )
#define RF_OP_MOD_LORA ( 0x80 )
// Low / High frequency select.
#define RF_OP_FREQ     ( 0x08 )
#define RF_OP_FREQ_HF  ( 0x00 )
#define RF_OP_FREQ_LF  ( 0x08 )
// Operation modes.
#define RF_OP_MD       ( 0x07 )
// 'sleep'
#define RF_OP_MD_SLEEP ( 0x00 )
// 'standby'
#define RF_OP_MD_STDBY ( 0x01 )
// 'transmit frequency synthesis'
#define RF_OP_MD_FSTX  ( 0x02 )
// 'transmit'
#define RF_OP_MD_TX    ( 0x03 )
// 'receive frequency synthesis'
#define RF_OP_MD_FSRX  ( 0x04 )
// 'continuous receive'
#define RF_OP_MD_RXC   ( 0x05 )
// 'single receive'
#define RF_OP_MD_RXS   ( 0x06 )
// 'channel activity detection'
#define RF_OP_MD_CAD   ( 0x07 )

/* IRQ flags register */
#define RF_IRQ_RX_TIMEOUT       ( 0x80 )
#define RF_IRQ_RX_DONE          ( 0x40 )
#define RF_IRQ_CRC_ERROR        ( 0x20 )
#define RF_IRQ_HDR_VALID        ( 0x10 )
#define RF_IRQ_TX_DONE          ( 0x08 )
#define RF_IRQ_CAD_DONE         ( 0x04 )
#define RF_IRQ_FHSS_CHANGE_CHAN ( 0x02 )
#define RF_IRQ_CAD_DETECTED     ( 0x01 )

/* LoRa modem status register */
// 'coding rate of the last received header'
#define RF_MDM_RX_RATE   ( 0xE0 )
// 'modem clear'
#define RF_MDM_CLEAR     ( 0x10 )
// 'header info valid'
#define RF_MDM_HDR_VALID ( 0x08 )
// 'receive ongoing'
#define RF_MDM_RX        ( 0x04 )
// 'signal synchronized'
#define RF_MDM_SYNC      ( 0x02 )
// 'signal detected'
#define RF_MDM_SIG       ( 0x01 )

#endif

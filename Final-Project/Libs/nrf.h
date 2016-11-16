#ifndef NRF
#define NRF

#include <stdint.h>

typedef uint8_t nrfCmd;

/* Register addresses and bitmasks (Table 27) */
#define NRF_CONFIG_ADDR         (0x00)                     // Configuration
#define NRF_EN_AA_ADDR          (0x01)                     // Enable 'Auto Acknowledge'
#define NRF_EN_RXADDR_ADDR      (0x02)                     // Enabled RX Addresses
#define NRF_RF_SETUP_ADDR       (0x06)
#define NRF_STATUS_ADDR         (0x07)                     // Enabled RX Addresses
#define NRF_RPD_ADDR            (0x09)
#define NRF_RX_PW_P0_ADDR       (0x11)
#define NRF_FEATURE_ADDR        (0x1D)

/* Bit masks and register configurations */
#define NRF_CONFIG_RX_DR_IIE    (0x40)
#define NRF_CONFIG_TX_DS_IIE    (0x20)
#define NRF_CONFIG_MAX_RT_IIE   (0x10)
#define NRF_CONFIG_EN_CRC       (0x08)
#define NRF_CONFIG_PWR_UP       (0x02)
#define NRF_CONFIG_PRIM_RX      (0x01)
#define NRF_CONFIG_PRIM_TX      (0x00)
#define NRF_RF_SETUP_CONT_WAVE  (0x80)
#define NRF_RF_SETUP_RF_DR_LOW  (0x20)
#define NRF_RF_SETUP_PLL_LOCK   (0x10)
#define NRF_RF_SETUP_RF_DR_HIGH (0x08)
#define NRF_RF_SETUP_RF_PWR_18  (0x00)
#define NRF_RF_SETUP_RF_PWR_12  (0x02)
#define NRF_RF_SETUP_RF_PWR_6   (0x04)
#define NRF_RF_SETUP_RF_PWR_0   (0x06)
#define NRF_STATUS_RX_DR        (0x40)
#define NRF_STATUS_TX_DS        (0x20)
#define NRF_STATUS_MAX_RT       (0x10)
#define NRF_STATUS_RX_P_NO      (0x0E)
#define NRF_FEATURE_EN_DYN_ACK  (0x01)

/* Command encodings (Table 19) */
#define NRF_R_REGISTER(addr)    ((nrfCmd) addr)        // Read specified register
#define NRF_W_REGISTER(addr)    ((nrfCmd) (0x20 | addr)) // Write specified register
#define NRF_R_RX_PAYLOAD        ((nrfCmd) 0x61)
#define NRF_W_TX_PAYLOAD        ((nrfCmd) 0xA0)
#define NRF_FLUSH_TX            ((nrfCmd) 0xE1)
#define NRF_FLUSH_RX            ((nrfCmd) 0xE2)
#define NRF_REUSE_TX_PL         ((nrfCmd) 0xE3)
#define NRF_R_RX_PL_WID         ((nrfCmd) 0x60)
#define NRF_W_ACK_PAYLOAD(pipe) ((nrfCmd) (0xA8 | pipe))
#define NRF_W_TX_PAYLOAD_NO_ACK ((nrfCmd) 0xB0)
#define NRF_NOP                 ((nrfCmd) 0xFF)
#define NRF_WAIT_FOR_DATA       ((nrfCmd) 0xFF)

#endif

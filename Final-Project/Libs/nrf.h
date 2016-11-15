#ifndef NRF
#define NRF

#include <stdint.h>

typedef uint8_t nrfCmd;

/* Register addresses and bitmasks (Table 27) */
#define NRF_CONFIF_ADDR         (0x00)                     // Configuration
#define NRF_EN_AA_ADDR          (0x01)                     // Enable 'Auto Acknowledge'
#define NRF_EN_RXADDR_ADDR      (0x02)                     // Enable RX Addresses


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

#endif

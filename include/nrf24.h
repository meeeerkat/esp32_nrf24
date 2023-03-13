#ifndef _NRF24_H_
#define _NRF24_H_

#include <stdint.h>
#include "consts.h"




typedef struct {
  uint8_t config,
          en_aa,
          en_rxaddr,
          setup_aw,
          setup_retr,
          rf_ch,
          rf_setup;
  uint8_t rx_addr_p0[5];    // full RX_ADDR_P0
  uint8_t rx_addr_base[4];  // first 4 bytes of RX_ADDR_P[1:5]
  uint8_t rx_addr_lsb_p[5]; // lsb bytes of each RX_ADDR_P[X+1]
  uint8_t tx_addr[5];
} nrf24_config_t;



typedef struct nrf24_s nrf24_t;


extern nrf24_t* nrf24__init(uint8_t ce_pin, uint8_t csn_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t clk_pin);

// Registers
extern uint8_t nrf24__read_register(nrf24_t* nrf, uint8_t reg);
extern void nrf24__buffread_register(nrf24_t* nrf, uint8_t reg, uint8_t* buff, uint8_t length);
extern void nrf24__write_register(nrf24_t* nrf, uint8_t reg, uint8_t value);
extern void nrf24__buffwrite_register(nrf24_t* nrf, uint8_t reg, uint8_t* buff, uint8_t length);

// Payload
extern void nrf24__read_payload(nrf24_t* nrf, uint8_t* data);
extern void nrf24__write_payload(nrf24_t* nrf, uint8_t* data);

// Flush
extern void nrf24__flush_tx(nrf24_t* nrf);
extern void nrf24__flush_rx(nrf24_t* nrf);

// Status
extern uint8_t nrf24__get_last_saved_status(nrf24_t* nrf);
extern uint8_t nrf24__update_status(nrf24_t* nrf);
extern void nrf24__print_status(nrf24_t* nrf);

// Config
extern void nrf24__get_config(nrf24_t* nrf, nrf24_config_t* c);
extern void nrf24__set_config(nrf24_t* nrf, nrf24_config_t* c);
extern void nrf24_config__print(nrf24_config_t* config);



#endif

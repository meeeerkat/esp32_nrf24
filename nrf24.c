
#include "nrf24.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"



typedef struct nrf24_s {
  uint8_t ce_pin, csn_pin;
  spi_device_handle_t* spi_handle;

  uint8_t status;
} nrf24_t;


/*
 * Public methods
 */

nrf24_t* nrf24__init(uint8_t ce_pin, uint8_t csn_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t clk_pin) {
  nrf24_t* nrf = malloc(sizeof(nrf24_t));
  nrf->ce_pin = ce_pin;
  nrf->csn_pin = csn_pin;

  gpio_set_direction(ce_pin, GPIO_MODE_OUTPUT);
  gpio_set_direction(csn_pin, GPIO_MODE_OUTPUT);

  // Initializing SPI bus
  spi_bus_config_t buscfg={
    .miso_io_num = miso_pin,
    .mosi_io_num = mosi_pin,
    .sclk_io_num = clk_pin,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1
  };
  spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED); 

  // Initializing slave interface
  spi_device_interface_config_t devcfg={
    .mode = 0,
    // 1 MHz, maybe would need to be configurable (10MHz doesn't work properly for 1 shift register)
    .clock_speed_hz = 1000000,
    .queue_size = 1,
    .command_bits = 8,
    .flags = SPI_DEVICE_NO_DUMMY// | SPI_DEVICE_BIT_LSBFIRST
  };
  spi_bus_add_device(SPI2_HOST, &devcfg, nrf->spi_handle);
  
  return nrf;
}



/*
 * Read & Write registers
 */

static void send_command(nrf24_t* nrf, spi_transaction_t* trans) {
  gpio_set_level(nrf->csn_pin, 0);
  esp_err_t err = spi_device_polling_transmit(*nrf->spi_handle, trans);
  gpio_set_level(nrf->csn_pin, 1);
  
  nrf->status = trans->rx_data[0];
}

// Read
uint8_t nrf24__read_register(nrf24_t* nrf, uint8_t reg) {
  spi_transaction_t trans = {
    .length = 2*8,
    .cmd = R_REGISTER | (REGISTER_MASK & reg),
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA
  };
  
  send_command(nrf, &trans);

  return trans.rx_data[1];
}
void nrf24__buffread_register(nrf24_t* nrf, uint8_t reg, uint8_t* buff, uint8_t length) {
  assert(length <= 4);

  spi_transaction_t trans = {
    .length = (length+1)*8,
    .cmd = R_REGISTER | (REGISTER_MASK & reg),
    .rx_buffer = buff
  };
  
  send_command(nrf, &trans);
}

// Write
void nrf24__write_register(nrf24_t* nrf, uint8_t reg, uint8_t value) {
  spi_transaction_t trans = {
    .length = 2*8,
    .cmd = W_REGISTER | (REGISTER_MASK & reg),
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA
  };
  trans.tx_data[0] = value;
  
  send_command(nrf, &trans);
}
void nrf24__buffwrite_register(nrf24_t* nrf, uint8_t reg, uint8_t* buff, uint8_t length) {
  spi_transaction_t trans = {
    .length = (1+length)*8,
    .cmd = W_REGISTER | (REGISTER_MASK & reg),
    .tx_buffer = buff
  };
  
  send_command(nrf, &trans);
}



/*
 * Read & Write payload
 */

void nrf24__read_payload(nrf24_t* nrf, uint8_t* data) {
  spi_transaction_t trans = {
    .length = 5*8,
    .cmd = R_RX_PAYLOAD,
    .rx_buffer = data
  };
  
  send_command(nrf, &trans);
}

void nrf24__write_payload(nrf24_t* nrf, uint8_t* data) {
  spi_transaction_t trans = {
    .length = 5*8,
    .cmd = W_TX_PAYLOAD,
    .tx_buffer = data
  };
  
  send_command(nrf, &trans);
}


/*
 * Flush TX & RX
 */

void nrf24__flush_tx(nrf24_t* nrf) {
  spi_transaction_t trans = {
    .length = 1*8,
    .cmd = FLUSH_TX
  };

  send_command(nrf, &trans);
}
void nrf24__flush_rx(nrf24_t* nrf) {
  spi_transaction_t trans = {
    .length = 1*8,
    .cmd = FLUSH_RX
  };

  send_command(nrf, &trans);
}


/*
 * Status
 */

uint8_t nrf24__get_last_saved_status(nrf24_t* nrf) {
  return nrf->status;
}
uint8_t nrf24__update_status(nrf24_t* nrf) {
  spi_transaction_t trans = {
    .length = 1*8,
    .cmd = NOP
  };

  send_command(nrf, &trans);
  return nrf->status;
}
void nrf24__print_status(nrf24_t* nrf) {
  printf("STATUS= 0x%x : {\n\tRX_DR=%x\n\tTX_DS=%x\n\tMAX_RT=%x\n\tRX_P_NO=%x\n\tTX_FULL=%x\n}\n",
      nrf->status,
      nrf->status & _BV(RX_DR),
      nrf->status & _BV(TX_DS),
      nrf->status & _BV(MAX_RT),
      (nrf->status >> RX_P_NO) & 0b111,
      nrf->status & _BV(TX_FULL)
  );
}




/*
 * CONFIG
 */

void nrf24__get_config(nrf24_t* nrf, nrf24_config_t* c) {
  c->config = nrf24__read_register(nrf, CONFIG);
  c->en_aa = nrf24__read_register(nrf, EN_AA);
  c->en_rxaddr = nrf24__read_register(nrf, EN_RXADDR);
  c->setup_aw = nrf24__read_register(nrf, SETUP_AW);
  c->setup_retr = nrf24__read_register(nrf, SETUP_RETR);
  c->rf_ch = nrf24__read_register(nrf, RF_CH);
  c->rf_setup = nrf24__read_register(nrf, RF_SETUP);

  // RX_ADDR_PX
  // P0
  nrf24__buffread_register(nrf, RX_ADDR_P0, c->rx_addr_p0, 5);
  // P1
  uint8_t buff[5];
  nrf24__buffread_register(nrf, RX_ADDR_P1, buff, 5);
  memcpy(c->rx_addr_base, buff+1, 4);
  c->rx_addr_lsb_p[0] = buff[0];
  // P2 -> P5
  for (uint8_t x=1; x < 5; x++)
    c->rx_addr_lsb_p[x] = nrf24__read_register(nrf, x+RX_ADDR_P1);

  // TX_ADDR
  nrf24__buffread_register(nrf, TX_ADDR, c->tx_addr, 5);
}

void nrf24__set_config(nrf24_t* nrf, nrf24_config_t* c) {
  nrf24__write_register(nrf, CONFIG, c->config);
  nrf24__write_register(nrf, EN_AA, c->en_aa);
  nrf24__write_register(nrf, EN_RXADDR, c->en_rxaddr);
  nrf24__write_register(nrf, SETUP_AW, c->setup_aw);
  nrf24__write_register(nrf, SETUP_RETR, c->setup_retr);
  nrf24__write_register(nrf, RF_CH, c->rf_ch);
  nrf24__write_register(nrf, RF_SETUP, c->rf_setup);

  // RX_ADDR_PX
  // P0
  nrf24__buffwrite_register(nrf, RX_ADDR_P0, c->rx_addr_p0, 5);
  // P1
  uint8_t buff[5];
  memcpy(buff+1, c->rx_addr_base, 4);
  buff[0] = c->rx_addr_lsb_p[0];
  nrf24__buffwrite_register(nrf, RX_ADDR_P1, buff, 5);
  // P2 -> P5
  for (uint8_t x=1; x < 5; x++)
    nrf24__write_register(nrf, x+RX_ADDR_P1, c->rx_addr_lsb_p[x]);

  // TX_ADDR
  nrf24__buffwrite_register(nrf, TX_ADDR, c->tx_addr, 5);
}

void nrf24_config__print(nrf24_config_t* c) {
  printf("CONFIG = {\n\t\
      .config=%x,\n\t\
      .en_aa=%x,\n\t\
      .en_rxaddr=%x,\n\t\
      .setup_aw=%x,\n\t\
      .setup_retr=%x,\n\t\
      .rf_ch=%x,\n\t\
      .rf_setup=%x,\n\t\
      .rx_addr_p0=%x,\n\t\
      .rx_addr_base=%x,\n\t",
      (unsigned) c->config,
      (unsigned) c->en_aa,
      (unsigned) c->en_rxaddr,
      (unsigned) c->setup_aw,
      (unsigned) c->setup_retr,
      (unsigned) c->rf_ch,
      (unsigned) c->rf_setup,
      (unsigned) c->rx_addr_p0,
      (unsigned) c->rx_addr_base);

  for (uint8_t x=0; x < 5; x++)
    printf(".rx_addr_lsb_p[%u]=%x,\n\t", (unsigned) x, (unsigned) c->rx_addr_lsb_p[x]);

  printf(".tx_addr=%x\n", (unsigned) c->tx_addr);

  printf("}");
}



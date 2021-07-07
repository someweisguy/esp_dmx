#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "soc/uart_struct.h"

#define REG_UART_BASE(i)      (DR_REG_UART_BASE + (i) * 0x10000 + ((i) > 1 ? 0xe000 : 0 ))
#define REG_UART_AHB_BASE(i)  (0x60000000 + (i) * 0x10000 + ((i) > 1 ? 0xe000 : 0 ))
#define UART_FIFO_AHB_REG(i)  (REG_UART_AHB_BASE(i) + 0x0)
#define UART_FIFO_REG(i)      (REG_UART_BASE(i) + 0x0)

/**
 * @brief Get the current length of the bytes in the rx FIFO.
 * 
 * @param hal Context of the HAL layer
 * 
 * @return Number of bytes in the rx FIFO
 */
static inline uint32_t dmx_hal_get_rxfifo_len(uart_dev_t *hw) 
{
    uint32_t fifo_cnt = hw->status.rxfifo_cnt;
    typeof(hw->mem_rx_status) rx_status = hw->mem_rx_status;
    uint32_t len = 0;

    // When using DPort to read fifo, fifo_cnt is not credible, we need to calculate the real cnt based on the fifo read and write pointer.
    // When using AHB to read FIFO, we can use fifo_cnt to indicate the data length in fifo.
    if (rx_status.wr_addr > rx_status.rd_addr) {
        len = rx_status.wr_addr - rx_status.rd_addr;
    } else if (rx_status.wr_addr < rx_status.rd_addr) {
        len = (rx_status.wr_addr + 128) - rx_status.rd_addr;
    } else {
        len = fifo_cnt > 0 ? 128 : 0;
    }

    return len;
}

/**
 * @brief Gets the number of bits the UART remains idle after transmitting data.
 * 
 * @param dev Pointer to a UART struct.
 * @return The number of bits the UART is idle after transmitting data. 
 */
static inline uint16_t dmx_hal_get_idle_num(uart_dev_t *dev) {
  return dev->idle_conf.tx_idle_num;
}

/**
 * @brief Gets the number of bits the UART sends as break.
 * 
 * @param dev Pointer to a UART struct.
 * @return The number of bits the UART sends as a break after transmitting.
 */
static inline uint8_t dmx_hal_get_break_num(uart_dev_t *dev) {
  return dev->idle_conf.tx_brk_num;
}

/**
 * @brief Gets the UART rx timeout (unit: time it takes for one word to be sent at current baud_rate).
 * 
 * @param dev Pointer to a UART struct.
 * @return The UART rx timeout.
 */
static inline uint8_t dmx_hal_get_rx_tout(uart_dev_t *dev) {
  return dev->conf1.rx_tout_en ? dev->conf1.rx_tout_thrhd : 0;
}

/**
 * @brief Inverts or uninverts tx line on the UART bus.
 * 
 * @param dev Pointer to a UART struct.
 * @param invert 1 to invert, 0 to un-invert.
 */
static inline void dmx_hal_inverse_txd_signal(uart_dev_t *dev, int invert) {
  dev->conf0.txd_inv = invert ? 1 : 0;
}

/**
 * @brief Inverts or uninverts rts line on the UART bus.
 * 
 * @param dev Pointer to a UART struct.
 * @param invert 1 to invert, 0 to un-invert.
 */
static inline void dmx_hal_inverse_rts_signal(uart_dev_t *dev, int invert) {
  dev->conf0.rts_inv = invert ? 1 : 0;
}

/**
 * @brief Gets the level of the rx line on the UART bus.
 * 
 * @param dev Pointer to a UART struct.
 * @return UART rx line level.
 */
static inline uint32_t dmx_hal_get_rx_level(uart_dev_t *dev) {
  return dev->status.rxd;
}

/**
 * @brief  Read the UART rxfifo.
 *
 * @param  hw Beginning address of the peripheral registers.
 * @param  buf The data buffer. The buffer size should be large than 128 byts.
 * @param  rd_len The data length needs to be read.
 *
 * @return None.
 */
static inline void dmx_hal_read_rxfifo(uart_dev_t *hw, uint8_t *buf, uint32_t rd_len)
{
    //Get the UART APB fifo addr. Read fifo, we use APB address
    for(int i = 0; i < rd_len; i++) {
        buf[i] = hw->fifo.rw_byte;
#ifdef CONFIG_COMPILER_OPTIMIZATION_PERF
        __asm__ __volatile__("nop"); // TODO: why is this here?
#endif
    }
}

/**
 * @brief Read the first num characters from the rxfifo.
 * 
 * @param dev Pointer to a UART struct.
 * @param buf Destination buffer to be read into
 * @param num The maximum number of characters to read
 * 
 * @return The number of characters read
 */
static inline int dmx_hal_readn_rxfifo(uart_dev_t *dev, uint8_t *buf, int num) {
  const uint16_t rxfifo_len = dmx_hal_get_rxfifo_len(dev);
  if (num > rxfifo_len) num = rxfifo_len;
  dmx_hal_read_rxfifo(dev, buf, num);
  return num;
}

static inline uint32_t dmx_hal_get_intsts_mask(uart_dev_t *dev) {
  return dev->int_st.val;
}

static inline void dmx_hal_disable_intr_mask(uart_dev_t *dev, uint32_t mask) {
  dev->int_ena.val &= (~mask);
}

static inline void dmx_hal_clr_intsts_mask(uart_dev_t *dev, uint32_t mask) {
  dev->int_clr.val = mask;
}

static inline void dmx_hal_ena_intr_mask(uart_dev_t *dev, uint32_t mask) {
  dev->int_ena.val |= mask;
}

static inline void dmx_hal_set_rts(uart_dev_t *dev, int set) { // 1 sets low, 0 sets high
  dev->conf0.sw_rts = set & 0x1;
}

static inline uint32_t dmx_hal_get_intr_ena_status(uart_dev_t *dev){
  return dev->int_ena.val;
}

static inline void dmx_hal_init(uart_dev_t *dev, dmx_port_t dmx_num) {
  // uart_hal_set_parity(&(dmx_context[dmx_num].dev), UART_PARITY_DISABLE);
  // uart_hal_set_data_bit_num(&(dmx_context[dmx_num].dev), UART_DATA_8_BITS);
  // uart_hal_set_stop_bits(&(dmx_context[dmx_num].dev), UART_STOP_BITS_2);
  // uart_hal_set_hw_flow_ctrl(&(dmx_context[dmx_num].dev), UART_HW_FLOWCTRL_DISABLE, 0);
  // uart_hal_set_mode(&(dmx_context[dmx_num].dev), UART_MODE_RS485_COLLISION_DETECT);
}

static inline void dmx_hal_set_baudrate(uart_dev_t *dev, uart_sclk_t source_clk, int baud_rate) {

}

static inline void dmx_hal_set_tx_idle_num(uart_dev_t *dev, uint16_t idle_num) {

}

static inline void dmx_hal_tx_break(uart_dev_t *dev, uint8_t break_num) {

}
static inline void dmx_hal_get_sclk(uart_dev_t *dev, uart_sclk_t *source_clk) {

}

static inline uint32_t dmx_hal_get_baudrate(uart_dev_t *dev) {
  uint32_t src_clk = dev->conf0.tick_ref_always_on ? APB_CLK_FREQ : REF_CLK_FREQ;
  typeof(dev->clk_div) div_reg = dev->clk_div;
  return ((src_clk << 4)) / ((div_reg.div_int << 4) | div_reg.div_frag);
}

static inline void dmx_hal_set_rx_timeout(uart_dev_t *dev, uint8_t rx_timeout_thresh) {

}

static inline void dmx_hal_set_rxfifo_full_thr(uart_dev_t *dev, uint8_t rxfifo_full_thresh) {

}

static inline void dmx_hal_set_txfifo_empty_thr(uart_dev_t *dev, uint8_t threshold) {

}

static inline void dmx_hal_rxfifo_rst(uart_dev_t *hw)
{
    //Hardware issue: we can not use `rxfifo_rst` to reset the hw rxfifo.
    uint16_t fifo_cnt;
    typeof(hw->mem_rx_status) rxmem_sta;
    //Get the UART APB fifo addr
    uint32_t fifo_addr = (hw == &UART0) ? UART_FIFO_REG(0) : (hw == &UART1) ? UART_FIFO_REG(1) : UART_FIFO_REG(2);
    do {
        fifo_cnt = hw->status.rxfifo_cnt;
        rxmem_sta.val = hw->mem_rx_status.val;
        if(fifo_cnt != 0 ||  (rxmem_sta.rd_addr != rxmem_sta.wr_addr)) {
            READ_PERI_REG(fifo_addr);
        } else {
            break;
        }
    } while(1);
}

static inline void dmx_ll_write_txfifo(uart_dev_t *hw, const uint8_t *buf, uint32_t wr_len) {
  //Get the UART AHB fifo addr, Write fifo, we use AHB address
  uint32_t fifo_addr = (hw == &UART0) ? UART_FIFO_AHB_REG(0) : (hw == &UART1) ? UART_FIFO_AHB_REG(1) : UART_FIFO_AHB_REG(2);
  for(int i = 0; i < wr_len; i++) {
    WRITE_PERI_REG(fifo_addr, buf[i]);
  }
}

static inline uint32_t dmx_ll_get_txfifo_len(uart_dev_t *hw) {
  // default fifo len - fifo count
  return 128 - hw->status.txfifo_cnt;
}

static inline void dmx_hal_write_txfifo(uart_dev_t *dev, const uint8_t *buf, uint32_t data_size, uint32_t *write_size) {
    uint16_t fill_len = dmx_ll_get_txfifo_len(dev);
    if(fill_len > data_size) {
        fill_len = data_size;
    }
    *write_size = fill_len;
    dmx_ll_write_txfifo(dev, buf, fill_len);
}

static inline void dmx_hal_txfifo_rst(uart_dev_t *hw) {
  /*
   * Note:   Due to hardware issue, reset UART1's txfifo will also reset UART2's txfifo.
   *         So reserve this function for UART1 and UART2. Please do DPORT reset for UART and its memory at chip startup
   *         to ensure the TX FIFO is reset correctly at the beginning.
   */
    if (hw == &UART0) {
        hw->conf0.txfifo_rst = 1;
        hw->conf0.txfifo_rst = 0;
    }
}

#ifdef __cplusplus
}
#endif

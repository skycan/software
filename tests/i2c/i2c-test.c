/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A quick program that blinks the LEDs
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "dev/leds.h"
#include "dev/i2c.h"

#define I2C_ADDR 0x55

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "i2c-test");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/

void
write_reg(uint8_t reg, uint8_t val) {
  uint8_t tx_buf[] = {reg, val};

  i2c_transmitinit(I2C_ADDR);
  while (i2c_busy());

  i2c_transmit_n(2, tx_buf);
  while (i2c_busy());
}

uint16_t
read_reg(uint8_t reg) {
  uint8_t rxbuf[2];
  uint8_t tx = reg;

  /* transmit the register to read */
  i2c_transmitinit(I2C_ADDR);
  while (i2c_busy());
  i2c_transmit_n(1, &tx);
  while (i2c_busy());

  /* receive the data */
  i2c_receiveinit(I2C_ADDR);
  while (i2c_busy());
  i2c_receive_n(2, (uint8_t*)&rxbuf);
  while (i2c_busy());

  return (rxbuf[0] << 8) | (rxbuf[1] & 0xFF);
}

PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_EXITHANDLER(goto exit;)
  PROCESS_BEGIN();

  static struct etimer et;
  i2c_enable();

  while(1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
    leds_on(LEDS_ALL);
    read_reg(0x06);
    leds_off(LEDS_ALL);
  }

 exit:
  leds_off(LEDS_ALL);
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

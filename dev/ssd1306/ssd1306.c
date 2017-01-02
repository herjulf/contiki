/*
 * Copyright (c) 2016, Copyright Robert Olsson
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
 *
 * Author  : Robert Olsson rolss@kth.se/robert@radio-sensors.com
 * Created : 2016-12-25
 */

/**
 * \file
 *        Basic functions for SSD1306 based on datasheet Rev X.X
 */

#include "contiki.h"
#include <string.h>
#include "ssd1306.h"
#include "ssd1306-arch.h"
#include "i2c.h"

#include "font6x8.h"
#include "font8x16.h"
//#include "img0_128x64c1.h"

int8_t display_buffer[1024];

void ssd1306_cmd(int8_t val)
{
  int8_t cmd = 0x00;
  i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
  i2c_write(cmd);
  i2c_write(val);
  i2c_stop();
}

void ssd1306_data(int8_t val)
{
  int8_t cmd = 0x40;
  i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
  i2c_write(cmd);
  i2c_write(val);
  i2c_stop();
}

void
delay_ms(uint16_t ms) 
{
  uint16_t i;
  for(i = 0; i < ms; i++) {
    clock_delay_usec(1000);
  }
}

char *str ="KALLE";


void setColAddress()
{
  ssd1306_cmd(SSD1306_COLUMNADDR); // 0x21 CMD
  ssd1306_cmd(0); // Column start address
  ssd1306_cmd(SSD1306_LCDWIDTH-1); // Column end address
}

// Used when doing Horizontal or Vertical Addressing
void setPageAddress()
{
  ssd1306_cmd(SSD1306_PAGEADDR); // 0x22 CMD
  ssd1306_cmd(0); // Start Page address
  ssd1306_cmd((SSD1306_LCDHEIGHT/8)-1);// End Page address
}

/* Transfer buffer to CGRAM */
void TransferBuffer()
{
  int16_t j=0;
  int8_t cmd = 0x40;
   
  setColAddress(); 
  setPageAddress();
  delay_ms(1); 
  i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
  i2c_write(0X40); /* data */
  for(j=0; j < sizeof(display_buffer); j++) {
    i2c_write(display_buffer[j]);
  }
  i2c_stop();
}

void ssd1306_setpos(uint8_t x, uint8_t y)
{
  i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
  i2c_write(0X0); /* Command */
  i2c_write(0xb0 + (y & 0x7));
  //i2c_write(0x21 + (x & 0x7));
  //i2c_write(((x & 0xf0) >> 4) | 0x10);
  ///i2c_write((x & 0x0f));
  i2c_stop();
  setColAddress();
}

void ssd1306_fill4(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) 
{
  uint16_t i;

  ssd1306_setpos(0, 0);

  //setColAddress(); 
  //setPageAddress();

  i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
  i2c_write(0X40); /* data */
  for (i = 0; i < 128 * 8 / 4; i++) {
    i2c_write(p1);
    i2c_write(p2);
    i2c_write(p3);
    i2c_write(p4);
  }
  i2c_stop();
}

void ssd1306_fill2(uint8_t p1, uint8_t p2) 
{
	ssd1306_fill4(p1, p2, p1, p2);
}

void ssd1306_fill(uint8_t p) 
{
	ssd1306_fill4(p, p, p, p);
}

void ssd1306_draw_bmp(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[])
{
  uint8_t y, x;
  uint16_t j = 0;

  if (y1 % 8 == 0) 
    y = y1 / 8;
  else 
    y = y1 / 8 + 1;

  for (y = y0; y < y1; y++)  {

    ssd1306_setpos(x0,y);

    i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
    i2c_write(0X40); /* data */

    for (x = x0; x < x1; x++)  {
      i2c_write(pgm_read_byte(&bitmap[j++]));
    }
    i2c_stop();
  }
}

void ssd1306_char_font6x8(char ch) 
{
  uint8_t i, c = ch - 32;
  i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
  i2c_write(0X40); /* data */
  for (i = 0; i < 6; i++)  {
    i2c_write(pgm_read_byte(&ssd1306xled_font6x8[c * 6 + i]));
  }
  i2c_stop();
}

void ssd1306_string_font6x8(char *s) {
  while (*s) {
    ssd1306_char_font6x8(*s++);
  }
}

void ssd1306_char_font(char *f, char ch) 
{
  uint8_t i, c = ch - 32;
  i2c_start(I2C_SSD1306_ADDR | I2C_WRITE);
  i2c_write(0X40); /* data */
  for (i = 0; i < 6; i++)  {
    i2c_write(pgm_read_byte(&f[c * 6 + i]));
  }
  i2c_stop();
}

void ssd1306_string_show(char *f, char *s) {
  while (*s) {
    ssd1306_char_font(f, *s++);
  }
}

void
ssd1306_clear(void)
{
  memset(display_buffer, 0X00, sizeof(display_buffer));
  TransferBuffer();
}

void
ssd1306_hello(void)
{
  ssd1306_clear();

  // ---- Fill out screen with patters ----
  //ssd1306_fill(0xAA); delay_ms(400);
  //ssd1306_fill2(0x55, 0xAA); delay_ms(400);
  //ssd1306_fill4(0xCC, 0xCC, 0x33, 0x33); delay_ms(400);
  //delay_ms(1000);

  // ---- Draw bitmap 
  //ssd1306_draw_bmp(0,0,128,8, img0_128x64c1);
  //ssd1306_draw_bmp(0,0,128,8, ssd1306xled_font8x16);
  //delay_ms(6000);


#if 0
  // ---- Print some small numbers on the screen ----
  for (j = 0; j < 8; j++) {
    ssd1306_setpos(0, j);
    for (i = 0; i < 7; i++) {
      ssd1306_numdec(n1++);
      ssd1306_string(" ");
    }
  }
#endif

   //ssd1306_string_font6x8(str) ;
  //ssd1306_string_font6x8("This is the");

  //ssd1306_setpos(20, 1);
  //ssd1306_string_show(ssd1306xled_font6x8, "Radio Sensors AB");
  ssd1306_setpos(20, 0);
  ssd1306_string_show(ssd1306xled_font6x8, "Contiki-OS");
  ssd1306_setpos(20, 1);
  ssd1306_string_show(ssd1306xled_font6x8, "0xDEADBEEF...");
  ssd1306_setpos(20, 2);
  ssd1306_string_show(ssd1306xled_font6x8, "Radio Sensors AB");
  ssd1306_setpos(20, 3);
  ssd1306_string_show(ssd1306xled_font6x8, "Uppsala");
  ssd1306_setpos(20, 4);
  ssd1306_string_show(ssd1306xled_font6x8, "SWEDEN");

}

void
ssd1306_init(void)
{
  uint8_t j;
  uint16_t i, n1 = 0;

  delay_ms(50); /* really needed ? */

  ssd1306_cmd(SSD1306_DISPLAYOFF);                    // 0xAE
  ssd1306_cmd(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  ssd1306_cmd(0x80);                 // the suggested ratio 0x80
  ssd1306_cmd(SSD1306_SETMULTIPLEX);                  // 0xA8
  ssd1306_cmd(0x3F);
  ssd1306_cmd(SSD1306_SETDISPLAYOFFSET);              // 0xD3
  ssd1306_cmd(0x0);                                   // no offset
  ssd1306_cmd(SSD1306_SETSTARTLINE);// | 0x0);        // line #0
  ssd1306_cmd(SSD1306_CHARGEPUMP);                    // 0x8D
  ssd1306_cmd(0x14);  // using internal VCC
  ssd1306_cmd(SSD1306_MEMORYMODE);                    // 0x20
  ssd1306_cmd(0x00);          // 0x00 horizontal addressing
  ssd1306_cmd(SSD1306_SEGREMAP | 0x1); // rotate screen 180
  ssd1306_cmd(SSD1306_COMSCANDEC); // rotate screen 180
  ssd1306_cmd(SSD1306_SETCOMPINS);                    // 0xDA
  ssd1306_cmd(0x12);
  ssd1306_cmd(SSD1306_SETCONTRAST);                   // 0x81
  ssd1306_cmd(0xCF);
  ssd1306_cmd(SSD1306_SETPRECHARGE);                  // 0xd9
  ssd1306_cmd(0xF1);
  ssd1306_cmd(SSD1306_SETVCOMDETECT);                 // 0xDB
  ssd1306_cmd(0x40);
  ssd1306_cmd(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  ssd1306_cmd(SSD1306_NORMALDISPLAY);                 // 0xA6
  //ssd1306_cmd(SSD1306_INVERTDISPLAY);                 // 0xA7
  ssd1306_cmd(SSD1306_DISPLAYON);                     //switch on OLED
  ssd1306_hello();
}

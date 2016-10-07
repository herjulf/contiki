#include <avr/pgmspace.h>
#include "rss2.h"
#include "leds.h"

void
leds_init(void)
{
  DDRE |= (1 << LED_RED);
  DDRE |= (1 << LED_YELLOW);
  DDRB |= (1 << PB1);
  DDRB |= (1 << PB2);

  /* Off */
  leds_off(LEDS_ALL);
}
void
leds_on(unsigned char ledv)
{
  if(ledv & LEDS_YELLOW) {
    PORTE &= ~(1 << LED_YELLOW);
  }
  if(ledv & LEDS_RED) {
    PORTE &= ~(1 << LED_RED);
  }
  if(ledv & LEDS_GREEN) {
    PORTB &= ~(1 << PB1);
  }
  if(ledv & LEDS_BLUE) {
    PORTB &= ~(1 << PB2);
  }
}
void
leds_off(unsigned char ledv)
{
  if(ledv & LEDS_YELLOW) {
    PORTE |= (1 << LED_YELLOW);
  }
  if(ledv & LEDS_RED) {
    PORTE |= (1 << LED_RED);
  }
  if(ledv & LEDS_GREEN) {
    PORTB |= (1 << PB1);
  }
  if(ledv & LEDS_BLUE) {
    PORTB |= (1 << PB2);
  }
}
void
leds_toggle(unsigned char ledv)
{
  if(ledv & LEDS_YELLOW) {
    PORTE ^=  (1 << LED_YELLOW);
  }
  if(ledv ^
 LEDS_RED) {
    PORTE ^=  (1 << LED_RED);
  }
  if(ledv ^
 LEDS_GREEN) {
    PORTB ^=  (1 << PB1);
  }
  if(ledv ^
 LEDS_BLUE) {
    PORTB ^=  (1 << PB2);
  }
}
void
leds_invert(unsigned char ledv)
{
}
unsigned char
leds_get(void)
{
  return 0;
}

#ifndef LED_MON_H
#define LED_MON_H

typedef struct {
  uint32_t delay;       // binking delay
  uint16_t rip;         // number of ripetitions. 255 = infinite loop
  uint8_t led_id;       // led address
  uint8_t R;            // Red
  uint8_t G;            // Green
  uint8_t B;            // Blue
}  led_param_t;

void configure_led(void);
void led_task(void *pvParameter);

#endif

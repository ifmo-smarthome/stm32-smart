#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <string.h>
#include <stdio.h>

#define ONE_WIRE_TX GPIOD, GPIO5
#define ONE_WIRE_RX GPIOA, GPIO0
#define SCRATCHPAD_SIZE 9

volatile uint32_t system_millis;
void sys_tick_handler(void)
{
  system_millis++;
}

static void msleep(uint32_t delay)
{
  uint32_t wake = system_millis + delay;
  while (wake > system_millis);
}

struct ds18b20 {
  uint8_t (*read_temperature)(struct ds18b20 *self);
  void    (*write)(struct ds18b20 *self, uint8_t value);
  uint8_t (*read)(struct ds18b20 *self);
  void    (*initialize)(struct ds18b20 *self);
  void    (*send)(struct ds18b20 *self, uint8_t cmd);
};

static uint8_t ds18b20_read_temperature(struct ds18b20 *self) {
  uint8_t i, j;
  uint8_t scratchpad[SCRATCHPAD_SIZE] = {0};

  self->initialize(self);
  self->send(self, 0xCC);
  self->send(self, 0x44);
  gpio_set(ONE_WIRE_TX);
  msleep(750000);

  self->initialize(self);
  self->send(self, 0xCC);
  self->send(self, 0xBE);

  for(i = 0; i < SCRATCHPAD_SIZE; i++)
    for(j = 0; j < 8; j++)
      scratchpad[i] |= self->read(self) << j;

  return ((scratchpad[1] & 0x7) << 4) | ((scratchpad[0] & 0xF0) >> 4);
}

static void ds18b20_write(struct ds18b20 *self, uint8_t value) {
  if(value) {
    gpio_clear(ONE_WIRE_TX);
    msleep(10);
    gpio_set(ONE_WIRE_TX);
    msleep(70);
  } else {
    gpio_clear(ONE_WIRE_TX);
    msleep(70);
    gpio_set(ONE_WIRE_TX);
    msleep(10);
  }
}

static uint8_t ds18b20_read(struct ds18b20 *self) {
  uint8_t value;

  gpio_clear(ONE_WIRE_TX);
  msleep(3);
  gpio_set(ONE_WIRE_TX);
  msleep(3);

  value = gpio_get(ONE_WIRE_RX);

  msleep(60); // Minimum of 60 mcs in duration
  return (value ? 1 : 0);
}

static void ds18b20_initialize(struct ds18b20 *self) {
  gpio_clear(ONE_WIRE_TX);
  msleep(480);
  gpio_set(ONE_WIRE_TX);
  msleep(60);
  if( gpio_get(ONE_WIRE_RX) == 0)
    gpio_set(GPIOD, GPIO12);

  msleep(300);
}

static void ds18b20_send(struct ds18b20 *self, uint8_t cmd) {
  uint8_t i;
  for( i = 0; i < 8; i++)
    self->write(self, cmd & (1 << i));

  msleep(300);
}

static void systick_setup(void)
{
  systick_set_reload(48);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();
  systick_interrupt_enable();
}

static void clock_setup(void)
{
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_48MHZ]);

  /* Enable GPIOA clock for communicate with bluetooth controller */
  rcc_periph_clock_enable(RCC_GPIOA);

  /* Enable GPIOD clock for communicate with temperature sensor */
  rcc_periph_clock_enable(RCC_GPIOD);

  /* Enable clocks for USART2 */
  rcc_periph_clock_enable(RCC_USART2);
}

static void usart_setup(void)
{
  /* Setup USART2 parameters */
  usart_set_baudrate(USART2, 9600);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART2);
}

static void gpio_setup(void)
{
  /* Setup GPIO pins for USART2 transmit */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

  /* Setup GPIO pins for USART2 receive */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO3);

  /* Setup USART2 TX pin as alternate function */
  gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // PA2 -- TX
  gpio_set_af(GPIOA, GPIO_AF7, GPIO3); // PA3 -- RX

  /* Setup pin for control relay */
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4); // PA4

  /* Setup pins for communicate with temperature sensor */
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

  gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

struct ds18b20 ds18b20 = {
  .read_temperature = ds18b20_read_temperature,
  .write            = ds18b20_write,
  .read             = ds18b20_read,
  .send             = ds18b20_send,
  .initialize       = ds18b20_initialize
};

void str_send(char *str) {
  uint8_t len = strlen(str);
  uint8_t i;
  for( i = 0; i < len; i++ )
    usart_send_blocking(USART2, str[i]);
}

int main(void)
{
  clock_setup();
  gpio_setup();
  systick_setup();
  usart_setup();

  char data_rx;
  uint8_t temp;
  char buf[1024];

  /* Opcodes
   * a: relay turn on
   * b: relay turn off
   * t: get temperature
   * */
  while(1) {
    data_rx = usart_recv_blocking(USART2);
    switch(data_rx) {
      case 'a':
        gpio_set(GPIOA, GPIO4);
        break;
      case 'b':
        gpio_clear(GPIOA, GPIO4);
        break;
      case 't':
        temp = ds18b20.read_temperature(&ds18b20);
        snprintf(buf, sizeof(buf), "Temperature is %d C\r\n", temp);
        str_send(buf);
        break;
    }
  }

  while(1){}
  return 0;
}

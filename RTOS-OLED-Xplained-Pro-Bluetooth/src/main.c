#include <asf.h>
#include <string.h>

#include "conf_board.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

// LEDs
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Botão
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX      11
#define BUT_IDX_MASK (1 << BUT_IDX)

// CAM PAUSE
#define CAMP_PIO      PIOA
#define CAMP_PIO_ID   ID_PIOA
#define CAMP_PIO_IDX  3
#define CAMP_IDX_MASK (1 << CAMP_PIO_IDX)

// CAM NEXT
#define CAMN_PIO      PIOA
#define CAMN_PIO_ID   ID_PIOA
#define CAMN_PIO_IDX  4
#define CAMN_IDX_MASK (1 << CAMN_PIO_IDX)

// CAM BACK
#define CAMB_PIO      PIOA
#define CAMB_PIO_ID   ID_PIOA
#define CAMB_PIO_IDX  2
#define CAMB_IDX_MASK (1 << CAMB_PIO_IDX)


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void io_init(void);
static void camp_callback(void);
static void camn_callback(void);
static void camb_callback(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

SemaphoreHandle_t xSemaphoreCamp;

SemaphoreHandle_t xSemaphoreCamn;

SemaphoreHandle_t xSemaphoreCamb;

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

static void camp_callback(void) {
	printf("Entrei no callback camp");
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreCamp, &xHigherPriorityTaskWoken);
}

static void camn_callback(void) {
	printf("Entrei no callback camn");
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreCamn, &xHigherPriorityTaskWoken);
}

static void camb_callback(void) {
	printf("Entrei no callback camb");
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreCamb, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void io_init(void) {

  // Ativa PIOs
  pmc_enable_periph_clk(LED_PIO_ID);
  pmc_enable_periph_clk(BUT_PIO_ID);

  // Configura Pinos
  pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
  pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
}

uint32_t usart_puts(uint8_t *pstring) {
  uint32_t i ;

  while(*(pstring + i))
  if(uart_is_tx_empty(USART_COM))
  usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
  usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
  uint timecounter = timeout_ms;
  uint32_t rx;
  uint32_t counter = 0;

  while( (timecounter > 0) && (counter < bufferlen - 1)) {
    if(usart_read(usart, &rx) == 0) {
      buffer[counter++] = rx;
    }
    else{
      timecounter--;
      vTaskDelay(1);
    }
  }
  buffer[counter] = 0x00;
  return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
  usart_put_string(usart, buffer_tx);
  usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
  sysclk_enable_peripheral_clock(ID_USART0);
  usart_serial_options_t config;
  config.baudrate = 9600;
  config.charlength = US_MR_CHRL_8_BIT;
  config.paritytype = US_MR_PAR_NO;
  config.stopbits = false;
  usart_serial_init(USART0, &config);
  usart_enable_tx(USART0);
  usart_enable_rx(USART0);

  // RX - PB0  TX - PB1
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
  char buffer_rx[128];
  usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
  vTaskDelay( 500 / portTICK_PERIOD_MS);
  usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
  vTaskDelay( 500 / portTICK_PERIOD_MS);
  usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMESmartcontrol", 100);
  vTaskDelay( 500 / portTICK_PERIOD_MS);
  usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
  vTaskDelay( 500 / portTICK_PERIOD_MS);
  usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
  vTaskDelay( 500 / portTICK_PERIOD_MS);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  gfx_mono_ssd1306_init();
  gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
  gfx_mono_draw_string("oii", 0, 20, &sysfont);

  for (;;)  {

	 /*
	  if(!pio_get(CAMP_PIO, PIO_INPUT, CAMP_IDX_MASK)){
		printf("LIGUEI");	
	  }
	  if (xSemaphoreTake(xSemaphoreCamp, 100)){
		  printf("LIGUEI");	
		  gfx_mono_draw_string("               ", 0, 0, &sysfont);
		  gfx_mono_draw_string("Pause", 0, 0, &sysfont);
		  
	  }
	  
	  if (xSemaphoreTake(xSemaphoreCamn, 100)){
		  gfx_mono_draw_string("               ", 0, 0, &sysfont);
		  gfx_mono_draw_string("Next", 0, 0, &sysfont);
		  
	  }
	  */
  }
}

void task_bluetooth(void) {
  printf("Task Bluetooth started \n");
  printf("Inicializando HC05 \n");
  #ifndef DEBUG_SERIAL
  config_usart0();
  #endif
  hc05_init();

  // configura LEDs e Botões
  io_init();

  char button1 = '0';
  char eof = 'X';

  // Task não deve retornar.
  while(1) {
    // atualiza valor do botão
    if(pio_get(BUT_PIO, PIO_INPUT, BUT_IDX_MASK) == 0) {
        button1 = '1';
      } else if (!pio_get(CAMP_PIO, PIO_INPUT, CAMP_IDX_MASK)){
		button1 = '2';
	  } else if (!pio_get(CAMN_PIO, PIO_INPUT, CAMN_IDX_MASK)){
		button1 = '3';
	  } else if (!pio_get(CAMB_PIO, PIO_INPUT, CAMB_IDX_MASK)){
		button1 = '4';
	  } else {
        button1 = '0';
    }

	
	

    // envia status botão
    while(!usart_is_tx_ready(USART_COM)) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    usart_write(USART_COM, button1);
    
    // envia fim de pacote
    while(!usart_is_tx_ready(USART_COM)) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    usart_write(USART_COM, eof);
	

    // dorme por 500 ms
    vTaskDelay(125 / portTICK_PERIOD_MS);
  }
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

void init_pins(void){
	pmc_enable_periph_clk(CAMP_PIO_ID);
	pmc_enable_periph_clk(CAMN_PIO_ID);
	pmc_enable_periph_clk(CAMB_PIO_ID);

	pio_configure(CAMP_PIO, PIO_INPUT, CAMP_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(CAMP_PIO, CAMP_IDX_MASK, 60);
	pio_handler_set(CAMP_PIO,
					CAMP_PIO_ID,
					CAMP_IDX_MASK,
					PIO_IT_EDGE,
					camp_callback);
					
	pio_enable_interrupt(CAMP_PIO, CAMP_IDX_MASK);
	pio_get_interrupt_status(CAMP_PIO);
	
	NVIC_EnableIRQ(CAMP_PIO_ID);
	NVIC_SetPriority(CAMP_PIO_ID, 4); // Prioridade 4
	
	//-----------------------------------------
	
	pio_configure(CAMN_PIO, PIO_INPUT, CAMN_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(CAMN_PIO, CAMN_IDX_MASK, 60);
	
					
	pio_handler_set(CAMN_PIO,
					CAMN_PIO_ID,
					CAMN_IDX_MASK,
					PIO_IT_EDGE,
					camn_callback);
					
	pio_enable_interrupt(CAMN_PIO, CAMN_IDX_MASK);
	pio_get_interrupt_status(CAMN_PIO);
	
	NVIC_EnableIRQ(CAMN_PIO_ID);
	NVIC_SetPriority(CAMN_PIO_ID, 4); // Prioridade 4
	
	//-----------------------------------------
	/*
	pio_configure(CAMB_PIO, PIO_INPUT, CAMB_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(CAMB_PIO, CAMB_IDX_MASK, 60);
	pio_handler_set(CAMB_PIO,
					CAMB_PIO_ID,
					CAMB_IDX_MASK,
					PIO_IT_EDGE,
					camb_callback);
	
	pio_enable_interrupt(CAMB_PIO, CAMB_IDX_MASK);
	pio_get_interrupt_status(CAMB_PIO);
	
	NVIC_EnableIRQ(CAMB_PIO_ID);
	NVIC_SetPriority(CAMB_PIO_ID, 4); // Prioridade 4
	*/
	
	
	
}


int main(void) {
	/* Initialize the SAM system */
	printf("oi");
	sysclk_init();
	board_init();
	init_pins();

	configure_console();

	/*
	xSemaphoreCamp = xSemaphoreCreateBinary();
	if (xSemaphoreCamp == NULL)
		printf("falha em criar o semaforo \n");
	
	xSemaphoreCamn = xSemaphoreCreateBinary();
	if (xSemaphoreCamn == NULL)
		printf("falha em criar o semaforo \n");
		
	xSemaphoreCamb = xSemaphoreCreateBinary();
	if (xSemaphoreCamb == NULL)
	printf("falha em criar o semaforo \n");

	*/
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

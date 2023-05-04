/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/************************************************************************/
/* defines                                                              */
/************************************************************************/

 
#define PIO_PA2A_URXD0  2
#define PIO_PA2A_URXD0_MASK (1 << PIO_PA2A_URXD0)

#define PIO_PA24A_UTXD0 24
#define PIO_PA24A_UTXD0_MASK (1 << PIO_PA24A_UTXD0)

 
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Botão
// #define BUT_PIO      PIOA
// #define BUT_PIO_ID   ID_PIOA
// #define BUT_IDX      11
// #define BUT_IDX_MASK (1 << BUT_IDX)

#define BUT_PIO				PIOA
#define BUT_PIO_ID			ID_PIOA
#define BUT_PIO_PIN			11
#define BUT_PIO_PIN_MASK	(1 << BUT_PIO_PIN)
#define ACTIVATE_IN			PIO_IT_FALL_EDGE

//PAUSE - CAM - P0
#define CAM_1_PIO			PIOA
#define CAM_1_PIO_ID		ID_PIOA
#define CAM_1_PIO_IDX		21
#define CAM_1_PIO_IDX_MASK (1u << CAM_1_PIO_IDX)

//NEXT - CAM - P1
#define CAM_2_PIO			PIOA
#define CAM_2_PIO_ID		ID_PIOA
#define CAM_2_PIO_IDX		3
#define CAM_2_PIO_IDX_MASK (1u << CAM_2_PIO_IDX)

//BACK - CAM - P2
#define CAM_3_PIO			PIOA
#define CAM_3_PIO_ID		ID_PIOA
#define CAM_3_PIO_IDX		4
#define CAM_3_PIO_IDX_MASK (1u << CAM_3_PIO_IDX)

//BACK
#define BUT_1_PIO			PIOD
#define BUT_1_PIO_ID		ID_PIOD
#define BUT_1_PIO_IDX		28
#define BUT_1_PIO_IDX_MASK (1u << BUT_1_PIO_IDX) 

//PAUSE
#define BUT_2_PIO			PIOC
#define BUT_2_PIO_ID		ID_PIOC
#define BUT_2_PIO_IDX		31
#define BUT_2_PIO_IDX_MASK (1u << BUT_2_PIO_IDX) 

// NEXT
#define BUT_3_PIO			PIOA
#define BUT_3_PIO_ID		ID_PIOA
#define BUT_3_PIO_IDX		19
#define BUT_3_PIO_IDX_MASK (1u << BUT_3_PIO_IDX) 


// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/
TimerHandle_t xTimer;


/* TASKS  */
#define TASK_LCD_STACK_SIZE					  (1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY	              (tskIDLE_PRIORITY)

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
afec_callback_t callback);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
volatile int volume;
int volatile integrate_volume = 1;
int volatile old_volume = 1;
SemaphoreHandle_t xSemaphoreNext;
SemaphoreHandle_t xSemaphoreBack;
SemaphoreHandle_t xSemaphorePause;
SemaphoreHandle_t xSemaphoreVolume;


/** Queue for msg log send data */
QueueHandle_t xQueueADC;

QueueHandle_t xQueueGeneral;

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

int volatile flag_sleep = 0;
void but_callback(void){
	flag_sleep = !flag_sleep;
}

static void next_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreNext, &xHigherPriorityTaskWoken);
}

static void back_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphoreBack, &xHigherPriorityTaskWoken);
}

static void pause_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xSemaphorePause, &xHigherPriorityTaskWoken);
}


static void AFEC_pot_callback(void) {
	int delta_volume;
	int value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	if(value < 256){
		volume = 1;
	}
	else if(value < 512){
		volume = 2;
	}
	else if(value < 768){
		volume = 3;
	}
	else if(value < 1024){
		volume = 4;
	}
	else if(value < 1280){
		volume = 5;
	}
	else if(value < 1536){
		volume = 6;
	}
	else if(value < 1796){
		volume = 7;
	}
	else if(value < 2048){
		volume = 8;
	}
	else if(value < 2304){
		volume = 9;
	}
	else if(value < 2560){
		volume = 10;
	}
	else if(value < 2816){
		volume = 11;
	}
	else if(value < 3072){
		volume = 12;
	}
	else if(value < 3328){
		volume = 13;
	}
	else if(value < 3584){
		volume = 14;
	}
	else if(value < 3840){
		volume = 15;
	}
	else if(value < 4096){
		volume = 16;
	}
	delta_volume = (volume - old_volume) + 16;
	integrate_volume += delta_volume - 16;
	if (delta_volume != 16){
		
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(xQueueADC, &delta_volume, &xHigherPriorityTaskWoken);
	}
	old_volume = volume;
}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
/**
 * \brief Configure the console UART.
 */


static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);

	// Configura Pinos
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP);
	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_handler_set(BUT_PIO,
					BUT_PIO_ID,
					BUT_PIO_PIN_MASK,
					PIO_IT_FALL_EDGE,
					but_callback);
	// Ativa interrup??o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
	// ---------------------------------------------------------------------
	
	
	pio_handler_set(BUT_1_PIO,
					BUT_1_PIO_ID,
					BUT_1_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					back_callback);
	// Ativa interrup??o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_1_PIO, BUT_1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4); // Prioridade 4
	// ---------------------------------------------------------------------
	
	
	pio_handler_set(BUT_2_PIO,
					BUT_2_PIO_ID,
					BUT_2_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					pause_callback);
	// Ativa interrup??o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_2_PIO, BUT_2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_2_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4); // Prioridade 4
	// ---------------------------------------------------------------------
	
	
	
	pio_handler_set(BUT_3_PIO,
					BUT_3_PIO_ID,
					BUT_3_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					next_callback);
	// Ativa interrup??o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_3_PIO, BUT_3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_3_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4); // Prioridade 4
	// ---------------------------------------------------------------------

}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
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
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEaipaipara", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void vTimerCallback(TimerHandle_t xTimer) {
	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

// static void task_feedback(void *pvParameters){
// 	
// 	
// 	
// 	while(1){
// 		char rx;
// 		while(!usart_is_rx_ready(USART_COM))
// 		vTaskDelay(10 / portTICK_PERIOD_MS);
// // 		usart_read(USART_COM, &rx);
// 		if (rx == 'K'){
// 			pin_toggle(LED_PIO, LED_IDX_MASK);
// 			vTaskDelay(100);
// 			pin_toggle(LED_PIO, LED_IDX_MASK);
// 			vTaskDelay(100);
// 		}
// 	}
// }



static void task_general(void *pvParameters){
	
	int msg;
	char to_send = '0';
	char eof = 'X';
	while (1){
		if (xQueueReceive(xQueueGeneral, &(msg), 10)) {
			
			if (msg == 1){
				to_send = '1';
			} else if (msg == 3){
				to_send = '3'; 
			} else if (msg == 5){
				to_send = '5';
			}
			
			
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, to_send);
			
			// envia fim de pacote
			while(!usart_is_tx_ready(USART_COM)) {
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			usart_write(USART_COM, eof);
		}
	}
}

static void task_adc(void *pvParameters) {

  // configura ADC e TC para controlar a leitura
  config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);

  xTimer = xTimerCreate(/* Just a text name, not used by the RTOS
                        kernel. */
                        "Timer",
                        /* The timer period in ticks, must be
                        greater than 0. */
                        100,
                        /* The timers will auto-reload themselves
                        when they expire. */
                        pdTRUE,
                        /* The ID is used to store a count of the
                        number of times the timer has expired, which
                        is initialised to 0. */
                        (void *)0,
                        /* Timer callback */
                        vTimerCallback);
  xTimerStart(xTimer, 0);

  // variável para recever dados da fila
  int delta_volume;
  while (1) {
    if (xQueueReceive(xQueueADC, &(delta_volume), 100)) {
		//int abs_volume = integrate_volume + 16;
		unsigned char byte = (unsigned char) (integrate_volume & 0xFF); // conversão de inteiro para byte
		// envia fim de pacote
 		while(!usart_is_tx_ready(USART_COM)) {
 			vTaskDelay(10 / portTICK_PERIOD_MS);
 		}
		usart_write(USART_COM,'V');
		
		
 		while(!usart_is_tx_ready(USART_COM)) {
 			vTaskDelay(10 / portTICK_PERIOD_MS);
 		}
		 usart_write(USART_COM, byte);
		 
		 while(!usart_is_tx_ready(USART_COM)) {
			 vTaskDelay(10 / portTICK_PERIOD_MS);
		 }
		 usart_write(USART_COM, 'X');
         //printf("ADC: %d \n", byte);
// 		printf("Integrate volume: %d \n", integrate_volume);
    }
  }
}


void task_bluetooth(void) {
	#ifndef DEBUG_SERIAL
	config_usart0();
	#endif
	hc05_init();
	// configura LEDs e Botões
	io_init();
	// Handshake
	//Send Hello and check OK
	char rx = 'R';
	while (rx != 'O') {
		while(!usart_is_tx_ready(USART_COM))
		vTaskDelay(10 / portTICK_PERIOD_MS);
		usart_write(USART_COM, 'H');
		while(!usart_is_rx_ready(USART_COM))
		vTaskDelay(10 / portTICK_PERIOD_MS);
		usart_read(USART_COM, &rx);
	}
	// Send Start
	while(!usart_is_tx_ready(USART_COM))
	vTaskDelay(10 / portTICK_PERIOD_MS);
	usart_write(USART_COM, 'S');

			
	
	// Task não deve retornar.
	while(1) {
		// botao A
		// BACK
		if(xSemaphoreTake(xSemaphoreBack, 1)) {
			BaseType_t xHigherPriorityTaskWoken = pdTRUE;
			int buttA = 1;
			xQueueSendFromISR(xQueueGeneral, &(buttA), &xHigherPriorityTaskWoken);
		} 

		// botao B
		// PAUSE
		if(xSemaphoreTake(xSemaphorePause, 1)) {
			BaseType_t xHigherPriorityTaskWoken = pdTRUE;
			int buttB = 3;
			xQueueSendFromISR(xQueueGeneral, &(buttB), &xHigherPriorityTaskWoken);
		}

		// botao C
		// NEXT
		if(xSemaphoreTake(xSemaphoreNext, 1)) {
			BaseType_t xHigherPriorityTaskWoken = pdTRUE;
			int buttC = 5;
			xQueueSendFromISR(xQueueGeneral, &(buttC), &xHigherPriorityTaskWoken);
		}
		
		// dorme por 150 ms
		vTaskDelay(150 / portTICK_PERIOD_MS);
	}
}


void CAM_pins_init(void){
	// Ativa PIOs
	
	pmc_enable_periph_clk(CAM_1_PIO_ID); // PAUSE
	pmc_enable_periph_clk(CAM_2_PIO_ID); // NEXT
	pmc_enable_periph_clk(CAM_3_PIO_ID); // BACK

	// Configura Pinos
	pio_configure(CAM_1_PIO, PIO_INPUT, CAM_1_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(CAM_2_PIO, PIO_INPUT, CAM_2_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(CAM_3_PIO, PIO_INPUT, CAM_3_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_handler_set(CAM_1_PIO,
					CAM_1_PIO_ID,
					CAM_1_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					pause_callback);
	
	// Ativa interrup??o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(CAM_1_PIO, CAM_1_PIO_IDX_MASK);
	pio_get_interrupt_status(CAM_1_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(CAM_1_PIO_ID);
	NVIC_SetPriority(CAM_1_PIO_ID, 4); // Prioridade 4
	// ---------------------------------------------
	
	pio_handler_set(CAM_2_PIO,
					CAM_2_PIO_ID,
					CAM_2_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					next_callback);
	
	// Ativa interrup??o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(CAM_2_PIO, CAM_2_PIO_IDX_MASK);
	pio_get_interrupt_status(CAM_2_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(CAM_2_PIO_ID);
	NVIC_SetPriority(CAM_2_PIO_ID, 4); // Prioridade 4
	// ---------------------------------------------
	
	pio_handler_set(CAM_3_PIO,
					CAM_3_PIO_ID,
					CAM_3_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					back_callback);
	
	// Ativa interrup??o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(CAM_3_PIO, CAM_3_PIO_IDX_MASK);
	pio_get_interrupt_status(CAM_3_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr?ximo de 0 maior)
	NVIC_EnableIRQ(CAM_3_PIO_ID);
	NVIC_SetPriority(CAM_3_PIO_ID, 4); // Prioridade 4
	// ---------------------------------------------
}





static void init_all(void) {

	sysclk_init();
	board_init();
	configure_console();
	
	CAM_pins_init();
	
	/* Inicializa o button/gate */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, ACTIVATE_IN , but_callback);
	

}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	printf("oi");
	
	init_all();
	
	
	xQueueADC = xQueueCreate(1000, sizeof(int));
	if (xQueueADC == NULL)
	printf("falha em criar a queue xQueueADC \n");
	
 	xQueueGeneral = xQueueCreate(1000, sizeof(int));
 	if (xQueueGeneral == NULL)
 	printf("falha em criar a queue xQueueADC \n");
	
	/* Attempt to create a semaphore. */
	xSemaphoreNext = xSemaphoreCreateBinary();
	if (xSemaphoreNext == NULL)
	printf("falha em criar o semaforo \n");
	
	/* Attempt to create a semaphore. */
	xSemaphoreBack = xSemaphoreCreateBinary();
	if (xSemaphoreBack == NULL)
	printf("falha em criar o semaforo \n");
	

	/* Attempt to create a semaphore. */
	xSemaphorePause = xSemaphoreCreateBinary();
	if (xSemaphorePause == NULL)
	printf("falha em criar o semaforo \n");
	
	/* Attempt to create a semaphore. */
	xSemaphoreVolume = xSemaphoreCreateBinary();
	if (xSemaphoreVolume == NULL)
	printf("falha em criar o semaforo \n");
	
// 	if (xTaskCreate(task_adc, "adc", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
// 		printf("Failed to create adc task\r\n");
// 	}

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	
	xTaskCreate(task_adc, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	
	xTaskCreate(task_general, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	
	//xTaskCreate(task_feedback, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

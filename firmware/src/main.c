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


/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

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

SemaphoreHandle_t xSemaphoreNext;
SemaphoreHandle_t xSemaphoreBack;
SemaphoreHandle_t xSemaphorePause;


/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/


void but_callback(void){
	int i = 1;
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

void TC0_Handler(void){    
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);
	printf("[Debug] TC0 IRQ \n");

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

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

	 // RX - PA2  TX - PA24
	 
	 pio_configure(PIOA, PIO_PERIPH_A, PIO_PA2A_URXD0_MASK, PIO_DEFAULT);
	 pio_configure(PIOA, PIO_PERIPH_A, PIO_PA24A_UTXD0_MASK, PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEagoravai", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/



void task_bluetooth(void) {
	printf("Task Bluetooth started \n");
	
	printf("Inicializando HC05 \n");
	config_usart0();
	hc05_init();

	// configura LEDs e Botões
	io_init();

	char buttonA = '0';
	char buttonB = '2';
	char buttonC = '4';
	char eof = 'X';

	// Task não deve retornar.
	while(1) {
		// botao A
		// BACK
		
		if(xSemaphoreTake(xSemaphoreBack, 1)) {
			buttonA = '1';
		} 
		else {
			buttonA = '0';
		}

		// envia status botão
		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, buttonA);
		
		// envia fim de pacote
		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);
		
		// botao B
		// PAUSE
		if(xSemaphoreTake(xSemaphorePause, 1)) {
			buttonB = '3';
		}
		else {
			buttonB = '2';
		}

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, buttonB);
	
		// envia fim de pacote
		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);
		
		
		// botao C
		// NEXT
		if(xSemaphoreTake(xSemaphoreNext, 1)) {
			buttonC = '5';
		}
		else {
			buttonC = '4';
		}
		
		
		
// 		if (xSemaphoreTake(xSemaphoreBack, 1)){
// 			printf("Back\n");
// 		}
// 		
// 		if (xSemaphoreTake(xSemaphorePause, 1)){
// 			printf("Pause\n");
// 		}
// 		
// 		if (xSemaphoreTake(xSemaphoreNext, 1)){
// 			printf("Next\n");
// 		}

		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, buttonC);
		
		// envia fim de pacote
		while(!usart_is_tx_ready(USART_COM)) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		usart_write(USART_COM, eof);
		// dorme por 500 ms
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
	
// 	if (xTaskCreate(task_adc, "adc", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
// 		printf("Failed to create adc task\r\n");
// 	}

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}

/******************************************************************************
********** Archivo fuente .c para configurar UART e interrupciones  **********
******************************************************************************/

//Prueba romoto
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"


#define LED 2
#define Tx1 4
#define Rx1 5
#define tamBUFFER 1024

// Variables globales
static intr_handle_t manejadorUART0; // Manejador para interrupciones UART0
static intr_handle_t manejadorUART1; // Manejador para interrupciones UART1
uint8_t bufferRx0[256];				// Buffer para guardar lo que se recibe Rx0
uint8_t bufferRx1[256];				// Buffer para guardar lo que se recibe Rx1
uint16_t longRx0;					// Longitud del dato recibido en UART0
uint16_t longRx1;					// Longitud del dato recibido en UART1

/* Rutina de atenci�n de interrupci�n en UART0 ----------------------------------*/
static void IRAM_ATTR intUART0(void *arg){
	uint16_t longFIFORx0,n;
	  uint16_t i=0;
	  char str[80];



	//  status = UART0.int_st.val; // read UART interrupt Status
	  longFIFORx0 = UART0.status.rxfifo_cnt; // read number of bytes in UART buffer
	  n=longFIFORx0;
	  while(longFIFORx0){
		  bufferRx0[i++] = UART0.fifo.rw_byte; // read all bytes
		  longFIFORx0--;
	 }

	 // Luego de recibir los bytes se limpia el estado de la bander de interrupci�n
	 uart_clear_intr_status(UART_NUM_0, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

	// Peque�a prueba que escribe un mensaje recibido en el UART correspondiente

	 uart_write_bytes(UART_NUM_0, (const char*) "Recibido en RX0\n\r", 15);
	 uint8_t m=sprintf(str, "numero de bytes = %d \n\r", n);
	 uart_write_bytes(UART_NUM_0, (const char*) str, m);
	 uart_write_bytes(UART_NUM_0, (const char*) bufferRx0, n);
}

/* Rutina de atenci�n de interrupci�n en UART1 ----------------------------------*/
static void IRAM_ATTR intUART1(void *arg){
	uint16_t longFIFORx1;
	uint16_t i=0;

	//  status = UART0.int_st.val; // read UART interrupt Status
	longFIFORx1 = UART1.status.rxfifo_cnt; // read number of bytes in UART buffer

	while(longFIFORx1){
		bufferRx1[i++] = UART1.fifo.rw_byte; // read all bytes
		longFIFORx1--;
	}

	// Luego de recibir los bytes se limpia el estado de la bander de interrupci�n
	uart_clear_intr_status(UART_NUM_1, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

	// Peque�a prueba que escribe un mensaje recibido en el UART correspondiente
	uart_write_bytes(UART_NUM_1, (const char*) "Recibido en RX1\n\r", 15);
}

void iniciarUART(){
	const int uart0 = UART_NUM_0;
	uart_config_t configUART0 = {
	    .baud_rate = 115200,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    //UART_HW_FLOWCTRL_CTS_RTS,
	    .rx_flow_ctrl_thresh = 122,
	};
	const int uart1 = UART_NUM_1;
	uart_config_t configUART1 = {
	    .baud_rate = 115200,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    //UART_HW_FLOWCTRL_CTS_RTS,
	    .rx_flow_ctrl_thresh = 122,
	};

	// Se configuran los par�metros UART1 y UART0
	uart_param_config(uart0, &configUART0);
	uart_param_config(uart1, &configUART1);

	// Se configuran los terminales para los UART
	uart_set_pin(uart0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_set_pin(uart1, Tx1, Rx1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// Se instalan los controladores UART de la configuraci�n anterior
	uart_driver_install(uart0, tamBUFFER * 2, 0, 0, NULL, 0);
	uart_driver_install(uart1, tamBUFFER * 2, 0, 0, NULL, 0);

	// Rutinas para liberar interrupciones y evitar errores
	uart_isr_free(UART_NUM_0);
	uart_isr_free(UART_NUM_1);

	// Rutinas para registrar las interrupciones correspondientes a UART0 y UART1
	uart_isr_register(UART_NUM_0,intUART0, NULL, 1, &manejadorUART0);
	uart_isr_register(UART_NUM_1,intUART1, NULL, ESP_INTR_FLAG_IRAM, &manejadorUART1);

	// Se deben habilitar las interrupciones necesarias
	uart_enable_rx_intr(UART_NUM_0);
	uart_enable_rx_intr(UART_NUM_1);
}


void tarea1(void *Para){
	gpio_pad_select_gpio(LED);
	gpio_set_direction(LED,GPIO_MODE_OUTPUT);
	while(1){
		gpio_set_level(LED,1);
		vTaskDelay(350/portTICK_PERIOD_MS);
		gpio_set_level(LED,0);
		vTaskDelay(350/portTICK_PERIOD_MS);
	}
}

void app_main(){
	iniciarUART();
	xTaskCreate(tarea1, "led_BLINK", 1024*2, NULL, 5, NULL);
}


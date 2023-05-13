#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "TM4C123GH6PM.h"
#include "macros.h"


#define PortA_IRQn 30

//define a Semaphore handle
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xMutex;
xQueueHandle xQueue;

void sensorButtonInit(void);
void timer0Init(void);
void timer0_Delay(int time);
void motorInit(void);
void limitInit(void);
void buttonsInit(void);
void lockButtonInit(void);


// Initialize timer0
void timer0Init(void)
{
  SYSCTL_RCGCTIMER_R |= 0x01; // Enable Timer 0 module clock
  TIMER0_CTL_R=0x00; // Disable timer
  TIMER0_CFG_R=0x00; // Configure as 32-bit timer
  TIMER0_TAMR_R=0x02; // Periodic mode
  TIMER0_CTL_R=0x03; // Enable timer and configure timer 0A as periodic
}



// Delay function using timer0
void timer0_Delay(int time)
{
  TIMER0_CTL_R=0x00; // Disable timer
  TIMER0_TAILR_R=16000*time-1; // Load the timer
  TIMER0_ICR_R=0x01; // Clear interrupt
  TIMER0_CTL_R |=0x03; // Enable timer and configure timer 0A as periodic
  while((TIMER0_RIS_R & 0x01)==0); // Wait until the timer has counted down
}


// Function executed by the jam task
void jamTask(void* pvParameters) {
       // Take the binary semaphore
    xSemaphoreTake(xBinarySemaphore, 0);
    while (1) {
        // Wait indefinitely for the binary semaphore
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

        // Set motor direction to reverse
        GPIO_PORTF_DATA_R |= (1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
       

        //timer0_Delay(500);

				// Stop motor
			  GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);


   }  
}


// Function executed by the receive queue task
void recieveQueue(void* pvParameters) {
		int Val;
		portBASE_TYPE xStatus;
		const portTickType xTicks=100/portTICK_RATE_MS;
	while(1)
	{
		xStatus=xQueueReceive(xQueue,&Val,xTicks);
		
		if(Val==0)
		{
				GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
		}
		else if(Val==1)
		{
				GPIO_PORTF_DATA_R |= (1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
		}
		else if(Val==2)
		{
				GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R |= (1 << 2);
		}		
		
	}
	
}

void driver(void* pvParameters){
		int Val;
	  portBASE_TYPE xStatus;
	  Val= (int) pvParameters;
		while(1)
		{
			xSemaphoreTake(xMutex,portMAX_DELAY );
			if (GET_BIT(GPIO_PORTD_DATA_R,2)==0){ //pulldown
				Val=1;
				xStatus = xQueueSendToBack(xQueue,&Val,0);
				vTaskDelay(1000); 
				if (GET_BIT(GPIO_PORTD_DATA_R,2)==0) //still pressing then it is manual
					{

						while(GET_BIT(GPIO_PORTD_DATA_R,2)==0);
					}
     
				else if (GET_BIT(GPIO_PORTD_DATA_R,2)==1) // then it will be automatic
				{   
							while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==0 | GET_BIT(GPIO_PORTD_DATA_R,2)==1 | GET_BIT(GPIO_PORTD_DATA_R,3)==1)); 
				}
				Val=0;
				xStatus = xQueueSendToBack(xQueue,&Val,0);
			}
			if (GET_BIT(GPIO_PORTD_DATA_R,3)==0){ //pulldown
					Val=2;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
					vTaskDelay(1000); 
					if (GET_BIT(GPIO_PORTD_DATA_R,3)==0) //still pressing then it is manual
					{

					while(GET_BIT(GPIO_PORTD_DATA_R,3)==0);
					}
     
					else if (GET_BIT(GPIO_PORTD_DATA_R,3)==1) // then it will be automatic
					{   
								while(!(GET_BIT(GPIO_PORTD_DATA_R,7)==0 | GET_BIT(GPIO_PORTD_DATA_R,3)==1 | GET_BIT(GPIO_PORTD_DATA_R,2)==1)); 
					}
					Val=0;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
			
				}
			
				if (GET_BIT(GPIO_PORTF_DATA_R,4)==0){
					
					vTaskPrioritySet(NULL,2);
					
				}
				else
				{
					vTaskPrioritySet(NULL,1);
				}

				xSemaphoreGive(xMutex);
				vTaskDelay(100);
		}
	}

void passenger(void* pvParameters){
	int Val;
	portBASE_TYPE xStatus;
	Val= (int) pvParameters;
	while(1)
	{
		xSemaphoreTake(xMutex,portMAX_DELAY ); // take the mutex, blocking indefinitely if it is not available

if (GET_BIT(GPIO_PORTD_DATA_R,2)==0){ //pulldown
	 Val=1;
   xStatus = xQueueSendToBack(xQueue,&Val,0); // send Val to the back of xQueue, blocking for 0 ticks if the queue is full
   vTaskDelay(1000); 
      if (GET_BIT(GPIO_PORTD_DATA_R,2)==0) //still pressing then it is manual
      {

      while(GET_BIT(GPIO_PORTD_DATA_R,2)==0);
      }
     
      else if (GET_BIT(GPIO_PORTD_DATA_R,2)==1) // then it will be automatic
      {   
            while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==0 | GET_BIT(GPIO_PORTD_DATA_R,2)==1 | GET_BIT(GPIO_PORTD_DATA_R,3)==1)); 
      }
      Val=0;
   xStatus = xQueueSendToBack(xQueue,&Val,0);
		}
if (GET_BIT(GPIO_PORTD_DATA_R,3)==0){ //pulldown
  	 Val=2;
   xStatus = xQueueSendToBack(xQueue,&Val,0);
   vTaskDelay(1000); 
      if (GET_BIT(GPIO_PORTD_DATA_R,3)==0) //still pressing then it is manual
      {

      while(GET_BIT(GPIO_PORTD_DATA_R,3)==0);
      }
     
      else if (GET_BIT(GPIO_PORTD_DATA_R,3)==1) // then it will be automatic
      {   
            while(!(GET_BIT(GPIO_PORTD_DATA_R,7)==0 | GET_BIT(GPIO_PORTD_DATA_R,3)==1 | GET_BIT(GPIO_PORTD_DATA_R,2)==1)); 
      }
        Val=0;
   xStatus = xQueueSendToBack(xQueue,&Val,0);
			
		}
    xSemaphoreGive(xMutex);
		vTaskDelay(100); 
	}
	}

                         /*main function*/
/*------------------------------------------------------------------------*/
int main( void )
{
	  xQueue = xQueueCreate(2,sizeof(int));
	  xMutex = xSemaphoreCreateMutex(); 
    sensorButtonInit();
	  lockButtonInit();
		buttonsInit();
		limitInit();
		motorInit();
	  //timer0Init();
		__ASM("CPSIE i");
		
		vSemaphoreCreateBinary(xBinarySemaphore);
		//xBinarySemaphore = xSemaphoreCreateBinary();
	if( xBinarySemaphore != NULL )
		{
			/* Create the 'handler' task. This is the task that will be synchronized
			with the interrupt. The handler task is created with a high priority to
			ensure it runs immediately after the interrupt exits. In this case a
			priority of 3 is chosen. */
			
			/* Create the 'recieveQueue' task, which receives and handles messages from the queue. 
This task has a priority of 3. */	
			xTaskCreate( jamTask, "jamTask", 200, NULL, 5, NULL );
			
				/* Create the 'passenger' task, which handles passenger button presses. 
This task has a priority of 1. */
			xTaskCreate( passenger, "passenger", 270, NULL, 1, NULL );
			

			/* Create the 'driver' task, which handles driver button presses. 
This task has a priority of 1. */\
			xTaskCreate( driver, "driver", 270, NULL, 1, NULL );
			
			
			xTaskCreate( recieveQueue, "recieveQueue", 200, NULL, 3, NULL );
			/* Start the scheduler so the created tasks start executing. */
			vTaskStartScheduler();
		}

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
    for( ;; );
}

/*------------------------------------------------------------------------*/


/*------------------------------------------------------------------------*/
//Port-F handler
void GPIOA_Handler(void)
{
    //Clear Interrupt Flag
    GPIO_PORTA_ICR_R |= (1<<1);
	
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    /* Clear the software interrupt bit using the interrupt controllers
    Clear Pending register. */
   // mainCLEAR_INTERRUPT();
	  /* 'Give' the semaphore to unblock the task. */
    xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void sensorButtonInit(void)
{
	
	    //Enable Port E
    SYSCTL_RCGCGPIO_R |= 0x01;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 1 in Port E as input
    GPIO_PORTA_DIR_R &= ~(1 << 2);
    GPIO_PORTA_CR_R |= (1 << 2);
    GPIO_PORTA_PUR_R |= (1 << 2);	
    GPIO_PORTA_DEN_R |= (1 << 2);
	
    //Enable Interrupt on PORT E & set priority to 0
	  NVIC_PRI0_R |= (1<<7) | (1<<6) | (1<<5);
    NVIC_EN0_R |= (1<<0);
    
	
    //Configure Interrupt on Pin 1 to detect FALLING edge
    GPIO_PORTA_IM_R &=0;
    GPIO_PORTA_IS_R &= ~(1<<2);
    GPIO_PORTA_IEV_R &= ~(1<<2);
    GPIO_PORTA_ICR_R |= (1<<2);
    GPIO_PORTA_IM_R |= (1<<2);	
	
}

void lockButtonInit(void)
{
	    //Enable Port E
    SYSCTL_RCGCGPIO_R |= 0x20;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 1 in Port E as input
    GPIO_PORTF_DIR_R &= ~(1 << 4);
    GPIO_PORTF_CR_R |= (1 << 4);
    GPIO_PORTF_PUR_R |= (1 << 4);	
    GPIO_PORTF_DEN_R |= (1 << 4);
	
}

void buttonsInit(void)
{
		    //Enable Port E
    SYSCTL_RCGCGPIO_R |= 0x08;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 1 in Port E as input
    GPIO_PORTD_DIR_R &= ~((1 << 0)|(1<<1)|(1<<2)|(1<<3));
    GPIO_PORTD_CR_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);
    GPIO_PORTD_PUR_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);	
    GPIO_PORTD_DEN_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);
	
	
}

void limitInit(void)
{
			    //Enable Port E
    SYSCTL_RCGCGPIO_R |= 0x08;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 1 in Port E as input
    GPIO_PORTD_DIR_R &= ~((1 << 6)|(1<<7));
    GPIO_PORTD_CR_R |= (1 << 6)|(1<<7);
    GPIO_PORTD_PUR_R |= (1 << 6)|(1<<7);	
    GPIO_PORTD_DEN_R |= (1 << 6)|(1<<7);
}


void motorInit(void)
{
			    //Enable Port E
    SYSCTL_RCGCGPIO_R |= 0x20;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 1 in Port E as input
    GPIO_PORTF_DIR_R |= ((1 << 2)|(1<<3));
    GPIO_PORTF_CR_R |= (1 << 2)|(1<<3);
    GPIO_PORTF_DEN_R |= (1 << 2)|(1<<3);

}
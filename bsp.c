#include <stdint.h>
#include <stdio.h>
#include "bsp.h"
#include "TM4C123GH6PM.h" 

#define MIN(A,B) 	(A <= B ? A : B)		// IF A less than/equal to B, return A else B
#define MAX(A,B)	(A >= B ? A : B)		// IF A greater than/equal to B, return A else B

#define PIN0		(1U << 0)
#define PIN1		(1U << 1)
#define PIN2		(1U << 2)		// Buf A
#define PIN3		(1U << 3)		// Buf B
#define PIN4		(1U << 4)		// Buf C

#define CH17PR  	(17 * 4)
#define CH17ALT		(CH17PR + 128)

#define ADC0_SS3_R	((volatile uint32_t *)0x400380A8)

// Message to be sent out through UART
char msg[12];

// Variable to store result from ADC
volatile unsigned int adcV;

// Counter for ADC SS3 Interrupt
int adcCount = 0;

// The circular buffers for the converted results from ADC0 SS3.
#define ADC_BUF_SIZE	50
uint32_t ui32BufA[ADC_BUF_SIZE];
uint32_t ui32BufB[ADC_BUF_SIZE];
uint32_t ui32BufC[ADC_BUF_SIZE];

// The count of ADC buffers filled, one for each buffer.
uint32_t ui32BufACount = 0;
uint32_t ui32BufBCount = 0;
uint32_t ui32BufCCount = 0;

// The circular buffer index for the DMA Ping-Pong buffer configuration
static unsigned long circularBuffer_Ping_idx = 0;
static unsigned long circularBuffer_Pong_idx = 0;

// Control table for uDMA
uint32_t ucControlTable[256] __attribute__ ((aligned(1024)));

void ADC0SS3_Handler(void){		
	// Increment counter to show interrupt has been hit
	adcCount++;

	// Read ADC conversion result from SS3 FIFO
	adcV = ADC0->SSFIFO3;	

	sprintf(msg, "\r\nADC = %3d\r", adcV);
	UART0_printString(msg);	
	
	if((ucControlTable[CH17PR + 2] & 0x7) == 0){		
		switch(circularBuffer_Ping_idx % 3){
			// Process data in C + A
			case 0:
				// Increment counter for primary buffer recent filled 
				ui32BufACount++;

				//Delay(1);
				
				GPIOA->DATA ^= PIN2;
			
				// Setup control table for next buffer used
				ucControlTable[CH17PR] = (uint32_t)ADC0_SS3_R;
				ucControlTable[CH17PR + 1] = (uint32_t)ui32BufC + 0xC4;
				ucControlTable[CH17PR + 2] = 0xAE000317;
				
				break;
			
			// Process data in B + C
			case 1:
				// Increment counter for primary buffer recent filled 
				ui32BufCCount++;
			
				//Delay(1);	
			
				GPIOA->DATA ^= PIN3;
			
				// Setup control table for next buffer used
				ucControlTable[CH17PR] = (uint32_t)ADC0_SS3_R;
				ucControlTable[CH17PR + 1] = (uint32_t)ui32BufB + 0xC4;
				ucControlTable[CH17PR + 2] = 0xAE000317;
			
				break;
			
			// Process data in A + B
			case 2:
				// Increment counter for primary buffer recent filled 
				ui32BufBCount++;
			
				//Delay(1);			
			
				GPIOA->DATA ^= PIN4;
			
				// Setup control table for next buffer used
				ucControlTable[CH17PR] = (uint32_t)ADC0_SS3_R;
				ucControlTable[CH17PR + 1] = (uint32_t)ui32BufA + 0xC4;
				ucControlTable[CH17PR + 2] = 0xAE000317;
				
				break;
		}
		
		// Increment circular buffer ping [PRI] to show a transfer has completed
		circularBuffer_Ping_idx++;
	}
			
	if((ucControlTable[CH17ALT + 2] & 0x7) == 0){
		switch(circularBuffer_Pong_idx % 3){
			// Process data in A + B
			case 0:
				// Increment counter for primary buffer recent filled 
				ui32BufBCount++;					

				//Delay(1);
			
				GPIOA->DATA ^= PIN4;
				
				// Setup control table for next buffer used
				ucControlTable[CH17ALT] = (uint32_t)ADC0_SS3_R;
				ucControlTable[CH17ALT + 1] = (uint32_t)ui32BufA + 0xC4;
				ucControlTable[CH17ALT + 2] = 0xAE000317;
				
				break;
			
			// Process data in C + A
			case 1:
				// Increment counter for primary buffer recent filled 
				ui32BufACount++;				

				//Delay(1);		

				GPIOA->DATA ^= PIN2;
				
				// Setup control table for next buffer used
				ucControlTable[CH17ALT] = (uint32_t)ADC0_SS3_R;
				ucControlTable[CH17ALT + 1] = (uint32_t)ui32BufC + 0xC4;
				ucControlTable[CH17ALT + 2] = 0xAE000317;
				
				break;
			
			// Process data in B + C
			case 2:
				// Increment counter for primary buffer recent filled 
				ui32BufCCount++;			

				//Delay(1);
	
				GPIOA->DATA ^= PIN3;
				
				// Setup control table for next buffer used
				ucControlTable[CH17ALT] = (uint32_t)ADC0_SS3_R;
				ucControlTable[CH17ALT + 1] = (uint32_t)ui32BufB + 0xC4;
				ucControlTable[CH17ALT + 2] = 0xAE000317;
				
				break;
		}
		
		// Increment circular buffer pong [ALT] to show a transfer has completed
		circularBuffer_Pong_idx++;
	}
	
	// Clear conversion flag for ADC Interrupt Status and Clear (ADCISC)
	ADC0->ISC = 8;		

	// Re-enable uDMA
	UDMA->ENASET |= (1U << 17);	
}

void BSP_init(void){
	// Initialize the ADC
	ADC_init();
	
	// Initialize the 32/64-bit Wide General Purpose Timer
	WTimer_init();
	
	// Initialize GPIO
	GPIO_init();
	
	// Initialize UART
	UART_init();
	
	// Initialize and start the uDMA
	DMA_init();
	DMA_start();
}

void GPIO_init(void){
	// Initialize GPIOs to test outputs from buffers
	// Start the clock for GPIO registers + UART
	SYSCTL->RCGCGPIO |= (1U << 0);
	
	// Delay for clock to start
	Delay(1);
	
	// Set direction of GPIO Port A as output 
	GPIOA->DIR |= (PIN2 | PIN3 | PIN4);
	
	// Set digital enable of GPIO Port A as output
	GPIOA->DEN |= (PIN2 | PIN3 | PIN4);	
	
	// Turn on GPIO A Pin 4 to simulate a recent transfer for uDMA [Buffer B recently completed]
	GPIOA->DATA = PIN4;
}

void WTimer_init(void){
	// Enable and provide clock to 32/64-bit wide general-purpose timer R0
	SYSCTL->RCGCWTIMER |= (1U << 0); 
	
	// Delay for clock to start
	Delay(1);
	
	// Disable WTimer before initialization
	WTIMER0->CTL = 0;		

	// Write the GPTM Configuration (GPTMCFG) register
	WTIMER0->CFG = 0x04;			 // 32-bit option
	
	// Write to the GPTM Timer Mode (GPTMTnMR)
	WTIMER0->TAMR = 0x02;			 // Periodic Mode and Down-counter
	
	// Load start value into GPTM Timer n Internal Load Register (GPTMTnILR)
	WTIMER0->TAILR = (500000 * 1);		 // WTimer A interval load value reg (1s) [50,000,000]
	
	// GPTM Timer A Output Trigger Enable
	WTIMER0->CTL |= 0x20;			 
	
	// Enable WTimer after initialization 
	WTIMER0->CTL |= 0x01;			 
}

void ADC_init(void){
	/* Initialize ADC (pg. 815) */
	// NOTE: The PLL must be enabled and programmed to a supported xtal frequency in the RCC register
	// Enable the ADC clock using the RCGCADC register
	SYSCTL->RCGCADC |= (1U << 0);
	
	// Delay for clock to start
	Delay(1);
	
	/* Sample Sequencer Configuration */
	/* Sequencer - # of samples - Depth of FIFO
	**    SS3			1				1
	**    SS2			4				4
	**    SS1			4				4
	**    SS0			8				8
	*/
	// Disable sample sequencer by disabling corresponding ASENn bit in the active sample sequencer
	// (ADCACTSS) register
	ADC0->ACTSS &= ~(1U << 3);
	
	// Configure the trigger event for the sample sequencer in the ADCEMUX register
	// Clear the trigger event
	ADC0->EMUX &= ~0xF000;		

	// Write trigger event
	ADC0->EMUX |= 0x5000;			
	
	// For each sample in the sample sequence, configure the corresponding input source in the ADCSSMUXn register
	ADC0->SSMUX3 = 0;					// Configure for Analog Channel (AN0) for sample sequencer 3 or SS3
	
	// For each sample in the sample sequence, configure the sample control bits in the corresponding nibble
	// in the ADCSSCTLn register. Ensure that the END bit is set, failure to do so causes unpredictable behavior
	ADC0->SSCTL3 |= ((1U << 3) | (1U << 2) | (1U << 1));	// 1st Sample Temp Sensor | Sample Interrupt Enable | End of Sequence

	// Enable ADC Interrupt
	ADC0->IM |= (1U << 3);								// Unmask ADC0 SS3 interrupt
	
	// Enable the sample sequencer logic by setting the corresponding ASENn bit in the ADCACTSS register
	ADC0->ACTSS |= (1U << 3);
}

void UART_init(void){
	/** Initialize and Configure UART Module 1 **/
	// Enable and provide a clock to UART module 1
	SYSCTL->RCGCUART |= (1U << 0);
	
	/* Configure UART0 GPIO Pins */
	// Enable clock for GPIO Port A
	// SYSCTL->RCGCGPIO |= (1U << 0);		// Clock already enabled in GPIO_init()
	
	// Delay for clock to start
	Delay(1);

	/* Configure UART */
	// Disable UART by clearing UARTEN bit	
	UART0->CTL = 0;

	/** Calculate UART Baud Rate **/
	/* UART Baud Rate = (f / 16 * baud divisor) 						
	** f = clock frequency of the UART module = system frequency (xtal) 
	** Example: Baud rate of 115200 (115200 bits per second) 			
	** 			115200 = (16MHz / 16 * baud divisor) 					
	**			Baud divisor = (1000000/115200) = 8.680					
	**			UARTIBRD = 8											
	** Value for fractional baud rate register can be calculated by 	
	** multiplying fraction value with 64 and  adding 0.5				
	**			0.680 * 64 + 0.5 = 44									
	**			UARTFBRD = 44												*/
	
	// Write integer portion of BRD
	UART0->IBRD = 104;	
	
	// Write fraction portion of BRD
	UART0->FBRD = 11;	
	
	// Write the desired serial parameters to the UARTLCRH (UART Line Control)
	// Word Length 8-bit, no parity bit, no FIFO
	UART0->LCRH = (0x3 << 5);
	
	// Configure UART clock source (set for Precision Internal Oscillator Operation (PIOSC))
	UART0->CC = 0x5;
	
	// Enable UART0 module, Bits - RXE (9), TXE (8), UARTEN(0)
	UART0->CTL = (1U << 0) | (1U << 8) | (1U << 9);
	
	// Set GPIO Pin 0 and Pin 1 as digital pins
	GPIOA->DEN |= (PIN0 | PIN1);
	
	// Set GPIO AFSEL bit for Pin 0 and Pin 1
	GPIOA->AFSEL |= (PIN0 | PIN1);
	
	// Set port control register (PCTL) for PA0 and PA1
	GPIOA->PCTL |= (1U << 0) | (1U << 4);
}

void DMA_init(void){
	/** Module Initialization (pg. 599) **/
	// Enable the uDMA clock by using the RCGCDMA register
	SYSCTL->RCGCDMA |= (1U << 0);
	
	// Delay for clock to start
	Delay(1);
	
	// Enable the uDMA controller by setting the MASTEREN bit of the DMA Configuration (DMACFG)
	UDMA->CFG |= (1U << 0);
	
	// Program the location of the channel control table by writing the base address of the table
	// to the DMA Channel Control Base Point (DMACTLBASE). Base address must be aligned on a 1024-byte
	UDMA->CTLBASE = (uint32_t)ucControlTable;
	
	// Configure the Channel Map of 
	UDMA->CHMAP2 = (0x0 << 4);
	
	/** Configure memory to memory transfer **/
	/* Configure the Channel Attributes */
	// Program the DMA Channel Priority Set (DMAPRIOSET) or DMA Channel Priority Clear
	// (DMAPRIOCLR) to set the channel to High priority or Default priority
	UDMA->PRIOCLR = (1U << 17);
	
	// Set the DMA Channel Primary Alternate Clear (DMAALTCLR) to select the primary
	// channel control structure for this transfer
	UDMA->ALTCLR = (1U << 17);

	// Set the DMA Channel Useburst Clear (DMAUSEBURSTCLR) to allow the uDMA controller
	// to respond to burst requests
	UDMA->USEBURSTSET = (1U << 17);
	
	// Set the DMA Channel Request Mask Clear (DMAREQMASKCLR) to allow the uDMA controller
	// to recognize requests for this channel
	UDMA->REQMASKCLR = (1U << 17);
}

void CTLTable_init(void){
	/* Configure the Channel Control Structure */
	// Program the source/destination end pointers to the address of the source/destination buffers
	// PRIMARY CONTROL ADDRESSED
	ucControlTable[CH17PR] = (uint32_t)ADC0_SS3_R;
	ucControlTable[CH17PR + 1] = (uint32_t)ui32BufA + 0xC4;
	ucControlTable[CH17PR + 2] = 0xAE000317;
	
	// SECONDARY CONTROL ADDRESSED
	ucControlTable[CH17ALT] = (uint32_t)ADC0_SS3_R;
	ucControlTable[CH17ALT + 1] = (uint32_t)ui32BufB + 0xC4;
	ucControlTable[CH17ALT + 2] = 0xAE000317;
	
	/* Channel Control Word Config 
	** DMACHCTL          Bits    Value        Description
	** DSTINC            31:30   10           32-bit destination address increment
	** DSTSIZE           29:28   10           32-bit destination data size
	** SRCINC            27:26   11	          No source address increment
	** SRCSIZE           25:24   10           32-bit source data size
	** reserved          23:18   000000       Reserved  
	** ARBSIZE           17:14   0000     	  Arbitrates after 1 transfer
	** XFERSIZE          13:4    0000110001   Transfer count items (50 - 1)
	** NXTUSEBURST       3       0     		  N/A for this transfer type
	** XFERMODE          2:0     001  	  	  Use basic transfer mode
	*/
	
	// 0xC4 = (32-bit int) * (# of items to be transfered [array size]) = (32 * 49) = 196
}

void DMA_start(void){
	// Initialize the control table for the uDMA
	CTLTable_init();
	
	/* Start the transfer */
	// Enable the channel by setting the DMA Channel Enable Set (DMAENASET)
	UDMA->ENASET |= (1U << 17);	
	
	// Enable IRQ17 for ADC0SS3	by setting the Interrupt Set Enable (EN0) register
	NVIC->ISER[0] |= (1U << 17);							
}

void Delay(unsigned long counter){	
	// This function creates a computational delay
	unsigned long i;
	
	for(i = 0; i < counter; i++){
		// do nothing
	}
}

char UART0_Rx(void){
	// Receive data on the RX register of UART0
	char data;
	
	while((UART0-> FR & (1U << 4)) != 0);
	data = UART0->DR;
	
	return (unsigned char) data;
}

void UART0_printChar(unsigned char data){
	// Wait until TxFF (UART Transmit FIFO Full) buffer is not full
	// 0 - transmitter not full
	// 1 - transmit register is full
	while((UART0->FR & (1U << 5)) != 0);
	
	// Transfer the data into the data register of UART0
	UART0->DR = data;
}

void UART0_printString(char * string){
	// Print out each string to UART0 Tx
	while(*string){
		UART0_printChar(*(string++));
	}
}

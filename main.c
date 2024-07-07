#include <stdint.h>
#include <assert.h>

typedef struct {
    uint32_t UE     	: 1;  // USART enable
    uint32_t reserved_0	: 1;
    uint32_t RE     	: 1;  // Receiver enable
    uint32_t TE     	: 1;  // Transmitter enable
    uint32_t IDLEIE 	: 1;  // IDLE interrupt enable
    uint32_t RXNEIE 	: 1;  // RXNE interrupt enable
    uint32_t TCIE   	: 1;  // Transmission complete interrupt enable
    uint32_t TXEIE  	: 1;  // TXE interrupt enable
    uint32_t PEIE   	: 1;  // PE interrupt enable
    uint32_t PS     	: 1;  // Parity selection
    uint32_t PCE    	: 1;  // Parity control enable
    uint32_t WAKE   	: 1;  // Receiver wakeup method
    uint32_t M0     	: 1;  // Word length bit 0
    uint32_t MME    	: 1;  // Mute mode enable
    uint32_t CMIE   	: 1;  // Character match interrupt enable
    uint32_t OVER8  	: 1;  // Oversampling mode
    uint32_t DEDT   	: 5;  // Driver Enable de-assertion time
    uint32_t DEAT   	: 5;  // Driver Enable assertion time
    uint32_t RTOIE  	: 1;  // Receiver timeout interrupt enable
    uint32_t EOBIE  	: 1;  // End of Block interrupt enable
    uint32_t M1     	: 1;  // Word length bit 1
    uint32_t reserved 	: 3;  // Reserved, must be kept at reset value
} USART_CR1_Bits;// res value in each bit = 0

typedef union {
    uint32_t reg;
    USART_CR1_Bits bits;
} USART_CR1_Reg;

typedef struct {
    uint32_t RESERVED1 : 4;  // Reserved
    uint32_t ADDM7     : 1;  // 7-bit Address Detection/4-bit Address Detection
    uint32_t LBDL      : 1;  // LIN break detection length
    uint32_t LBDIE     : 1;  // LIN break detection interrupt enable
    uint32_t RESERVED2 : 1;  // Reserved
    uint32_t LBCL      : 1;  // Last bit clock pulse
    uint32_t CPHA      : 1;  // Clock phase
    uint32_t CPOL      : 1;  // Clock polarity
    uint32_t CLKEN     : 1;  // Clock enable
    uint32_t STOP      : 2;  // STOP bits
    uint32_t LINEN     : 1;  // LIN mode enable
    uint32_t SWAP      : 1;  // Swap TX/RX pins
    uint32_t RXINV     : 1;  // RX pin active level inversion
    uint32_t TXINV     : 1;  // TX pin active level inversion
    uint32_t DATAINV   : 1;  // Binary data inversion
    uint32_t MSBFIRST  : 1;  // Most significant bit first
    uint32_t ABREN     : 1;  // Auto baud rate enable
    uint32_t ABRMOD0   : 1;  // Auto baud rate mode [0]
    uint32_t ABRMOD1   : 1;  // Auto baud rate mode [1]
    uint32_t RTOEN     : 1;  // Receiver timeout enable
    uint32_t ADD0_3    : 4;  // Address of the USART node [3:0]
    uint32_t ADD4_7    : 4;  // Address of the USART node [7:4]
} USART_CR2_Bits;

typedef union {
    uint32_t reg;
    USART_CR2_Bits bits;
} USART_CR2_Reg;

typedef struct {
    uint32_t EIE        : 1;  // Bit 0: Error interrupt enable
    uint32_t IREN       : 1;  // Bit 1: IrDA mode enable
    uint32_t IRLP       : 1;  // Bit 2: IrDA low-power
    uint32_t HDSEL      : 1;  // Bit 3: Half-duplex selection
    uint32_t NACK       : 1;  // Bit 4: Smartcard NACK enable
    uint32_t SCEN       : 1;  // Bit 5: Smartcard mode enable
    uint32_t DMAR       : 1;  // Bit 6: DMA enable receiver
    uint32_t DMAT       : 1;  // Bit 7: DMA enable transmitter
    uint32_t RTSE       : 1;  // Bit 8: RTS enable
    uint32_t CTSE       : 1;  // Bit 9: CTS enable
    uint32_t CTSIE      : 1;  // Bit 10: CTS interrupt enable
    uint32_t ONEBIT     : 1;  // Bit 11: One sample bit method enable
    uint32_t OVRDIS     : 1;  // Bit 12: Overrun Disable
    uint32_t DDRE       : 1;  // Bit 13: DMA Disable on Reception Error
    uint32_t DEM        : 1;  // Bit 14: Driver enable mode
    uint32_t DEP        : 1;  // Bit 15: Driver enable polarity selection
    uint32_t RESERVED1  : 1;  // Bit 16: Reserved
    uint32_t SCARCNT0   : 1;  // Bit 17: Smartcard auto-retry count bit 0
    uint32_t SCARCNT1   : 1;  // Bit 18: Smartcard auto-retry count bit 1
    uint32_t SCARCNT2   : 1;  // Bit 19: Smartcard auto-retry count bit 2
    uint32_t RESERVED2  : 12;  // Bits 20-22: Reserved
} USART_CR3_Bits;

typedef union {
    uint32_t reg;
    USART_CR3_Bits bits;
} USART_CR3_Reg;


typedef struct {
    uint32_t PE         : 1;  // Bit 0: Parity error
    uint32_t FE         : 1;  // Bit 1: Framing error
    uint32_t NF         : 1;  // Bit 2: Noise detected flag
    uint32_t ORE        : 1;  // Bit 3: Overrun error
    uint32_t IDLE       : 1;  // Bit 4: IDLE line detected
    uint32_t RXNE       : 1;  // Bit 5: Read data register not empty
    uint32_t TC         : 1;  // Bit 6: Transmission complete
    uint32_t TXE        : 1;  // Bit 7: Transmit data register empty
    uint32_t LBDF       : 1;  // Bit 8: LIN break detection flag
    uint32_t CTSIF      : 1;  // Bit 9: CTS interrupt flag
    uint32_t CTS        : 1;  // Bit 10: CTS flag
    uint32_t RTOF       : 1;  // Bit 11: Receiver timeout
    uint32_t EOBF       : 1;  // Bit 12: End of block flag
    uint32_t RESERVED1  : 1;  // Bit 13: Reserved
    uint32_t ABRE       : 1;  // Bit 14: Auto baud rate error
    uint32_t ABRF       : 1;  // Bit 15: Auto baud rate flag
    uint32_t BUSY       : 1;  // Bit 16: Busy flag
    uint32_t CMF        : 1;  // Bit 17: Character match flag
    uint32_t SBKF       : 1;  // Bit 18: Send break flag
    uint32_t RWU        : 1;  // Bit 19: Receiver wakeup from Mute mode
    uint32_t RESERVED2  : 1;  // Bit 20: Reserved
    uint32_t TEACK      : 1;  // Bit 21: Transmit enable acknowledge flag
    uint32_t RESERVED3  : 10;  // Bits 22-31: Reserved
} USART_ISR_Bits;// read-only

typedef union {
    uint32_t reg;
    USART_ISR_Bits bits;
} USART_ISR_Reg;

typedef struct {
    uint32_t PECF   : 1;  // Parity error clear flag
    uint32_t FECF   : 1;  // Framing error clear flag
    uint32_t NCF    : 1;  // Noise detected clear flag
    uint32_t ORECF  : 1;  // Overrun error clear flag
    uint32_t IDLECF : 1;  // Idle line detected clear flag
    uint32_t RES1   : 1;  // Reserved, must be kept at reset value
    uint32_t TCCF   : 1;  // Transmission complete clear flag
    uint32_t RES2   : 1;  // Reserved, must be kept at reset value
    uint32_t LBDCF  : 1;  // LIN break detection clear flag
    uint32_t CTSCF  : 1;  // CTS clear flag
    uint32_t RES3   : 1;  // Reserved, must be kept at reset value
    uint32_t RTOCF  : 1;  // Receiver timeout clear flag
    uint32_t EOBCF  : 1;  // End of block clear flag
    uint32_t RES4   : 4;  // Reserved, must be kept at reset value
    uint32_t CMCF   : 1;  // Character match clear flag
    uint32_t RES5   : 14;  // Reserved, must be kept at reset value
} USART_ICR_TypeDef;

typedef union {
    uint32_t reg;
    USART_ICR_TypeDef bits;
} USART_IСR_Reg;

typedef struct{
	uint32_t	value		: 9;
	uint32_t	reserved	: 23;
} USART_RDR;


typedef struct {
    uint32_t ABRRQ		: 1;  // Bit 0: Auto baud rate request
    uint32_t SBKRQ		: 1;  // Bit 1: Send break request
    uint32_t MMRQ		: 1;  // Bit 2: Mute mode request
    uint32_t RXFRQ		: 1;  // Bit 3: Receive data flush request
    uint32_t TXFRQ		: 1;  // Bit 4: Transmit data flush request
    uint32_t Reserved	: 27; // Bits 5-31: Reserved
} USART_RQR_TypeDef;

typedef union {
    uint32_t reg;
    USART_RQR_TypeDef bits;
} USART_RQR_Reg;

#define   	__I     volatile const      /*!< Defines 'read only' permissions */
#define     __O     volatile            /*!< Defines 'write only' permissions */
#define     __IO    volatile            /*!< Defines 'read / write' permissions */

typedef struct {
    __IO	USART_CR1_Reg CR1; /*!< USART Control register 1,                 Address offset: 0x00 */
    __IO	USART_CR2_Reg CR2; /*!< USART Control register 2,                 Address offset: 0x04 */
    __IO	USART_CR3_Reg CR3; /*!< USART Control register 3,                 Address offset: 0x08 */
    __IO	uint32_t BRR;      /*!< USART Baud rate register,                 Address offset: 0x0C */
    __IO	uint32_t GTPR;     /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    __IO	uint32_t RTOR;     /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    __O		USART_RQR_Reg RQR; /*!< USART Request register,                   Address offset: 0x18 */
    __I		USART_ISR_Reg ISR; /*!< USART Interrupt and status register,      Address offset: 0x1C */
    __O		USART_IСR_Reg ICR; /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    __I		USART_RDR RDR;     /*!< USART Receive Data register,              Address offset: 0x24 */
    __O		uint32_t TDR;      /*!< USART Transmit Data register,             Address offset: 0x28 */
} USART_TypeDef;

#define PERIPH_BASE             0x40000000UL /*!< Base address of : AHB/ABP Peripherals  */
#define APB1PERIPH_BASE         PERIPH_BASE
#define AHB1PERIPH_BASE         (PERIPH_BASE + 0x20000UL)
#define USART3_BASE             (APB1PERIPH_BASE + 0x4800UL)
#define USART3                  ((USART_TypeDef *) USART3_BASE)
#define RCC_BASE                (AHB1PERIPH_BASE + 0x3800UL)
#define RCC_AHB1ENR             (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB1ENR             (*(volatile uint32_t *)(RCC_BASE + 0x40UL))
#define GPIOD_BASE              (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOD_MODER             (*(volatile uint32_t *) GPIOD_BASE)
#define GPIOD_AFR_1             (*(volatile uint32_t *)(GPIOD_BASE + 0x24))


#define GPIO_MODER_MODER8_Pos            (16U)
#define GPIO_MODER_MODER8_Msk            (0x3UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00030000 */
#define GPIO_MODER_MODER8                GPIO_MODER_MODER8_Msk
#define GPIO_MODER_MODER8_1              (0x2UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00020000 */

#define GPIO_MODER_MODER9_Pos            (18U)
#define GPIO_MODER_MODER9_Msk            (0x3UL << GPIO_MODER_MODER9_Pos)       /*!< 0x000C0000 */
#define GPIO_MODER_MODER9                GPIO_MODER_MODER9_Msk
#define GPIO_MODER_MODER9_1              (0x2UL << GPIO_MODER_MODER9_Pos)       /*!< 0x00080000 */


#define RCC_APB1ENR_USART3EN_Pos           (18U)
#define RCC_APB1ENR_USART3EN_Msk           (0x1UL << RCC_APB1ENR_USART3EN_Pos)  /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN               RCC_APB1ENR_USART3EN_Msk

#define SET		0x1
#define RESET	0x0

#define GPIOD_EN 0x8
#define AF07	 0x07

void GPIO_UART_init(){
	RCC_AHB1ENR	&= ~GPIOD_EN;
	RCC_AHB1ENR	|= GPIOD_EN;
	GPIOD_MODER	|= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOD_AFR_1	|= GPIOD_AFR_1 |= AF07 | (AF07 << 4); // Set alternate function 7 (AF7) for PD8 and PD9
}

enum stop_bits { _1_STB = 0, _2_STB = 0b10 };

enum mode_frame_format { _SB_8Databit_STB = 0, _SB_7Databit_PB_STB, _SB_9Databit_STB, _SB_8Databit_PB_STB, _SB_7Databit_STB, _SB_6Databit_PB_STB };
const uint8_t M0_values[] 	= {0, 0, 0, 0, 1, 1};
const uint8_t M1_values[] 	= {0, 0, 1, 1, 0, 0};
const uint8_t PCE_values[] 	= {0, 1, 0, 1, 0, 1};

enum parity { even = 0, odd };

static const enum mode_frame_format mode_frame = _SB_8Databit_PB_STB;

void UART_init(uint32_t baudrate){
	GPIO_UART_init();
	// enable uart_clock
	RCC_APB1ENR |= RCC_APB1ENR_USART3EN;

	USART3->CR1.reg	&= ~0x1;
	USART3->CR1.bits.OVER8	= 0;
	uint32_t usart_div = 16000000 / baudrate;
	assert(usart_div > 65535);// bit fields resolution for BRR = 16 bit = 65535
	USART3->BRR = usart_div;

	USART3->CR1.bits.M0 	= M0_values[mode_frame];
	USART3->CR1.bits.M1 	= M1_values[mode_frame];
	USART3->CR1.bits.PCE	= PCE_values[mode_frame];

	USART3->CR1.bits.PS = even;
	USART3->CR2.bits.STOP = _1_STB;

	USART3->RTOR = 0;				// DISABLE Receiver timeout
	USART3->CR3.bits.OVRDIS = RESET;// ENABLE OVERRUN DETECTION
	USART3->CR3.bits.ONEBIT = RESET;// ENABLE Noise detection

	USART3->CR1.bits.UE = SET;
	USART3->CR1.bits.TE = SET;
	USART3->CR1.bits.RE = SET;
}

void UART_write(uint8_t x)
{
	USART3->TDR =(x);
	while(!(USART3->ISR.bits.TC)){;}
}

uint8_t UART_Read(){
	while (!(USART3->ISR.bits.RXNE) && ( ! ( USART3->ISR.bits.BUSY ))) {;}
	return	( mode_frame == _SB_8Databit_STB )		? ( USART3->RDR.value ) :
			( mode_frame == _SB_7Databit_PB_STB )	? ( USART3->RDR.value & 0x7F ):
			( mode_frame == _SB_9Databit_STB )		? ( USART3->RDR.value ) : // 1 high bit slice ERROR!
			( mode_frame == _SB_8Databit_PB_STB )	? ( USART3->RDR.value ) :
			( mode_frame == _SB_7Databit_STB )		? ( USART3->RDR.value ) :
			( mode_frame == _SB_6Databit_PB_STB )	? ( USART3->RDR.value & 0x3F ) : USART3->RDR.value;
}


__attribute__((weak)) void doSomethingForSheduleError();

/*
 * Common logic this function , if error detected - reset flag
 */
void UART_Check_Error(){
	if(USART3->ISR.bits.ORE)
		USART3->ICR.bits.ORECF	= SET;

	if(USART3->ISR.bits.NF)
		USART3->ICR.bits.NCF	= SET;

	if(USART3->ISR.bits.FE)
		USART3->ICR.bits.FECF	= SET;

	if(USART3->ISR.bits.PE)
		USART3->ICR.bits.PECF	= SET;

	if(USART3->ISR.bits.IDLE)
		USART3->ICR.bits.IDLECF	= SET;

	if(USART3->ISR.bits.RXNE)
		USART3->RQR.bits.RXFRQ	= SET;

	doSomethingForSheduleError();
}


int main(void)
{

	//Configure GPIOD PD8 as TX pin
	UART_init(115200);


	while(1)
	{
		UART_write('a');
		UART_write('\r');
		UART_write('\n');

		for(volatile int i=0;i<100000;i++);
	}


}

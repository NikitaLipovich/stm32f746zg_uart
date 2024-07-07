# Problem Statement:
The function prototype given in the task imposes limitations on returning error statuses. This restriction affects the range of returnable values, leading to the conclusion that returning an error status is not feasible. Consequently, if returning an error status is not possible, there is no point in implementing error state handling. This results in non-functional prototype code.

# Solution:
If this restriction did not exist, there are three solutions, but the most typical and flexible one is as follows:
In each potentially infinite loop, create a counter that, upon reaching a certain number of ticks or after a specific time in milliseconds, interrupts the wait or pauses execution, returning from the function with a TIMEOUT error code.

Since we are writing the task without an operating system, just on registers, the available solution will be based on a local counter. Otherwise, we could use either a hardware timer or the state of an operating system timer.

# Pseudocode:
The pseudocode below is meant to convey the idea, so any syntax errors are intentional.

```C
	const uint32_t freq = GetSysClock();
	#define TIMEOUT_ERR	0xFFFF

	uint16_t UART_Read() {
		for(uint32_t timeout = freq / 10; !(USART3->ISR.bits.RXNE) && ( ! ( USART3->ISR.bits.BUSY )); timeout--)
			if(!timeout)
				return TIMEOUT_ERR;

		return ( mode_frame == _SB_8Databit_STB ) ? ( USART3->RDR.value );
	}
```
Well, accordingly, since we have returned states, now when there is an error, we can call the error handler function.
```C
  uint16_t c = UART_Read();
  if(c == TIMEOUT_ERR)
    UART_Check_Error();
```

## Explanations on the method of implementation.

Due to the fact that there were no requirements for the format of the solution and my questions were not answered by email, the problem was solved by declaring structures with bit fields. This allows for greater flexibility, faster development speed, and less chance of error when calculating values ​​and writing prototyping code.

Due to a limitation in the function prototype
```C
uint8_t UART_Read();
```
when the transfer format is _SB_9Databit_STB. The return value from the function will not be complete. return 9 bit data not 8bit.

If this task were implemented in C++, the code could look much better, because Using classes, you could create your own local functions to adjust the register. This would make the code more readable.
## EXAMPLE
```C++
#include <cstdint>
#include <cassert>

// Example bit-field structure for the CR1 register
struct USART_CR1_Bits {
    uint32_t UE      : 1;  // USART enable
    uint32_t reserved_0 : 1;
    uint32_t RE      : 1;  // Receiver enable
    uint32_t TE      : 1;  // Transmitter enable
    uint32_t IDLEIE  : 1;  // IDLE interrupt enable
    uint32_t RXNEIE  : 1;  // RXNE interrupt enable
    uint32_t TCIE    : 1;  // Transmission complete interrupt enable
    uint32_t TXEIE   : 1;  // TXE interrupt enable
    uint32_t PEIE    : 1;  // PE interrupt enable
    uint32_t PS      : 1;  // Parity selection
    uint32_t PCE     : 1;  // Parity control enable
    uint32_t WAKE    : 1;  // Receiver wakeup method
    uint32_t M0      : 1;  // Word length bit 0
    uint32_t MME     : 1;  // Mute mode enable
    uint32_t CMIE    : 1;  // Character match interrupt enable
    uint32_t OVER8   : 1;  // Oversampling mode
    uint32_t DEDT    : 5;  // Driver Enable de-assertion time
    uint32_t DEAT    : 5;  // Driver Enable assertion time
    uint32_t RTOIE   : 1;  // Receiver timeout interrupt enable
    uint32_t EOBIE   : 1;  // End of Block interrupt enable
    uint32_t M1      : 1;  // Word length bit 1
    uint32_t reserved : 3;  // Reserved, must be kept at reset value
};

class USART_CR1 {
public:
    USART_CR1(uintptr_t addr) : addr(addr) {}
    uintptr_t addr;

    void set_UE(bool enable) {
        volatile USART_CR1_Bits* bits = reinterpret_cast<volatile USART_CR1_Bits*>(addr); // it can be how private variable it's depends from design
        bits->UE = enable ? 1 : 0;
    }
    
    // Similarly, you can add methods for other bits
};

// Define base addresses for peripherals
#define PERIPH_BASE             0x40000000UL /*!< Base address of : AHB/ABP Peripherals  */
#define APB1PERIPH_BASE         PERIPH_BASE
#define USART3_BASE             (APB1PERIPH_BASE + 0x4800UL)
#define USART3_CR1_ADDR         (USART3_BASE + 0x00UL)

// Main USART class
class USART {
public:
    USART_CR1 CR1;
    // Declare other registers as class members

    USART(uintptr_t base_addr) : CR1(base_addr) {}

    void init(uint32_t baudrate) {
        // Example initialization
        CR1.set_UE(false);
        // Additional initialization
    }

    void write(uint8_t data) {
        // Example data write
    }

    uint8_t read() {
        // Example data read
    }

    void check_error() {
        // Example error checking
    }
};

int main() {
    USART usart(USART3_BASE);
    usart.init(115200);

    while (true) {
        usart.write('a');
        usart.write('\r');
        usart.write('\n');
        for (volatile int i = 0; i < 100000; i++);
    }

    return 0;
}
```

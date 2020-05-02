#include "main.h"

// Default system clock frequency.
uint32_t core_clock_hz = 4000000;
// SysTick delay counter.
volatile uint32_t systick = 0;

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

// Delay for a specified number of milliseconds.
// TODO: Prevent rollover bug on the 'systick' value.
void delay_ms( uint32_t ms ) {
  // Calculate the 'end of delay' tick value, then wait for it.
  uint32_t next = systick + ms;
  while ( systick < next ) { __asm__( "WFI" ); }
}

// Override the 'write' clib method to implement 'printf' over UART.
int _write( int handle, char* data, int size ) {
  int count = size;
  while( count-- ) {
    while( !( USART2->ISR & USART_ISR_TXE ) ) {};
    USART2->TDR = *data++;
  }
  return size;
}

// Send a byte of data over SPI (blocking)
void spi_w8( SPI_TypeDef *SPIx, uint8_t dat ) {
  // Wait for 'transmit buffer empty' bit to be set, then send data.
  while ( !( SPIx->SR & SPI_SR_TXE ) ) {};
  *( uint8_t* )&( SPIx->DR ) = dat;
  // Dummy receive.
  while( !( SPIx->SR & SPI_SR_RXNE ) ) {};
  uint8_t dr = *( uint8_t* )&( SPIx->DR );
}

// Receive a byte of data over SPI (blocking)
uint8_t spi_r8( SPI_TypeDef *SPIx ) {
  // Transmit a dummy byte once the peripheral is ready.
  while ( !( SPIx->SR & SPI_SR_TXE ) ) {};
  *( uint8_t* )&( SPIx->DR ) = 0xFF;
  // Wait to receive a byte of data, then return it.
  while( !( SPIx->SR & SPI_SR_RXNE ) ) {};
  return *( uint8_t* )&( SPIx->DR );
}

// Logic to read a register from the RFM95W module.
uint8_t read_rf_reg( uint8_t addr ) {
  // Assert CS signal.
  GPIOA->ODR &= ~( 0x1 << 11 );
  // Write the address byte with the read/write bit set to 0.
  spi_w8( SPI1, addr & 0x7F );
  // Receive the current register contents from the module.
  uint8_t rx = spi_r8( SPI1 );
  // Release CS signal, then return the received value.
  GPIOA->ODR |=  ( 0x1 << 11 );
  return rx;
}

// Logic to write a register in the RFM95W module.
void write_rf_reg( uint8_t addr, uint8_t data ) {
  // Assert CS signal.
  GPIOA->ODR &= ~( 0x1 << 11 );
  // Write the address with the read/write bit set to 1.
  spi_w8( SPI1, addr | 0x80 );
  // Write the byte of data.
  spi_w8( SPI1, data );
  // Release CS signal.
  GPIOA->ODR |=  ( 0x1 << 11 );
}

/**
 * Main program.
 */
int main(void) {
  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit. (Required for 'printf' formatting)
  SCB->CPACR    |=  ( 0xF << 20 );

  // Use the 16MHz HSI oscillator for the system core clock.
  RCC->CR |=  ( RCC_CR_HSION );
  while ( !( RCC->CR & RCC_CR_HSIRDY ) ) {};
  RCC->CFGR &= ~( RCC_CFGR_SW );
  RCC->CFGR |=  ( RCC_CFGR_SW_HSI );
  while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI ) {};
  core_clock_hz = 16000000;

  // Setup the SysTick peripheral to generate 1ms ticks.
  SysTick_Config( core_clock_hz / 1000 );

  // Enable peripheral clocks: GPIOA, GPIOB, SPI1, USART2.
  RCC->APB1ENR1 |= ( RCC_APB1ENR1_USART2EN );
  RCC->APB2ENR  |= ( RCC_APB2ENR_SPI1EN );
  RCC->AHB2ENR  |= ( RCC_AHB2ENR_GPIOAEN |
                     RCC_AHB2ENR_GPIOBEN );

  // UART TX pin setup (AF7).
  GPIOA->MODER    &= ~( 0x3 << ( 2 * 2 ) );
  GPIOA->MODER    |=  ( 0x2 << ( 2 * 2 ) );
  GPIOA->OTYPER   &= ~( 0x1 << 2 );
  GPIOA->OSPEEDR  &= ~( 0x3 << ( 2 * 2 ) );
  GPIOA->OSPEEDR  |=  ( 0x2 << ( 2 * 2 ) );
  GPIOA->AFR[ 0 ] &= ~( 0xF << ( 2 * 4 ) );
  GPIOA->AFR[ 0 ] |=  ( 0x7 << ( 2 * 4 ) );

  // SPI pins setup.
  // PA11: software-controlled CS pin.
  GPIOA->MODER    &= ~( 0x3 << ( 11 * 2 ) );
  GPIOA->MODER    |=  ( 0x1 << ( 11 * 2 ) );
  GPIOA->OTYPER   &= ~( 0x1 << 11 );
  GPIOA->OSPEEDR  &= ~( 0x3 << ( 11 * 2 ) );
  GPIOA->OSPEEDR  |=  ( 0x1 << ( 11 * 2 ) );
  GPIOA->ODR      |=  ( 0x1 << 11 );
  // PB3, PB4, PB5: hardware-controlled SPI SCK/MISO/MOSI pins (AF5).
  GPIOB->MODER    &= ~( ( 0x3 << ( 3 * 2 ) ) |
                        ( 0x3 << ( 4 * 2 ) ) |
                        ( 0x3 << ( 5 * 2 ) ) );
  GPIOB->MODER    |=  ( ( 0x2 << ( 3 * 2 ) ) |
                        ( 0x2 << ( 4 * 2 ) ) |
                        ( 0x2 << ( 5 * 2 ) ) );
  GPIOB->OSPEEDR  &= ~( ( 0x3 << ( 3 * 2 ) ) |
                        ( 0x3 << ( 4 * 2 ) ) |
                        ( 0x3 << ( 5 * 2 ) ) );
  GPIOB->OSPEEDR  |=  ( ( 0x2 << ( 3 * 2 ) ) |
                        ( 0x2 << ( 4 * 2 ) ) |
                        ( 0x2 << ( 5 * 2 ) ) );
  GPIOB->AFR[ 0 ] &= ~( ( 0xF << ( 3 * 4 ) ) |
                        ( 0xF << ( 4 * 4 ) ) |
                        ( 0xF << ( 5 * 4 ) ) );
  GPIOB->AFR[ 0 ] |=  ( ( 0x5 << ( 3 * 4 ) ) |
                        ( 0x5 << ( 4 * 4 ) ) |
                        ( 0x5 << ( 5 * 4 ) ) );

  // UART setup: 115200 baud, transmit only.
  USART2->BRR  = ( core_clock_hz / 115200 );
  USART2->CR1 |= ( USART_CR1_TE | USART_CR1_UE );

  // SPI setup: standard host mode.
  SPI1->CR1 |= ( SPI_CR1_SSM |
                 SPI_CR1_SSI |
                 SPI_CR1_MSTR |
                 SPI_CR1_CPOL |
                 SPI_CR1_CPHA );
  // Set RX FIFO threshold to one byte instead of two.
  // Without this, the RXNE flag will not get set for single bytes.
  SPI1->CR2 |= ( SPI_CR2_FRXTH );
  SPI1->CR1 |= ( SPI_CR1_SPE );

  // Debug: wait 5 seconds before starting, to give a meatbag time
  // to connect to the UART output after plugging the device in.
  delay_ms( 5000 );

  uint8_t reg;
  printf( "Configure RFM95W for LoRa communication...\r\n" );
  // Set 'RegOpMode' to sleep mode / high-frequency.
  write_rf_reg( 0x01, 0x00 );
  // Verify the new 'RegOpMode'.
  reg = read_rf_reg( 0x01 );
  printf( "RegOpMode: 0x%02X\r\n", reg );
  // Set 'RegOpMode' to LoRa communication while in sleep mode.
  write_rf_reg( 0x01, 0x80 );
  // Verify the new 'RegOpMode'.
  reg = read_rf_reg( 0x01 );
  printf( "RegOpMode: 0x%02X\r\n", reg );
  // Put the module back into 'standby' mode.
  write_rf_reg( 0x01, reg | 0x01 );
  // Wait for the module to finish exiting 'sleep' mode.
  while ( reg != 0x81 ) {
    reg = read_rf_reg( 0x01 );
    printf( "RegOpMode: 0x%02X\r\n", reg );
  }

  // Set carrier frequency. With the default 32MHz crystal, each bit
  // is worth 61.035Hz. North 'muricans should use 0xE4C026 (~915MHz)
  write_rf_reg( 0x06, 0xE4 );
  write_rf_reg( 0x07, 0xC0 );
  write_rf_reg( 0x08, 0x26 );

  // Clear all interrupt request flags.
  write_rf_reg( 0x12, 0xFF );

  // 'Receiver' half of the test program.
  #if ( MODE == RX_MODE )
    // Set the base FIFO address to 0.
    write_rf_reg( 0x0F, 0x00 );
    // Enter 'continuous receive' mode.
    reg = read_rf_reg( 0x01 );
    reg &= 0xF8;
    reg |= 0x05;
    write_rf_reg( 0x01, reg );
    // Receive data loop.
    while( 1 ) {
      // Wait for a 'receive done' or 'CRC error' interrupt request.
      // (There is no 'receive timeout' in continuous mode)
      reg = read_rf_reg( 0x12 );
      while( ( reg & 0x60 ) == 0 ) {
        reg = read_rf_reg( 0x12 );
        printf( "ReqIrqFlag: 0x%02X\r\n", reg );
        // Check receive status every half-second.
        delay_ms( 500 );
      }
      // Clear all interrupt request flags.
      write_rf_reg( 0x12, 0xFF );
      // Set FIFO address to the start of received data.
      reg = read_rf_reg( 0x10 );
      printf( "RX@: 0x%02X\r\n", reg );
      write_rf_reg( 0x0D, reg );
      // Check how many bytes of data were received.
      uint8_t rx_len = read_rf_reg( 0x13 );
      printf( "RXL: 0x%02X\r\n", rx_len );
      // Read data out of the FIFO and print it.
      for ( uint8_t i = 0; i < rx_len; ++i ) {
        reg = read_rf_reg( 0x00 );
        printf( "R%d: 0x%02X\r\n", i, reg );
      }
    }

  // 'Transmitter' half of the test program.
  #elif ( MODE == TX_MODE )
    // Set the base FIFO address to 0.
    write_rf_reg( 0x0E, 0x00 );
    // Set payload length to 4 bytes.
    write_rf_reg( 0x22, 0x04 );
    // Transmit loop: send '0x01234567' forever. Hey, it's about as
    // useful a message as anything I ever say...
    while( 1 ) {
      // Reset FIFO address pointer.
      write_rf_reg( 0x0D, 0x00 );
      // Set data in the FIFO.
      write_rf_reg( 0x00, 0x01 );
      write_rf_reg( 0x00, 0x23 );
      write_rf_reg( 0x00, 0x45 );
      write_rf_reg( 0x00, 0x67 );
      // Enter 'transmit' mode.
      reg = read_rf_reg( 0x01 );
      reg &= 0xF8;
      reg |= 0x03;
      write_rf_reg( 0x01, reg );
      // Wait for the device to finish and enter 'standby' mode.
      reg = read_rf_reg( 0x01 );
      while( ( reg & 0x07 ) != 0x01 ) {
        reg = read_rf_reg( 0x01 );
        delay_ms( 100 );
      }
      // Read the interrupt status, then clear the 'TX done' bit.
      reg = read_rf_reg( 0x12 );
      printf( "ReqIrqFlag: 0x%02X\r\n", reg );
      write_rf_reg( 0x12, 0x08 );
      // Sleep for 10 seconds before transmitting again.
      printf( "TX done.\r\n" );
      delay_ms( 10000 );
    }
  #endif

  // These end blocks should never be reached.
  while ( 1 ) {};
  return 0;
}

// SysTick interrupt handler: increment the global 'systick' value.
void SysTick_IRQn_handler( void ) {
  ++systick;
}

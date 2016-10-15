#include <avr/io.h>
#include <avr/interrupt.h>



//These are for Timer1
#define PRESCALE0_1 _BV(CS10)
#define PRESCALE0_8 _BV(CS11)
#define PRESCALE0_64 (_BV(CS10) | _BV(CS11))
#define PRESCALE0_256 _BV(CS12)
#define PRESCALE0_1024 (_BV(CS12) | _BV(CS10))

#define SEND_BUFFER_SIZE 64
#define RECV_BUFFER_SIZE 64

#define wrapSendNum(x) ((x) % SEND_BUFFER_SIZE)
#define wrapRecvNum(x) ((x) % RECV_BUFFER_SIZE)

//=========================

// Send helper variables
byte sendBuffer[SEND_BUFFER_SIZE];
volatile uint8_t sendHead = 0;
volatile uint8_t sendTail = 0;
volatile uint8_t sendBitNum = 0;

//=========================

// Receive helper variables
byte recvBuffer[RECV_BUFFER_SIZE];
boolean isReceiving = false;
boolean lastRead = false;
volatile uint8_t recvHead = 0;
volatile uint8_t recvTail = 0;
volatile uint8_t recvBitNum = 0;

//=========================

void setup() {
  // put your setup code here, to run once:

  OSCCAL = 0x79;

  // Set up data direction
  DDRA |= (1 << PA6); // set LED pin as output
  
  DDRA |= (1 << PA0); // Set Serial Output as Output
  DDRB &= ~(1 << PB0); // Set Serial Input as input

  // Set up timers
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCCR1B |= (1 << WGM12); // configure timer1 for CTC mode
  TIMSK1 |= (1 << OCIE1A); // enable the CTC interrupt
  
  // sei(); // enable global interrupts
  OCR1A = 833; // Should return frequency ~1000hz
  TCCR1B |= PRESCALE0_1;

  // Initialize Serial Output and LED pin to HIGH
  PORTA |= (1 << PA0);
  PORTA |= (1 << PA6);
  

  if (sendTail == 0) {
    PORTA &= ~(1 << PA6);
  }

  // Initial test
  byte i = 0;
  for (i=0; i<50; i++)
    sendByte(i);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (isAvailable()) {
    sendByte(readByte());
  }

}


// this function is called every time the timer reaches the threshold we set
ISR (TIM1_COMPA_vect) { 
  // PORTA ^= (1 << PA6); // toggle the LED
  // PORTA |= (1 << PA6);


  // Write out Send buffer
  if (sendHead != sendTail) {
    // PORTA &= ~(1 << PA6);
    // PORTA |= (1 << PA6);
    
    sendByteBitNumber(sendBuffer[sendHead], sendBitNum);
    sendBitNum++; // Go to next bit
    
    // Byte is complete including stop bit
    if (sendBitNum == 10) { 
      sendHead = wrapSendNum(sendHead + 1);
      sendBitNum = 0;
    }
  }

  // Read buffer
  // TODO:

  // Read PB0 from Serial In
  lastRead = (PINB >> PB0) & 0x01;
  
  if (isReceiving) {
    PORTA ^= (1 << PA6); // toggle the LED
    
    // Continue receive the rest of the bits
    // recvBitNum runs from 0 to 7
    if (recvBitNum < 8) {
      // Shift right by 1
      recvBuffer[recvTail] >>= 1;
      if (lastRead)
        recvBuffer[recvTail] |= 0x80;

      recvBitNum++;
    } else if (recvBitNum == 8) {
      // Done with the bit
      isReceiving = false;
      recvTail = wrapRecvNum(recvTail + 1);
    }
  } else if (!isReceiving && lastRead == 0) {
    // If not receiving, set receiving flag if Start bit received
    // Start bit received
    isReceiving = true;
    recvBitNum = 0;
    recvBuffer[recvTail] = 0;
  } 

}


int readByte() {
  // Check for empty buffer
  if (recvHead == recvTail)
    return -1;

  // Return recvHead and increment
  uint8_t tmp = recvBuffer[recvHead];
  recvHead = wrapRecvNum(recvHead + 1);
  return tmp;
}

boolean isAvailable() {
  return (recvHead != recvTail);
}



void sendByte(byte outByte){
  // TODO: Check for collision between sendHead and sendTail
  
  
  // Manipulate sendBuffer and sendTail
  sendBuffer[sendTail] = outByte;
  sendTail = wrapSendNum(sendTail + 1);

  // sendHead will be consumed by ISR
}


void sendByteBitNumber(byte tmp, uint8_t bitNum) {
  switch(bitNum) {
    case 0:
      PORTA &= ~(1 << PA0);
      break;
    case 9:
      PORTA |= (1 << PA0);
      break;
    default:
      if ((tmp>>(bitNum-1)) & 0x01)
        PORTA |= (1 << PA0);
      else
        PORTA &= ~(1 << PA0);
  }
}

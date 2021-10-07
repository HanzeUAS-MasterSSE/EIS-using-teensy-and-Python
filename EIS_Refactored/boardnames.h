#ifndef boardnames_h
#define  boardnames_h

#if defined(__AVR_ATmega32U4__)
  #define BOARD "Teensy 2.0"
#elif defined(__AVR_AT90USB1286__)       
  #define BOARD "Teensy++ 2.0"
#elif defined(__MK20DX128__)       
  #define BOARD "Teensy 3.0"
#elif defined(__MK20DX256__)       
  #define BOARD "Teensy 3.2" // and Teensy 3.1 (obsolete)
#elif defined(__MKL26Z64__)       
  #define BOARD "Teensy LC"
#elif defined(__MK64FX512__)
  #define BOARD "Teensy 3.5"
#elif defined(__MK66FX1M0__)
  #define BOARD "Teensy 3.6"
#elif defined(ARDUINO_TEENSY40)
  #define BOARD_TYPE "TEENSY 4.0"
#elif defined(ARDUINO_TEENSY41)
  #define BOARD_TYPE "TEENSY 4.1"
#else
  #define BOARD "Unknown board"
#endif

#endif

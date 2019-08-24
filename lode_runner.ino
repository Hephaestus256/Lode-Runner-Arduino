/*
 * Lode Runner for Arduino UNO
 * by Nathan G. Smith
 * 
 * Original by Doug Smith (no relation, don't I wish)
 * and Broderbund Software (please don't sue me for this, I just want to learn, create, and teach people about Arduino!)
 * 
 * You can play Lode Runner for free online.  So maybe this will get them off my back, since I plugged their site:
 * http://loderunnerwebgame.com/game/
 * 
 * -If you can think of ways to improve this, please let me know.  I'm still a bit new to Arduino... this project was to force me to learn it.
 * -Sorry, this only works with the ILI9481 screen.  I got it from Banggood for about $11 including shipping (but I think it was on sale)
 * -If you can get it to work with other screens, I will incorporate it into this program.  I'm thinking it could be controlled with a define statement, so it doesn't
 *     take up flash space for no reason.
 * -I had to route all of the controls through A5, because the screen takes up all of the pins =(  Resistor dividers make different voltages which are interpreted by
 *     the program.  Refer to the circuit diagram for correct scheme.  You need: two 10k resistors, one 2.2k, one 3.3k, one 4.7k, one 33k
 * -The levels are loaded from the SD card.  You can make your own levels with a raw text editor.
 * -I would have made some sound effects, but I'm out of pins.  If you can find a pin that won't distort it when blitting like pin 0 and 1 do, then please let me know.
 *     I made some sound effects, but they are in a different program.  I might have to optimise it for flash space in order to not max it out if I combine it 
 *     with this program.
 * -If any of the documentation is wrong, please let me know.  Also let me know about bugs.  I am aware of a few, but there are probably some that escaped my testing.
 */
  
  /*

  Todo:

  -Guards trying to go down the same ladder will get stuck against each other (and sometimes horizontally too) (done, but sometimes still doesn't work)
  -Can player get killed by a spawning guard?  If not, prevent
  -Sound effects? (rewrite and optimize sound engine) need to find a pin for it
  -When guards wiggle in a ditch, sometimes it doesn't show a frame.  This is because when the ditch fills in,
   it re-blits the image of guard without the horizontal offset.  Incorporate this offset into the re-blit.
  -Optimize for flash space
  -Polish monkey animation
  -Clean up graphics to look more like original
  -Sometimes a dug ditch fills with time, but it doesn't re-blit the brick.  This happens when you dig below it when it's still filling.
  
*/

/*
   Get gold = 250
   Ditch enemy = 75
   Kill enemy = 75
   Level = 1500
*/


/*
Thanks to David Prentice for writing the original UTFTGLUE library.  I chopped it to pieces and Frankensteined it to get all of this to work.  Hope that's okay.
*/

/*************************** START OF UTFTGLUE.h ************************************/


/*
 * utftglue.h
 *
 * Created: 02/03/2013 14:25:06
 *  Author: David Prentice
 */ 


#ifndef UTFTGLUE_H_
#define UTFTGLUE_H_

#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

#define PORTRAIT 0
#define LANDSCAPE 1

/************************ MCUFRIEND_kbv.h *************************/

/*
 * MCUFRIEND_kbv class inherits from Adafruit_GFX class and the Arduino Print class.
 * Any use of MCUFRIEND_kbv class and examples is dependent on Adafruit and Arduino licenses
 * The license texts are in the accompanying license.txt file
 */

#ifndef MCUFRIEND_KBV_H_
#define MCUFRIEND_KBV_H_   298

#include "Adafruit_GFX.h"

class MCUFRIEND_kbv : public Adafruit_GFX {
  
public:
  MCUFRIEND_kbv(int CS=0, int RS=0, int WR=0, int RD=0, int RST=0);  //dummy arguments
  void     reset(void);                                       // you only need the constructor
  void     begin(uint16_t ID = 0x9341);                       // you only need the constructor
  virtual void     drawPixel(int16_t x, int16_t y, uint16_t color);  // and these three
  void     WriteCmdData(uint16_t cmd, uint16_t dat);                 // ?public methods !!!
  void     pushCommand(uint16_t cmd, uint8_t * block, int8_t N);
  void blit(int16_t x1, int16_t y1, int16_t x2, int16_t y2, byte* dat, uint16_t* colors);
  void blit_man(int16_t x, int16_t y, int16_t y1, byte* dat, uint16_t color, int rev);
  void blit_x_init(int x, int end);
  void blit_x_fwd(int x, byte d, uint16_t color);
  void blit_x_rev(int x, byte d, uint16_t color);
  void blit_x_level(int x1, int x2, int y, int y1, byte* dat, uint16_t* colors, bool left);
  void set_range_y(int y);
  void blit_man_erase(
            int prev_x, int prev_y, int curr_x, int curr_y,
            byte* level, const uint32_t* gfx, const uint8_t* blits, const uint16_t* man,
            uint16_t* const colors, uint16_t color, int dir);
  virtual void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  virtual void     fillScreen(uint16_t color)                                     { fillRect(0, 0, _width, _height, color); }
  virtual void     setRotation(uint8_t r);
  virtual void     invertDisplay(boolean i);
  
  uint16_t readReg(uint16_t reg, int8_t index=0);
  int16_t  readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h);
  void     setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
  void     pushColors(uint16_t *block, int16_t n, bool first);
  void     pushColors(uint8_t *block, int16_t n, bool first);
  void     pushColors(const uint8_t *block, int16_t n, bool first, bool bigend = false);
  void     vertScroll(int16_t top, int16_t scrollines, int16_t offset);
  
protected:
  uint32_t readReg32(uint16_t reg);
  uint32_t readReg40(uint16_t reg);
  uint16_t  _lcd_xor, _lcd_capable;
  
private:
  uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;
};

// New color definitions.  thanks to Bodmer
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

#endif


/************************* end of MCUFRIEND_kbv.h *****************/


//#include <Adafruit_GFX.h>
//#if !defined(AVR) && !defined(ESP32)
//#if defined(__arm)
//#include <avr/dtostrf.h>
//#endif

#undef _GFXFONT_H_     //comment this line to enable FreeFonts

class UTFTGLUE : public MCUFRIEND_kbv
{
  public:
  UTFTGLUE(byte model, int RS, int WR,int CS, int RST, int RD = A0) : MCUFRIEND_kbv(CS, RS, WR, RD, RST) {}
  void InitLCD(byte orientation=LANDSCAPE) {
    MCUFRIEND_kbv::reset();
    uint16_t ID = 0x9481;
     MCUFRIEND_kbv::begin(ID);
     MCUFRIEND_kbv::setRotation(_orient = orientation);
     _radius = 4;
     _fontsize = 1;
    }
  void blit(int16_t x1, int16_t y1, int16_t x2, int16_t y2, byte* dat, uint16_t* colors)
    { MCUFRIEND_kbv::blit(x1, y1, x2, y2, dat, colors); }
  void blit_man(int16_t x, int16_t y, int16_t y1, byte* dat, uint16_t color, int rev)
    { MCUFRIEND_kbv::blit_man(x, y, y1, dat, color, rev); }
  void blit_man_erase(
      int prev_x, int prev_y, int curr_x, int curr_y,
    byte* level, const uint32_t* gfx, const uint8_t* blits, const uint16_t* man,
    const uint16_t* colors, uint16_t color, int dir)
    {
      MCUFRIEND_kbv::blit_man_erase(prev_x, prev_y, curr_x, curr_y,
           level, gfx, blits, man, colors, color, dir);
    }
  protected:
  uint16_t _fcolor;
  uint16_t _bcolor;
  uint8_t _fontsize;
  uint8_t _radius;
  uint8_t _orient;
  
};


#ifndef MCUFRIEND_KBV_H_
#define MCUFRIEND_KBV_H_   298

class MCUFRIEND_kbv : public Adafruit_GFX {
  
public:
  MCUFRIEND_kbv(int CS=0, int RS=0, int WR=0, int RD=0, int RST=0);  //dummy arguments
  void     reset(void);                                       // you only need the constructor
  void     begin(uint16_t ID = 0x9341);                       // you only need the constructor
  virtual void     drawPixel(int16_t x, int16_t y, uint16_t color);  // and these three
  void     WriteCmdData(uint16_t cmd, uint16_t dat);                 // ?public methods !!!
  void     pushCommand(uint16_t cmd, uint8_t * block, int8_t N);
  void blit(int16_t x1, int16_t y1, int16_t x2, int16_t y2, byte* dat, uint16_t* colors);
  void blit_man(int16_t x, int16_t y, int16_t y1, byte* dat, uint16_t color, int rev);
  void blit_x_init(int x, int end);
  void blit_x_fwd(int x, byte d, uint16_t color);
  void blit_x_rev(int x, byte d, uint16_t color);
  void blit_x_level(int x1, int x2, int y, int y1, byte* dat, uint16_t* colors, bool left);
  void set_range_y(int y);
  void blit_man_erase(
            int prev_x, int prev_y, int curr_x, int curr_y,
            byte* level, const uint32_t* gfx, const uint8_t* blits, const uint16_t* man,
            uint16_t* const colors, uint16_t color, int dir);
  virtual void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  virtual void     fillScreen(uint16_t color)                                     { fillRect(0, 0, _width, _height, color); }
  virtual void     setRotation(uint8_t r);
  virtual void     invertDisplay(boolean i);
  
  uint16_t readReg(uint16_t reg, int8_t index=0);
  int16_t  readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h);
  void     setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
  void     pushColors(uint16_t *block, int16_t n, bool first);
  void     pushColors(uint8_t *block, int16_t n, bool first);
  void     pushColors(const uint8_t *block, int16_t n, bool first, bool bigend = false);
  void     vertScroll(int16_t top, int16_t scrollines, int16_t offset);
  
protected:
  uint32_t readReg32(uint16_t reg);
  uint32_t readReg40(uint16_t reg);
  uint16_t  _lcd_xor, _lcd_capable;
  
private:
  uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;
};

// New color definitions.  thanks to Bodmer
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

#endif

#if !defined(USE_SPECIAL) || defined (USE_SPECIAL_FAIL)

#if 0
//################################### UNO ##############################
#elif defined(__AVR_ATmega328P__)       //regular UNO shield on UNO
#define RD_PORT PORTC
#define RD_PIN  0
#define WR_PORT PORTC
#define WR_PIN  1
#define CD_PORT PORTC
#define CD_PIN  2
#define CS_PORT PORTC
#define CS_PIN  3
#define RESET_PORT PORTC
#define RESET_PIN  4

#define BMASK         0x03              //more intuitive style for mixed Ports
#define DMASK         0xFC              //does exactly the same as previous
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) & BMASK); PORTD = (PORTD & ~DMASK) | ((x) & DMASK); }
#define read_8()      ( (PINB & BMASK) | (PIND & DMASK) )
#define setWriteDir() { DDRB |=  BMASK; DDRD |=  DMASK; }
#define setReadDir()  { DDRB &= ~BMASK; DDRD &= ~DMASK; }
#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))


//################################### UNO SHIELD on BOBUINO ##############################
#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__) //UNO shield on BOBUINO
#warning regular UNO shield on BOBUINO
#define RD_PORT PORTA
#define RD_PIN  7
#define WR_PORT PORTA
#define WR_PIN  6
#define CD_PORT PORTA
#define CD_PIN  5
#define CS_PORT PORTA
#define CS_PIN  4
#define RESET_PORT PORTA
#define RESET_PIN  3

#define BMASK         0x0F              //
#define DMASK         0x6C              //
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) >> 4); \
PORTD = (PORTD & ~DMASK) | ((x) & 0x0C) | (((x) & 0x03) << 5); }
#define read_8()      ( (PINB << 4) | (PIND & 0x0C) | ((PIND & 0x60) >> 5) )
#define setWriteDir() { DDRB |=  BMASK; DDRD |=  DMASK; }
#define setReadDir()  { DDRB &= ~BMASK; DDRD &= ~DMASK; }
#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))


//####################################### STM32 ############################
// NUCLEO:   ARDUINO_NUCLEO_xxxx from ST Core or ARDUINO_STM_NUCLEO_F103RB from MapleCore
// BLUEPILL: ARDUINO_NUCLEO_F103C8 / ARDUINO_BLUEPILL_F103C8 from ST Core or ARDUINO_GENERIC_STM32F103C from MapleCore
// MAPLE_REV3: n/a from ST Core or ARDUINO_MAPLE_REV3 from MapleCore
// ST Core:   ARDUINO_ARCH_STM32
// MapleCore: __STM32F1__
#elif defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32)   //MapleCore or ST Core
#define IS_NUCLEO64 ( defined(ARDUINO_STM_NUCLEO_F103RB) \
|| defined(ARDUINO_NUCLEO_F030R8) || defined(ARDUINO_NUCLEO_F091RC) \
|| defined(ARDUINO_NUCLEO_F103RB) || defined(ARDUINO_NUCLEO_F303RE) \
|| defined(ARDUINO_NUCLEO_F401RE) || defined(ARDUINO_NUCLEO_F411RE) \
|| defined(ARDUINO_NUCLEO_F446RE) || defined(ARDUINO_NUCLEO_L053R8) \
|| defined(ARDUINO_NUCLEO_L152RE) || defined(ARDUINO_NUCLEO_L476RG) \
)
// F1xx, F4xx, L4xx have different registers and styles.  General Macros
#if defined(__STM32F1__)   //weird Maple Core
#define REGS(x) regs->x
#else                      //regular ST Core
#define REGS(x) x
#endif
#define PIN_HIGH(port, pin)   (port)-> REGS(BSRR) = (1<<(pin))
#define PIN_LOW(port, pin)    (port)-> REGS(BSRR) = (1<<((pin)+16))
#define PIN_MODE2(reg, pin, mode) reg=(reg&~(0x3<<((pin)<<1)))|(mode<<((pin)<<1))
#define GROUP_MODE(port, reg, mask, val)  {port->REGS(reg) = (port->REGS(reg) & ~(mask)) | ((mask)&(val)); }

// Family specific Macros.  F103 needs ST and Maple compatibility
// note that ILI9320 class of controller has much slower Read cycles
#if 0
#elif defined(__STM32F1__) || defined(ARDUINO_NUCLEO_F103C8) || defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_NUCLEO_F103RB)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#if defined(__STM32F1__)  //MapleCore crts.o does RCC.  not understand regular syntax anyway
#define GPIO_INIT()
#else
#define GPIO_INIT()   { RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; \
AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;}
#endif
#define GP_OUT(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x33333333)
#define GP_INP(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x44444444)
#define PIN_OUTPUT(port, pin) {\
if (pin < 8) {GP_OUT(port, CRL, 0xF<<((pin)<<2));} \
else {GP_OUT(port, CRH, 0xF<<((pin&7)<<2));} \
}
#define PIN_INPUT(port, pin) { \
if (pin < 8) { GP_INP(port, CRL, 0xF<<((pin)<<2)); } \
else { GP_INP(port, CRH, 0xF<<((pin&7)<<2)); } \
}

// should be easy to add F030, F091, F303, L053, ...
#elif defined(STM32F030x8)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F091xC)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F303xE)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; \
/* AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1; */ }

#elif defined(STM32F401xE)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F411xE)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F446xx)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32L053xx)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32L152xE)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32L476xx)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#else
#error unsupported STM32
#endif

#if 0
#elif defined(ARDUINO_GENERIC_STM32F103C) || defined(ARDUINO_NUCLEO_F103C8) || defined(ARDUINO_BLUEPILL_F103C8)
#warning Uno Shield on BLUEPILL
#define RD_PORT GPIOB
//#define RD_PIN  5
#define RD_PIN  0  //hardware mod to Adapter.  Allows use of PB5 for SD Card
#define WR_PORT GPIOB
#define WR_PIN  6
#define CD_PORT GPIOB
#define CD_PIN  7
#define CS_PORT GPIOB
#define CS_PIN  8
#define RESET_PORT GPIOB
#define RESET_PIN  9

// configure macros for the data pins
#define write_8(d)    { GPIOA->REGS(BSRR) = 0x00FF << 16; GPIOA->REGS(BSRR) = (d) & 0xFF; }
#define read_8()      (GPIOA->REGS(IDR) & 0xFF)
//                                         PA7 ..PA0
#define setWriteDir() {GP_OUT(GPIOA, CRL, 0xFFFFFFFF); }
#define setReadDir()  {GP_INP(GPIOA, CRL, 0xFFFFFFFF); }

#elif IS_NUCLEO64 // Uno Shield on NUCLEO
#warning Uno Shield on NUCLEO
#define RD_PORT GPIOA
#define RD_PIN  0
#define WR_PORT GPIOA
#define WR_PIN  1
#define CD_PORT GPIOA
#define CD_PIN  4
#define CS_PORT GPIOB
#define CS_PIN  0
#define RESET_PORT GPIOC
#define RESET_PIN  1

// configure macros for the data pins
#define write_8(d) { \
GPIOA->REGS(BSRR) = 0x0700 << 16; \
GPIOB->REGS(BSRR) = 0x0438 << 16; \
GPIOC->REGS(BSRR) = 0x0080 << 16; \
GPIOA->REGS(BSRR) = (  ((d) & (1<<0)) << 9) \
| (((d) & (1<<2)) << 8) \
| (((d) & (1<<7)) << 1); \
GPIOB->REGS(BSRR) = (  ((d) & (1<<3)) << 0) \
| (((d) & (1<<4)) << 1) \
| (((d) & (1<<5)) >> 1) \
| (((d) & (1<<6)) << 4); \
GPIOC->REGS(BSRR) = (  ((d) & (1<<1)) << 6); \
}

#define read_8() (       (  (  (GPIOA->REGS(IDR) & (1<<9)) >> 9) \
| ((GPIOC->REGS(IDR) & (1<<7)) >> 6) \
| ((GPIOA->REGS(IDR) & (1<<10)) >> 8) \
| ((GPIOB->REGS(IDR) & (1<<3)) >> 0) \
| ((GPIOB->REGS(IDR) & (1<<5)) >> 1) \
| ((GPIOB->REGS(IDR) & (1<<4)) << 1) \
| ((GPIOB->REGS(IDR) & (1<<10)) >> 4) \
| ((GPIOA->REGS(IDR) & (1<<8))  >> 1)))


#if defined(ARDUINO_NUCLEO_F103RB) || defined(ARDUINO_STM_NUCLEO_F103RB) //F103 has unusual GPIO modes
//                                 PA10,PA9,PA8                       PB10                   PB5,PB4,PB3                             PC7
#define setWriteDir() {GP_OUT(GPIOA, CRH, 0xFFF); GP_OUT(GPIOB, CRH, 0xF00); GP_OUT(GPIOB, CRL, 0xFFF000); GP_OUT(GPIOC, CRL, 0xF0000000); }
#define setReadDir()  {GP_INP(GPIOA, CRH, 0xFFF); GP_INP(GPIOB, CRH, 0xF00); GP_INP(GPIOB, CRL, 0xFFF000); GP_INP(GPIOC, CRL, 0xF0000000); }
#else      //F0xx, F3xx, F4xx, L0xx, L1xx, L4xx use MODER
//                                   PA10,PA9,PA8           PB10,PB5,PB4,PB3                      PC7
#define setWriteDir() { setReadDir(); \
GPIOA->MODER |=  0x150000; GPIOB->MODER |=  0x100540; GPIOC->MODER |=  0x4000; }
#define setReadDir()  { GPIOA->MODER &= ~0x3F0000; GPIOB->MODER &= ~0x300FC0; GPIOC->MODER &= ~0xC000; }
#endif

#elif defined(ARDUINO_MAPLE_REV3) // Uno Shield on MAPLE_REV3 board
#warning Uno Shield on MAPLE_REV3 board
#define RD_PORT GPIOC
#define RD_PIN  0
#define WR_PORT GPIOC
#define WR_PIN  1
#define CD_PORT GPIOC
#define CD_PIN  2
#define CS_PORT GPIOC
#define CS_PIN  3
#define RESET_PORT GPIOC
#define RESET_PIN  4

// configure macros for the data pins
#define write_8(d) { \
GPIOA->REGS(BSRR) = 0x0703 << 16; \
GPIOB->REGS(BSRR) = 0x00E0 << 16; \
GPIOA->REGS(BSRR) = (  ((d) & (1<<0)) << 10) \
| (((d) & (1<<2)) >> 2) \
| (((d) & (1<<3)) >> 2) \
| (((d) & (1<<6)) << 2) \
| (((d) & (1<<7)) << 2); \
GPIOB->REGS(BSRR) = (  ((d) & (1<<1)) << 6) \
| (((d) & (1<<4)) << 1) \
| (((d) & (1<<5)) << 1); \
}

#define read_8()  (     (   (  (GPIOA->REGS(IDR) & (1<<10)) >> 10) \
| ((GPIOB->REGS(IDR) & (1<<7)) >> 6) \
| ((GPIOA->REGS(IDR) & (1<<0)) << 2) \
| ((GPIOA->REGS(IDR) & (1<<1)) << 2) \
| ((GPIOB->REGS(IDR) & (1<<5)) >> 1) \
| ((GPIOB->REGS(IDR) & (1<<6)) >> 1) \
| ((GPIOA->REGS(IDR) & (1<<8)) >> 2) \
| ((GPIOA->REGS(IDR) & (1<<9)) >> 2)))

//                                 PA10,PA9,PA8                   PA1,PA0                     PB7,PB6,PB5
#define setWriteDir() {GP_OUT(GPIOA, CRH, 0xFFF); GP_OUT(GPIOA, CRL, 0xFF); GP_OUT(GPIOB, CRL, 0xFFF00000); }
#define setReadDir()  {GP_INP(GPIOA, CRH, 0xFFF); GP_INP(GPIOA, CRL, 0xFF); GP_INP(GPIOB, CRL, 0xFFF00000); }

#else
#error REGS group
#endif

#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; WR_IDLE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

//################################### ESP32 ##############################
#elif defined(ESP32)       //regular UNO shield on TTGO D1 R32 (ESP32)
#define LCD_RD  2  //LED
#define LCD_WR  4
#define LCD_RS 15  //hard-wired to A2 (GPIO35)
#define LCD_CS 33  //hard-wired to A3 (GPIO34)
#define LCD_RST 32 //hard-wired to A4 (GPIO36)

#define LCD_D0 12
#define LCD_D1 13
#define LCD_D2 26
#define LCD_D3 25
#define LCD_D4 17
#define LCD_D5 16
#define LCD_D6 27
#define LCD_D7 14

#define RD_PORT GPIO.out
#define RD_PIN  LCD_RD
#define WR_PORT GPIO.out
#define WR_PIN  LCD_WR
#define CD_PORT GPIO.out
#define CD_PIN  LCD_RS
#define CS_PORT GPIO.out1.val
#define CS_PIN  LCD_CS
#define RESET_PORT GPIO.out1.val
#define RESET_PIN  LCD_RST

static inline uint32_t map_8(uint32_t d)
{
  return (
      0
      | ((d & (1 << 0)) << (LCD_D0 - 0))
      | ((d & (1 << 1)) << (LCD_D1 - 1))
      | ((d & (1 << 2)) << (LCD_D2 - 2))
      | ((d & (1 << 3)) << (LCD_D3 - 3))
      | ((d & (1 << 4)) << (LCD_D4 - 4))
      | ((d & (1 << 5)) << (LCD_D5 - 5))
      | ((d & (1 << 6)) << (LCD_D6 - 6))
      | ((d & (1 << 7)) << (LCD_D7 - 7))
      );
}

static inline uint8_t map_32(uint32_t d)
{
  return (
      0
      | ((d & (1 << LCD_D0)) >> (LCD_D0 - 0))
      | ((d & (1 << LCD_D1)) >> (LCD_D1 - 1))
      | ((d & (1 << LCD_D2)) >> (LCD_D2 - 2))
      | ((d & (1 << LCD_D3)) >> (LCD_D3 - 3))
      | ((d & (1 << LCD_D4)) >> (LCD_D4 - 4))
      | ((d & (1 << LCD_D5)) >> (LCD_D5 - 5))
      | ((d & (1 << LCD_D6)) >> (LCD_D6 - 6))
      | ((d & (1 << LCD_D7)) >> (LCD_D7 - 7))
      );
}

static inline void write_8(uint16_t data)
{
  GPIO.out_w1tc = map_8(0xFF);  //could define once as DMASK
  GPIO.out_w1ts = map_8(data);
}

static inline uint8_t read_8()
{
  return map_32(GPIO.in);
}
static void setWriteDir()
{
  pinMode(LCD_D0, OUTPUT);
  pinMode(LCD_D1, OUTPUT);
  pinMode(LCD_D2, OUTPUT);
  pinMode(LCD_D3, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D7, OUTPUT);
}

static void setReadDir()
{
  pinMode(LCD_D0, INPUT);
  pinMode(LCD_D1, INPUT);
  pinMode(LCD_D2, INPUT);
  pinMode(LCD_D3, INPUT);
  pinMode(LCD_D4, INPUT);
  pinMode(LCD_D5, INPUT);
  pinMode(LCD_D6, INPUT);
  pinMode(LCD_D7, INPUT);
}

#define WRITE_DELAY { }
#define READ_DELAY  { }

#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (digitalWrite(b, LOW))
#define PIN_HIGH(p, b)       (digitalWrite(b, HIGH))
#define PIN_OUTPUT(p, b)     (pinMode(b, OUTPUT))

#else
#error MCU unsupported
#endif                          // regular UNO shields on Arduino boards

#endif                          //!defined(USE_SPECIAL) || defined (USE_SPECIAL_FAIL)

#define RD_ACTIVE  PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE    PIN_HIGH(RD_PORT, RD_PIN)
#define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)
#define WR_ACTIVE  PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE    PIN_HIGH(WR_PORT, WR_PIN)
#define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)
#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

// General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }       //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE      //PWLR=TRDL=150ns, tDDR=100ns

#if !defined(GPIO_INIT)
#define GPIO_INIT()
#endif
#define CTL_INIT()   { GPIO_INIT(); RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write16(x); CD_DATA; }
#define WriteData(x) { write16(x); }

#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

MCUFRIEND_kbv::MCUFRIEND_kbv(int CS, int RS, int WR, int RD, int RST):Adafruit_GFX(240, 320)
{
  // we can not access GPIO pins until AHB has been enabled.
}

static uint8_t done_reset, is8347, is555;


void MCUFRIEND_kbv::reset(void)
{
  done_reset = 1;
  setWriteDir();
  CTL_INIT();
  CS_IDLE;
  RD_IDLE;
  WR_IDLE;
  RESET_IDLE;
  delay(50);
  RESET_ACTIVE;
  delay(100);
  RESET_IDLE;
  delay(100);
}

static void writecmddata(uint16_t cmd, uint16_t dat)
{
  CS_ACTIVE;
  WriteCmd(cmd);
  WriteData(dat);
  CS_IDLE;
}

void MCUFRIEND_kbv::WriteCmdData(uint16_t cmd, uint16_t dat) { writecmddata(cmd, dat); }

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
  CS_ACTIVE;
  WriteCmd(cmd);
  while (N-- > 0) {
    uint8_t u8 = *block++;
    write8(u8);
    if (N && is8347) {
      cmd++;
      WriteCmd(cmd);
    }
  }
  CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
  uint8_t d[4];
  d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
  WriteCmdParamN(cmd, 4, d);
}

void MCUFRIEND_kbv::pushCommand(uint16_t cmd, uint8_t * block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void)
{
  uint16_t ret;
  uint8_t lo;
  READ_8(ret);
  READ_8(lo);
  return (ret << 8) | lo;
}

uint16_t MCUFRIEND_kbv::readReg(uint16_t reg, int8_t index)
{
  uint16_t ret;
  uint8_t lo;
  if (!done_reset)
    reset();
  CS_ACTIVE;
  WriteCmd(reg);
  setReadDir();
  delay(1);    //1us should be adequate
  do { ret = read16bits(); }while (--index >= 0);  //need to test with SSD1963
  RD_IDLE;
  CS_IDLE;
  setWriteDir();
  return ret;
}


void MCUFRIEND_kbv::setRotation(uint8_t r)
{
  uint16_t GS, SS_v, ORG, REV = _lcd_rev;
  uint8_t val, d[3];
  rotation = r & 3;           // just perform the operation ourselves on the protected variables
  _width = (rotation & 1) ? HEIGHT : WIDTH;
  _height = (rotation & 1) ? WIDTH : HEIGHT;
  
  val = 0x28;
  
  if (_lcd_capable & INVERT_GS)
    val ^= 0x80;
  if (_lcd_capable & INVERT_SS)
    val ^= 0x40;
  if (_lcd_capable & INVERT_RGB)
    val ^= 0x08;
  if (_lcd_capable & MIPI_DCS_REV1) {
    if (val & 0x80)
      val |= 0x01;    //GS
    if ((val & 0x40))
      val |= 0x02;    //SS
    if (_lcd_ID == 0x1963) val &= ~0xC0;
    if (_lcd_ID == 0x9481) val &= ~0xD0;
    if (_lcd_ID == 0x1511) {
      val &= ~0x10;   //remove ML
      val |= 0xC0;    //force penguin 180 rotation
    }
    _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
    WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
    _lcd_madctl = val;
  }
  // cope with 9320 variants
  else {
    switch (_lcd_ID) {
      default:
        _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
        GS = (val & 0x80) ? (1 << 15) : 0;
        WriteCmdData(0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
      common_SS:
        SS_v = (val & 0x40) ? (1 << 8) : 0;
        WriteCmdData(0x01, SS_v);     // set Driver Output Control
      common_ORG:
        ORG = (val & 0x20) ? (1 << 3) : 0;
        
        if (val & 0x08)
          ORG |= 0x1000;  //BGR
        _lcd_madctl = ORG | 0x0030;
        WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
        break;
    }
  }
  if ((rotation & 1) && ((_lcd_capable & MV_AXIS) == 0)) {
    uint16_t x;
    x = _MC, _MC = _MP, _MP = x;
    x = _SC, _SC = _SP, _SP = x;    //.kbv check 0139
    x = _EC, _EC = _EP, _EP = x;    //.kbv check 0139
  }
  setAddrWindow(0, 0, width() - 1, height() - 1);
  vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}


void MCUFRIEND_kbv::blit_x_level(int x1, int x2, int y, int y1, byte* dat, uint16_t* colors, bool left)
{
  // count how many pixels are written.  display that with x1, x2
  // highest order bits to lowest
  int ct = 0;
  
  if (x1 >= x2) return;
  
  //blit_x_init(x1, x2 - 1);
  
  int c;
  int i1 = 3 - ((x1 / 4) & 3); // byte begin
  int i2 = 3 - (((x2 + 3) / 4 - 1) & 3); // byte end
  int j1 = (3 - (x1 & 3)) * 2; // bit begin
  int j2 = (3 - ((x2 - 1) & 3)) * 2; // bit end
  
  //y1 &= 0xfff0;
  y -= y1;
  
  if (!(!left && (i1 == i2))) {
    // left of non-byte-aligned part of blit
    c = pgm_read_byte(dat + y * 4 + i1);
    for (int j = j1; j >= 0; j -= 2) {
      WriteData(colors[(c >> j) & 0b00000011]);
      ct++;
    }
  }
  
  //if (left) i2--;
  
  // byte-aligned part of blit
  for (int i = i1 - 1; i >= i2 + 1; i--) {
    c = pgm_read_byte(dat + y * 4 + i);
    for (int j = 6; j >= 0; j -= 2) {
      WriteData(colors[(c >> j) & 0b00000011]);
      ct++;
    }
  }
  
  //if (ct >= x2 - x1) return;
  
  //if ((x1 & 15) != 13) {
  // right, non-byte-aligned part of blit
  if (!(left && (i1 == i2))) {
    c = pgm_read_byte(dat + y * 4 + i2);
    for (int j = 6; j >= j2; j -= 2) {
      WriteData(colors[(c >> j) & 0b00000011]);
      ct++;
    }
  }
  
}


void MCUFRIEND_kbv::set_range_y(int y)
{
  // write Y
  WriteCmd(_MP);
  CD_DATA;
  write16(y);
  write16(y);
}


void MCUFRIEND_kbv::blit_man_erase(
                   int prev_x, int prev_y, int curr_x, int curr_y,
                   byte* level, const uint32_t* gfx, const uint8_t* blits, const uint16_t* man,
                   uint16_t* colors, uint16_t color, int dir)
{
  uint16_t test_colors1[4] = {
    TFT_ORANGE, TFT_PINK, TFT_GREEN, TFT_PURPLE
  };
  
  uint16_t test_colors2[4] = {
    TFT_PINK, TFT_GREEN, TFT_PURPLE, TFT_ORANGE
  };
  
  int mid1, mid2;
  int border_x1 = prev_x & 0xfff0;
  int border_y1 = prev_y & 0xfff0;
  int border_x2 = (border_x1 + 16) & 0xfff0;
  int border_y2 = (border_y1 + 16) & 0xfff0;
  int lev, tile, cx, cy;
  int y;
  
  cx = prev_x >> 4;
  cy = prev_y >> 4;
  
  lev = (level[((cx + 0) + 30 * (cy + 0)) / 2] >> (((cx + 0) & 1) * 4)) & 0x0f;
  tile = blits[lev];
  byte* dat11 = (byte*)(&gfx[tile * 16]);
  
  lev = (level[((cx + 1) + 30 * (cy + 0)) / 2] >> (((cx + 1) & 1) * 4)) & 0x0f;
  tile = blits[lev];
  byte* dat12 = (byte*)(&gfx[tile * 16]);
  
  lev = (level[((cx + 0) + 30 * (cy + 1)) / 2] >> (((cx + 0) & 1) * 4)) & 0x0f;
  tile = blits[lev];
  byte* dat21 = (byte*)&gfx[tile * 16];
  
  lev = (level[((cx + 1) + 30 * (cy + 1)) / 2] >> (((cx + 1) & 1) * 4)) & 0x0f;
  tile = blits[lev];
  byte* dat22 = (byte*)&gfx[tile * 16];
  
  CS_ACTIVE;
  
  if (prev_y < curr_y) { // going downward
    // do top of erase from prev_y to curr_y - 1
    
    for (y = prev_y; y < min(curr_y, border_y2); y++) {
      set_range_y(y);
      blit_x_init(prev_x, prev_x + 15);
      
      // erase from prev_x to border between cells - 1
      blit_x_level(prev_x, border_x2 - 0, y, prev_y & 0xfff0, dat11, colors, true);
      // erase from border between cells to prev_x + 15
      blit_x_level(border_x2, prev_x + 16, y, prev_y & 0xfff0, dat12, colors, false);
    }
    
    for (y = border_y2; y < curr_y; y++) {
      set_range_y(y);
      blit_x_init(prev_x, prev_x + 15);
      
      // erase from prev_x to border between cells - 1
      blit_x_level(prev_x, border_x2 - 0, y, prev_y & 0xfff0, dat21, colors, true);
      // erase from border between cells to prev_x + 15
      blit_x_level(border_x2, prev_x + 16, y, prev_y & 0xfff0, dat22, colors, false);
    }
    
    mid1 = curr_y;
    mid2 = prev_y + 16;
  } else { // going upward or no vertical movement
    // do top of blit from curr_y to prev_y - 1
    for (y = curr_y; y < prev_y; y++) {
      // blit from curr_x to curr_x + 15
      set_range_y(y);
      blit_man(curr_x, y, curr_y, (byte*)man, color, dir);
    }
    mid1 = prev_y;
    mid2 = curr_y + 16;
  }
  
  // erase mid and then blit mid for each y
  
  // mid 1 of 2
  
  for (y = mid1; y < min(mid2, border_y2); y++) {
    set_range_y(y);
    blit_x_init(prev_x, prev_x + 15);
    
    // erase from prev_x to border between cells
    blit_x_level(prev_x, border_x2 - 0, y, prev_y & 0xfff0, dat11, colors, true);
    
    // erase from border between cells to prev_x + 15
    blit_x_level(border_x2, prev_x + 16, y, prev_y & 0xfff0, dat12, colors, false);
    
    // blit from curr_x to curr_x + 15
    blit_man(curr_x, y, curr_y, (byte*)man, color, dir);
  }
  
  // mid 2 of 2
  
  for (y = border_y2; y < mid2; y++) {
    set_range_y(y);
    blit_x_init(prev_x, prev_x + 15);
    
    // erase from prev_x to border between cells
    blit_x_level(prev_x, border_x2 - 0, y, border_y2, dat21, colors, true);
    // erase from border between cells to prev_x + 15
    blit_x_level(border_x2, prev_x + 16, y, border_y2, dat22, colors, false);
    // blit from curr_x to curr_x + 15
    blit_man(curr_x, y, curr_y, (byte*)man, color, dir);
  }
  
  if (prev_y < curr_y) { // going downward
    // do bottom of blit from prev_y + 16 to curr_y + 15
    
    for (y = prev_y + 16; y < curr_y + 16; y++) {
      set_range_y(y);
      blit_x_init(prev_x, prev_x + 15);
      blit_man(curr_x, y, curr_y, (byte*)man, color, dir);
    }
  } else { // going upward or no vertical movement
    // do bottom of erase from curr_y + 16 to prev_y + 15
    
    for (y = curr_y + 16; y < border_y2; y++) {
      set_range_y(y);
      blit_x_init(prev_x, prev_x + 15);
      
      // erase from prev_x to border between cells
      blit_x_level(prev_x, border_x2 - 0, y, prev_y & 0xfff0, dat11, colors, true);
      // erase from border between cells to prev_x + 15
      blit_x_level(border_x2, prev_x + 16, y, prev_y & 0xfff0, dat12, colors, false);
    }
    
    for (y = max(curr_y + 16, border_y2); y < prev_y + 16; y++) {
      set_range_y(y);
      blit_x_init(prev_x, prev_x + 15);
      
      // erase from prev_x to border between cells
      blit_x_level(prev_x, border_x2 - 0, y, border_y2, dat21, colors, true);
      // erase from border between cells to prev_x + 15
      blit_x_level(border_x2, prev_x + 16, y, border_y2, dat22, colors, false);
    }
  }
  
  CS_IDLE;
}


void MCUFRIEND_kbv::blit(
             int16_t x1, int16_t y1, int16_t x2, int16_t y2,
             byte* dat, uint16_t* colors)
{
  CS_ACTIVE;
  
  for (int y = y1; y <= y2; y++) {
    // write Y
    WriteCmd(_MP);
    CD_DATA;
    write16(y);
    write16(y);
    
    // write X
    WriteCmd(_MC);
    CD_DATA;
    write16(x1);
    write16(x2);
    
    WriteCmd(_MW);
    
    blit_x_level(x1, x2, y, y1, dat, colors, true);
    
  }
  
  CS_IDLE;
}


void MCUFRIEND_kbv::blit_x_init(int x1, int x2)
{
  // write X bounds
  WriteCmd(_MC);
  CD_DATA;
  write16(x1);
  write16(x2);
  
  WriteCmd(_MW);
}


void MCUFRIEND_kbv::blit_x_fwd(int x, byte d, uint16_t color)
{
  byte mask = 0b10000000;
  int end = x + 7;
  
  do {
    blit_x_init(x, end);
    
    while (d & mask) { // write color for 1's
      WriteData(color);
      mask >>= 1;
      x++;
    };
    
    d = ~d;
    
    while (d & mask) { // skip pixels for 0's
      mask >>= 1;
      x++;
    };
    
    d = ~d;
  } while (mask);
}


void MCUFRIEND_kbv::blit_x_rev(int x, byte d, uint16_t color)
{
  byte mask = 0b00000001;
  int end = x + 7;
  
  do {
    blit_x_init(x, end);
    
    while (d & mask) { // write color for 1's
      WriteData(color);
      mask <<= 1;
      x++;
    };
    
    d = ~d;
    
    while (d & mask) { // skip pixels for 0's
      mask <<= 1;
      x++;
    };
    
    d = ~d;
  } while (mask);
}


void MCUFRIEND_kbv::blit_man(
               int16_t x, int16_t y, int16_t y1,
               byte* dat, uint16_t color, int rev = 0)
{
  dat += (y - y1) * 2;
  
  if (rev == 0) {
    dat++;
    blit_x_fwd(x, pgm_read_byte(dat), color);
    dat--;
    blit_x_fwd(x + 8, pgm_read_byte(dat), color);
  } else {
    blit_x_rev(x, pgm_read_byte(dat), color);
    dat++;
    blit_x_rev(x + 8, pgm_read_byte(dat), color);
  }
  
}


void MCUFRIEND_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  // MCUFRIEND just plots at edge if you try to write outside of the box:
  if (x < 0 || y < 0 || x >= width() || y >= height())
    return;
  setAddrWindow(x, y, x, y);
  WriteCmdData(_MW, color);
}


void MCUFRIEND_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
  if (_lcd_ID == 0x1526 && (rotation & 1)) {
    int16_t dx = x1 - x, dy = y1 - y;
    if (dy == 0) { y1++; }
    else if (dx == 0) { x1 += dy; y1 -= dy; }
  }
  
  if (_lcd_capable & MIPI_DCS_REV1) {
    WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1);   //Start column instead of _MC
    WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1);   //
    if (is8347 && _lcd_ID == 0x0065) {             //HX8352-B has separate _MC, _SC
      uint8_t d[2];
      d[0] = x >> 8; d[1] = x;
      WriteCmdParamN(_MC, 2, d);                 //allows !MV_AXIS to work
      d[0] = y >> 8; d[1] = y;
      WriteCmdParamN(_MP, 2, d);
    }
  } else {
    WriteCmdData(_MC, x);
    WriteCmdData(_MP, y);
    if (!(x == x1 && y == y1)) {  //only need MC,MP for drawPixel
      if (_lcd_capable & XSA_XEA_16BIT) {
        if (rotation & 1)
          y1 = y = (y1 << 8) | y;
        else
          x1 = x = (x1 << 8) | x;
      }
      WriteCmdData(_SC, x);
      WriteCmdData(_SP, y);
      WriteCmdData(_EC, x1);
      WriteCmdData(_EP, y1);
    }
  }
}

void MCUFRIEND_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  int16_t end;
  
  if (w < 0) {
    w = -w;
    x -= w;
  }                           //+ve w
  end = x + w;
  if (x < 0)
    x = 0;
  if (end > width())
    end = width();
  w = end - x;
  if (h < 0) {
    h = -h;
    y -= h;
  }                           //+ve h
  end = y + h;
  if (y < 0)
    y = 0;
  if (end > height())
    end = height();
  h = end - y;
  setAddrWindow(x, y, x + w - 1, y + h - 1);
  CS_ACTIVE;
  WriteCmd(_MW);
  if (h > w) {
    end = h;
    h = w;
    w = end;
  }
  uint8_t hi = color >> 8, lo = color & 0xFF;
  while (h-- > 0) {
    end = w;
#if USING_16BIT_BUS
#if defined(__SAM3X8E__)
#define STROBE_16BIT {WR_ACTIVE;WR_ACTIVE;WR_ACTIVE;WR_IDLE;WR_IDLE;}
#else
#define STROBE_16BIT {WR_ACTIVE; WR_IDLE;}
#endif
    write_16(color);        //we could just do the strobe
    lo = end & 7;
    hi = end >> 3;
    if (hi)
      do {
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
        STROBE_16BIT;
      } while (--hi > 0);
    while (lo-- > 0) {
      STROBE_16BIT;
    }
#else
    do {
      write8(hi);
      write8(lo);
    } while (--end != 0);
#endif
  }
  CS_IDLE;
  if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
    setAddrWindow(0, 0, width() - 1, height() - 1);
}

static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, bool first, uint8_t flags)
{
  uint16_t color;
  uint8_t h, l;
  bool isconst = flags & 1;
  bool isbigend = (flags & 2) != 0;
  CS_ACTIVE;
  if (first) {
    WriteCmd(cmd);
  }
  while (n-- > 0) {
    if (isconst) {
      h = pgm_read_byte(block++);
      l = pgm_read_byte(block++);
    } else {
      h = (*block++);
      l = (*block++);
    }
    color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);
    write16(color);
  }
  CS_IDLE;
}

void MCUFRIEND_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
  pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void MCUFRIEND_kbv::pushColors(uint8_t * block, int16_t n, bool first)
{
  pushColors_any(_MW, (uint8_t *)block, n, first, 2);   //regular bigend
}
void MCUFRIEND_kbv::pushColors(const uint8_t * block, int16_t n, bool first, bool bigend)
{
  pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}


void MCUFRIEND_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
  int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
  int16_t vsp;
  int16_t sea = top;
  
  if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
  vsp = top + offset; // vertical start position
  if (offset < 0)
    vsp += scrollines;          //keep in unsigned range
  sea = top + scrollines - 1;
  if (_lcd_capable & MIPI_DCS_REV1) {
    uint8_t d[6];           // for multi-byte parameters
    d[0] = top >> 8;        //TFA
    d[1] = top;
    d[2] = scrollines >> 8; //VSA
    d[3] = scrollines;
    d[4] = bfa >> 8;        //BFA
    d[5] = bfa;
    WriteCmdParamN(is8347 ? 0x0E : 0x33, 6, d);
    //        if (offset == 0 && rotation > 1) vsp = top + scrollines;   //make non-valid
    d[0] = vsp >> 8;        //VSP
    d[1] = vsp;
    WriteCmdParamN(is8347 ? 0x14 : 0x37, 2, d);
    if (is8347) {
      d[0] = (offset != 0) ? (_lcd_ID == 0x8347 ? 0x02 : 0x08) : 0;
      WriteCmdParamN(_lcd_ID == 0x8347 ? 0x18 : 0x01, 1, d);  //HX8347-D
    } else if (offset == 0 && (_lcd_capable & MIPI_DCS_REV1)) {
      WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
    }
    return;
  }
  // cope with 9320 style variants:
  switch (_lcd_ID) {
    default:
      // 0x6809, 0x9320, 0x9325, 0x9335, 0xB505 can only scroll whole screen
      WriteCmdData(0x61, (1 << 1) | _lcd_rev);        //!NDL, VLE, REV
      WriteCmdData(0x6A, vsp);        //VL#
      break;
  }
}


void MCUFRIEND_kbv::invertDisplay(boolean i)
{
  WriteCmdData(0x61, _lcd_rev);
}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F
static void init_table(const void *table, int16_t size)
{
  uint8_t *p = (uint8_t *) table, dat[24];            //R61526 has GAMMA[22]
  
  while (size > 0) {
    uint8_t cmd = pgm_read_byte(p++);
    uint8_t len = pgm_read_byte(p++);
    if (cmd == TFTLCD_DELAY8) {
      delay(len);
      len = 0;
    } else {
      for (uint8_t i = 0; i < len; i++)
        dat[i] = pgm_read_byte(p++);
      WriteCmdParamN(cmd, len, dat);
    }
    size -= len + 2;
  }
}

static void init_table16(const void *table, int16_t size)
{
  uint16_t *p = (uint16_t *) table;
  while (size > 0) {
    uint16_t cmd = pgm_read_word(p++);
    uint16_t d = pgm_read_word(p++);
    if (cmd == TFTLCD_DELAY)
      delay(d);
    else {
      writecmddata(cmd, d);                      //static function
    }
    size -= 2 * sizeof(int16_t);
  }
}

void MCUFRIEND_kbv::begin(uint16_t ID)
{
  int16_t *p16;               //so we can "write" to a const protected variable.
  const uint8_t *table8_ads = NULL;
  int16_t table_size;
  reset();
  _lcd_xor = 0;
  
  _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_BGR;
common_9481:
  static const uint8_t ILI9481_regValues[] PROGMEM = {    // Atmel MaxTouch
    0xB0, 1, 0x00,              // unlocks E0, F0
    0xB3, 4, 0x02, 0x00, 0x00, 0x00, //Frame Memory, interface [02 00 00 00]
    0xB4, 1, 0x00,              // Frame mode [00]
    0xD0, 3, 0x07, 0x42, 0x18,  // Set Power [00 43 18] x1.00, x6, x3
    0xD1, 3, 0x00, 0x07, 0x10,  // Set VCOM  [00 00 00] x0.72, x1.02
    0xD2, 2, 0x01, 0x02,        // Set Power for Normal Mode [01 22]
    0xD3, 2, 0x01, 0x02,        // Set Power for Partial Mode [01 22]
    0xD4, 2, 0x01, 0x02,        // Set Power for Idle Mode [01 22]
    0xC0, 5, 0x12, 0x3B, 0x00, 0x02, 0x11, //Panel Driving BGR for 1581 [10 3B 00 02 11]
    0xC1, 3, 0x10, 0x10, 0x88,  // Display Timing Normal [10 10 88]
    0xC5, 1, 0x03,      //Frame Rate [03]
    0xC6, 1, 0x02,      //Interface Control [02]
    0xC8, 12, 0x00, 0x32, 0x36, 0x45, 0x06, 0x16, 0x37, 0x75, 0x77, 0x54, 0x0C, 0x00,
    0xCC, 1, 0x00,      //Panel Control [00]
  };
  table8_ads = ILI9481_regValues, table_size = sizeof(ILI9481_regValues);
  p16 = (int16_t *) & HEIGHT;
  *p16 = 480;
  p16 = (int16_t *) & WIDTH;
  *p16 = 320;
  
  _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0);
  if (table8_ads != NULL) {
    static const uint8_t reset_off[] PROGMEM = {
      0x01, 0,            //Soft Reset
      TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
      0x28, 0,            //Display Off
      0x3A, 1, 0x55,      //Pixel read=565, write=565.
    };
    static const uint8_t wake_on[] PROGMEM = {
      0x11, 0,            //Sleep Out
      TFTLCD_DELAY8, 150,
      0x29, 0,            //Display On
    };
    init_table(&reset_off, sizeof(reset_off));
    init_table(table8_ads, table_size);   //can change PIXFMT
    init_table(&wake_on, sizeof(wake_on));
  }
  setRotation(0);             //PORTRAIT
  invertDisplay(false);
  
}


/* start of .cpp file */

#ifndef MCUFRIEND_KBV_H_
#define MCUFRIEND_KBV_H_   298

//#include "Adafruit_GFX.h"

class MCUFRIEND_kbv : public Adafruit_GFX {
  
public:
  MCUFRIEND_kbv(int CS=0, int RS=0, int WR=0, int RD=0, int RST=0);  //dummy arguments
  void     reset(void);                                       // you only need the constructor
  void     begin(uint16_t ID = 0x9341);                       // you only need the constructor
  virtual void     drawPixel(int16_t x, int16_t y, uint16_t color);  // and these three
  void     WriteCmdData(uint16_t cmd, uint16_t dat);                 // ?public methods !!!
  void     pushCommand(uint16_t cmd, uint8_t * block, int8_t N);
  void blit(int16_t x1, int16_t y1, int16_t x2, int16_t y2, byte* dat, uint16_t* colors);
  void blit_man(int16_t x, int16_t y, int16_t y1, byte* dat, uint16_t color, int rev);
  void blit_x_init(int x, int end);
  void blit_x_fwd(int x, byte d, uint16_t color);
  void blit_x_rev(int x, byte d, uint16_t color);
  void blit_x_level(int x1, int x2, int y, int y1, byte* dat, uint16_t* colors, bool left);
  void set_range_y(int y);
  void blit_man_erase(
            int prev_x, int prev_y, int curr_x, int curr_y,
            byte* level, const uint32_t* gfx, const uint8_t* blits, const uint16_t* man,
            uint16_t* const colors, uint16_t color, int dir);
  virtual void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  virtual void     fillScreen(uint16_t color)                                     { fillRect(0, 0, _width, _height, color); }
  virtual void     setRotation(uint8_t r);
  virtual void     invertDisplay(boolean i);
  
  uint16_t readReg(uint16_t reg, int8_t index=0);
  int16_t  readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h);
  void     setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
  void     pushColors(uint16_t *block, int16_t n, bool first);
  void     pushColors(uint8_t *block, int16_t n, bool first);
  void     pushColors(const uint8_t *block, int16_t n, bool first, bool bigend = false);
  void     vertScroll(int16_t top, int16_t scrollines, int16_t offset);
  
protected:
  uint32_t readReg32(uint16_t reg);
  uint32_t readReg40(uint16_t reg);
  uint16_t  _lcd_xor, _lcd_capable;
  
private:
  uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;
};

// New color definitions.  thanks to Bodmer
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

#endif

#if !defined(USE_SPECIAL) || defined (USE_SPECIAL_FAIL)

#if 0
//################################### UNO ##############################
#elif defined(__AVR_ATmega328P__)       //regular UNO shield on UNO
#define RD_PORT PORTC
#define RD_PIN  0
#define WR_PORT PORTC
#define WR_PIN  1
#define CD_PORT PORTC
#define CD_PIN  2
#define CS_PORT PORTC
#define CS_PIN  3
#define RESET_PORT PORTC
#define RESET_PIN  4

#define BMASK         0x03              //more intuitive style for mixed Ports
#define DMASK         0xFC              //does exactly the same as previous
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) & BMASK); PORTD = (PORTD & ~DMASK) | ((x) & DMASK); }
#define read_8()      ( (PINB & BMASK) | (PIND & DMASK) )
#define setWriteDir() { DDRB |=  BMASK; DDRD |=  DMASK; }
#define setReadDir()  { DDRB &= ~BMASK; DDRD &= ~DMASK; }
#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))


//################################### UNO SHIELD on BOBUINO ##############################
#elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__) //UNO shield on BOBUINO
#warning regular UNO shield on BOBUINO
#define RD_PORT PORTA
#define RD_PIN  7
#define WR_PORT PORTA
#define WR_PIN  6
#define CD_PORT PORTA
#define CD_PIN  5
#define CS_PORT PORTA
#define CS_PIN  4
#define RESET_PORT PORTA
#define RESET_PIN  3

#define BMASK         0x0F              //
#define DMASK         0x6C              //
#define write_8(x)    { PORTB = (PORTB & ~BMASK) | ((x) >> 4); \
PORTD = (PORTD & ~DMASK) | ((x) & 0x0C) | (((x) & 0x03) << 5); }
#define read_8()      ( (PINB << 4) | (PIND & 0x0C) | ((PIND & 0x60) >> 5) )
#define setWriteDir() { DDRB |=  BMASK; DDRD |=  DMASK; }
#define setReadDir()  { DDRB &= ~BMASK; DDRD &= ~DMASK; }
#define write8(x)     { write_8(x); WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))


//####################################### STM32 ############################
// NUCLEO:   ARDUINO_NUCLEO_xxxx from ST Core or ARDUINO_STM_NUCLEO_F103RB from MapleCore
// BLUEPILL: ARDUINO_NUCLEO_F103C8 / ARDUINO_BLUEPILL_F103C8 from ST Core or ARDUINO_GENERIC_STM32F103C from MapleCore
// MAPLE_REV3: n/a from ST Core or ARDUINO_MAPLE_REV3 from MapleCore
// ST Core:   ARDUINO_ARCH_STM32
// MapleCore: __STM32F1__
#elif defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32)   //MapleCore or ST Core
#define IS_NUCLEO64 ( defined(ARDUINO_STM_NUCLEO_F103RB) \
|| defined(ARDUINO_NUCLEO_F030R8) || defined(ARDUINO_NUCLEO_F091RC) \
|| defined(ARDUINO_NUCLEO_F103RB) || defined(ARDUINO_NUCLEO_F303RE) \
|| defined(ARDUINO_NUCLEO_F401RE) || defined(ARDUINO_NUCLEO_F411RE) \
|| defined(ARDUINO_NUCLEO_F446RE) || defined(ARDUINO_NUCLEO_L053R8) \
|| defined(ARDUINO_NUCLEO_L152RE) || defined(ARDUINO_NUCLEO_L476RG) \
)
// F1xx, F4xx, L4xx have different registers and styles.  General Macros
#if defined(__STM32F1__)   //weird Maple Core
#define REGS(x) regs->x
#else                      //regular ST Core
#define REGS(x) x
#endif
#define PIN_HIGH(port, pin)   (port)-> REGS(BSRR) = (1<<(pin))
#define PIN_LOW(port, pin)    (port)-> REGS(BSRR) = (1<<((pin)+16))
#define PIN_MODE2(reg, pin, mode) reg=(reg&~(0x3<<((pin)<<1)))|(mode<<((pin)<<1))
#define GROUP_MODE(port, reg, mask, val)  {port->REGS(reg) = (port->REGS(reg) & ~(mask)) | ((mask)&(val)); }

// Family specific Macros.  F103 needs ST and Maple compatibility
// note that ILI9320 class of controller has much slower Read cycles
#if 0
#elif defined(__STM32F1__) || defined(ARDUINO_NUCLEO_F103C8) || defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_NUCLEO_F103RB)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#if defined(__STM32F1__)  //MapleCore crts.o does RCC.  not understand regular syntax anyway
#define GPIO_INIT()
#else
#define GPIO_INIT()   { RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN; \
AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;}
#endif
#define GP_OUT(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x33333333)
#define GP_INP(port, reg, mask)           GROUP_MODE(port, reg, mask, 0x44444444)
#define PIN_OUTPUT(port, pin) {\
if (pin < 8) {GP_OUT(port, CRL, 0xF<<((pin)<<2));} \
else {GP_OUT(port, CRH, 0xF<<((pin&7)<<2));} \
}
#define PIN_INPUT(port, pin) { \
if (pin < 8) { GP_INP(port, CRL, 0xF<<((pin)<<2)); } \
else { GP_INP(port, CRH, 0xF<<((pin&7)<<2)); } \
}

// should be easy to add F030, F091, F303, L053, ...
#elif defined(STM32F030x8)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F091xC)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F303xE)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; \
/* AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1; */ }

#elif defined(STM32F401xE)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F411xE)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32F446xx)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32L053xx)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32L152xE)
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#elif defined(STM32L476xx)
#define WRITE_DELAY { WR_ACTIVE; WR_ACTIVE; }
#define READ_DELAY  { RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE; }
#define GPIO_INIT()   { RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN; }
#define PIN_OUTPUT(port, pin) PIN_MODE2((port)->MODER, pin, 0x1)

#else
#error unsupported STM32
#endif

#if 0
#elif defined(ARDUINO_GENERIC_STM32F103C) || defined(ARDUINO_NUCLEO_F103C8) || defined(ARDUINO_BLUEPILL_F103C8)
#warning Uno Shield on BLUEPILL
#define RD_PORT GPIOB
//#define RD_PIN  5
#define RD_PIN  0  //hardware mod to Adapter.  Allows use of PB5 for SD Card
#define WR_PORT GPIOB
#define WR_PIN  6
#define CD_PORT GPIOB
#define CD_PIN  7
#define CS_PORT GPIOB
#define CS_PIN  8
#define RESET_PORT GPIOB
#define RESET_PIN  9

// configure macros for the data pins
#define write_8(d)    { GPIOA->REGS(BSRR) = 0x00FF << 16; GPIOA->REGS(BSRR) = (d) & 0xFF; }
#define read_8()      (GPIOA->REGS(IDR) & 0xFF)
//                                         PA7 ..PA0
#define setWriteDir() {GP_OUT(GPIOA, CRL, 0xFFFFFFFF); }
#define setReadDir()  {GP_INP(GPIOA, CRL, 0xFFFFFFFF); }

#elif IS_NUCLEO64 // Uno Shield on NUCLEO
#warning Uno Shield on NUCLEO
#define RD_PORT GPIOA
#define RD_PIN  0
#define WR_PORT GPIOA
#define WR_PIN  1
#define CD_PORT GPIOA
#define CD_PIN  4
#define CS_PORT GPIOB
#define CS_PIN  0
#define RESET_PORT GPIOC
#define RESET_PIN  1

// configure macros for the data pins
#define write_8(d) { \
GPIOA->REGS(BSRR) = 0x0700 << 16; \
GPIOB->REGS(BSRR) = 0x0438 << 16; \
GPIOC->REGS(BSRR) = 0x0080 << 16; \
GPIOA->REGS(BSRR) = (  ((d) & (1<<0)) << 9) \
| (((d) & (1<<2)) << 8) \
| (((d) & (1<<7)) << 1); \
GPIOB->REGS(BSRR) = (  ((d) & (1<<3)) << 0) \
| (((d) & (1<<4)) << 1) \
| (((d) & (1<<5)) >> 1) \
| (((d) & (1<<6)) << 4); \
GPIOC->REGS(BSRR) = (  ((d) & (1<<1)) << 6); \
}

#define read_8() (       (  (  (GPIOA->REGS(IDR) & (1<<9)) >> 9) \
| ((GPIOC->REGS(IDR) & (1<<7)) >> 6) \
| ((GPIOA->REGS(IDR) & (1<<10)) >> 8) \
| ((GPIOB->REGS(IDR) & (1<<3)) >> 0) \
| ((GPIOB->REGS(IDR) & (1<<5)) >> 1) \
| ((GPIOB->REGS(IDR) & (1<<4)) << 1) \
| ((GPIOB->REGS(IDR) & (1<<10)) >> 4) \
| ((GPIOA->REGS(IDR) & (1<<8))  >> 1)))


#if defined(ARDUINO_NUCLEO_F103RB) || defined(ARDUINO_STM_NUCLEO_F103RB) //F103 has unusual GPIO modes
//                                 PA10,PA9,PA8                       PB10                   PB5,PB4,PB3                             PC7
#define setWriteDir() {GP_OUT(GPIOA, CRH, 0xFFF); GP_OUT(GPIOB, CRH, 0xF00); GP_OUT(GPIOB, CRL, 0xFFF000); GP_OUT(GPIOC, CRL, 0xF0000000); }
#define setReadDir()  {GP_INP(GPIOA, CRH, 0xFFF); GP_INP(GPIOB, CRH, 0xF00); GP_INP(GPIOB, CRL, 0xFFF000); GP_INP(GPIOC, CRL, 0xF0000000); }
#else      //F0xx, F3xx, F4xx, L0xx, L1xx, L4xx use MODER
//                                   PA10,PA9,PA8           PB10,PB5,PB4,PB3                      PC7
#define setWriteDir() { setReadDir(); \
GPIOA->MODER |=  0x150000; GPIOB->MODER |=  0x100540; GPIOC->MODER |=  0x4000; }
#define setReadDir()  { GPIOA->MODER &= ~0x3F0000; GPIOB->MODER &= ~0x300FC0; GPIOC->MODER &= ~0xC000; }
#endif

#elif defined(ARDUINO_MAPLE_REV3) // Uno Shield on MAPLE_REV3 board
#warning Uno Shield on MAPLE_REV3 board
#define RD_PORT GPIOC
#define RD_PIN  0
#define WR_PORT GPIOC
#define WR_PIN  1
#define CD_PORT GPIOC
#define CD_PIN  2
#define CS_PORT GPIOC
#define CS_PIN  3
#define RESET_PORT GPIOC
#define RESET_PIN  4

// configure macros for the data pins
#define write_8(d) { \
GPIOA->REGS(BSRR) = 0x0703 << 16; \
GPIOB->REGS(BSRR) = 0x00E0 << 16; \
GPIOA->REGS(BSRR) = (  ((d) & (1<<0)) << 10) \
| (((d) & (1<<2)) >> 2) \
| (((d) & (1<<3)) >> 2) \
| (((d) & (1<<6)) << 2) \
| (((d) & (1<<7)) << 2); \
GPIOB->REGS(BSRR) = (  ((d) & (1<<1)) << 6) \
| (((d) & (1<<4)) << 1) \
| (((d) & (1<<5)) << 1); \
}

#define read_8()  (     (   (  (GPIOA->REGS(IDR) & (1<<10)) >> 10) \
| ((GPIOB->REGS(IDR) & (1<<7)) >> 6) \
| ((GPIOA->REGS(IDR) & (1<<0)) << 2) \
| ((GPIOA->REGS(IDR) & (1<<1)) << 2) \
| ((GPIOB->REGS(IDR) & (1<<5)) >> 1) \
| ((GPIOB->REGS(IDR) & (1<<6)) >> 1) \
| ((GPIOA->REGS(IDR) & (1<<8)) >> 2) \
| ((GPIOA->REGS(IDR) & (1<<9)) >> 2)))

//                                 PA10,PA9,PA8                   PA1,PA0                     PB7,PB6,PB5
#define setWriteDir() {GP_OUT(GPIOA, CRH, 0xFFF); GP_OUT(GPIOA, CRL, 0xFF); GP_OUT(GPIOB, CRL, 0xFFF00000); }
#define setReadDir()  {GP_INP(GPIOA, CRH, 0xFFF); GP_INP(GPIOA, CRL, 0xFF); GP_INP(GPIOB, CRL, 0xFFF00000); }

#else
#error REGS group
#endif

#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; WR_IDLE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

//################################### ESP32 ##############################
#elif defined(ESP32)       //regular UNO shield on TTGO D1 R32 (ESP32)
#define LCD_RD  2  //LED
#define LCD_WR  4
#define LCD_RS 15  //hard-wired to A2 (GPIO35)
#define LCD_CS 33  //hard-wired to A3 (GPIO34)
#define LCD_RST 32 //hard-wired to A4 (GPIO36)

#define LCD_D0 12
#define LCD_D1 13
#define LCD_D2 26
#define LCD_D3 25
#define LCD_D4 17
#define LCD_D5 16
#define LCD_D6 27
#define LCD_D7 14

#define RD_PORT GPIO.out
#define RD_PIN  LCD_RD
#define WR_PORT GPIO.out
#define WR_PIN  LCD_WR
#define CD_PORT GPIO.out
#define CD_PIN  LCD_RS
#define CS_PORT GPIO.out1.val
#define CS_PIN  LCD_CS
#define RESET_PORT GPIO.out1.val
#define RESET_PIN  LCD_RST

static inline uint32_t map_8(uint32_t d)
{
  return (
      0
      | ((d & (1 << 0)) << (LCD_D0 - 0))
      | ((d & (1 << 1)) << (LCD_D1 - 1))
      | ((d & (1 << 2)) << (LCD_D2 - 2))
      | ((d & (1 << 3)) << (LCD_D3 - 3))
      | ((d & (1 << 4)) << (LCD_D4 - 4))
      | ((d & (1 << 5)) << (LCD_D5 - 5))
      | ((d & (1 << 6)) << (LCD_D6 - 6))
      | ((d & (1 << 7)) << (LCD_D7 - 7))
      );
}

static inline uint8_t map_32(uint32_t d)
{
  return (
      0
      | ((d & (1 << LCD_D0)) >> (LCD_D0 - 0))
      | ((d & (1 << LCD_D1)) >> (LCD_D1 - 1))
      | ((d & (1 << LCD_D2)) >> (LCD_D2 - 2))
      | ((d & (1 << LCD_D3)) >> (LCD_D3 - 3))
      | ((d & (1 << LCD_D4)) >> (LCD_D4 - 4))
      | ((d & (1 << LCD_D5)) >> (LCD_D5 - 5))
      | ((d & (1 << LCD_D6)) >> (LCD_D6 - 6))
      | ((d & (1 << LCD_D7)) >> (LCD_D7 - 7))
      );
}

static inline void write_8(uint16_t data)
{
  GPIO.out_w1tc = map_8(0xFF);  //could define once as DMASK
  GPIO.out_w1ts = map_8(data);
}

static inline uint8_t read_8()
{
  return map_32(GPIO.in);
}
static void setWriteDir()
{
  pinMode(LCD_D0, OUTPUT);
  pinMode(LCD_D1, OUTPUT);
  pinMode(LCD_D2, OUTPUT);
  pinMode(LCD_D3, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D7, OUTPUT);
}

static void setReadDir()
{
  pinMode(LCD_D0, INPUT);
  pinMode(LCD_D1, INPUT);
  pinMode(LCD_D2, INPUT);
  pinMode(LCD_D3, INPUT);
  pinMode(LCD_D4, INPUT);
  pinMode(LCD_D5, INPUT);
  pinMode(LCD_D6, INPUT);
  pinMode(LCD_D7, INPUT);
}

#define WRITE_DELAY { }
#define READ_DELAY  { }

#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; }
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define PIN_LOW(p, b)        (digitalWrite(b, LOW))
#define PIN_HIGH(p, b)       (digitalWrite(b, HIGH))
#define PIN_OUTPUT(p, b)     (pinMode(b, OUTPUT))

#else
#error MCU unsupported
#endif                          // regular UNO shields on Arduino boards

#endif                          //!defined(USE_SPECIAL) || defined (USE_SPECIAL_FAIL)

#define RD_ACTIVE  PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE    PIN_HIGH(RD_PORT, RD_PIN)
#define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)
#define WR_ACTIVE  PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE    PIN_HIGH(WR_PORT, WR_PIN)
#define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)
#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

// General macros.   IOCLR registers are 1 cycle when optimised.
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }       //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE      //PWLR=TRDL=150ns, tDDR=100ns

#if !defined(GPIO_INIT)
#define GPIO_INIT()
#endif
#define CTL_INIT()   { GPIO_INIT(); RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write16(x); CD_DATA; }
#define WriteData(x) { write16(x); }

/*
#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)
*/




#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F


#endif /* UTFTGLUE_H_ */


/*************************** END OF UTFTGLUE.h **************************************/


/* Below here is all my code - NS */

#include <SD.h>

#define MAP_WIDTH 30
#define MAP_HEIGHT 18
#define MAX_ENEMY 5

byte _men;
word _level;
long long _score;


// well excpet for this line - NS
UTFTGLUE myGLCD(0x9481, A2, A1, A3, A4, A0); // 23% of flash


/*
   Sound:
    Notes:
      highest 4 bits: note (0x0-0xB)
        lowest 4 bits: duration (2 ^ 0x0-0xF)
      highest 4 bits: special (0xC-0xF)
        0xC: set Octave, lowest 4: (0x0-0xF)
        0xD: set fractional note, lowest 4: (0x0-0xF)
        0xE: set pause (2 ^ 0x0-0xE), 0xF: tune done
        0xF: move note pointer back relative (0x1 + 0x0-0xF)

const uint16_t note_values[] PROGMEM = {
  440, // 0
  466, // 1
  494, // 2
  523, // 3
  554, // 4
  587, // 5
  622, // 6
  659, // 7
  698, // 8
  740, // 9
  784, // 10
  831  // 11
};


const uint8_t sound_values[] PROGMEM = {
  0x1 << 4 | 0x8,
  0x2 << 4 | 0x8,
  0x1 << 4 | 0x8,
  0x3 << 4 | 0x8,
  0x8 << 4 | 0x8,
  0xB << 4 | 0x8
};


template <int spk_pin = 1>
class sound {

  public:

    int duration_count;
    bool spk_state;
    int note_n;
    byte octave;
    byte frac;
    
    void init()
    {
      spk_state = false;
      duration_count = 0;
      octave = 0;
      frac = 0;
      
      pinMode(spk_pin, OUTPUT);
    }

    bool next_note()
    {
      byte b = pgm_read_byte(sound_values + note_n);
      byte freq = b >> 4;
      byte dur = b & 0x0f;

      if (freq == 0xC) { // go back relative
        note_n -= dur;
      } else if (freq == 0xD) { // set octave
        octave = dur;
      } else if (freq == 0xE) { // set fractional note
        frac = dur;
      } else if (freq == 0xF) { // silent note or stop
        
      } else {
        play_note(freq, dur);
      }

      note_n += 1;

      return true;
    }

    void start_tune()
    {
      cli(); // stop interrupts
      stop_tune();
      note_n = 0;
      duration_count = 0;
      TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
      next_note();
      sei(); // start interrupts
    }

    void stop_tune()
    {
      TCCR1A = 0; // set entire TCCR1A register to 0
      TCCR1B = 0; // set prescaler 1 to zero to stop
    }

    void pulse_spkr()
    {
      digitalWrite(spk_pin, spk_state);
      byte x, y;
      byte dig;
      
      if (!spk_state) {
        //pinMode(A5, INPUT);
        //user_input(&x, &y, dig);
        //pinMode(A5, OUTPUT);
        //if (dig != 0) exit(3);
      }
      
      spk_state = !spk_state;
      duration_count--;
    }

    void play_note(int freq, int dur)
    {
      long d = 1 << dur;
      long f = pgm_read_word(note_values + freq);

      TCNT1 = 0; // initialize counter value to 0

      if (f == -1) { // silent note
        duration_count = d;
        OCR1A = 16000 - 1; // set compare match register for X Hz
        TCCR1B = (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
      } else { // audible note
        long int ps_n;
        int ps_bits;

        ps_bits = 1 << WGM12;

        if (f >= 245) {
          // prescaler of 1
          ps_n = 1;
          ps_bits |= (0 << CS12) | (0 << CS11) | (1 << CS10);
        } else if (freq >= 31) {
          // prescaler of 8
          ps_n = 8;
          ps_bits |= (0 << CS12) | (1 << CS11) | (0 << CS10);
        }

        duration_count = int( 2L * (long(d) * long(f)) / 1000L );
        OCR1A = 8000000L / (ps_n * f) - 1; // set compare match register for X Hz
        TCCR1B = ps_bits;
      }
    }
};
sound<> _sound;


ISR(TIMER1_COMPA_vect)
{
  _sound.pulse_spkr();

  if (_sound.duration_count <= 0) {
    _sound.next_note();
  }
}
*/

template <class gen>
byte sign(gen x1, gen x2)
{
  if (x1 < x2) {
    return byte(-1);
  } else if (x1 > x2) {
    return byte(1);
  } else {
    return byte(0);
  }
}

template <class gen>
class point {
  public:

    gen x, y;

    inline point ()
    {
    }

    inline point (gen p_x, gen p_y)
    {
      x = p_x;
      y = p_y;
    }

    inline point operator + (point p)
    {
      return point(x + p.x, y + p.y);
    }

    inline point operator += (point p)
    {
      x += p.x;
      y += p.y;
    }

    inline bool operator == (point p)
    {
      return (x == p.x) && (y == p.y);
    }

    inline bool operator != (point p)
    {
      return (x != p.x) || (y != p.y);
    }

};
typedef point<char> pointb;
typedef point<int> pointw;


template <byte dig_size = 16, byte dig_states = 4, byte time0 = 60, byte time1 = 10, byte divide_by = 2>
class dig_log {

    class dig_type {
      public:

        byte frame;
        pointb loc;

        dig_type()
        {
        }

        dig_type(byte f, byte x, byte y)
        {
          frame = f;
          loc.x = x;
          loc.y = y;
        }
    };

    dig_type digs[dig_size];
    byte states[dig_states]; // beginning of block of contiguous states
    byte frame;
    byte divider;

    bool past(byte st)
    {
      if (states[st] != states[st - (byte)1]) {
        byte f = digs[states[st]].frame + time0 + byte(time1 * (st - (byte)1));

        //print_num(f, 10);

        if (frame > f) {
          return frame - f < (byte)128;
        } else {
          return f - frame >= (byte)128;
        }
      }

      return false;

      //print_num(frame, 0);

      // return (states[st] != states[st - 1]) && (frame == f);
    }

    byte inc_state(byte n)
    {
      byte ret = states[n];
      states[n] = (states[n] + (byte)1) % dig_size;
      return ret;
    }

  public:

    dig_log()
    {
    }

    void init()
    {
      frame = 0;
      divider = 1;

      for (int i = 0; i < dig_states; i++) {
        states[i] = 0;
      }
    }

    void log_ditch(pointb loc)
    {
      digs[inc_state(0)] = dig_type(frame, loc.x, loc.y);
    }

    bool check_states(pointb& pos, byte& st)
    {
      bool ret = false;

      if (divider >= divide_by) {
        for (byte i = (byte)1; i <= dig_states - (byte)1; i++) {
          if (past(i)) {
            pos = digs[inc_state(i)].loc;
            st = i;
            ret = true;
            break;
          }
        }

        divider = 0;
        frame++;
      }

      divider++;
      return ret;
    }

};
dig_log<> _dig_log;

/*

  Level map:
  0, // 0 brick blasted 0 (empty)
  6, // 1 brick healed 1
  7, // 2 brick healed 2
  0, // 3 blank
  0, // 4 safety ladder
  2, // 5 goal
  5, // 6 trap door
  3, // 7 bar
  1, // 8 ladder
  5, // 9 diggable brick
  4, // 10 undiggable brick
  4, // 11 unused
  4, // 12 unused
  4, // 13 unused
  4, // 14 unused
  4  // 15 unused

  Level graphics:
  0: blank
  1: ladder
  2: goal
  3: bar
  4: undiggable brick
  5: diggable brick
  6: brick healed 1
  7: brick healed 2
  8: bottom border

*/


/********************************************************
*********************************************************
**                                                     **
**                  Graphics data                      **
**                                                     **
*********************************************************
*********************************************************/

const uint16_t gfx_colors[] = {
  //  00         01        10        11
  TFT_BLACK, TFT_WHITE, TFT_CYAN, TFT_RED
};

const uint16_t sound_goal[] PROGMEM = {

};

const uint16_t gfx_man[] PROGMEM = {

  //0123456789ABCDEF
  0b0000001100000000, // 0       0 run 0
  0b0000011110000000, // 1
  0b0000011110000000, // 2
  0b0000011110000000, // 3
  0b0000000110000000, // 4
  0b0000011111000000, // 5
  0b0001111111100000, // 6
  0b0011011111110000, // 7
  0b0110011111110000, // 8
  0b0000000110000000, // 9
  0b0000111110000000, // A
  0b0000111110000000, // B
  0b0000111111000000, // C
  0b0000000011110000, // D
  0b0000000011100000, // E
  0b0000000011000000, // F

  //0123456789ABCDEF
  0b0000001100000000, // 0       1 run 1
  0b0000011110000000, // 1
  0b0000011110000000, // 2
  0b0000011110000000, // 3
  0b0000000110000000, // 4
  0b0000011111100000, // 5
  0b0001111110011000, // 6
  0b0110011110001100, // 7
  0b0000011110000000, // 8
  0b0000000110000000, // 9
  0b0000000111111100, // A
  0b0000011001111100, // B
  0b0001100000000000, // C
  0b0001100000000000, // D
  0b0001100000000000, // E
  0b0001100000000000, // F

  //0123456789ABCDEF
  0b0000001100000000, // 0       2 run 2
  0b0000011110000000, // 1
  0b0000011110000000, // 2
  0b0000011110000000, // 3
  0b0000000110000000, // 4
  0b0000011111100000, // 5
  0b0010011110011000, // 6
  0b0011111110001100, // 7
  0b0000011110000000, // 8
  0b0000000110000000, // 9
  0b0000000110000000, // A
  0b0000011011000000, // B
  0b0001100001100000, // C
  0b0001100000110000, // D
  0b0000000000011000, // E
  0b0000000000011000, // F

  //0123456789ABCDEF
  0b0011000011000011, // 0       3 fall 0
  0b0011000111100011, // 1
  0b0011000111100011, // 2
  0b0011000111100011, // 3
  0b0001111111111110, // 4
  0b0000000001100000, // 5
  0b0000000001100000, // 6
  0b0000000001100000, // 7
  0b0000011111100000, // 8
  0b0000110001100000, // 9
  0b0000110001100000, // A
  0b0000110001100000, // B
  0b0000110001100000, // C
  0b0000110001100000, // D
  0b0000000001100000, // E
  0b0000000001100000,  // F

  //0123456789ABCDEF
  0b0000000000000000, // 0       4 climb 0
  0b0000001111000000, // 1
  0b0000001111000000, // 2
  0b0000001111000110, // 3
  0b0000001111111110, // 4
  0b0110001111100000, // 5
  0b0110001111100000, // 6
  0b0111111111100000, // 7
  0b0000001111100000, // 8
  0b0000001111100000, // 9
  0b0000011100110000, // A
  0b0000111000111000, // B
  0b0000111000111100, // C
  0b0000111000000000, // D
  0b0001111000000000, // E
  0b0011111000000000,  // F

  //0123456789ABCDEF
  0b00011000000000000, // 0       5 monkey 0 (drop backward arm, move +0)
  0b00011000000000000, // 1
  0b00011000000000000, // 2
  0b0001100111000000, // 3
  0b0001100111000000, // 4
  0b0000110111000000, // 5
  0b0000011111111000, // 6
  0b0000000111001110, // 7
  0b0000000111000111, // 8
  0b0000000111000000, // 9
  0b0000011111100000, // A
  0b0000110000110000, // B
  0b0000110000110000, // C
  0b0000110000110000, // D
  0b0000110000110000, // E
  0b0000110000110000, // F

  //0123456789ABCDEF
  0b0000000000011000, // 0       6 monkey 1 (swing forward, move +8)
  0b0000000000011000, // 1
  0b0000000000011000, // 2
  0b0000001110011000, // 3
  0b1110001110011000, // 4
  0b0011101110110000, // 5
  0b0001111111100000, // 6
  0b0000001110000000, // 7
  0b0000001110000000, // 8
  0b0000001110000000, // 9
  0b0000111111000000, // A
  0b0001100001100000, // B
  0b0001100001100000, // C
  0b0001100001100000, // D
  0b0001100001100000, // E
  0b0001100001100000, // F

  //0123456789ABCDEF
  0b0001100000011000, // 0       7 monkey 2 (grab arm with forward hand to hang, move +0)
  0b0001100000011000, // 1
  0b0001100000011000, // 2
  0b0001101110011000, // 3
  0b0001101110011000, // 4
  0b0000111111011000, // 5
  0b0000001111110000, // 6
  0b0000001110000000, // 7
  0b0000001110000000, // 8
  0b0000001110000000, // 9
  0b0000001111110000, // A
  0b0000001100011000, // B
  0b0000001100011000, // C
  0b0000001100011000, // D
  0b0000001100001000, // E
  0b0000000110000000, // F

  //0123456789ABCDEF
  0b0000000000000000, // 0       8 player dig 0
  0b0000000001100000, // 1
  0b0000000011100000, // 2
  0b0000000011100000, // 3
  0b0000000001100000, // 4
  0b0000000111111100, // 5
  0b0000011011101100, // 6
  0b1000110011101100, // 7
  0b1001100011100000, // 8
  0b1000000111100000, // 9
  0b0000001111111000, // A
  0b0000011000011000, // B
  0b0000011000011000, // C
  0b0000011000011000, // D
  0b0000011000011000, // E
  0b0000011000011000, // F

  //0123456789ABCDEF
  0b0000000000000000, // 0       9 enemy spawn 0
  0b0000000000000000, // 1
  0b0000000000000000, // 2
  0b0000000000000000, // 3
  0b0000000000000000, // 4
  0b0000000000000000, // 5
  0b0000000000000000, // 6
  0b0000000000000000, // 7
  0b0000000000000000, // 8
  0b0000000000000000, // 9
  0b0000111111110000, // A
  0b0000111111110000, // B
  0b1111111111111111, // C
  0b1111111111111111, // D
  0b0000000000000000, // E
  0b0000000000000000, // F

  //0123456789ABCDEF
  0b0000000000000000, // 0       10 enemy spawn 1
  0b0000000000000000, // 1
  0b0000000000000000, // 2
  0b0000000000000000, // 3
  0b0000000000000000, // 4
  0b0000000000000000, // 5
  0b0000000000000000, // 6
  0b0000111111110000, // 7
  0b0000111111110000, // 8
  0b1111111111111111, // 9
  0b1111111111111111, // A
  0b1111111111111111, // B
  0b1111111111111111, // C
  0b1111111111111111, // D
  0b0000000000000000, // E
  0b0000000000000000, // F

  //0123456789ABCDEF
  0b0000000000000000, // 0       11 man blank
  0b0000000000000000, // 1
  0b0000000000000000, // 2
  0b0000000000000000, // 3
  0b0000000000000000, // 4
  0b0000000000000000, // 5
  0b0000000000000000, // 6
  0b0000000000000000, // 7
  0b0000000000000000, // 8
  0b0000000000000000, // 9
  0b0000000000000000, // A
  0b0000000000000000, // B
  0b0000000000000000, // C
  0b0000000000000000, // D
  0b0000000000000000, // E
  0b0000000000000000  // F

};

const uint16_t gfx_nums[] PROGMEM = {

  //0123456789ABCDEF
  0b0011111111111100, // 0       0
  0b0011111111111100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011100000011100, // 6
  0b0011100011111100, // 7
  0b0011100011111100, // 8
  0b0011100011111100, // 9
  0b0011100011111100, // A
  0b0011100011111100, // B
  0b0011100011111100, // C
  0b0011100011111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0001111111000000, // 0       1
  0b0001111111000000, // 1
  0b0001111111000000, // 2
  0b0001111111000000, // 3
  0b0000000111000000, // 4
  0b0000000111000000, // 5
  0b0000000111000000, // 6
  0b0000000111000000, // 7
  0b0000000111000000, // 8
  0b0000000111000000, // 9
  0b0000000111000000, // A
  0b0000000111000000, // B
  0b0001111111111000, // C
  0b0001111111111000, // D
  0b0001111111111000, // E
  0b0001111111111000, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       2
  0b0011111111111100, // 1
  0b0011110000011100, // 2
  0b0011110000011100, // 3
  0b0000000000011100, // 4
  0b0000000000011100, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0011110000000000, // 8
  0b0011110000000000, // 9
  0b0011110000000000, // A
  0b0011110000000000, // B
  0b0011110001111100, // C
  0b0011110001111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       3
  0b0011111111111100, // 1
  0b0011100000111100, // 2
  0b0011100000111100, // 3
  0b0000000000111100, // 4
  0b0000000000111100, // 5
  0b0000111111111100, // 6
  0b0000111111111100, // 7
  0b0000000000111100, // 8
  0b0000000000111100, // 9
  0b0000000000111100, // A
  0b0000000000111100, // B
  0b0011100000111100, // C
  0b0011100000111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111000011100, // 0       4
  0b0011111000011100, // 1
  0b0011111000011100, // 2
  0b0011111000011100, // 3
  0b0011111000011100, // 4
  0b0011111000011100, // 5
  0b0011111000011100, // 6
  0b0011111111111100, // 7
  0b0011111111111100, // 8
  0b0000000000011100, // 9
  0b0000000000011100, // A
  0b0000000000011100, // B
  0b0000000000011100, // C
  0b0000000000011100, // D
  0b0000000000011100, // E
  0b0000000000011100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       5
  0b0011111111111100, // 1
  0b0011100000000000, // 2
  0b0011100000000000, // 3
  0b0011100000000000, // 4
  0b0011100000000000, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0000000111111100, // 8
  0b0000000111111100, // 9
  0b0000000111111100, // A
  0b0000000111111100, // B
  0b0000000111111100, // C
  0b0000000111111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       6
  0b0011111111111100, // 1
  0b0011100000011100, // 2
  0b0011100000000000, // 3
  0b0011100000000000, // 4
  0b0011100000000000, // 5
  0b0011100000000000, // 6
  0b0011111111111100, // 7
  0b0011111111111100, // 8
  0b0011100001111100, // 9
  0b0011100001111100, // A
  0b0011100001111100, // B
  0b0011100001111100, // C
  0b0011100001111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       7
  0b0011111111111100, // 1
  0b0000000011111100, // 2
  0b0000000011111100, // 3
  0b0000000011111100, // 4
  0b0000000011111100, // 5
  0b0000000011111100, // 6
  0b0000001111100000, // 7
  0b0000001111100000, // 8
  0b0000011111000000, // 9
  0b0000011100000000, // A
  0b0000011100000000, // B
  0b0000011100000000, // C
  0b0000011100000000, // D
  0b0000011100000000, // E
  0b0000011100000000, // F

  //0123456789ABCDEF
  0b0000011111111100, // 0       8
  0b0000011111111100, // 1
  0b0000011100011100, // 2
  0b0000011100011100, // 3
  0b0000011100011100, // 4
  0b0000011100011100, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0011100000011100, // 8
  0b0011100000011100, // 9
  0b0011100000011100, // A
  0b0011100000011100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       9
  0b0011111111111100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0000000111111100, // 8
  0b0000000111111100, // 9
  0b0000000111111100, // A
  0b0000000111111100, // B
  0b0000000111111100, // C
  0b0000000111111100, // D
  0b0000000111111100, // E
  0b0000000111111100  // F
};

const uint16_t gfx_letters[] PROGMEM = {

  //0123456789ABCDEF
  0b0000011111111100, // 0       A
  0b0000011111111100, // 1
  0b0000011100011100, // 2
  0b0000011100011100, // 3
  0b0000011100011100, // 4
  0b0000011100011100, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0011100000011100, // 8
  0b0011100000011100, // 9
  0b0011100000011100, // A
  0b0011100000011100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011100001111100, // E
  0b0011100001111100, // F

  //0123456789ABCDEF
  0b0011111111100000, // 0       B
  0b0011111111100000, // 1
  0b0011100011100000, // 2
  0b0011100011100000, // 3
  0b0011100011100000, // 4
  0b0011100011100000, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0011100000011100, // 8
  0b0011100000011100, // 9
  0b0011100000011100, // A
  0b0011100000011100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       C
  0b0011111111111100, // 1
  0b0011000000011100, // 2
  0b0011000000011100, // 3
  0b0011000000000000, // 4
  0b0011000000000000, // 5
  0b0011000000000000, // 6
  0b0011000000000000, // 7
  0b0011000000000000, // 8
  0b0011000000000000, // 9
  0b0011111100000000, // A
  0b0011111100000000, // B
  0b0011111100011100, // C
  0b0011111100011100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111100000, // 0       D
  0b0011111111100000, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011100000011100, // 6
  0b0011100000011100, // 7
  0b0011111100011100, // 8
  0b0011111100011100, // 9
  0b0011111100011100, // A
  0b0011111100011100, // B
  0b0011111100011100, // C
  0b0011111100011100, // D
  0b0011111111100000, // E
  0b0011111111100000, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       E
  0b0011111111111100, // 1
  0b0011111100000000, // 2
  0b0011111100000000, // 3
  0b0011111100000000, // 4
  0b0011111100000000, // 5
  0b0011111111110000, // 6
  0b0011111111110000, // 7
  0b0011100000000000, // 8
  0b0011100000000000, // 9
  0b0011100000000000, // A
  0b0011100000000000, // B
  0b0011100000000000, // C
  0b0011100000000000, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       F
  0b0011111111111100, // 1
  0b0011111100000000, // 2
  0b0011111100000000, // 3
  0b0011111100000000, // 4
  0b0011111100000000, // 5
  0b0011111111110000, // 6
  0b0011111111110000, // 7
  0b0011100000000000, // 8
  0b0011100000000000, // 9
  0b0011100000000000, // A
  0b0011100000000000, // B
  0b0011100000000000, // C
  0b0011100000000000, // D
  0b0011100000000000, // E
  0b0011100000000000, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       G
  0b0011111111111100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000000000, // 4
  0b0011100000000000, // 5
  0b0011100000000000, // 6
  0b0011100000000000, // 7
  0b0011100111111100, // 8
  0b0011100111111100, // 9
  0b0011100111111100, // A
  0b0011100000011100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011100000011100, // 0       H
  0b0011100000011100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011100000011100, // 6
  0b0011111111111100, // 7
  0b0011111111111100, // 8
  0b0011111100011100, // 9
  0b0011111100011100, // A
  0b0011111100011100, // B
  0b0011111100011100, // C
  0b0011111100011100, // D
  0b0011111100011100, // E
  0b0011111100011100,  // F

  //0123456789ABCDEF
  0b0000011100000000, // 0       I
  0b0000011100000000, // 1
  0b0000011100000000, // 2
  0b0000011100000000, // 3
  0b0000011100000000, // 4
  0b0000011100000000, // 5
  0b0000011100000000, // 6
  0b0000011111100000, // 7
  0b0000011111100000, // 8
  0b0000011111100000, // 9
  0b0000011111100000, // A
  0b0000011111100000, // B
  0b0000011111100000, // C
  0b0000011111100000, // D
  0b0000011111100000, // E
  0b0000011111100000, // F

  //0123456789ABCDEF
  0b0000000001100000, // 0       J
  0b0000000001100000, // 1
  0b0000000001100000, // 2
  0b0000000001100000, // 3
  0b0000000001100000, // 4
  0b0000000001100000, // 5
  0b0000000001111100, // 6
  0b0000000001111100, // 7
  0b0000000001111100, // 8
  0b0000000001111100, // 9
  0b0000000001111100, // A
  0b0000000001111100, // B
  0b0011100001111100, // C
  0b0011100001111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b1100000000001100, // 0       K
  0b1100000000001100, // 1
  0b1100000011111100, // 2
  0b1100000011000000, // 3
  0b1100000011000000, // 4
  0b1111111111000000, // 5
  0b1111111111000000, // 6
  0b1111111111000000, // 7
  0b1111111111111100, // 8
  0b1111111111111100, // 9
  0b1111111111111100, // A
  0b1111110000001100, // B
  0b1111110000001100, // C
  0b1111110000001100, // D
  0b1111110000001100, // E
  0b1111110000001100, // F

  //0123456789ABCDEF
  0b0011100000000000, // 0       L
  0b0011100000000000, // 1
  0b0011100000000000, // 2
  0b0011100000000000, // 3
  0b0011100000000000, // 4
  0b0011100000000000, // 5
  0b0011100000000000, // 6
  0b0011100000000000, // 7
  0b0011111110000000, // 8
  0b0011111110000000, // 9
  0b0011111110000000, // A
  0b0011111110000000, // B
  0b0011111110000000, // C
  0b0011111110000000, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011100000000000, // 0       M
  0b0011100000000000, // 1
  0b0011100000000000, // 2
  0b0011111100000000, // 3
  0b0011111100000000, // 4
  0b0011111100000000, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0011100000011100, // 8
  0b0011100000011100, // 9
  0b0011100000011100, // A
  0b0011100000011100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011100000011100, // E
  0b0011100000011100, // F

  //0123456789ABCDEF
  0b0011100000011100, // 0       N
  0b0011100000011100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011111100011100, // 4
  0b0011111100011100, // 5
  0b0011111100011100, // 6
  0b0011111111111100, // 7
  0b0011100011111100, // 8
  0b0011100011111100, // 9
  0b0011100011111100, // A
  0b0011100011111100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011100000011100, // E
  0b0011100000011100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       O
  0b0011111111111100, // 1
  0b0011100011111100, // 2
  0b0011100011111100, // 3
  0b0011100011111100, // 4
  0b0011100011111100, // 5
  0b0011100011111100, // 6
  0b0011100000011100, // 7
  0b0011100000011100, // 8
  0b0011100000011100, // 9
  0b0011100000011100, // A
  0b0011100000011100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       P
  0b0011111111111100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011100000011100, // 6
  0b0011111111111100, // 7
  0b0011111111111100, // 8
  0b0011111100000000, // 9
  0b0011111100000000, // A
  0b0011111100000000, // B
  0b0011111100000000, // C
  0b0011111100000000, // D
  0b0011111100000000, // E
  0b0011111100000000, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       Q
  0b0011111111111100, // 1
  0b0011000011111100, // 2
  0b0011000011111100, // 3
  0b0011000011111100, // 4
  0b0011000000001100, // 5
  0b0011000000001100, // 6
  0b0011000000001100, // 7
  0b0011000000001100, // 8
  0b0011000000001100, // 9
  0b0011000000001100, // A
  0b0011000000001100, // B
  0b0011000001110000, // C
  0b0011000001110000, // D
  0b0011111100011100, // E
  0b0011111100011100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       R
  0b0011111111111100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011100000011100, // 6
  0b0011111111111100, // 7
  0b0011111111111100, // 8
  0b0011111111100000, // 9
  0b0011111111100000, // A
  0b0011111111100000, // B
  0b0011111111111100, // C
  0b0011111100011100, // D
  0b0011111100011100, // E
  0b0011111100011100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       S
  0b0011111111111100, // 1
  0b0011100000011100, // 2
  0b0011100000000000, // 3
  0b0011100000000000, // 4
  0b0011100000000000, // 5
  0b0011100000000000, // 6
  0b0011111111111100, // 7
  0b0011111111111100, // 8
  0b0000000011111100, // 9
  0b0000000011111100, // A
  0b0000000011111100, // B
  0b0000000011111100, // C
  0b0011100011111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       T
  0b0011111111111100, // 1
  0b0000011100000000, // 2
  0b0000011100000000, // 3
  0b0000011100000000, // 4
  0b0000011100000000, // 5
  0b0000011100000000, // 6
  0b0000011100000000, // 7
  0b0000011100000000, // 8
  0b0000011111100000, // 9
  0b0000011111100000, // A
  0b0000011111100000, // B
  0b0000011111100000, // C
  0b0000011111100000, // D
  0b0000011111100000, // E
  0b0000011111100000, // F

  //0123456789ABCDEF
  0b0011100000011100, // 0       U
  0b0011100000011100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011100000011100, // 6
  0b0011100000011100, // 7
  0b0011111100011100, // 8
  0b0011111100011100, // 9
  0b0011111100011100, // A
  0b0011111100011100, // B
  0b0011111100011100, // C
  0b0011111100011100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0011111000011100, // 0       V
  0b0011111000011100, // 1
  0b0011111000011100, // 2
  0b0011111000011100, // 3
  0b0011111000011100, // 4
  0b0011111000011100, // 5
  0b0011111000011100, // 6
  0b0011111000011100, // 7
  0b0011111000011100, // 8
  0b0011111000011100, // 9
  0b0011111000011100, // A
  0b0011111111100000, // B
  0b0000111111100000, // C
  0b0000111100000000, // D
  0b0000111000000000, // E
  0b0000111000000000, // F

  //0123456789ABCDEF
  0b0011100000011100, // 0       W
  0b0011100000011100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0011100000011100, // 6
  0b0011100000011100, // 7
  0b0011111111111100, // 8
  0b0011111111111100, // 9
  0b0011111111111100, // A
  0b0011111111111100, // B
  0b0011111100011100, // C
  0b0011111100011100, // D
  0b0011100000011100, // E
  0b0011100000011100, // F

  //0123456789ABCDEF
  0b0011100000011100, // 0       X
  0b0011100000011100, // 1
  0b0011100000011100, // 2
  0b0011100000011100, // 3
  0b0011100000011100, // 4
  0b0011100000011100, // 5
  0b0000011111100000, // 6
  0b0000011111100000, // 7
  0b0000011111100000, // 8
  0b0000011111100000, // 9
  0b0011100000011100, // A
  0b0011100000011100, // B
  0b0011100000011100, // C
  0b0011100000011100, // D
  0b0011100000011100, // E
  0b0011100000011100, // F

  //0123456789ABCDEF
  0b0011100011111100, // 0       Y
  0b0011100011111100, // 1
  0b0011100011111100, // 2
  0b0011100011111100, // 3
  0b0011100011111100, // 4
  0b0011100011111100, // 5
  0b0011111111111100, // 6
  0b0011111111111100, // 7
  0b0000011111100000, // 8
  0b0000011111100000, // 9
  0b0000011111100000, // A
  0b0000011111100000, // B
  0b0000011111100000, // C
  0b0000011111100000, // D
  0b0000011111100000, // E
  0b0000011111100000, // F

  //0123456789ABCDEF
  0b0011111111111100, // 0       Z
  0b0011111111111100, // 1
  0b0011000000001100, // 2
  0b0011000000001100, // 3
  0b0011000000000000, // 4
  0b0000000011100000, // 5
  0b0000000011100000, // 6
  0b0000011111100000, // 7
  0b0000011111100000, // 8
  0b0011000000000000, // 9
  0b0011000000000000, // A
  0b0011000000000000, // B
  0b0011000001111100, // C
  0b0011000001111100, // D
  0b0011111111111100, // E
  0b0011111111111100, // F

  //0123456789ABCDEF
  0b0000000011111000, // 0      (
  0b0000000011111000, // 1
  0b0000011111000000, // 2
  0b0000011111000000, // 3
  0b0011110000000000, // 4
  0b0011110000000000, // 5
  0b0011110000000000, // 6
  0b0011110000000000, // 7
  0b0011110000000000, // 8
  0b0011110000000000, // 9
  0b0011110000000000, // A
  0b0011110000000000, // B
  0b0000011111000000, // C
  0b0000011111000000, // D
  0b0000000011111000, // E
  0b0000000011111000, // F

  //0123456789ABCDEF
  0b0001111100000000, // 0       )
  0b0001111100000000, // 1
  0b0000001111100000, // 2
  0b0000001111100000, // 3
  0b0000000000111100, // 4
  0b0000000000111100, // 5
  0b0000000000111100, // 6
  0b0000000000111100, // 7
  0b0000000000111100, // 8
  0b0000000000111100, // 9
  0b0000000000111100, // A
  0b0000000000111100, // B
  0b0000001111100000, // C
  0b0000001111100000, // D
  0b0001111100000000, // E
  0b0001111100000000  // F

};


/*

  Level graphics:
  0: blank
  1: ladder
  2: goal
  3: bar
  4: undiggable brick
  5: diggable brick
  6: brick healed 1
  7: brick healed 2

*/

const byte gfx_blits[] = {
  0, // 0 brick blasted 0 (empty)
  6, // 1 brick healed 1
  7, // 2 brick healed 2
  0, // 3 blank
  0, // 4 safety ladder
  2, // 5 goal
  5, // 6 trap door
  3, // 7 bar
  1, // 8 ladder
  5, // 9 diggable brick
  4, // 10 undiggable brick
  8, // 11 border
  8, // 12 border
  8, // 13 border
  8, // 14 border
  8  // 15 border
};

const uint32_t gfx_level[] PROGMEM = {

  0b00000000000000000000000000000000,   // 0 blank
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b01010100000000000000000101010000,   // 1 ladder
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,
  0b01010101010101010101010101010000,
  0b01010101010101010101010101010000,
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,
  0b01010101010101010101010101010000,
  0b01010101010101010101010101010000,
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,
  0b01010100000000000000000101010000,

  0b00000000000000000000000000000000,   // 2 gold
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b01010101010101010101010101010000,
  0b01010111111111111111110101010000,
  0b01010111111111111111110101010000,
  0b01010111111111111111110101010000,
  0b01010111111111111111110101010000,
  0b01010111111111111111110101010000,
  0b01010111111111111111110101010000,
  0b01010101010101010101010101010000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b00000000000000000000000000000000,   // 3 bar
  0b01010101010101010101010101010101,
  0b01010101010101010101010101010101,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b11111111111111111111111111111111,   // 4 solid brick
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b11111111111111111111000011111111,   // 5 diggable brick
  0b11111111111111111111000011111111,
  0b11111111111111111111000011111111,
  0b11111111111111111111000011111111,
  0b11111111111111111111000011111111,
  0b11111111111111111111000011111111,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b11110000111111111111111111111111,
  0b11110000111111111111111111111111,
  0b11110000111111111111111111111111,
  0b11110000111111111111111111111111,
  0b11110000111111111111111111111111,
  0b11110000111111111111111111111111,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b00000000000000000000000000000000,   // 6 brick healed 0
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b11110000000000000000000000111111,
  0b11110000000000000000000000111111,
  0b11110000000000000000000000111111,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b00000000000000000000000000000000,   // 7 brick healed 1
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b11110000000000000000000000111111,
  0b11110000000000000000000000111111,
  0b11110000000000000000000000111111,
  0b11110000000000000000000000111111,
  0b11110000000000000000000000111111,
  0b11110000000000000000000000111111,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b11111111111111111111111111111111,   // 8  bottom border
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,

  0b00000000000000000000000000000000,   // 9  dig animation 0
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000011110000000000,
  0b00000000000000000011000000000000,
  0b00000011110011001111001010000000,
  0b00000000110000001111001010000000,
  0b00000000001111000010100000000000,
  0b00000000001111000010100000000000,
  0b00000011110000101000000000000000,

  0b00000000000000000000000000000000,   // 10  dig animation 0 reverse
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000011110000000000000000000,
  0b00000000000011000000000000000000,
  0b00000001010011110011001111000000,
  0b00000001010011110000001100000000,
  0b00000000000101000011110000000000,
  0b00000000000101000011110000000000,
  0b00000000000000010100001111000000,

  0b00000000000000000000000000000000,   // 11  dig animation 1
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000011000000000000,
  0b00000000000000000000000000000000,
  0b00000000000011000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000001100000000000000,
  0b00001100000000000000001111000000,
  0b00000000000000000000001111000000,
  0b00000000000011110000000000000000,
  0b00001100000011110000000000000000,
  0b00111100000000000011000000000000,
  0b00000000000011110000001100000000,
  0b00000000000000001100001111000000,
  0b00111100000000110000000000000000,
  0b00111100000000000000001111000000,
  0b00000000000010100010100011000000,
  0b00000000101010100010100000000000,
  0b11001010101010101010100000111111,

  0b00000000000000000000000000000000,   // 12  dig animation 2
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000011110000000000,
  0b00000000000000000011110000000000,
  0b00001100000011110000000000000000,
  0b00001100000011110000000000000000,
  0b00000000000000000000111100000000,
  0b00000011110000001100111100000000,
  0b00000011110000000000000000000000,
  0b00000000000000001111000000000000,
  0b00000011110010101010000000000000,
  0b11110011110010101010001111000000,
  0b00111010101010101010001111000000,
  0b00001010101010101010101010000000,

  0b00000000000000000000000000000000,   // 13  dig animation 3
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000010101010000000000000,
  0b00000000000010101010000000000000,
  0b11110000111110101010101010000000,
  0b11111010101010101010101010000000,

  0b00000000000000000000000000000000,   // 14  dig animation 4
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000000000000000000000000,
  0b00000000000010101010000000000000,
  0b00000000000010101010000000000000,
  0b00001010101010101010000000000000,
  0b00001010101010101010101010000000

};


class level_map {
    byte level[MAP_WIDTH * (MAP_HEIGHT + 2) / 2];
    byte enemy_ct;
    char dig_progress;

  public:

    int goal_ct;

    bool safety_done;

    level_map()
    {
    }

    byte enemy_count()
    {
      return enemy_ct;
    }

    void dig()
    {
      if (dig_progress == -1) {
        dig_progress = 0;
      }
    }

    void cancel_dig(pointw pos)
    {
      dig_progress = -1;
      // void draw_tile(int x, int y, char tile = -1, int from = 0, byte h = 15)
      //draw_tile(pos.x >> 4, (pos.y >> 4), 3);
      write_map(pos.x >> 4, (pos.y >> 4) + 1, 9);
    }

    int seq(int n, bool face)
    {
      if (n == 0) {
        if (face) {
          return 16;
        } else {
          return 0;
        }
      } else if (n == 1) {
        return 16 + 16;
      } else if (n == 2) {
        return 16 + 16 + 20;
      } else if (n == 3) {
        return 16 + 16 + 20 + 24;
      } else if (n == 4) {
        return 16 + 16 + 20 + 24 + 28;
      }
    }

    bool done_digging()
    {
      return dig_progress == 5;
    }

    bool digging(pointw curr, bool face)
    {
      if (dig_progress != -1) {
        pointb loc;

        if (face) {
          loc = pointb((curr.x + 16 + 8) >> 4, (curr.y + 16 + 8) >> 4);
        } else {
          loc = pointb((curr.x - 8) >> 4, (curr.y + 16 + 8) >> 4);
        }

        //draw_tile(int x, int y, char tile = -1, int from = 0, byte h = 15)

        if (dig_progress == 5) {
          _dig_log.log_ditch(loc);
          write_map(loc.x, loc.y, 0);
          dig_progress = -1;
        } else {
          char d = seq(dig_progress, face);
          draw_tile(loc.x << 4, (loc.y << 4) - 16, 0, 9 * 16 + d, 15 + dig_progress * 4);
          dig_progress++;
        }

        return true;
      } else {
        return false;
      }
    }

    // x, y are pixel-space (NOT tile space)
    byte read_map(int x, int y)
    {
      if (x < 0 || x >= MAP_WIDTH * 16 || y < 0) {
        return 10;
      } else {
        x >>= 4;
        y >>= 4;
        return byte(level[(x + y * MAP_WIDTH) / 2] >> ((x & 1) * 4)) & (byte)0x0f;
      }
    }

    void write_map(int x, int y, byte tile, bool redraw = true, int os = 0)
    {
      int ind = (x + y * MAP_WIDTH) / 2;
      int sh = (x & 1) * 4;
      level[ind] &= (0xf0 >> sh);
      level[ind] |= (tile << sh);
      if (redraw) draw_tile(x << 4, (y << 4) + os);
    }

    void add_tile(pointb& pos, byte tile = 0, bool redraw = true, int os = 0)
    {
      write_map(pos.x, pos.y, tile, redraw, os);

      if (pos.x >= MAP_WIDTH - 1) {
        pos.x = 0;
        pos.y++;
      } else {
        pos.x++;
      }
    }

    /*
      0, // 0 brick blasted 0 (empty)
      6, // 1 brick healed 1
      7, // 2 brick healed 2
      0, // 3 blank
      0, // 4 safety ladder
      2, // 5 goal
      5, // 6 trap door
      3, // 7 bar
      1, // 8 ladder
      5, // 9 diggable brick
      4, // 10 undiggable brick
      4, // 11 unused
      4, // 12 unused
      4, // 13 unused
      4, // 14 unused
      4  // 15 unused
    */

    void draw_tile(int x, int y, char tile = -1, int from = 0, byte h = 15)
    {
      if (tile == -1) {
        byte lev = read_map(x, y);
        tile = gfx_blits[lev];
        from = 16 * tile + (y & 0x000f);
      }

      myGLCD.blit(x, y, x + 16, y + h, (byte*)&gfx_level[from], gfx_colors);
    }

    bool load_level(int level_n, pointb& player, pointb* enemy)
    {
      bool ret;
      bool mute = false;
      char fn[] = "/lr/000.txt";
      pointb pos;
      File level_file;

      for (byte i = 4 + 2; i >= 4 + 0; i--) {
        fn[i] = '0' + char(level_n % 10);
        level_n = level_n / 10;
      }

      goal_ct = 0;
      enemy_ct = 0;
      dig_progress = -1;
      pos.x = 0;
      pos.y = 0;
      player.x = player.y = 0;
      
      // open the file for reading:
      level_file = SD.open(fn);

      if (level_file) {
        // read from the file until there's nothing else in it:
        while (level_file.available()) { // && pos.y < MAP_HEIGHT) {
          unsigned char b = level_file.read();

          if (b == 10) {
            // End of line
            mute = false;
          } else if (!mute) {

            /*
              0, // 0 brick blasted 0 (empty)
              6, // 1 brick healed 1
              7, // 2 brick healed 2
              0, // 3 blank
              0, // 4 safety ladder
              2, // 5 goal
              5, // 6 trap door
              3, // 7 bar
              1, // 8 ladder
              5, // 9 diggable brick
              4, // 10 undiggable brick
              4, // 11 unused
              4, // 12 unused
              4, // 13 unused
              4, // 14 unused
              4  // 15 unused
            */

            switch (b) {
              case '!':
                // comment
                mute = true;
                break;
              case '#':
                // diggable brick
                add_tile(pos, 9);
                break;
              case '=':
                // UNdiggable brick
                add_tile(pos, 10);
                break;
              case 'H':
                // ladder
                add_tile(pos, 8);
                break;
              case '-':
                // bar
                add_tile(pos, 7);
                break;
              case '+':
                // goal
                goal_ct++;
                add_tile(pos, 5);
                break;
              case 'P':
                // Player
                player = pos;
                add_tile(pos, 3);
                break;
              case 'E':
                // Enemy
                if (enemy_ct < MAX_ENEMY)
                  enemy[enemy_ct++] = pos;
                add_tile(pos, 3);
                break;
              case 'T':
                // trap door
                add_tile(pos, 6);
                break;
              case 'S':
                // safety ladder
                add_tile(pos, 4);
                break;
              case '.':
                // space
                add_tile(pos, 3);
                break;
            }
          }
        }

        for (byte i = 0; i < MAP_WIDTH; i++) {
          add_tile(pos, 15);
        }
        for (byte i = 0; i < MAP_WIDTH; i++) {
          add_tile(pos, 3);
        }

        ret = true;
      } else {
        ret = false;
      }

      // close the file
      level_file.close();

      return ret;
    }

    void get_goal(pointw pos, bool player)
    {
      write_map(pos.x >> 4, pos.y >> 4, 3);

      if (player) {
        goal_ct--;
        inc_score(250);
      }
    }

    void put_goal(pointw pos)
    {
      write_map(pos.x >> 4, pos.y >> 4, 5);
    }

    inline bool level_clear()
    {
      return goal_ct <= 0;
    }

    void safety_ladder()
    {
      if (!safety_done) {
        for (byte y = 0; y < MAP_HEIGHT; y++) {
          for (byte x = 0; x < MAP_WIDTH; x++) {
            if (read_map(x * 16, y * 16) == 4) {
              write_map(x, y, 8);
            }
          }
        }

        safety_done = true;
      }
    }

    byte* level_address()
    {
      return level;
    }
};
level_map _map;


/*
  void blit_erase_man (int prev_x, int prev_y, int curr_x, int curr_y, uint16_t color = TFT_ORANGE, int anim = 0, int dir = 0)
  {
  myGLCD.blit_man_erase(
    prev_x, prev_y, curr_x, curr_y,
    _map.level_address(), &gfx_level[0], &gfx_blits[0], &gfx_man[anim * 16],
    gfx_colors, color, dir
  );
  }
*/


void print_char(char c, int x, int y, uint16_t color)
{
  uint16_t* ou;
  
  if (c >= 'A' && c <= 'Z') {
    ou =  &gfx_letters[(c - 'A') * 16];
  } else if (c >= '0' && c <= '9') {
    ou = &gfx_nums[(c - '0') * 16];
  } else if (c == '(' || c == ')') {
    ou = &gfx_letters[(c - '(' + 26) * 16];
  } else if (c == ' ') {
    ou = &gfx_man[11 * 16];
  }

  myGLCD.blit_man_erase(
    x, y, x, y,
    _map.level_address(), &gfx_level[0], &gfx_blits[0], ou, gfx_colors, color, 0
  );

}

byte print_tex(char* text, byte pos = 0, uint16_t color = TFT_RED, byte row = 19, int os = -2)
{
  int x = pos * 16;
  int y = row * 16 + os;

  for (int i = 0; text[i] != '\0'; i++) {
    char c = text[i];

    // void print_tex(char c, byte pos, byte row, uint16_t color)
    print_char(c, x, y, color);

    x += 16;
  }

  return x >> 4;
}


void print_num(long long num, byte pos = 0, byte len = 5, uint16_t color = TFT_CYAN, int os = -2)
{
  int x = 16 * (pos + len - 1);
  int y = os + 320 - 16 * 1;

  while (len--) {
    myGLCD.blit_man_erase(
      x, y, x, y,
      _map.level_address(), &gfx_level[0], &gfx_blits[0], &gfx_nums[(num % 10) * 16],
      gfx_colors, color, 0
    );

    x -= 16;
    num /= 10;
  }
}


void print_value(char* text, long long num, byte pos, byte len)
{
  print_tex(text, pos);
  print_num(num, pos, len);
}


class man {

  protected:

    enum state {st_none, st_spawn, st_run, st_monkey, st_climb, st_dig, st_fall, st_dead};

    byte frame;
    byte face;
    bool has_gold;
    char spawn;
    char escape;
    bool reverse;

  public:

    pointb dest;
    pointw curr;
    pointw prev;

    man () {}

    byte static hit_man(pointw c, pointw tent)
    {      
      return
        //(tent.x <= c.x + 15 && tent.x + 15 >= c.x
         //&& tent.y <= c.y + 15 && tent.y + 15 <= c.y)

        (c.x + 15 >= tent.x && c.x <= tent.x + 15
         && c.y + 15 >= tent.y && c.y <= tent.y + 15)
        ;
    }

    bool hit_man(man* m, char ct, pointw tent, char man_n)
    {
      for (int i = 0; i < ct; i++) {
        if (i != man_n) {
          if (hit_man(m[i].curr, tent)) {
            return true;
          }

          // prevent enemies from trying to take the exact same route
          if ((dest.x != MAP_WIDTH + 1) && (dest == m[i].dest)) {
            int i1 = abs(curr.x - dest.x * 16) + abs(curr.y - dest.y * 16);
            int i2 = abs(m[i].curr.x - m[i].dest.x * 16) + abs(m[i].curr.y - m[i].dest.y * 16);

            if (i1 > i2) {
              //print_num(i1);
              //print_num(i2, 10);
              
              return true;
            }
          }
        }
      }

      return false;
    }

    void teleport(pointb pos, bool reverse, uint16_t color)
    {
      curr = prev = pointw(pos.x * 16, pos.y * 16);
      frame = 0;
      spawn = -1;
      escape = -1;
      dest.x = MAP_WIDTH + 1;
      has_gold = false;
      face = int(reverse);
      blit(1, reverse, color);
    }

    void en_die()
    {
      uint16_t x = millis();

      // spawn randomly
      do {
        x = (x + 1) % MAP_WIDTH;
      } while (_map.read_map(x * 16, 0) != 3);

      curr = prev = pointw(x * 16, 0 * 16);
      spawn = 1;
      escape = -1;

      inc_score(75);

      //print_num(curr.x);
      //print_num(curr.y, 10);

    }

    bool en_fall(byte tile, bool below = false)
    {
      return (tile >= 3 && tile <= 5) || (tile == 7 && below);
    }

    bool can_fall(byte tile, bool player, bool below = false)
    {
      return
        (
          tile <= 6 // non-solid objects
          || (tile == 7 && ((curr.y & 0x000f) || below)) // should fall through bar except when square vertically
        )
        && !(tile <= 2 && !player && !below && (curr.y & 0x000f) == 0
            );
    }

    /*
      0, // 0 brick blasted 0 (empty)
      6, // 1 brick healed 1
      7, // 2 brick healed 2
      0, // 3 blank
      0, // 4 safety ladder
      2, // 5 goal
      5, // 6 trap door
      3, // 7 bar
      1, // 8 ladder
      5, // 9 diggable brick
      4, // 10 undiggable brick
      4, // 11 unused
      4, // 12 unused
      4, // 13 unused
      4, // 14 unused
      4  // 15 unused
    */

    bool can_dig(byte next, byte below_next)
    {
      if ((next <= 4) && (below_next == 9)) {
        return true;
      } else {
        return false;
      }
    }

    bool can_climb(byte tile)
    {
      return tile == 8;
    }

    bool en_traverse(byte tile)
    {
      return ((tile >= 3 && tile <= 5) || tile == 7 || tile == 8);
    }

    bool can_traverse(byte tile, bool player)
    {
      return (tile <= 5 || tile == 7 || tile == 8) & !(tile <= 2 && !player);
    }

    inline int round_tile(int dim)
    {
      return (dim + int(8)) & int(0xfff0);
    }

    inline void round_tilex()
    {
      curr.x = round_tile(curr.x);
    }

    inline void round_tiley()
    {
      curr.y = round_tile(curr.y);
    }

    void blit_man(bool pl)
    {
      uint16_t color;

      if (pl) {
        color = TFT_WHITE;
      } else {
        color = TFT_CYAN;
      }

      blit(frame, face ^ reverse, color);
    }

    // mov.x, mov.y = 0 or +/-1
    bool walk(pointb mov, int push_dig, man* others, byte other_ct, char man_n)
    {
      /*
        0, // 0 brick blasted 0 (empty)
        6, // 1 brick healed 1
        7, // 2 brick healed 2
        0, // 3 blank
        0, // 4 safety ladder
        2, // 5 goal
        5, // 6 trap door
        3, // 7 bar
        1, // 8 ladder
        5, // 9 diggable brick
        4, // 10 undiggable brick
        4, // 11 unused
        4, // 12 unused
        4, // 13 unused
        4, // 14 unused
        4  // 15 unused
      */

      bool player = (man_n == char(-1));
      pointw delta;
      byte curr_tile = _map.read_map(curr.x + 8, curr.y + 15);
      byte below_tile = _map.read_map(curr.x + 8, curr.y + 16);
      byte nex_tile;
      byte nex_round;
      byte nex_below;
      state st;
      bool esc = false;
      bool done = false;

      reverse = false;

      //face = (int)push_dig;
      //mov.x = (int)push_dig;

      if (player) {
        done = _map.done_digging();

        if (_map.digging(curr, face)) {
          int os;

          if (face) os = 1;
          else os = -1;

          pointw dg = curr + pointw(16 * os, 0);

          for (byte i = 0; i < other_ct; i++) {
            if (man::hit_man(others[i].curr, dg)) {
              _map.cancel_dig(dg);
              done = true;
            }
          }

          mov = pointb(0, 0);
        }
      } else { // enemy


        if (spawn == -1 && curr_tile == 9) {
        } else if (escape == -1 && curr_tile <= 2 && (curr.y % 16) == 0) { // enemy is ditched
          inc_score(75);
          escape = 10 * 6;
          dest.x = MAP_WIDTH + 1;
        }
      }

      if (spawn > -1) {
        if (!hit_man(others, other_ct, curr, man_n)) {
          frame = 9 + 1 - spawn;
          face = 0;
          st = st_spawn;
          delta = pointw(0, 0);
          spawn--;
        }
      } else {
        if (curr_tile == 9) {
          st = st_dead;
          mov = pointb(0, 0);
        } else if (escape > -1) {
          // enemy escaping from ditch

          if (escape < 10 * 3) { // climb out
            if (curr.y % 16 == 0 && curr_tile > 2) {
              escape = 0;
              mov.y = 0;
              if (mov.x == 0) mov.x = 1;
              curr.x += mov.x;
              st = st_run;
            } else {
              mov.y = -1;
              st = st_climb;
            }
          } else if (escape < 10 * 4) { // wiggle
            if (escape & 1)
              mov = pointb(1, 0);
            else
              mov = pointb(-1, 0);

            st = st_fall;
          } else { // sit in the ditch for a moment
            mov = pointb(0, 0);
            st = st_none;
          }

          escape--;
        } else if (can_fall(curr_tile, player, false) && can_fall(below_tile, player, true) && !hit_man(others, other_ct, curr + pointw(0, 1), man_n)) {
          // falling
          st = st_fall;
          mov = pointb((byte)0, (byte)1);
          if (!player && has_gold && curr_tile <= 2) {
            _map.put_goal(curr + pointw(0, 0));
            has_gold = false;
          }

          if (curr.y & 0x000f == 0) {
            en_fall(75);
          }
        } else {
          // trying to go left/right
          if (mov.x < (byte)0) { // trying to go left
            nex_tile = _map.read_map(curr.x - 1, curr.y + 8);
          } else if (mov.x > (byte)0) { // trying to go right
            nex_tile = _map.read_map(curr.x + 16, curr.y + 8);
          }

          // trying to go up/down
          if (mov.y < 0) { // trying to go up
            if (!can_climb(curr_tile)) mov.y = 0;
          } else if (mov.y > 0) { // trying to go down
            if (!can_traverse(below_tile, player)) mov.y = 0;
          }

          // round y to nearest tile when leaving ladder
          if (mov.x && can_traverse(nex_tile, player) && !(curr_tile <= 2 && !player)) {
            if (nex_tile != 8) { // && dy == 0)
              round_tiley(); // curr.y = (curr.y + 8) & 0xfff0;
              mov.y = 0;
            }
          } else {
            mov.x = 0;
          }

          nex_round = _map.read_map(curr.x - 8 + ((byte)32 * (byte)face), curr.y + 8);
          nex_below = _map.read_map(curr.x - 8 + ((byte)32 * (byte)face), curr.y + 16 + 8);

          if (done) {
            frame = 0;
            st = st_run;
          } else if (push_dig && can_dig(nex_round, nex_below)) {
            st = st_dig;
          } else if (mov.y) {
            st = st_climb;
          } else if (curr_tile == 7) {
            st = st_monkey;
          } else {
            st = st_run;
          }
        }

        if (st != st_dead && !done && st != st_dig && mov == pointb(0, 0)) {
          st = st_none;
        }

        delta = pointw(4, 4);

        if (st == st_dead) {
          frame = 11;
        } else if (st == st_dig) {
          frame = 8;
          round_tilex();
          round_tiley();
          _map.dig();
        } else if (st == st_fall) {
          delta = pointw(1, 4);
          frame = 3;
          round_tilex();
        } else if (st == st_climb) {
          frame = 4;
          reverse = (curr.y / 4) & 1;
          round_tilex();
        } else if (st == st_monkey) {
          frame = 5 + (frame + 1) % 3;
          face = (mov.x > 0);

          if (frame == 5 || frame == 7)
            delta.x = 0;
          else
            delta.x = 12;
        } else if (st == st_run) { // run
          frame = 0 + (frame + 1) % 3;
          if (!done) face = (mov.x > 0);
        }
      }

      if (st != st_none) {
        pointw d = pointw((int)mov.x * delta.x, (int)mov.y * delta.y);

        if (hit_man(others, other_ct, curr + d, man_n)) {
          dest.x = MAP_WIDTH + 1;
          return true;
        } else {
          curr += d;

          // if man gets gold, change to blank
          if ((player || !has_gold) && curr_tile == (byte)5 && (curr.x & 0x000f) == 0 && (curr.y & 0x000f) == 0) {
            _map.get_goal(curr, player);
            has_gold = true;
          }

          blit_man(player);

          if (st != st_fall) frame++;
          prev = curr;
        }
      } else {
        dest.x = MAP_WIDTH + 1;
      }

      if (st == st_dead) {
        if (player) {
          return true;
        } else {
          en_die();
          return false;
        }
      } else {
        return false;
      }
    }

    void blit(int anim, bool reverse, uint16_t color)
    {

      // void blit_erase_man (int prev_x, int prev_y, int curr_x, int curr_y, uint_16 color = RGB_ORANGE, int anim = 0, int dir = 0)

      //blit_erase_man(prev.x, prev.y, curr.x, curr.y, color, anim, reverse);
      myGLCD.blit_man_erase(
        prev.x, prev.y, curr.x, curr.y,
        _map.level_address(), &gfx_level[0], &gfx_blits[0], &gfx_man[anim * 16],
        gfx_colors, color, reverse
      );
    }

    /*
      0, // 0 brick blasted 0 (empty)
      6, // 1 brick healed 1
      7, // 2 brick healed 2
      0, // 3 blank
      0, // 4 safety ladder
      2, // 5 goal
      5, // 6 trap door
      3, // 7 bar
      1, // 8 ladder
      5, // 9 diggable brick
      4, // 10 undiggable brick
      4, // 11 unused
      4, // 12 unused
      4, // 13 unused
      4, // 14 unused
      4  // 15 unused
    */

    bool en_reach(byte across, byte below)
    {
      if (across == 7 || across == 8 || below == 7) {
        return true;
      } else {
        return en_traverse(across) && !en_fall(below, true);
      }
    }

    bool reach(char& x, pointb pl, pointb en, char out)
    {
      byte obj = _map.read_map(x * 16, en.y * 16);
      byte below = _map.read_map(x * 16, (en.y + 1) * 16);

      if (pl.y < en.y) { // player is above, so find a way to climb up and get him!
        if (!en_reach(obj, below)) { // !en_traverse(obj) || en_fall(below, true)) { //
          x = out; // can't go up without falling or being blocked first
        } else if (can_climb(obj)) {
          return true; // can go up
        }
      } else if (pl.y > en.y) { // player is below or across, so find a way to drop down and get him!
        if (!en_traverse(obj)) {
          x = out; // can't reach anything
        } else if (en_traverse(below)) {
          return true; // found a way to drop down
        }
      } else { // same level, just go over if enemy can reach
        if (x == pl.x) {
          return true; // found a way
        } else if (!en_reach(obj, below)) {
          x = out; // can't reach anything
        }
      }

      return false; // keep looking along line
    }

    bool enemy_think(man* others, pointw player, byte other_ct, char man_n)
    {
      pointb mov;
      char x, y, best;
      pointb pl = pointb((player.x + 8) >> 4, (player.y + 8) >> 4);
      pointb en = pointb((curr.x + 8) >> 4, (curr.y + 8) >> 4);

      // scan left
      for (best = en.x; !reach(best, pl, en, -1) && best >= 0; best--);

      // scan right
      for (x = en.x; !reach(x, pl, en, MAP_WIDTH) && x < MAP_WIDTH; x++);

      if (
        (best < 0)
        || (
          x < MAP_WIDTH
          && abs(x - pl.x) + (x - en.x) < abs(best - pl.x) + (en.x - best)
        )
      ) {
        best = x;
      }

      mov = pointb(0, 0);

      if (best >= 0 && best < MAP_WIDTH) { // best is valid
        if (best == en.x) { // enemy caught up with best
          mov.y = sign(pl.y, en.y);
        } else {
          mov.x = sign(best, en.x);
        }
      }

      dest = en + mov;
      walk(mov, 0, others, other_ct, man_n);

      bool ret = hit_man(player, curr);

      return ret;
    }

} _enemy[MAX_ENEMY], _player;


void init_sd()
{
  pinMode(10, OUTPUT);

  if (!SD.begin(10)) {
    // deal with failure
    //Serial.println("Can't initialize SD");
    exit(1);
  }
}


bool init_level(word level, bool disp = true)
{
  _map.safety_done = false;

  _dig_log.init();

  pointb player_pos, enemy_pos[MAX_ENEMY];
  if (_map.load_level(level, player_pos, enemy_pos)) {
    _player.teleport(player_pos, true, TFT_WHITE);

    for (int i = 0; i < _map.enemy_count(); i++) {
      _enemy[i].teleport(enemy_pos[i], false, TFT_CYAN);
    }

    print_tex("SCORE", 0);
    print_num(_score, 5, 7);

    print_tex("MEN", 14);
    print_num(_men, 17, 3);

    print_tex("LEVEL", 22);
    print_num(level, 27, 3);

    return true;
  } else {
    if (disp) print_tex("CANT LOAD ", 12);
    return false;
  }
}


void init_game()
{
  //pinMode(A5, OUTPUT);
  
  //pinMode(0, OUTPUT);
  //pinMode(1, OUTPUT);

  myGLCD.InitLCD(); // 3% of flash
  init_sd();
  //_sound.init();
  //_sound.start_tune();
  //_sound.play_note(5, 3);

  show_title();

  delay(500);

  _score = 0;
  _men = 5;
  _level = 1;
}


void change_map(byte x, byte y, byte tile)
{
  bool drawn = false;

  int px = x * 16;
  int py = y * 16;

  //_map.write_map(x, y, tile, false);

  if (_player.prev.x == px && _player.prev.y == py) {
    //_player.blit_man(true);
    // void draw_tile(int x, int y, char tile = -1, int from = 0, byte h = 15)
    //_map.draw_tile(px, _player.prev.y + 16, -1, 0,
    drawn = true;
  } else {
    for (byte i = 0; i < _map.enemy_count(); i++) {
      if (_enemy[i].prev.x == px && _enemy[i].prev.y == py) {
        //_enemy[i].blit_man(false);
        // void draw_tile(int x, int y, char tile = -1, int from = 0, byte h = 15)
        //_map.draw_tile(px, _enemy[i].prev.y + 16, -1, 0, py - _enemy[i].prev.y);
        drawn = true;
        break;
      }
    }
  }


  if (!drawn) {
    _map.write_map(x, y, tile, true);
  }

  // bool static hit_man(pointw c, pointw tent)
  if (man::hit_man(point<int>(px, py), _player.curr)) {
    _player.blit_man(true);
  }

  for (byte i = 0; i < _map.enemy_count(); i++) {
    if (man::hit_man(point<int>(px, py), _enemy[i].curr)) {
      _enemy[i].blit_man(false);
    }
  }
}


void update_digs()
{
  pointb dl;
  byte st;

  if (_dig_log.check_states(dl, st)) {
    if (st == 3) {
      _map.write_map(dl.x, dl.y, 9);
      //change_map(dl.x, dl.y, 9);
    } else {
      //_map.write_map(dl.x, dl.y, st);
      change_map(dl.x, dl.y, st);
    }
  }
}


bool user_input(byte* x, byte* y, byte& dig)
{
  int a = analogRead(A5);
  dig = 0;
  pointb input = pointb(0, 0);

  //if (a < 84) {
  //  dig = -1;
  //} else 
  
  if (a < 255) { // dig
    dig = 1;
  } else if (a < 417) { // move up
    input = pointb(0, -1);
  } else if (a < 578) { // move down
    input = pointb(0, 1);
  } else if (a < 755) { // move left
    input = pointb(-1, 0);
  } else if (a < 936) { // move right
    input = pointb(1, 0);
  } else {              // no direction buttions pushed
  }
  
  *x = input.x;
  *y = input.y;

  //print_num(dig, 0, 4);

  return dig || input.x || input.y;
}


bool wait_player(int dur = 250)
{
  bool ret;
  byte dig;
  byte x, y;
  long long t = millis();

  //return true;
  
  do {
    ret = user_input(&x, &y, dig);
  } while (!ret && int(millis() - t) < dur);

  return ret;
}


void player_die()
{
  _men--;
  print_num(_men, 17, 3);
  delay(500);
}


void inc_score(int inc)
{
  _score += inc;
  print_num(_score, 7);
}


void show_title()
{
  pointb pos;
  File title;
  bool mute = false;
  int os = 0;

  title = SD.open("/lr/title.txt");

  pos.x = 0;
  pos.y = 0;

  for (int i = 0; i < 30 * 20; i++) {
    //_map.add_tile(pos, 3);
  }

  pos.x = 0;
  pos.y = 0;

  while (title.available()) {
    byte b = title.read();

    if (b == 10) {
      mute = false;
    } else if (!mute) {
      if (b == '#') {
        // diggable brick
        _map.add_tile(pos, 9, true);
      } else if (b == '.') {
        _map.add_tile(pos, 3, true);
      } else if (b == '!') {
        mute = true;
      } else if (b > ' ') {
        if (pos.x == 0) {
          os += 2;
        }

        _map.add_tile(pos, 3, true);
        print_char(b, int(pos.x - 1) * 16, int(pos.y * 16) + (pos.y - 13) * 0, TFT_CYAN);
      }
    }
  }

  title.close();

  delay(5 * 1000);
}


void setup()
{
  bool playing = true;

  init_game();

  while (_men > 0) {
    if (!init_level(0, false)) {
      init_level(_level);
    }

    //while (wait_player(0));

    // blink player and wait for user to press any button
    for (;;) {
      _player.blit(1, true, TFT_WHITE);
      if (wait_player(250)) break;

      _map.draw_tile(_player.curr.x, _player.curr.y);
      if (wait_player(250)) break;
    }

    bool turn = false;
    playing = true;

    long long frame_time;

    //pinMode(0, INPUT);
    //pinMode(1, INPUT);

    while (playing) {
      frame_time = millis();

      pointb input;
      byte dig;

      user_input(&input.x, &input.y, dig);
      
      if (_player.walk(input, dig, _enemy, _map.enemy_count(), -1)) {
        player_die();
        playing = false;
      } else {
        update_digs();

        if (_map.level_clear()) {
          _map.safety_ladder();
          if (_player.curr.y <= 0) {
            _level++;
            _men++;
            inc_score(1500);
            playing = false;
          }
        }

        if (playing) {
          pointw pl = _player.curr;

          for (byte i = 0; i < _map.enemy_count(); i++) {
            if (bool(i & 1) == turn) {
              pointw en = _enemy[i].curr;
              
              if (_enemy[i].enemy_think(_enemy, _player.curr, _map.enemy_count(), i)) {
                player_die();
                playing = false;
              }
            }
          }

          turn = !turn;

          //digitalWrite(0, (_player.curr.x & 15) >= 8);

          //user_input(&input.x, &input.y, dig);

          //pinMode(A5, INPUT);
          while (millis() - frame_time < 50) {
          }

        }
      }
    }
  }

  for (byte i = 12; i < 12 + 9; i++) {
    _map.write_map(i, 9, 3);
  }

  print_tex("GAME OVER", 12, TFT_CYAN, 9);
}


void loop()
{
}

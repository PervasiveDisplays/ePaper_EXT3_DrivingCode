/* 
 * This example code is for sub 5 inch with iTC driver
 * including 1.54", 2.13", 2.66", 2.71", 2.87", 3.70", 4.20" and 4.37"
 * of Aurora or Spectra EPD made by Pervasive Displays Inc (PDi) for EXT3 board 
 * which is verified by Arduino M0 Pro and TI LaunchPad EK-TM4C123GXL, MSP-EXP430F5529LP.
 * And it should be able to be compiled on Arduino/Energia IDE supported boards.
 * You will need a level shifter 5V --> 3V if using Arduino Due or Arduino Uno to work with EXT3 board.
 * The driving code below will update black/white colors image, delay 3 seconds, then black/white/red colors image.
 * If you are using EPD with Aurora imaging film, you could comment out the 2nd image data.
 * For more information about PDi EPD products and EXT3 board, please visit 
 * https://www.pervasivedisplays.com/products/ and https://www.pervasivedisplays.com/product/epd-extension-kit-gen-3-ext3/
 */

#include <SPI.h>

#define EPD_154 0 //1.54 inch PDi EPD (iTC)
#define EPD_213 1 //2.13 inch PDi EPD (iTC)
#define EPD_266 2 //2.66 inch PDi EPD (iTC)
#define EPD_271 3 //2.71 inch PDi EPD (iTC)
#define EPD_287 4 //2.87 inch PDi EPD (iTC)
#define EPD_370 5 //3.70 inch PDi EPD (iTC)
#define EPD_420 6 //4.20 inch PDi EPD (iTC)
#define EPD_437 7 //4.37 inch PDi EPD (iTC)

long image_data_size[] = { 2888, 2756, 5624, 5808, 4736, 12480, 15000, 10560 };  //followed by the index above

// Please define which size of PDi's EPD you will work with from one of the definitions above
#define PDI_EPD_Size EPD_154

#if (PDI_EPD_Size >= 5)   //3.70", 4.20", 4.37"
    //0x00, soft-reset, temperature, active temp., PSR0, PSR1
    uint8_t register_data[] = { 0x00, 0x0e, 0x19, 0x02, 0x0f, 0x89 };
#else   //other small sizes
    uint8_t register_data[] = { 0x00, 0x0e, 0x19, 0x02, 0xcf, 0x8d };
#endif

#if (PDI_EPD_Size==0)   //1.54"
    #include "image_data\1.54\image_154_152x152_BW.c"
    #include "image_data\1.54\image_154_152x152_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_154_152x152_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_154_152x152_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_154_152x152_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_154_152x152_BWR_redBuffer	  
#elif(PDI_EPD_Size==1)   //2.13"
    #include "image_data\2.13\image_213_212x104_BW.c"
    #include "image_data\2.13\image_213_212x104_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_213_212x104_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_213_212x104_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_213_212x104_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_213_212x104_BWR_redBuffer  
#elif(PDI_EPD_Size==2)   //2.66"
    #include "image_data\2.66\image_266_296x152_BW.c"
    #include "image_data\2.66\image_266_296x152_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_266_296x152_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_266_296x152_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_266_296x152_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_266_296x152_BWR_redBuffer 
#elif(PDI_EPD_Size==3)   //2.71"
    #include "image_data\2.71\image_271_264x176_BW.c"
    #include "image_data\2.71\image_271_264x176_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_271_264x176_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_271_264x176_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_271_264x176_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_271_264x176_BWR_redBuffer
#elif(PDI_EPD_Size==4)   //2.87"
    #include "image_data\2.87\image_287_296x128_BW.c"
    #include "image_data\2.87\image_287_296x128_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_287_296x128_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_287_296x128_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_287_296x128_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_287_296x128_BWR_redBuffer
#elif(PDI_EPD_Size==5)   //3.70"
    #include "image_data\3.70\image_370_416x240_BW.c"
    #include "image_data\3.70\image_370_416x240_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_370_416x240_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_370_416x240_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_370_416x240_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_370_416x240_BWR_redBuffer
#elif(PDI_EPD_Size==6)   //4.2"
    #include "image_data\4.20\image_420_400x300_BW.c"	//BW image error 
    #include "image_data\4.20\image_420_400x300_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_420_400x300_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_420_400x300_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_420_400x300_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_420_400x300_BWR_redBuffer    
#elif(PDI_EPD_Size==7)   //4.37"
    #include "image_data\4.37\image_437_480x176_BW.c" 
    #include "image_data\4.37\image_437_480x176_BWR.c"
    #define BW_monoBuffer        (uint8_t *) & image_437_480x176_BW_mono
    #define BW_0x00Buffer        (uint8_t *) & image_437_480x176_BW_0x00
    #define BWR_blackBuffer      (uint8_t *) & image_437_480x176_BWR_blackBuffer
    #define BWR_redBuffer        (uint8_t *) & image_437_480x176_BWR_redBuffer
#endif

#if defined(ENERGIA)	// Valid pins for LaunchPad on Energia
#define SCL_PIN 7     // EXT3 board J4 pin 2 SCK
#define BUSY_PIN 11   // EXT3 board J4 pin 3 BUSY
#define DC_PIN 12     // EXT3 board J4 pin 4 D/C
#define RESET_PIN 13  // EXT3 board J4 pin 5 RST (RESET)
#define SDA_PIN 15    // EXT3 board J4 pin 7 MOSI
#define CS_PIN 19     // EXT3 board J4 pin 9 ECSM (EPD CS Master)
//EXT3 board J4 pin 1 connects 3V3
//EXT3 board J4 pin 10 connects GND

// SPI protocl setup
void sendIndexData( uint8_t index, const uint8_t *data, uint32_t len ) {
	SPI.begin (); 
	SPI.setDataMode(SPI_MODE3);
	SPI.setClockDivider(SPI_CLOCK_DIV32);
	SPI.setBitOrder(MSBFIRST);
	digitalWrite( DC_PIN, LOW );      //DC Low
	digitalWrite( CS_PIN, LOW );      //CS Low
	delayMicroseconds(500);
	SPI.transfer( index );
	delayMicroseconds(500);
	digitalWrite( CS_PIN, HIGH );     //CS High
	digitalWrite( DC_PIN, HIGH );     //DC High
	digitalWrite( CS_PIN, LOW );      //CS Low
	delayMicroseconds(500);
	for ( int i = 0; i < len; i++ ) SPI.transfer( data[ i ] );
	delayMicroseconds(500);
	digitalWrite( CS_PIN, HIGH );     //CS High
}

#else	// Valid pins for Arduino board, like M0 Pro
#define SCL_PIN 13      // EXT3 board J4 pin 2 SCK
#define BUSY_PIN 8      // EXT3 board J4 pin 3 BUSY
#define DC_PIN 10       // EXT3 board J4 pin 4 D/C
#define RESET_PIN 9     // EXT3 board J4 pin 5 RST (RESET)
#define SDA_PIN 12      // EXT3 board J4 pin 7 MOSI
#define CS_PIN 11       // EXT3 board J4 pin 9 ECSM (EPD CS Master)
//EXT3 board J4 pin 1 connects 3V3
//EXT3 board J4 pin 10 connects GND

// Software SPI setup
void softwareSpi( uint8_t data ) {
	for ( int i = 0; i < 8; i++ ) {
		if ((( data >> (7 - i) ) & 0x01 ) == 1 ) digitalWrite( SDA_PIN, HIGH );
		else digitalWrite( SDA_PIN, LOW );
		digitalWrite( SCL_PIN, HIGH );
		digitalWrite( SCL_PIN, LOW );
	}
}

// Software SPI protocl setup
void sendIndexData( uint8_t index, const uint8_t *data, uint32_t len ) {
	digitalWrite( DC_PIN, LOW );      //DC Low
	digitalWrite( CS_PIN, LOW );      //CS Low
	softwareSpi( index );
	digitalWrite( CS_PIN, HIGH );     //CS High
	digitalWrite( DC_PIN, HIGH );     //DC High
	digitalWrite( CS_PIN, LOW );      //CS High
	for ( int i = 0; i < len; i++ ) softwareSpi( data[ i ] );
	digitalWrite( CS_PIN, HIGH );     //CS High
}
#endif

void setup() {            
	Serial.begin( 115200 );	//begin a Serial link
	pinMode( SCL_PIN, OUTPUT );
	pinMode( SDA_PIN, OUTPUT );
	pinMode( CS_PIN, OUTPUT );
	pinMode( DC_PIN, OUTPUT );
	pinMode( RESET_PIN, OUTPUT );
	pinMode( BUSY_PIN, INPUT );			//All Pins 0
	delay( 5 );
	digitalWrite( RESET_PIN, HIGH );	//RES# = 1
	delay( 5 ); 
	digitalWrite( RESET_PIN, LOW );
	delay( 10 );
	digitalWrite( RESET_PIN, HIGH );
	delay( 5 );
	digitalWrite( CS_PIN, HIGH );		//CS# = 1

	sendIndexData( 0x00, &register_data[1], 1 );	//Soft-reset
	while( digitalRead( BUSY_PIN ) != HIGH );

	sendIndexData( 0xe5, &register_data[2], 1 );	//Input Temperature: 25C
	sendIndexData( 0xe0, &register_data[3], 1 );	//Active Temperature
	sendIndexData( 0x00, &register_data[4], 2 );	//PSR

	// Send 1st image data for black and white colors
	sendIndexData( 0x10, BW_monoBuffer, image_data_size[PDI_EPD_Size] );	//First frame: black frame where 1=black and 0=white pixel
	sendIndexData( 0x13, BW_0x00Buffer, image_data_size[PDI_EPD_Size] );	//Second frame: all 0x00

	sendIndexData( 0x04, &register_data[0], 1 );	//Power on
	while( digitalRead( BUSY_PIN ) != HIGH );
	sendIndexData( 0x12, &register_data[0], 1 );	//Display Refresh
	while( digitalRead( BUSY_PIN ) != HIGH );

	delay( 3000 );

	// Send 2nd image data for black, white and red colors
	sendIndexData( 0x10, BWR_blackBuffer, image_data_size[PDI_EPD_Size] );	//First frame: black frame where 1=black and 0=white or color pixel
	sendIndexData( 0x13, BWR_redBuffer, image_data_size[PDI_EPD_Size] );	//Second frame: color frame where 1=color and 0=white or black pixel

	sendIndexData( 0x12, &register_data[0], 1 );	//Display Refresh
	while( digitalRead( BUSY_PIN ) != HIGH );

	//Turn-off DC/DC
	sendIndexData( 0x02, &register_data[0], 1 );	//Turn off DC/DC
	while( digitalRead( BUSY_PIN ) != HIGH );
	digitalWrite( DC_PIN, LOW );
	digitalWrite( CS_PIN, LOW );
	digitalWrite( SDA_PIN, LOW );
	digitalWrite( SCL_PIN, LOW );
	digitalWrite( BUSY_PIN, LOW );
	delay( 150 );
	digitalWrite( RESET_PIN, LOW );
}

void loop() {
}

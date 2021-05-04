//
// This example code is for 5.81 to 7.4 inch with iTC driver
// of Aurora or Spectra EPD made by Pervasive Displays Inc (PDi) for EXT3 board
// which is verified by TI LaunchPad EK-TM4C123GXL.
// And it should be able to be compiled on Energia IDE supported boards.
// You will need a level shifter 5V --> 3V if using Arduino Due or Arduino Uno to work with EXT3 board.
// The driving code below will update black/white colours image, delay 3 seconds, then black/white/red colours image.
// If you are using EPD with Aurora imaging film, you could comment out the 2nd image data.
// For more information about PDi EPD products and EXT3 board, please visit
// https://www.pervasivedisplays.com/products/ and https://www.pervasivedisplays.com/product/epd-extension-kit-gen-3-ext3/
//

#include <SPI.h>

#define EPD_581 0 // 5.81 inch PDi EPD (iTC)
#define EPD_740 1 // 7.40 inch PDi EPD (iTC)

long image_data_size[] = {23040, 48000}; //followed by the index above

// Please define which size of PDi's EPD you will work with from one of the definitions above
#define PDI_EPD_Size EPD_581

#if (PDI_EPD_Size == 0) // 5.81"

#include "image_data/5.81/image_581_720x256_BW.c"
#include "image_data/5.81/image_581_720x256_BWR.c"
#define BW_monoBuffer (uint8_t *)&image_581_720x256_BW_mono
#define BW_0x00Buffer (uint8_t *)&image_581_720x256_BW_0x00
#define BWR_blackBuffer (uint8_t *)&image_581_720x256_BWR_blackBuffer
#define BWR_redBuffer (uint8_t *)&image_581_720x256_BWR_redBuffer

#elif (PDI_EPD_Size == 1) // 7.4"

#include "image_data\7.40\image_740_800x480_BW.c"
#include "image_data\7.40\image_740_800x480_BWR.c"
#define BW_monoBuffer (uint8_t *)&image_740_800x480_BW_mono
#define BW_0x00Buffer (uint8_t *)&image_740_800x480_BW_0x00
#define BWR_blackBuffer (uint8_t *)&image_740_800x480_BWR_blackBuffer
#define BWR_redBuffer (uint8_t *)&image_740_800x480_BWR_redBuffer
#endif

/// Pins definition
#if defined(ENERGIA) // Valid pins for LaunchPad on Energia

#define PANEL_SCL_PIN 7       // EXT3 board J4 pin 2 SCK  Brown
#define PANEL_BUSY_PIN 11     // EXT3 board J4 pin 3 BUSY Red
#define PANEL_DC_PIN 12       // EXT3 board J4 pin 4 D/C  Orange
#define PANEL_RESET_PIN 13    // EXT3 board J4 pin 5 RST(RESET) Yellow
#define PANEL_SDA_PIN 15      // EXT3 board J4 pin 7 MOSI Blue
#define FLASH_CS_PIN 18       // EXT3 board J4 pin 8 FCSW Violet
#define PANEL_CS_PIN 19       // EXT3 board J4 pin 9 ECSW Grey
// If not used, comment out
//#define PANEL_CSS_PIN 39      // EXT3 board J4 pin 12 ECSS Violet2
//#define FLASH_CSS_PIN 38      // EXT3 board J4 pin 20 FCSS Black2

//EXT3 board J4 pin 1 connects 3V3
//EXT3 board J4 pin 10 connects GND

// SPI protocl setup
void _sendIndexData(uint8_t index, const uint8_t * data, uint32_t len)
{
    digitalWrite(PANEL_DC_PIN, LOW); // DC Low= Command
    digitalWrite(PANEL_CS_PIN, LOW); // CS Low= Select

    delayMicroseconds(50);
    SPI.transfer(index);
    delayMicroseconds(50);

    digitalWrite(PANEL_CS_PIN, HIGH); // CS High= Unselect
    digitalWrite(PANEL_DC_PIN, HIGH); // DC High= Data
    digitalWrite(PANEL_CS_PIN, LOW); // CS Low= Select

    delayMicroseconds(50);
    for (uint32_t i = 0; i < len; i++)
    {
        SPI.transfer(data[i]);
    }
    delayMicroseconds(50);

    digitalWrite(PANEL_CS_PIN, HIGH); // CS High
}

#else // Valid pins for Arduino board, like M0 Pro

#define PANEL_SCL_PIN 13  // EXT3 board J4 pin 2 SCK
#define PANEL_BUSY_PIN 8  // EXT3 board J4 pin 3 BUSY
#define PANEL_DC_PIN 10   // EXT3 board J4 pin 4 D/C
#define PANEL_RESET_PIN 9 // EXT3 board J4 pin 5 RST (RESET)
#define PANEL_SDA_PIN 12  // EXT3 board J4 pin 7 MOSI
#define PANEL_CS_PIN 11   // EXT3 board J4 pin 9 ECSM (EPD CS Master)
//#define FLASH_CS_PIN 18   // EXT3 board J4 pin 8 FCSM
//#define PANEL_CSS_PIN 5   // EXT3 board J4 pin 12 ECSS (EPD CS Slave)
//EXT3 board J4 pin 1 connects 3V3
//EXT3 board J4 pin 10 connects GND

// Software SPI setup
void softwareSpi(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        if (((data >> (7 - i)) & 0x01) == 1)
        {
            digitalWrite(PANEL_SDA_PIN, HIGH);
        }
        else
        {
            digitalWrite(PANEL_SDA_PIN, LOW);
        }
        digitalWrite(PANEL_SCL_PIN, HIGH);
        digitalWrite(PANEL_SCL_PIN, LOW);
    }
}

// Software SPI protocl setup
void _sendIndexData(uint8_t index, const uint8_t * data, uint32_t len)
{
    digitalWrite(PANEL_DC_PIN, LOW); //DC Low
    digitalWrite(PANEL_CS_PIN, LOW); //CS Low
    softwareSpi(index);
    digitalWrite(PANEL_CS_PIN, HIGH); //CS High
    digitalWrite(PANEL_DC_PIN, HIGH); //DC High
    digitalWrite(PANEL_CS_PIN, LOW);  //CS High
    for (int i = 0; i < len; i++)
    {
        softwareSpi(data[i]);
    }
    digitalWrite(PANEL_CS_PIN, HIGH); //CS High
}

#endif

void wait(uint8_t second)
{
    for (uint8_t i = second; i > 0; i--)
    {
        Serial.print(" > ");
        Serial.print(i);
        Serial.print("  \r");
        delay(1000);
    }
    Serial.print("         \r\n");
}

void _reset(uint32_t ms1, uint32_t ms2, uint32_t ms3, uint32_t ms4, uint32_t ms5)
{
    delay(ms1);
    digitalWrite(PANEL_RESET_PIN, HIGH); // RES# = 1
    delay(ms2);
    digitalWrite(PANEL_RESET_PIN, LOW);
    delay(ms3);
    digitalWrite(PANEL_RESET_PIN, HIGH);
    delay(ms4);
    digitalWrite(PANEL_CS_PIN, HIGH); // CS# = 1
    delay(ms5);
}

void sendImageData(uint8_t * blackBuffer, uint8_t * redBuffer)
{
    Serial.print("reset... ");
    _reset(200, 20, 200, 50, 5);

    // Send image data
#if (PDI_EPD_Size == 0) // 5.81"

    uint8_t data1[] = {0x00, 0x1f, 0x50, 0x00, 0x1f, 0x03}; // DUW
    _sendIndexData(0x13, data1, 6); // DUW
    uint8_t data2[] = {0x00, 0x1f, 0x00, 0xc9}; // DRFW
    _sendIndexData(0x90, data2, 4); // DRFW
    uint8_t data3[] = {0x1f, 0x50, 0x14}; // RAM_RW
    _sendIndexData(0x12, data3, 3); // RAM_RW

#elif (PDI_EPD_Size == 1) // 7.40"

    uint8_t data1[] = {0x00, 0x3b, 0x00, 0x00, 0x1f, 0x03}; // DUW
    _sendIndexData(0x13, data1, 6); // DUW
    uint8_t data2[] = {0x00, 0x3b, 0x00, 0xc9}; // DRFW
    _sendIndexData(0x90, data2, 4); // DRFW
    uint8_t data3[] = {0x3b, 0x00, 0x14}; // RAM_RW
    _sendIndexData(0x12, data3, 3); // RAM_RW
#endif

    /// @todo Check and adapt this parameter
    uint8_t dtcl = 0x08; // 0=IST7232, 8=IST7236
    _sendIndexData(0x01, &dtcl, 1); //DCTL 0x10 of MTP

    Serial.print("1st frame... ");
    _sendIndexData(0x10, blackBuffer, image_data_size[PDI_EPD_Size]); // First frame

#if (PDI_EPD_Size == 0) // 5.81"

    // uint8_t data3[] = {0x1f, 0x50, 0x14}; // RAM_RW
    _sendIndexData(0x12, data3, 3); // RAM_RW

#elif (PDI_EPD_Size == 1) // 7.40"

    // uint8_t data3[] = {0x3b, 0x00, 0x14}; // RAM_RW
    _sendIndexData(0x12, data3, 3); // RAM_RW

#endif

    Serial.print("2nd frame... ");
    _sendIndexData(0x11, redBuffer, image_data_size[PDI_EPD_Size]); // Second frame

    // Initial COG
    Serial.print("init... ");

    uint8_t data4[] = {0x7d};
    _sendIndexData(0x05, data4, 1);
    delay(200);
    uint8_t data5[] = {0x00};
    _sendIndexData(0x05, data5, 1);
    delay(10);
    uint8_t data6[] = {0x3f};
    _sendIndexData(0xc2, data6, 1);
    delay(1);
    uint8_t data7[] = {0x00};
    _sendIndexData(0xd8, data7, 1); // MS_SYNC mtp_0x1d
    uint8_t data8[] = {0x00};
    _sendIndexData(0xd6, data8, 1); // BVSS mtp_0x1e
    uint8_t data9[] = {0x10};
    _sendIndexData(0xa7, data9, 1);
    delay(100);
    _sendIndexData(0xa7, data5, 1);
    delay(100);
    // uint8_t data10[] = {0x00, 0x02 };

#if (PDI_EPD_Size == 0) // 5.81

    uint8_t data10[] = {0x00, 0x01}; // OSC
    _sendIndexData(0x03, data10, 2); // OSC mtp_0x12

#elif (PDI_EPD_Size == 1) // 7.40
    uint8_t data10[] = {0x00, 0x01}; // OSC
    _sendIndexData(0x03, data10, 2); // OSC mtp_0x12

#endif

    _sendIndexData(0x44, data5, 1);
    uint8_t data11[] = {0x80};
    _sendIndexData(0x45, data11, 1);
    _sendIndexData(0xa7, data9, 1);
    delay(100);
    _sendIndexData(0xa7, data7, 1);
    delay(100);
    uint8_t data12[] = {0x06};
    _sendIndexData(0x44, data12, 1);
    uint8_t data13[] = {0x82};
    _sendIndexData(0x45, data13, 1); // Temperature 0x82@25C
    _sendIndexData(0xa7, data9, 1);
    delay(100);
    _sendIndexData(0xa7, data7, 1);
    delay(100);
    uint8_t data14[] = {0x25};
    _sendIndexData(0x60, data14, 1); // TCON mtp_0x0b
    // uint8_t data15[] = {0x01 };

#if (PDI_EPD_Size == 0) // 5.81

    uint8_t data15[] = {0x00}; // STV_DIR
    _sendIndexData(0x61, data15, 1); // STV_DIR mtp_0x1c

#elif (PDI_EPD_Size == 1) // 7.40

    uint8_t data15[] = {0x00}; // STV_DIR
    _sendIndexData(0x61, data15, 1); // STV_DIR mtp_0x1c

#endif

    uint8_t data16[] = {0x00};
    _sendIndexData(0x01, data16, 1); // DCTL mtp_0x10
    uint8_t data17[] = {0x00};
    _sendIndexData(0x02, data17, 1); // VCOM mtp_0x11

    // DC-DC soft-start
    Serial.print("on... ");

    uint8_t index51[] = {0x50, 0x01, 0x0a, 0x01};
    _sendIndexData(0x51, &index51[0], 2);
    uint8_t index09[] = {0x1f, 0x9f, 0x7f, 0xff};

    for (int value = 1; value <= 4; value++)
    {
        _sendIndexData(0x09, &index09[0], 1);
        index51[1] = value;
        _sendIndexData(0x51, &index51[0], 2);
        _sendIndexData(0x09, &index09[1], 1);
        delay(2);
    }
    for (int value = 1; value <= 10; value++)
    {
        _sendIndexData(0x09, &index09[0], 1);
        index51[3] = value;
        _sendIndexData(0x51, &index51[2], 2);
        _sendIndexData(0x09, &index09[1], 1);
        delay(2);
    }
    for (int value = 3; value <= 10; value++)
    {
        _sendIndexData(0x09, &index09[2], 1);
        index51[3] = value;
        _sendIndexData(0x51, &index51[2], 2);
        _sendIndexData(0x09, &index09[3], 1);
        delay(2);
    }
    for (int value = 9; value >= 2; value--)
    {
        _sendIndexData(0x09, &index09[2], 1);
        index51[2] = value;
        _sendIndexData(0x51, &index51[2], 2);
        _sendIndexData(0x09, &index09[3], 1);
        delay(2);
    }
    _sendIndexData(0x09, &index09[3], 1);
    delay(10);

    // Display Refresh Start
    Serial.print("start... ");

    while (digitalRead(PANEL_BUSY_PIN) != HIGH)
    {
        delay(100);
    }
    uint8_t data18[] = {0x3c};
    _sendIndexData(0x15, data18, 1); //Display Refresh
    delay(5);

    // DC-DC off
    Serial.print("off... ");

    while (digitalRead(PANEL_BUSY_PIN) != HIGH)
    {
        delay(100);
    }
    uint8_t data19[] = {0x7f};
    _sendIndexData(0x09, data19, 1);
    uint8_t data20[] = {0x7d};
    _sendIndexData(0x05, data20, 1);
    _sendIndexData(0x09, data5, 1);
    delay(200);

    while (digitalRead(PANEL_BUSY_PIN) != HIGH)
    {
        delay(100);
    }
    digitalWrite(PANEL_DC_PIN, LOW);
    digitalWrite(PANEL_CS_PIN, LOW);
    digitalWrite(PANEL_RESET_PIN, LOW);
    // digitalWrite(PNLON_PIN, LOW); // PANEL_OFF# = 0

    digitalWrite(PANEL_CS_PIN, HIGH); // CS# = 1
}

void setup()
{
    Serial.begin(115200); //begin a Serial link
    delay(500);
    Serial.println();
    Serial.println("=== " __FILE__);
    Serial.println("=== " __DATE__ " " __TIME__);
    Serial.println();

    pinMode(PANEL_SCL_PIN, OUTPUT);
    pinMode(PANEL_SDA_PIN, OUTPUT);
    pinMode(PANEL_CS_PIN, OUTPUT);
    pinMode(PANEL_DC_PIN, OUTPUT);
    pinMode(PANEL_RESET_PIN, OUTPUT);
    pinMode(PANEL_BUSY_PIN, INPUT); //All Pins 0
    delay(5);

    // Initialise Flash /CS as HIGH
#if defined(FLASH_CS_PIN)
#if (FLASH_CS_PIN > 0)
    pinMode(FLASH_CS_PIN, OUTPUT);
    digitalWrite(FLASH_CS_PIN, HIGH);
#endif // FLASH_CS_PIN
#endif // FLASH_CS_PIN

#ifndef SPI_CLOCK_MAX
#define SPI_CLOCK_MAX 16000000
#endif

    // Initialise SPI
    Serial.print("SPI.begin... ");
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    // SPI.setClockDivider(SPI_CLOCK_DIV32); // 16 MHz / 32
    SPI.setClockDivider(SPI_CLOCK_MAX / min(SPI_CLOCK_MAX, 4000000));
    SPI.setBitOrder(MSBFIRST);
    Serial.println("done");

    Serial.println("--- Image_Mono");
    sendImageData((uint8_t *)BW_monoBuffer, (uint8_t *)BW_0x00Buffer);
    Serial.println("done");

    wait(3);

    Serial.println("--- Image_BWR");
    sendImageData((uint8_t *)BWR_blackBuffer, (uint8_t *)BWR_redBuffer);
    Serial.println("done");

    wait(3);

    Serial.println("--- Power-off");
    // Turn-off DC/DC
    uint8_t register_turnOff[] = {0x7f, 0x7d, 0x00};
    _sendIndexData(0x09, register_turnOff, 3);
    delay(200);

    digitalWrite(PANEL_DC_PIN, LOW);
    digitalWrite(PANEL_CS_PIN, LOW);
    digitalWrite(PANEL_SDA_PIN, LOW);
    digitalWrite(PANEL_SCL_PIN, LOW);
    digitalWrite(PANEL_BUSY_PIN, LOW);
    delay(150);
    digitalWrite(PANEL_RESET_PIN, LOW);

    Serial.println("===");
}

void loop()
{
}

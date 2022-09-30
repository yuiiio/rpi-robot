/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
//#define ANALOGIN_MEDIAN_FILTER      0

#include "mbed.h"
#include "stdio.h"

class TextLCD : public Stream {
public:

    /** LCD panel format */
    enum LCDType {
        LCD16x2     /**< 16x2 LCD panel (default) */
        , LCD16x2B  /**< 16x2 LCD panel alternate addressing */
        , LCD20x2   /**< 20x2 LCD panel */
        , LCD20x4   /**< 20x4 LCD panel */
    };

    /** Create a TextLCD interface
     *
     * @param rs    Instruction/data control line
     * @param e     Enable line (clock)
     * @param d4-d7 Data lines for using as a 4-bit interface
     * @param type  Sets the panel size/addressing mode (default = LCD16x2)
     */
    TextLCD(PinName rs, PinName e, PinName d4, PinName d5, PinName d6, PinName d7, LCDType type = LCD16x2B);

#if DOXYGEN_ONLY
    /** Write a character to the LCD
     *
     * @param c The character to write to the display
     */
    int putc(int c);

    /** Write a formated string to the LCD
     *
     * @param format A printf-style format string, followed by the
     *               variables to use in formating the string.
     */
    int printf(const char* format, ...);
#endif

    virtual int _putc(int value);

    /** Locate to a screen column and row
     *
     * @param column  The horizontal position from the left, indexed from 0
     * @param row     The vertical position from the top, indexed from 0
     */
    void locate(int column, int row);

    /** Clear the screen and locate to 0,0 */
    void cls();

    int rows();
    int columns();

protected:

    // Stream implementation functions
    //virtual int _putc(int value);
    virtual int _getc();

    int address(int column, int row);
    void character(int column, int row, int c);
    void writeByte(int value);
    void writeCommand(int command);
    void writeData(int data);

    DigitalOut _rs, _e;
    BusOut _d;
    LCDType _type;

    int _column;
    int _row;
};

TextLCD::TextLCD(PinName rs, PinName e, PinName d4, PinName d5,
                 PinName d6, PinName d7, LCDType type) : _rs(rs),
        _e(e), _d(d4, d5, d6, d7),
        _type(type) {

    _e  = 1;
    _rs = 0;            // command mode

    wait_us(15000);        // Wait 15ms to ensure powered up

    // send "Display Settings" 3 times (Only top nibble of 0x30 as we've got 4-bit bus)
    for (int i=0; i<3; i++) {
        writeByte(0x3);
        wait_us(1640);  // this command takes 1.64ms, so wait for it
    }
    writeByte(0x2);     // 4-bit mode
    wait_us(40);    // most instructions take 40us

    writeCommand(0x28); // Function set 001 BW N F - -
    writeCommand(0x0C);
    writeCommand(0x6);  // Cursor Direction and Display Shift : 0000 01 CD S (CD 0-left, 1-right S(hift) 0-no, 1-yes
    cls();
}

void TextLCD::character(int column, int row, int c) {
    int a = address(column, row);
    writeCommand(a);
    writeData(c);
}

void TextLCD::cls() {
    writeCommand(0x01); // cls, and set cursor to 0
    wait_us(1640);     // This command takes 1.64 ms
    locate(0, 0);
}

void TextLCD::locate(int column, int row) {
    _column = column;
    _row = row;
}

int TextLCD::_putc(int value) {
    if (value == '\n') {
        _column = 0;
        _row++;
        if (_row >= rows()) {
            _row = 0;
        }
    } else {
        character(_column, _row, value);
        _column++;
        if (_column >= columns()) {
            _column = 0;
            _row++;
            if (_row >= rows()) {
                _row = 0;
            }
        }
    }
    return value;
}

int TextLCD::_getc() {
    return -1;
}

void TextLCD::writeByte(int value) {
    _d = value >> 4;
    wait_us(40); // most instructions take 40us
    _e = 0;
    wait_us(40);
    _e = 1;
    _d = value >> 0;
    wait_us(40);
    _e = 0;
    wait_us(40); // most instructions take 40us
    _e = 1;
}

void TextLCD::writeCommand(int command) {
    _rs = 0;
    writeByte(command);
}

void TextLCD::writeData(int data) {
    _rs = 1;
    writeByte(data);
}

int TextLCD::address(int column, int row) {
    switch (_type) {
        case LCD20x4:
            switch (row) {
                case 0:
                    return 0x80 + column;
                case 1:
                    return 0xc0 + column;
                case 2:
                    return 0x94 + column;
                case 3:
                    return 0xd4 + column;
            }
        case LCD16x2B:
            return 0x80 + (row * 40) + column;
        case LCD16x2:
        case LCD20x2:
        default:
            return 0x80 + (row * 0x40) + column;
    }
}

int TextLCD::columns() {
    switch (_type) {
        case LCD20x4:
        case LCD20x2:
            return 20;
        case LCD16x2:
        case LCD16x2B:
        default:
            return 16;
    }
}

int TextLCD::rows() {
    switch (_type) {
        case LCD20x4:
            return 4;
        case LCD16x2:
        case LCD16x2B:
        case LCD20x2:
        default:
            return 2;
    }
}

TextLCD lcd(dp17, dp18, dp24, dp14, dp26, dp28); // rs, e, d4-d7

AnalogIn adcIns[] = {
    AnalogIn (dp4), //(dp9), //1114 10bit-adc
    AnalogIn (dp9),
    AnalogIn (dp10),
    AnalogIn (dp11),
};
const unsigned int adcIns_Size = sizeof(adcIns)/sizeof(*adcIns);

const unsigned int SAMPLE_SIZE = 50; //833ms = 50*rate(19)

SPISlave device(dp2, dp1, dp6, dp25); // mosi, miso, sclk, ssel

unsigned int passed_time = 0;
unsigned int avg[4] = {0};
uint16_t filter_avg[4] = {0};

Ticker lcd_flipper;
void print_lcd()
{
    lcd.cls();
    const char* text = "avg: fil: rate:\n";
    for ( unsigned int i = 0; i < strlen((char*)text); i++ )
    {
        lcd._putc(text[i]);
    }

    char* string;
    utoa(avg[0], string, 10);
    for ( unsigned int i = 0; i < strlen((char*)string); i++ )
    {
        lcd._putc(string[i]);
    }

    for ( unsigned int i = 0; i < (5 - strlen((char*)string)); i++ )
    {
        lcd._putc(*" ");
    }

    char* string2;
    utoa(filter_avg[0], string2, 10);
    for ( unsigned int i = 0; i < strlen((char*)string2); i++ )
    {
        lcd._putc(string2[i]);
    }

    for ( unsigned int i = 0; i < (5 - strlen((char*)string)); i++ )
    {
        lcd._putc(*" ");
    }

    unsigned int rate = passed_time / ((SAMPLE_SIZE) * adcIns_Size);
    char* string3;
    utoa(rate, string3, 10);
    for ( unsigned int i = 0; i < strlen((char*)string3); i++ )
    {
        lcd._putc(string3[i]);
    }
}

int main() {
    device.format(8, 0); // 8bit, mode 0
    device.frequency(1000000); // 1MHz
    //device.reply(0x00); // Prime SPI with first reply

    lcd_flipper.attach(&print_lcd, 500ms);
    while (true) {

        unsigned short analogHexes[adcIns_Size][SAMPLE_SIZE] = {{0}};
        Timer t;
        t.start();
        for ( unsigned int j = 0; j < adcIns_Size; j++ )
        {
            for ( unsigned int i = 0; i < SAMPLE_SIZE; i++ )
            {
                analogHexes[j][i] = adcIns[j].read_u16();  // Get ADC data
            }
        }
        t.stop();
        passed_time = t.read_us();

        for ( unsigned int j = 0; j < adcIns_Size; j++ )
        {
            for ( unsigned int i = 0; i < SAMPLE_SIZE; i++ )
            {
                analogHexes[j][i] = analogHexes[j][i] >> 6;  // adjsut 10bit
            }

            unsigned short analogHexes_low_pass[SAMPLE_SIZE] = {0};
            //low pass filter
            for ( unsigned int i = 1; i < (SAMPLE_SIZE - 1); i++ )
            {
                analogHexes_low_pass[i] = ( analogHexes[j][i-1] + analogHexes[j][i] + analogHexes[j][i+1] ) / 3;  // 3 point avg
            }

            unsigned int sum = 0;

            for ( unsigned int i = 1; i < (SAMPLE_SIZE - 1); i++ )
            {
                sum += analogHexes_low_pass[i];
            }
            avg[j] = sum / (SAMPLE_SIZE - 2);

            unsigned short analogHexes_high_pass[SAMPLE_SIZE] = {0};
            //high pass filter
            for ( unsigned int i = 1; i < (SAMPLE_SIZE - 1); i++ )
            {
                if (analogHexes_low_pass[i] > avg[j]) {
                    analogHexes_high_pass[i] = (analogHexes_low_pass[i] - avg[j]) * 10;
                } else {
                    analogHexes_high_pass[i] = (avg[j] - analogHexes_low_pass[i]) * 10;
                }
                //diff avg and raw data
            }

            unsigned int filter_sum = 0;

            for ( unsigned int i = 1; i < (SAMPLE_SIZE - 1); i++ )
            {
                filter_sum += analogHexes_high_pass[i];
            }
            filter_avg[j] = filter_sum / (SAMPLE_SIZE - 2);
        }

        // send to SPI master device

        if (device.receive()) {
            if (device.read() == 0x40) {
                device.reply(0x40); //first_seg
                for ( unsigned int j = 0; j < adcIns_Size; j++ ) {
                    uint16_t send_data = filter_avg[j];
                    uint8_t send_data_H = ((send_data >> 8) & 0xff); //filter_avg
                    uint8_t send_data_L = ((send_data >> 0) & 0xff); //filter_avg
                    device.reply(send_data_H);
                    device.reply(send_data_L);
                }
            }
        }
    }
}

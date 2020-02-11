#include <Wire.h>
#define WIRE_MAX_BYTES	0x18

void OLED::_convert_float(char *buf, double num, int width, byte prec)
{
	dtostrf(num, width, prec, buf);
}

void OLED::_initTWI()
{
	Wire.begin(_sda_pin, _scl_pin); // SDA, SCL
}

void OLED::update()
{
//	noInterrupts();
	ESP.wdtDisable();
	_sendTWIcommand(SSD1306_SET_COLUMN_ADDR);
	_sendTWIcommand(0);
	_sendTWIcommand(127);

	_sendTWIcommand(SSD1306_SET_PAGE_ADDR);
	_sendTWIcommand(0);
	_sendTWIcommand(7);
	
	Wire.beginTransmission(SSD1306_ADDR);
	Wire.write(SSD1306_DATA_CONTINUE);

	for (uint16_t b=0; b<_bufsize; b++)		// Send data
	{
		Wire.write(scrbuf[b]);
		if ((b % WIRE_MAX_BYTES) == (WIRE_MAX_BYTES -1))
		{
			Wire.endTransmission();
			Wire.beginTransmission(SSD1306_ADDR);
			Wire.write(SSD1306_DATA_CONTINUE);
		}
	}
	Wire.endTransmission();
//	interrupts();
}

void OLED::_sendTWIcommand(uint8_t value)
{
	Wire.beginTransmission(SSD1306_ADDR);
	Wire.write(SSD1306_COMMAND);
	Wire.write(value);
	Wire.endTransmission();
}

// *** Hardwarespecific defines ***

#define fontbyte(x) cfont.font[x]  
#define bitmapbyte(x) bitmap[x]

#define PROGMEM
#define bitmapdatatype unsigned char*

#ifndef TWI_FREQ
	#define TWI_FREQ 400000L
#endif



#ifndef LCD_H__
#define LCD_H__

void LCDinit(void);
void putChar(const char *letter, unsigned short x, unsigned short y, unsigned short color);
void writeString(const char *message, unsigned short x, unsigned short y, unsigned short color);
void makeBar(unsigned short x, unsigned short y, unsigned short color, int length, int progress);
void clearScreen(void);
void fillScreen(unsigned short color);


#endif
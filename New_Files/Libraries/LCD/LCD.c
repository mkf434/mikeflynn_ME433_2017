

#include <xc.h>                         // Pretty standard header files
#include <stdio.h>
#include <stdlib.h>
#include "ILI9163C.h"
#include <math.h>


void LCDinit(void){
	SPI1_init();
    LCD_init();
    
    TRISAbits.TRISA4 = 0;           // A7 is a digital output
    LATAbits.LATA4 = 1;             // A7 is on initially
    
    LCD_clearScreen(BLACK);
}

void putChar(const char *letter, unsigned short x, unsigned short y, unsigned short color){

    int ii = 0;
    int jj = 0;
    
    char bits[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
    int num_iis = *(&letter + 1) - letter;
   
        while(ii<5){  
            while(jj<8){
                        
                if(bits[7-jj]&&(bits[7-jj]&(letter[ii]))){
                    if((x+ii) < 130 && (x-ii) > 0 && (y+jj) < 131 && (y-jj) > 0){
                        LCD_drawPixel(x+ii,y+jj,color);
                    }            
                }            
 
                jj++;  
            }

            jj = 0;
            ii++;   
        }
}

void writeString(const char *message, unsigned short x, unsigned short y, unsigned short color){
	
	int ii = 0; 
    int letter = 0; 
    int nextline = 0;
    
    int x1 = x;
    int y1 = y;
    
    while(message[ii]){
        
        letter = (int) message[ii];
        letter = letter - 32;
        if(x+(6*ii)>125){ nextline = nextline + 1; x1 = 5; }
        putChar(ASCII[letter],x1+(6*ii),y1+(9*nextline), color);
        ii++;    
        
    }
}

void makeBar(unsigned short x, unsigned short y, unsigned short color, int length, int progress){
	int xx = 0; 
    const char bar[5] = {0xFF, 0x00, 0x00, 0x00, 0x00};
    const char bar2[5] = {0x81, 0x00, 0x00, 0x00, 0x00};
    
    putChar(bar,15,62,WHITE);
    //_CP0_SET_COUNT(0);
    //while(_CP0_GET_COUNT()<4800000){;}
    //sprintf(string1, "     Percent");
    //loopString(string1, 28, 90, WHITE);
    
    while(1){
    
    while(xx<length){
        
        //loopString(black,28, 90, BLACK);
        //sprintf(string1, "%d",xx);
        loopString(string1, 28, 90, WHITE);
        
        putChar(bar,15+xx,62,WHITE);
        xx++;
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<24000000){;}
        
    }
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<4800000){;}
    
    putChar(bar,15+xx,62,WHITE);
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<4800000){;}
    
    sprintf(string1, "100  Percent");
    loopString(string1, 28, 90, WHITE);
    
    }
}

void clearScreen(void){
	 
	LCD_clearScreen(BLACK);
}

void fillScreen(unsigned short color){

	LCD_clearScreen(color);

}
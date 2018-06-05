

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
                }   else { LCD_drawPixel(x+ii,y+jj,BLACK); }          
 
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
	int yy = (progress/length)*length;

    putChar(0xFF,x,y,WHITE);

    while(xx<yy){  
        putChar(0x81,x+xx,y,WHITE);
        
    }
    
    putChar(0xFF,x+xx,y,WHITE);
}

void clearScreen(void){
	 
	LCD_clearScreen(BLACK);
}

void fillScreen(unsigned short color){

	LCD_clearScreen(color);

}

void clearChar(unsigned short x, unsigned short y){
    
    int ii = 0;
    int jj = 0;
    
    while(ii<5){ 
        
        while(jj<8){
            
            LCD_drawPixel(x+ii,y+jj,BLACK);
            jj++;        
            
        }            
 
        jj = 0;
        ii++;
            
    } 
}
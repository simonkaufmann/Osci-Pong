/*  Pong at the Oscilloscope - let you play pong at an analog oscilloscope in xy-mode
    Copyright (C) 2014  Simon Kaufmann - HeKa

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//experimental with new plotting engine


//crystal frequency: 16MHz!!


#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <math.h>

//#define CHAR_TESTER  //set this define if you want a char-tester program
#define ENABLE_LED_MULTIPLEX //comment it out if you want to disable the led matrix!
#define ENABLE_EASTEREGG1 //enables the easteregg that your paddle follows the ball automatically
 //get into easteregg by pressing the pause-button at your controller until countdown is zero

#define POINTS_START 0

#define BEEP_DDR DDRD
#define BEEP_PORT PORTD
#define BEEP PD1 //loudspeaker is connected via rs232-connector of pcb -> tx-line
#define BEEP_ON() (BEEP_PORT |= (1<<BEEP))
#define BEEP_OFF() (BEEP_PORT &= ~(1<<BEEP))
#define BEEP_TIME 5 //how many cykles of TIMER0 (the timer which reads the adc values)

#define POINTS_TO_WIN 10

#define LDAC_DDR DDRD //for synchronising the two channels of DAC (cable later added by soldering)
#define LDAC_PORT PORTD
#define LDAC PD3 //if LDAC is put from high to low -> both buffered channels are set to their previously buffered value!

#define LED_MATRIX_DDR DDRC
#define LED_MATRIX_PORT PORTC

#define LED_MUX_DDR DDRA
#define LED_MUX_PORT PORTA
//the mux pins must be edited in programm (it is needed for shifting!)

#define SPI_PORT PORTB
#define SPI_DDR DDRB
#define MOSI PB5
#define SCK PB7

//chip select for SPI:
#define CS PB4
#define CS_PORT PORTB
#define CS_DDR DDRB

//pause switches:
#define SW_PAUSE_PIN PINA
#define SW_START_PIN PINA
#define SW_START PA6
#define SW_PAUSE  PA7

#define CONT_SWITCH_PIN PINB
#define C1S1 PB3 //Controller1 switch1
#define C1S2 PB2 //controller1 switch1
#define C2S1 PB1 //Controller2 switch2
#define C2S2 PB0 //controller2 switch2

#define STATUS_LED PD2
#define STATUS_LED_PORT PORTD
#define STATUS_LED_DDR DDRD

#define STATUS_LED_TOGGLE()	STATUS_LED_PORT^=(1<<STATUS_LED)
#define STATUS_LED_ON()	STATUS_LED_PORT|=(1<<STATUS_LED)
#define STATUS_LED_OFF() STATUS_LED_PORT&=~(1<<STATUS_LED)

#define OK 0
#define ERR 1
#define PROGRESS 2

#define X 0 //for DAC-channel choice
#define Y 1 //for DAC-channel choice

#define OFF_X_L 20 //Offset_X_LEFT
#define OFF_X_R 235 //= 255 - OFF_X_L, if symmetrical,  Offset_X_RIGHT (how far right paddle is away from border)
//it's the other way round -> U (upper) is in real Down, and D (down) is in real up on the screen!
#define OFF_Y_U 255 //highest point reachable for ball in y-direction (the wall)
#define OFF_Y_D	0 //lowes point reachable for ball in y-direction (the wall)
#define PAD_SIZE 50 //length of paddle
#define BALL_SIZE 10 //diameter of ball

#define POTI_OFF 70 //is subtracted from ADC-value of paddle-poti
#define POTI_MULT 1.3 //subtracted ADC-value ist multiplied by this value

#define INCY_MAX 3

#define INCY_STD 1
#define INCX_STD 1

#define Y_SCALE 1.25 //x-values have to be multiplied with Y_SCALE so that it would be as long in the y-direction
					//because osci is not square!

#define T_1 1 //first t - switching point (says what is plotted -> line or ball...)
			//bigger T_1 increases brightness of left line
#define T_2 2 //first t - switching point (says what is plotted -> line or ball...)
			//bigger T_2 - T_1 increases brightness of right line
			//ball is only one time plotted! -> the darkest object (but is bright,
			//because it is a ball -> areas are brighter than lines

//defines for pm (plot mode)
#define READY 0
#define VERT_LINE 1
#define HORIZ_LINE 2


#define NUM_SIZE_X 15
#define NUM_SIZE_Y 30

#define NUM11_X 70
#define NUM11_Y 10
#define NUM12_X 100 //position of first number x
#define NUM12_Y 10 //position of first number y
#define NUM2_X 140 //if number 2 is below 10 -> if only one number has to be shown
#define NUM2_Y 10 // -||-
#define NUM21_X 130 // if number 2 is 10 or higher -> two numbers must be shown
#define NUM21_Y 10
#define NUM22_X 160  // the second number of number 2 if two numbers must be shown!
#define NUM22_Y 10
#define NUM222_X 170 // a bit righter than NUM22_X -> is shown if first digit is more than 1 (then it must be righter)
#define NUM222_Y 10
#define COLON_X	128 //position of colon between numbers
#define COLON_Y 10 //position of colon between numbers

#define PRELOAD_TIMER2 220//255-10	//together with prescaler 8 -> frequency of 20kHz, now slower, so that the whole plotting calculations can be done in isr
#define PRELOAD_TIMER0 150
#define PRELOAD_TIMER1 65500 //-> 100Hz for led-multiplex!

//game status
#define INIT 4 //initalize at start -> show welcome screen
#define STOP 0 //set all to zero
#define PAUSE 1 //just ball stopped
#define RUNNING 2 //running mode
#define WON 3 //one has won

#define HEADING_X 85 //position of "PONG" headline x
#define HEADING_Y 70 //position of "PONG" headline y
#define HEADING_SIZE_X 15
#define HEADING_SIZE_Y 30
#define HEADING_SPACE 8

#define HEADING "PONG"
#define HEADING_CHARS 4

#define SUBHEADING_X 30 //position of "PONG" headline x
#define SUBHEADING_Y 155 //position of "PONG" headline y
#define SUBHEADING_SIZE_X 8
#define SUBHEADING_SIZE_Y 16
#define SUBHEADING_SPACE 5

#define SUBHEADING "A HEKA-PROJECT"
#define SUBHEADING_CHARS 14

#define PAUSE_MESSAGE "PAUSE"
#define PAUSE_CHARS 5


#define PAUSE_X 90
#define PAUSE_Y 165
#define PAUSE_SIZE_X 10
#define PAUSE_SIZE_Y 20
#define PAUSE_SPACE 6

#define PLAYER_MESSAGE	"PLAYER"
#define PLAYER_CHARS 6

#define PLAYER_X 75
#define PLAYER_Y 100
#define PLAYER_SIZE_X 10
#define PLAYER_SIZE_Y 20
#define PLAYER_SPACE 6

#define WIN_MESSAGE	"WINS"
#define WIN_CHARS 4

#define WIN_X 100
#define WIN_Y 160
#define WIN_SIZE_X 10
#define WIN_SIZE_Y 20
#define WIN_SPACE 7

#define BALL_INIT_X 123 //init position for a ball
#define BALL_INIT_Y 123 //init position for a ball

#define COUNTDOWN 3

#define COUNTDOWN_X 123
#define COUNTDOWN_Y 165
#define COUNTDOWN_SIZE_X 10
#define COUNTDOWN_SIZE_Y 20

//for countdown-timing at timer 0:
#define COUNTDOWN_COUNT_MAX 100


void sendDAC(uint8_t channel, uint8_t value);

uint8_t plotVLine(uint8_t x, uint8_t y1, uint8_t y2);
uint8_t plotHLine(uint8_t x1, uint8_t x2, uint8_t y);
uint8_t plotLine(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2);
uint8_t plotCircle(uint8_t d, uint8_t xm, uint8_t ym);
uint8_t plotNumber(uint8_t num, uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey);
uint8_t plotColon(uint8_t x, uint8_t y);
uint8_t plotBall(uint8_t size, uint8_t ballx, uint8_t bally);

const char heading[]=HEADING;
const char subheading[]=SUBHEADING;
const  char pause_message[]=PAUSE_MESSAGE;
const  char player_message[]=PLAYER_MESSAGE;
const  char win_message[]=WIN_MESSAGE;

//only capital chars are supported (A-Z)!!!
uint8_t plotChar(char ch, uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey);

//are changed by TIM0 -> therefore volatile!
volatile float posx = BALL_INIT_X;//kugel posx
volatile float posy = BALL_INIT_Y;//kugel posy

volatile float incx = INCX_STD; //increment of ballposx in 10ms
volatile float incy = INCY_STD; //increment of ballposy in 10ms (TIMER0-frequency)

volatile int16_t pl,pr; //position of paddle left and position right
volatile int16_t pl_old, pr_old; //old position needed for speed of paddle!

volatile uint8_t t=0;

volatile uint8_t adc_c=0;

volatile uint8_t c=0;
volatile uint8_t c2=0;

volatile uint8_t lx1 = 0; //line x
volatile uint8_t lx2 = 0; //line x2
volatile uint8_t ly1 = 0; //line x
volatile uint8_t ly2 = 0;
volatile uint8_t pm = READY; //plot-mode: 0 -> inactive, 1 -> vertical line (see defines plotmode!)

volatile uint8_t points1 = 0;
volatile uint8_t points2 = 0;

volatile uint8_t countdown_count = 0; //waits 100 times for Timer0 -> to get about 1sec delay
volatile uint8_t countdown = 0; //if 0 -> wait two seconds until ball moves!
								//if 1 -> ball is moving!
volatile uint8_t countdown_show = 0; //1-> show countdown, 0-> don't show


volatile uint8_t gamestatus = INIT;
volatile uint8_t playerwon = 1; //shows who has won (1-> player 1), (2-> player2)

const uint8_t numbers[10]={
	//format : Bit7: no meaning!, Bit6: top line, Bit5: right upper, Bit4: right lower, Bit3: down
	// Bit2: left lower, Bit1: left upper, Bit0: middle line
	//Bit=1 -> line is switched on, Bit=0 -> line is switched off
	/* 0 */ 0b01111110,
	/* 1 */ 0b00110000,
	/* 2 */ 0b01101101,
	/* 3 */ 0b01111001,
	/* 4 */ 0b00110011,
	/* 5 */ 0b01011011,
	/* 6 */ 0b01011111,
	/* 7 */ 0b01110000,
	/* 8 */ 0b01111111,
	/* 9 */ 0b01111011 };

//for chars (14-segment-display)
//format : Bit7: no meaning!, Bit6: top line, Bit5: right upper, Bit4: right lower, Bit3: down
// Bit2: left lower, Bit1: left upper, Bit0: middle line left
#ifdef CHAR_TESTER
volatile ch2 = 'A';
#endif
const uint8_t char1[27]= {
 /* A */ 0b01110111,
 /* B */ 0b00000111,
 /* C */ 0b01001110,
 /* D */ 0b00000110, //D and B have some special lines -> see plotChar
 /* E */ 0b01001111,
 /* F */ 0b01000111,
 /* G */ 0b01011110,
 /* H */ 0b00110111,
 /* I */ 0b00000000,
 /* J */ 0b00111100,
 /* K */ 0b00000111,
 /* L */ 0b00001110,
 /* M */ 0b00110110,
 /* N */ 0b00110110,
 /* O */ 0b01111110,
 /* P */ 0b01100111,
 /* Q */ 0b01111110,
 /* R */ 0b01100111,
 /* S */ 0b01011011,
 /* T */ 0b01000000,
 /* U */ 0b00111110,
 /* V */ 0b00000000, //V has some special lines -> see plot Char
 /* W */ 0b00110110,
 /* X */ 0b00000000,
 /* Y */ 0b00000000,
 /* Z */ 0b01001000,
 /* - */ 0b00000001, /* hyphen for heka-project*/
};

//format : Bit7: no meaning!, Bit6: middle line right, Bit5: upper left diagonal line,
// Bit4: upper right diagonal line,
// Bit3: lower left diagonal line, Bit2: lower right diagonal line
// Bit1: upper vertical middle line, Bit0: lower vertical middle line
const uint8_t char2[27] = {
 /* A */ 0b01000000,
 /* B */ 0b01000000,
 /* C */ 0b00000000,
 /* D */ 0b00000000, //D and B have some special lines -> see plotChar
 /* E */ 0b01000000,
 /* F */ 0b01000000,
 /* G */ 0b01000000,
 /* H */ 0b01000000,
 /* I */ 0b00000011,
 /* J */ 0b00000000,
 /* K */ 0b00010100,
 /* L */ 0b00000000,
 /* M */ 0b00110000,
 /* N */ 0b00100100,
 /* O */ 0b00000000,
 /* P */ 0b01000000,
 /* Q */ 0b00000100,
 /* R */ 0b01000100,
 /* S */ 0b01000000,
 /* T */ 0b00000011,
 /* U */ 0b00000000,
 /* V */ 0b00000000, //V has some special lines -> see plot Char
 /* W */ 0b00001100,
 /* X */ 0b00111100,
 /* Y */ 0b00110001,
 /* Z */ 0b00011000,
 /* - */ 0b01000000,
};

const uint8_t matrix[10] = {
	//Bit7: Point (not connected!), Bit6: middle line, Bit5: left upper line,
	//Bit4: left lower line, Bit3: down line, Bit2: right down line, Bit1: right upper line
	//Bit0: upper line
	// -> see connection of LED_MATRIX_PORT!
	/* 0 */ 0b00111111,
	/* 1 */ 0b00000110,
	/* 2 */ 0b01011011,
	/* 3 */ 0b01001111,
	/* 4 */ 0b01100110,
	/* 5 */ 0b01101101,
	/* 6 */ 0b01111101,
	/* 7 */ 0b00000111,
	/* 8 */ 0b01111111,
	/* 9 */ 0b01101111
};

volatile uint8_t easteregg_c1 =0, easteregg_c11=0; //if you press button at beginning of a game up to countdown is zero -> your paddle is following the ball automatically
volatile uint8_t easteregg_c2 =0, easteregg_c22=0; //for controller 2
//easteregg_c11 -> set by controller 2 for easteregg in controller1
//easteregg_c22 -> set by controller 1 for easteregg in controller2 (when you press from countdown = 3 to countdown = 1 and then don't press it)

uint8_t beep_c = 0; //beepcount if it is set to BEEP_TIME and decreased in ISR of TIMER0 -> if it is down to zero -> BEEP_OFF()!

int main(void)	{


	//configure SPI-Interface (Master, enable SPI, LSB is first transmitted, frequency = clock /128)
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR);//|(1<<SPR0)|(1<<SPR1);

	//config ADC -> left adjusted result!
	ADMUX = (1<<ADLAR);
	//ADC -> enable, start conversion, enable auto trigger, set frequency to clk/64 -> about 250kHz
	ADCSRA= (1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	//TIMER0 -> for moving the ball and for countdown at beginning
	//set TIMER0 to prescaler 1024 (frequency of timer interrupt should be about 100Hz)
	TCCR0|=(1<<CS02)|(1<<CS00);

	//TIMER1 -> led-multiplex!
	//prescaler = 1024
	TCCR1B|=(1<<CS22)|(1<<CS20);
	TCNT1H=(PRELOAD_TIMER1>>8); //you must first write to TCNT1H!!! (see accessing 16-bit registers in datasheet)
	TCNT1L=PRELOAD_TIMER1;


	//set timer overflow interrupts
	TIMSK|=(1<<TOIE2)|(1<<TOIE0)|(1<<TOIE1);
	sei();
	//set timer2 to prescaler 32//8
	TCCR2|=(1<<CS21);

	//initialize the LED-Matrix
	LED_MUX_DDR |= (1<<5)|(1<<4)|(1<<3)|(1<<2);
	LED_MATRIX_DDR = 0xFF;

	//LDAC for synchronising channels
	LDAC_DDR |=(1<<LDAC);

	//init beep
	BEEP_DDR |= (1<<BEEP);

	//set status led
	STATUS_LED_DDR |= (1<<STATUS_LED);
	//set SCK and MOSI as Output
	SPI_DDR |= (1<<MOSI)|(1<<SCK);
	//turn on status led
	STATUS_LED_ON();

	//set CS for DAC as output and set it to high (idle)
	CS_PORT|=(1<<CS);
	CS_DDR |= (1<<CS);

	uint8_t top=0; //toplot
	while(1)	{

		static uint8_t sw_pause, sw_pause_high;
		static uint16_t sw_c; //sw_pause_high -> if =0 -> sw_pause was
		//high and has to be considered this time
		//otherwise it was always pressed and therefore should not have an effect
		sw_pause=SW_PAUSE_PIN&(1<<SW_PAUSE);
		if(sw_pause==0&&sw_pause_high==1)	{
			//sw-pause pressed
			sw_pause_high=0;
			sw_c=0;
			if (gamestatus==PAUSE)	{
				gamestatus=RUNNING;
			}
			else if (gamestatus==RUNNING)	{
				gamestatus=PAUSE;
			}
			else if (gamestatus==WON||gamestatus==STOP||gamestatus==INIT)	{
				resetGame();
			}
		}
		else	{
			sw_c++;
			if(sw_c==1000)	{
				sw_pause_high=1;
			}//Entprellung!
		}



		static uint8_t c1s1, c1s1_high;
		static uint16_t c1s1_c; //sw_pause_high -> if =0 -> sw_pause was
		static uint8_t pressedc1=0;
		static uint8_t pre_beg_c1=0; //pressed at beginning -> for easter egg
		//high and has to be considered this time
		//otherwise it was always pressed and therefore should not have an effect
		c1s1=CONT_SWITCH_PIN&(1<<C1S1);
		if(c1s1==0&&c1s1_high==1)	{
			//sw-pause pressed
			if(countdown==COUNTDOWN)	{
				pre_beg_c1=1;
			}
			if(countdown==0&&pre_beg_c1==1)	{
				easteregg_c1=1;
				easteregg_c22=0;
				//first pause is not executed if you go to easteregg
			}
			if(countdown==1&&pre_beg_c1==1)	{
				//if only to one than the other player has easteregg
				easteregg_c22=1;
				//first pause is not executed if you go to easteregg
			}
			if(pre_beg_c1!=1)	{
				pressedc1=1;
			}
			c1s1_c=0;
		}
		else	{
			c1s1_c++;
			if(c1s1_c==1000)	{
				c1s1_high=1;
				pre_beg_c1=0; //first pause is not executed if you go to easteregg
			}//Entprellung!
			if (pressedc1==1&&pre_beg_c1!=1)	{
				c1s1_high=0;

				if (gamestatus==PAUSE)	{
					gamestatus=RUNNING;
				}
				else if (gamestatus==RUNNING)	{
					gamestatus=PAUSE;
				}
				else if (gamestatus==WON||gamestatus==STOP||gamestatus==INIT)	{
					resetGame();
				}
				pressedc1=0;
			}
		}

		static uint8_t c2s1, c2s1_high;
		static uint16_t c2s1_c; //sw_pause_high -> if =0 -> sw_pause was
		static uint8_t pressedc2=0;
		static uint8_t pre_beg_c2=0; //pressed at beginning -> for easter egg
		//high and has to be considered this time
		//otherwise it was always pressed and therefore should not have an effect
		c2s1=CONT_SWITCH_PIN&(1<<C2S1);
		if(c2s1==0&&c2s1_high==1)	{
			//sw-pause pressed
			if(countdown==COUNTDOWN)	{
				pre_beg_c2=1;
			}
			if(countdown==0&&pre_beg_c2==1)	{
				easteregg_c2=1;
				easteregg_c11=0;
				//first pause is not executed if you go to easteregg
			}
			if(countdown==1&&pre_beg_c2==1)	{
				//if only to one than the other player has easteregg
				easteregg_c11=1;
				//first pause is not executed if you go to easteregg
			}
			if(pre_beg_c2!=1)	{
				pressedc2=1;
			}
			c2s1_c=0;
		}
		else	{
			c2s1_c++;
			if(c2s1_c==1000)	{
				c2s1_high=1;
				pre_beg_c2=0; //first pause is not executed if you go to easteregg
			}//Entprellung!
			if (pressedc2==1&&pre_beg_c2!=1)	{
				c2s1_high=0;
				if (gamestatus==PAUSE)	{
					gamestatus=RUNNING;
				}
				else if (gamestatus==RUNNING)	{
					gamestatus=PAUSE;
				}
				else if (gamestatus==WON||gamestatus==STOP||gamestatus==INIT)	{
					resetGame();
				}
				pressedc2=0;
			}
		}


		if((SW_START_PIN&(1<<SW_START))==0)	{
			//sw-start pressed
			resetGame();
		}






		if (pm==READY)	{
		//	plotChar('F', 50,50,50,50);
		/*
			if(gamestatus==INIT)	{
				//show welcome screen
				if(top>=0&&top<HEADING_CHARS)	{
					if(plotChar(heading[top], HEADING_X+top*(HEADING_SIZE_X+HEADING_SPACE), HEADING_Y, HEADING_SIZE_X, HEADING_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				//draw border so that oscilloscope can be adjusted!
				else if(top==HEADING_CHARS)	{
					plotVLine(0,0,20);
					top++;
				}
				else if(top==HEADING_CHARS+1)	{
					plotVLine(255,0,20);
					top++;
				}
				else if(top==HEADING_CHARS+2)	{
					plotHLine(0,20,0);
					top++;
				}
				else if(top==HEADING_CHARS+3)	{
					plotHLine(0,20,255);
					top++;
				}
				else if(top==HEADING_CHARS+4)	{
					plotVLine(0,235,255);
					top++;
				}
				else if(top==HEADING_CHARS+5)	{
					plotVLine(255,235,255);
					top++;
				}
				else if(top==HEADING_CHARS+6)	{
					plotHLine(235,255,0);
					top++;
				}
				else if(top==HEADING_CHARS+7)	{
					plotHLine(235,255,255);
					top++;
				}
				if(top>=HEADING_CHARS+8&&top<HEADING_CHARS+8+SUBHEADING_CHARS)	{
					if(plotChar(subheading[top-HEADING_CHARS-8], SUBHEADING_X+(top-HEADING_CHARS-7)*(SUBHEADING_SIZE_X+SUBHEADING_SPACE), SUBHEADING_Y, SUBHEADING_SIZE_X, SUBHEADING_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}

				if(top>HEADING_CHARS+7+SUBHEADING_CHARS)	{
					top=0;
				}
			}

			else if (gamestatus==WON)	{
				if(top>=0&&top<PLAYER_CHARS)	{
					if(plotChar(player_message[top], PLAYER_X+top*(PLAYER_SIZE_X+PLAYER_SPACE), PLAYER_Y, PLAYER_SIZE_X, PLAYER_SIZE_Y)==OK)	{
						top++;
					}
				}
				else if(top==PLAYER_CHARS)	{
					//playerspace once more added so that there is a gap between "player" and the number
					if(plotNumber(playerwon, PLAYER_X+top*(PLAYER_SIZE_X+PLAYER_SPACE)+PLAYER_SPACE*2, PLAYER_Y, PLAYER_SIZE_X, PLAYER_SIZE_Y)==OK)	{
						top++;
					}
				}
				else if(top>=PLAYER_CHARS+1&&top<PLAYER_CHARS+2+WIN_CHARS)	{
					if(plotChar(win_message[top-PLAYER_CHARS-2], WIN_X+(top-PLAYER_CHARS-2)*(WIN_SIZE_X+WIN_SPACE), WIN_Y, WIN_SIZE_X, WIN_SIZE_Y)==OK)	{
						top++;
					}
				}
				else if (top==PLAYER_CHARS+2+WIN_CHARS)	{
					if(points1/10<1)	{
						top++;
					}
					else	{
						if(plotNumber(points1/10, NUM11_X, NUM11_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}
				else if (top==PLAYER_CHARS+WIN_CHARS+3)	{
					if(plotNumber(points1%10, NUM12_X, NUM12_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else if (top==PLAYER_CHARS+WIN_CHARS+4)	{
					if(points2/10<1)	{
						top++;
					}
					else	{
						if(plotNumber(points2/10, NUM21_X, NUM21_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}
				else if (top==PLAYER_CHARS+WIN_CHARS+5)	{
					if(points2/10<1)	{
						//put second number mor to the left
						if(plotNumber(points2%10, NUM2_X, NUM2_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
					else	{
						if(plotNumber(points2%10, NUM22_X, NUM22_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}
				else if (top==PLAYER_CHARS+WIN_CHARS+6)	{
					if(plotColon(COLON_X, COLON_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else if (top==PLAYER_CHARS+WIN_CHARS+7)	{
					plotVLine(OFF_X_L, pl, pl+PAD_SIZE);
					top++;
				}
				else if (top==PLAYER_CHARS+WIN_CHARS+8)	{
					plotVLine(OFF_X_R, pr, pr+PAD_SIZE);
					top++;
				}


				if(top>PLAYER_CHARS+WIN_CHARS+8)	{
					top=0;
				}
			}

			else {
				//if balls, paddle or number plotting is changed here,
				//change it also at gamestatus == WON!

				//show ball and all the things!
				if (top==0)	{
					plotVLine(OFF_X_L, pl, pl+PAD_SIZE);
					top++;
				}
				else if (top==1)	{
					plotVLine(OFF_X_R, pr, pr+PAD_SIZE);
					top++;
				}
				else if (top==2)	{
					static uint8_t ballx=0, bally=0;
					if(plotBall(BALL_SIZE, ballx, bally)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
						ballx=posx; bally=posy; //so that the position changes not while plotting the ball
						//because then there is a ugly stripe on the screen (when putting in a new ball)
					}
				}
				else if (top==3)	{
					if(points1/10<1)	{
						top++;
					}
					else	{
						if(plotNumber(points1/10, NUM11_X, NUM11_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}
				else if (top==4)	{
					if(plotNumber(points1%10, NUM12_X, NUM12_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else if (top==5)	{
					if(points2/10<1)	{
						top++;
					}
					else	{
						if(plotNumber(points2/10, NUM21_X, NUM21_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}
				else if (top==6)	{
					if(points2/10<1)	{
						//put second number mor to the left
						if(plotNumber(points2%10, NUM2_X, NUM2_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
					else	{
						if(plotNumber(points2%10, NUM22_X, NUM22_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}
				else if (top==7)	{
					if(plotColon(COLON_X, COLON_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else if(gamestatus==PAUSE)	{
					if(top>=8&&top<PAUSE_CHARS+8)	{
						if(plotChar(pause_message[top-8], PAUSE_X+(top-8)*(PAUSE_SIZE_X+PAUSE_SPACE), PAUSE_Y, PAUSE_SIZE_X, PAUSE_SIZE_Y)==OK)	{
								top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}
				else {
					if(top==8)	{
						if(plotNumber(countdown, COUNTDOWN_X, COUNTDOWN_Y, COUNTDOWN_SIZE_X, COUNTDOWN_SIZE_Y)==OK)	{
								top++; //otherwise plotCircle must be redrawed when pm is next ready!
						}
					}
				}




				if(gamestatus == PAUSE)	{
					if(top>PAUSE_CHARS+7)	{
						top=0;
					}
				}
				else if	(countdown_show==1)	{
					if(top>8)	{
						top=0;
					}
				}
				else	{
					if(top>7)	{
						top=0;
					}
				}

			}*/
		}
	}

	return 0;
}


//sends data to DAC
// channel = 0 -> channel A, channel = 1 -> channel B
// value: the least 12 bit are used as value for the eprom (0b0000111111111111 means 5V)
void sendDAC(uint8_t channel, uint8_t value)	{
	if(channel==Y)	{
		value=255-value;
	 }
	uint8_t dh,dl; //datahigh and datalow
	//Bit 7: channel, Bit 6: 1->buffered, Bit 5: 1-> Gain = 1 otherwise gain would be doubled, Bit 4: 1 means not shutdown
	//Bit 3 - Bit 0: DataBits 11 - DataBits 8
	dh=(channel<<7)|(1<<6)|(1<<5)|(1<<4)|((value>>4)&(0b00001111));
	//Bit 7 - Bit 0: DataBits 7 - DataBits 0 of value!
	dl=(value<<4);

	//send SPI:
	CS_PORT&=~(1<<CS); //CS low
	SPDR=dh;
	while(!(SPSR & (1<<SPIF))); //wait until transmit complete (SPIF ist resettet by writing into SPDR)
	SPDR=dl;
	while(!(SPSR & (1<<SPIF))); //wait transmit complete
	CS_PORT|=(1<<CS);
}


//initiates plotting a vertical line by timer 2,
//x -> x-value
//y1 -> first value of vertical line
//y2 -> second value of vertical line
//return value: if timer 2 is not free to plot something -> return ERR, else -> return OK
uint8_t plotVLine(uint8_t x, uint8_t y1, uint8_t y2)	{
	if(pm!=READY)	{
		return ERR;
	}
	lx1=x;
	//ensure that ly2 is always bigger or equal than ly1!
	if (y2<y1)	{
		ly1=y2;
		ly2=y1;
	}
	else	{
		ly1=y1;
		ly2=y2;
	}
	pm=VERT_LINE;
	return OK;
}

//initiates plotting a horizontal line by timer 2,
//x -> x-value
//y1 -> first value of vertical line
//y2 -> second value of vertical line
//return value: if timer 2 is not free to plot something -> return ERR, else -> return OK
uint8_t plotHLine(uint8_t x1, uint8_t x2, uint8_t y)	{
	if(pm!=READY)	{
		return ERR;
	}
	ly1=y;
	//ensure that lx2 is always bigger or equal than lx1!
	if (x2<x1)	{
		lx1=x2;
		lx2=x1;
	}
	else	{
		lx1=x1;
		lx2=x2;
	}
	pm=HORIZ_LINE;
	return OK;
}

//plots a line (only use it if it is not a vertical or horizontal line!)
//return value:
// must so often be called until it returns OK (then circle is ready drawed)
// otherwise it returns progress -> must be recalled when pm is READY again!
uint8_t plotLine(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2)	{
	if(pm!=READY)	{
		return ERR;
	}
	if (x2<x1)	{
		//so that x2 is always bigger than x1
		uint8_t temp=x1;
		x1=x2;
		x2=temp;
		temp=y1;
		y1=y2;
		y2=temp;
	}
	static uint8_t first=0;
	static uint8_t x;
	static float k=0; //rising
	static uint8_t ynew, yold=0;
	if(first==0)	{
		x=x1;
		//first=1; //first is set to one some lines underneath
		yold=0;
		k=(float)(y2-y1)/(float)(x2-x1);
	}
	ynew=k*(x-x1)+y1;
	if	(first!=0)	{
		//first x is not printed (is a dirty very bright point!)
		plotVLine(x, yold, ynew);
	}
	else	{
		first=1;
	}
	x++;
	yold=ynew;
	if(x>x2||x==255)	{ //x==255, so that x won't overflow to zero!
		first=0;
		return OK;
	}
	else	{
		return PROGRESS; //circle isn't ready yet (there are some lines to be drawed!)
	}
}

//plots a circle
// xm -> x coordinate of center
// ym -> y coordinate of center
// d -> diameter
//return value:
// must so often be called until it returns OK (then circle is ready drawed)
// otherwise it returns progress -> must be recalled when pm is READY again!
uint8_t plotCircle(uint8_t d, uint8_t xm, uint8_t ym)	{
	//xm + d/2 + 1 must not be bigger than 255!
	if(pm!=READY)	{
		return ERR;
	}
	static uint8_t first=0;
	static uint8_t x;
	static uint8_t ynew, yold=0;
	if(first==0)	{
		x=xm-d/2;
		//first=1; //first is set to one some lines underneath
		yold=0;
	}
	uint16_t dd = d*d/4;
	int16_t xx = (x-xm)*(x-xm);
	ynew=sqrt(dd-xx);
	ynew*=Y_SCALE;
	if(first!=0)	{
		//first x is not printed (is a dirty very bright point!)
		plotVLine(x, ym+yold, ym+ynew);
		while(pm!=READY);
		plotVLine(x, ym-yold, ym-ynew);
	}
	else	{
		first=1;
	}
	x++;
	yold=ynew;
	if(x>xm+d/2)	{
		first=0;
		return OK;
	}
	else	{
		return PROGRESS; //circle isn't ready yet (there are some lines to be drawed!)
	}
}

uint8_t plotBall(uint8_t size, uint8_t ballx, uint8_t bally)	{
	static uint8_t first=0;
	static uint8_t x=0;
	if(first==0)	{
		x=ballx;
		first=1;
	}
	plotVLine(x, bally, bally+size);
	x++;
	if(x>ballx+size)	{
		first=0;
		return OK;
	}
	else	{
		return PROGRESS;
	}
}

uint8_t plotNumber(uint8_t num, uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey)	{
	if(pm!=READY)	{
		return ERR;
	}
	if(num>9)	{
		num=9;
	}
	static uint8_t count = 0;
	if(count==0)	{
		//top line
		if(numbers[num]&(1<<6))	{
			plotHLine(x, x+sizex, y);
		}
	}
	else if(count==1)	{
		//right upper line
		if(numbers[num]&(1<<5))	{
			plotVLine(x+sizex, y, y+sizey/2);
		}
	}
	else if(count==2)	{
		//right lower line
		if(numbers[num]&(1<<4))	{
			plotVLine(x+sizex, y+sizey/2, y+sizey);
		}
	}
	else if(count==3)	{
		//down line
		if(numbers[num]&(1<<3))	{
			plotHLine(x, x+sizex, y+sizey);
		}
	}
	else if(count==4)	{
		//left lower line
		if(numbers[num]&(1<<2))	{
			plotVLine(x, y+sizey/2, y+sizey);
		}
	}
	else if(count==5)	{
		//left up line
		if(numbers[num]&(1<<1))	{
			plotVLine(x, y, y+sizey/2);
		}
	}
	else if(count==6)	{
		//middle line
		if(numbers[num]&(1<<0))	{
			plotHLine(x, x+sizex, y+sizey/2);
		}
	}
	count++;
	if(count>6)	{
		count=0;
		return OK;
	}
	else	{
		return PROGRESS;
	}
}


//only capital chars are supported (A-Z)!!!
uint8_t plotChar(char ch, uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey)	{
	if(pm!=READY)	{
		return ERR;
	}
	if(ch=='-')	{
		ch=26; //constant value, only char not out of capital chars that is supported
	}
	else	{
		ch -= 'A'; //subtract 'A' so that 'A' is 0 and 'Z' is 25!
	}
	if(ch>=27)	{
		return OK; //plot a space if the char is not recognised!
	}
	static uint8_t count = 0;
	if(count==0)	{
		//top line
		if(char1[ch]&(1<<6))	{
			plotHLine(x, x+sizex, y);
		}
		else	{
			count++;
		}
	}
	if(count==1)	{
		//right upper line
		if(char1[ch]&(1<<5))	{
			plotVLine(x+sizex, y, y+sizey/2);
		}
		else	{
			count++;
		}
	}
	if(count==2)	{
		//right lower line
		if(char1[ch]&(1<<4))	{
			plotVLine(x+sizex, y+sizey/2, y+sizey);
		}
		else	{
			count++;
		}
	}
	if(count==3)	{
		//down line
		if(char1[ch]&(1<<3))	{
			plotHLine(x, x+sizex, y+sizey);
		}
		else	{
			count++;
		}
	}
	if(count==4)	{
		//left lower line
		if(char1[ch]&(1<<2))	{
			plotVLine(x, y+sizey/2, y+sizey);
		}
		else	{
			count++;
		}
	}
	if(count==5)	{
		//left up line
		if(char1[ch]&(1<<1))	{
			plotVLine(x, y, y+sizey/2);
		}
		else	{
			count++;
		}
	}
	if(count==6)	{
		//middle line left
		if(char1[ch]&(1<<0))	{
			plotHLine(x, x+sizex/2, y+sizey/2);
		}
		else	{
			count++;
		}
	}
	if(count==7)	{
		//middle line right
		if(char2[ch]&(1<<6))	{
			plotHLine(x+sizex/2, x+sizex, y+sizey/2);
		}
		else	{
			count++;
		}
	}
	if(count==8)	{
		//upper left diagonal line
		if(char2[ch]&(1<<5))	{
			if(plotLine(x, x+sizex/2, y, y+sizey/2)!=OK)	{
				return PROGRESS;
			};
		}
		else	{
			count++;
		}
	}
	if(count==9)	{
		//upper right diagonal line
		if(char2[ch]&(1<<4))	{
			if(plotLine(x+sizex/2, x+sizex, y+sizey/2, y)!=OK)	{
				return PROGRESS;
			}
		}
		else	{
			count++;
		}
	}
	if(count==10)	{
		//lower left diagonal line
		if(char2[ch]&(1<<3))	{
			if(plotLine(x, x+sizex/2, y+sizey, y+sizey/2)!=OK)	{
				return PROGRESS;
			}
		}
		else	{
			count++;
		}
	}
	if(count==11)	{
		//lower right diagonal line
		if(char2[ch]&(1<<2))	{
			if(plotLine(x+sizex/2, x+sizex, y+sizey/2, y+sizey)!=OK)	{
				return PROGRESS;
			}
		}
		else	{
			count++;
		}
	}
	if(count==12)	{
		//vertical middle line up
		if(char2[ch]&(1<<1))	{
			plotVLine(x+sizex/2, y, y+sizey/2);
		}
		else	{
			count++;
		}
	}
	if(count==13)	{
		//vertical middle line down
		if(char2[ch]&(1<<0))	{
			plotVLine(x+sizex/2, y+sizey/2, y+sizey);
		}
		else	{
			count++;
		}
	}
	if(count==14)	{
		if(ch=='D'-'A'||ch=='B'-'A')	{
			//right upper diagonal line only for D and B
			if(plotLine(x+sizex*3/4, x+sizex, y, y+sizey*1/4)!=OK)	{
				return PROGRESS;
			}
		}
		else	{
			count++;
		}
	}
	if(count==15)	{
		if(ch=='D'-'A'||ch=='B'-'A')	{
			//right lower diagonal line only for D and B
			if(plotLine(x+sizex*3/4, x+sizex, y+sizey, y+sizey*3/4)!=OK)	{
				return PROGRESS;
			}
		}
		else	{
			count++;
		}
	}
	if(count==16)	{
		//right vertical line middle for B
		if(ch=='D'-'A'||ch=='B'-'A')	{
			plotVLine(x+sizex, y+sizey*1/4, y+sizey*3/4);
		}
		else	{
			count++;
		}
	}
	if(count==17)	{
		//right upper middle horizontal line for B and D
		if(ch=='D'-'A'||ch=='B'-'A')	{
			plotHLine(x, x+sizex*3/4, y);
		}
		else	{
			count++;
		}
	}
	if(count==18)	{
		//right lower middle horizontal line for B and D
		if(ch=='D'-'A'||ch=='B'-'A')	{
			plotHLine(x, x+sizex*3/4, y+sizey);
		}
		else	{
			count++;
		}
	}
	if(count==19)	{
		//right lower middle horizontal line for B and D
		if(ch=='V'-'A')	{
			if(plotLine(x, x+sizex/2, y, y+sizey)!=OK)	{
				return PROGRESS;
			}
		}
		else	{
			count++;
		}
	}
	if(count==20)	{
		//right lower middle horizontal line for B and D
		if(ch=='V'-'A')	{
			if(plotLine(x+sizex/2, x+sizex, y+sizey, y)!=OK)	{
				return PROGRESS;
			}
		}
		else	{
			count++;
		}
	}

	count++;
	if(count>20)	{
		count=0;
		return OK;
	}
	else	{
		return PROGRESS;
	}
}

uint8_t plotColon(uint8_t x, uint8_t y)	{
	if(pm!=READY)	{
		return ERR;
	}
	static uint8_t count;
	if(count==0)	{
		plotVLine(x, y+NUM_SIZE_Y, y+NUM_SIZE_Y);
		count++;
		return PROGRESS;
	}
	else if(count==1)	{
		plotVLine(x, y+(NUM_SIZE_Y-NUM_SIZE_Y/2), y+(NUM_SIZE_Y-NUM_SIZE_Y/2));
		count=0;
		return OK;
	}
}

void resetGame()	{
	points1=POINTS_START;
	points2=POINTS_START;
	posx=BALL_INIT_X;
	posy=BALL_INIT_Y;
	TCNT1H=(PRELOAD_TIMER1>>8);
	TCNT1L=PRELOAD_TIMER1;
	countdown_show=1;
	countdown_count=0;
	countdown=COUNTDOWN;
	gamestatus=RUNNING;
	easteregg_c1=0;
	easteregg_c2=0;
	easteregg_c11=0;
	easteregg_c22=0;
}


ISR(TIMER2_OVF_vect)	{
	TCNT2=PRELOAD_TIMER2;
	if(pm==VERT_LINE)	{
		//plot a vertical line:
		//ly1 must be lesser or equal than ly2!
		LDAC_PORT |= (1<<LDAC);
		sendDAC(X, lx1);
		sendDAC(Y, ly1);
		LDAC_PORT &= ~(1<<LDAC);
		if(ly1>=ly2)	{
			pm=READY; //line plotting ready
		}
		ly1++;
	}
	else if(pm==HORIZ_LINE)	{
		//plot a horizontal line:
		//lx1 must be lesser or equal than lx2!
		LDAC_PORT |= (1<<LDAC);
		sendDAC(X, lx1);
		sendDAC(Y, ly1);
		LDAC_PORT &= ~(1<<LDAC);
		if(lx1>=lx2)	{
			pm=READY; //line plotting ready
		}
		lx1++;
	}
	/*else	{
		//ready
		LDAC_PORT |= (1<<LDAC);
		sendDAC(X, 0);
		sendDAC(Y, 0);
		LDAC_PORT &= ~(1<<LDAC);
	}*/


	//plot screen:::
	static 	uint8_t top=0; //toplot
	STATUS_LED_TOGGLE();
	if (pm==READY)	{
#ifdef CHAR_TESTER
		plotChar(ch2, 20,20,20,40);
#else
		if(gamestatus==INIT)	{
			//show welcome screen
			if(top>=0&&top<HEADING_CHARS)	{
				if(plotChar(heading[top], HEADING_X+top*(HEADING_SIZE_X+HEADING_SPACE), HEADING_Y, HEADING_SIZE_X, HEADING_SIZE_Y)==OK)	{
					top++; //otherwise plotCircle must be redrawed when pm is next ready!
				}

			}

			//draw border so that oscilloscope can be adjusted!
			else if(top==HEADING_CHARS)	{
				plotVLine(0,0,20);
				top++;
			}
			else if(top==HEADING_CHARS+1)	{
				plotVLine(255,0,20);
				top++;
			}
			else if(top==HEADING_CHARS+2)	{
				plotHLine(0,20,0);
				top++;
			}
			else if(top==HEADING_CHARS+3)	{
				plotHLine(0,20,255);
				top++;
			}
			else if(top==HEADING_CHARS+4)	{
				plotVLine(0,235,255);
				top++;
			}
			else if(top==HEADING_CHARS+5)	{
				plotVLine(255,235,255);
				top++;
			}
			else if(top==HEADING_CHARS+6)	{
				plotHLine(235,255,0);
				top++;
			}
			else if(top==HEADING_CHARS+7)	{
				plotHLine(235,255,255);
				top++;
			}
			if(top>=HEADING_CHARS+8&&top<HEADING_CHARS+8+SUBHEADING_CHARS)	{
				if(plotChar(subheading[top-HEADING_CHARS-8], SUBHEADING_X+(top-HEADING_CHARS-7)*(SUBHEADING_SIZE_X+SUBHEADING_SPACE), SUBHEADING_Y, SUBHEADING_SIZE_X, SUBHEADING_SIZE_Y)==OK)	{
					top++; //otherwise plotCircle must be redrawed when pm is next ready!
				}
			}

			if(top>HEADING_CHARS+7+SUBHEADING_CHARS)	{
				top=0;
			}
		}

		else if (gamestatus==WON)	{
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//if balls, paddle or number plotting is changed here,
			//change it also at gamestatus == WON!

			if(top>=0&&top<PLAYER_CHARS)	{
				if(plotChar(player_message[top], PLAYER_X+top*(PLAYER_SIZE_X+PLAYER_SPACE), PLAYER_Y, PLAYER_SIZE_X, PLAYER_SIZE_Y)==OK)	{
					top++;
				}
			}
			else if(top==PLAYER_CHARS)	{
				//playerspace once more added so that there is a gap between "player" and the number
				if(plotNumber(playerwon, PLAYER_X+top*(PLAYER_SIZE_X+PLAYER_SPACE)+PLAYER_SPACE*2, PLAYER_Y, PLAYER_SIZE_X, PLAYER_SIZE_Y)==OK)	{
					top++;
				}
			}
			else if(top>=PLAYER_CHARS+1&&top<PLAYER_CHARS+2+WIN_CHARS)	{
				if(plotChar(win_message[top-PLAYER_CHARS-2], WIN_X+(top-PLAYER_CHARS-2)*(WIN_SIZE_X+WIN_SPACE), WIN_Y, WIN_SIZE_X, WIN_SIZE_Y)==OK)	{
					top++;
				}
			}
			else if (top==PLAYER_CHARS+2+WIN_CHARS)	{
				if(points1/10<1)	{
					top++;
				}
				else {
					if(plotNumber(points1/10, NUM11_X, NUM11_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}
			else if (top==PLAYER_CHARS+WIN_CHARS+3)	{
				if(plotNumber(points1%10, NUM12_X, NUM12_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
					top++; //otherwise plotCircle must be redrawed when pm is next ready!
				}
			}
			else if (top==PLAYER_CHARS+WIN_CHARS+4)	{
				if(points2/10<1)	{
					top++;
				}
				else if(points2/10<2)	{
					// a bit more left because 1 is so small
					if(plotNumber(points2/10, NUM21_X, NUM21_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else	{
					//a bit more right than if points2/10 == 1
					if(plotNumber(points2/10, NUM2_X, NUM2_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}
			else if (top==PLAYER_CHARS+WIN_CHARS+5)	{
				if(points2/10<1)	{
					//put second number mor to the left
					if(plotNumber(points2%10, NUM2_X, NUM2_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else if(points2/10<2)	{
					//first digit is 1 -> go only a little bit to the right (not as much as if first digit would be 2 ore more)
					if(plotNumber(points2%10, NUM22_X, NUM22_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else	{
					//a bit more right than if points2/10 == 1
					if(plotNumber(points2%10, NUM222_X, NUM222_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}
			else if (top==PLAYER_CHARS+WIN_CHARS+6)	{
				if(plotColon(COLON_X, COLON_Y)==OK)	{
					top++; //otherwise plotCircle must be redrawed when pm is next ready!
				}
			}
			else if (top==PLAYER_CHARS+WIN_CHARS+7)	{
				plotVLine(OFF_X_L, pl, pl+PAD_SIZE);
				top++;
			}
			else if (top==PLAYER_CHARS+WIN_CHARS+8)	{
				plotVLine(OFF_X_R, pr, pr+PAD_SIZE);
				top++;
			}


			if(top>PLAYER_CHARS+WIN_CHARS+8)	{
				top=0;
			}
		}

		else {
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//if balls, paddle or number plotting is changed here,
			//change it also at gamestatus == WON!

			//show ball and all the things!
			if (top==0)	{
				plotVLine(OFF_X_L, pl, pl+PAD_SIZE);
				top++;
			}
			else if (top==1)	{
				plotVLine(OFF_X_R, pr, pr+PAD_SIZE);
				top++;
			}
			else if (top==2)	{
				static uint8_t ballx=0, bally=0;
				if(plotBall(BALL_SIZE, ballx, bally)==OK)	{
					top++; //otherwise plotCircle must be redrawed when pm is next ready!
					ballx=posx; bally=posy; //so that the position changes not while plotting the ball
					//because then there is a ugly stripe on the screen (when putting in a new ball)
				}
			}
			else if (top==3)	{
					if(points1/10<1)	{
					top++;
				}
				else	{
					if(plotNumber(points1/10, NUM11_X, NUM11_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}
			else if (top==4)	{
				if(plotNumber(points1%10, NUM12_X, NUM12_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
					top++; //otherwise plotCircle must be redrawed when pm is next ready!
				}
			}
			else if (top==5)	{
				if(points2/10<1)	{
					top++;
				}
				else if(points2/10<2)	{
					// a bit more left because 1 is so small
					if(plotNumber(points2/10, NUM21_X, NUM21_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else	{
					//a bit more right than if points2/10 == 1
					if(plotNumber(points2/10, NUM2_X, NUM2_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}
			else if (top==6)	{
				if(points2/10<1)	{
					//put second number mor to the left
					if(plotNumber(points2%10, NUM2_X, NUM2_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else if(points2/10<2)	{
					//first digit is 1 -> go only a little bit to the right (not as much as if first digit would be 2 ore more)
					if(plotNumber(points2%10, NUM22_X, NUM22_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
				else	{
					//a bit more right than if points2/10 == 1
					if(plotNumber(points2%10, NUM222_X, NUM222_Y, NUM_SIZE_X, NUM_SIZE_Y)==OK)	{
						top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}
			else if (top==7)	{
				if(plotColon(COLON_X, COLON_Y)==OK)	{
					top++; //otherwise plotCircle must be redrawed when pm is next ready!
				}
			}
			else if(gamestatus==PAUSE)	{
				if(top>=8&&top<PAUSE_CHARS+8)	{
					if(plotChar(pause_message[top-8], PAUSE_X+(top-8)*(PAUSE_SIZE_X+PAUSE_SPACE), PAUSE_Y, PAUSE_SIZE_X, PAUSE_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}
			else {
				if(top==8)	{
					if(plotNumber(countdown, COUNTDOWN_X, COUNTDOWN_Y, COUNTDOWN_SIZE_X, COUNTDOWN_SIZE_Y)==OK)	{
							top++; //otherwise plotCircle must be redrawed when pm is next ready!
					}
				}
			}




			if(gamestatus == PAUSE)	{
				if(top>PAUSE_CHARS+7)	{
							top=0;
				}
			}
			else if	(countdown_show==1)	{
				if(top>8)	{
						top=0;
				}
			}
			else	{
				if(top>7)	{
					top=0;
				}
			}

		}
#endif


	}

}

ISR(TIMER1_OVF_vect)	{
	TCNT1H=(PRELOAD_TIMER1>>8);
	TCNT1L=PRELOAD_TIMER1;

	if(beep_c>0)	{
		beep_c--;
	}
	else	{
		BEEP_OFF();
	}

	#ifdef ENABLE_LED_MULTIPLEX
			//LED matrix multiplexing
			if(gamestatus == INIT)	{
				//dont multiplex matrix
				LED_MUX_PORT &= ~((1<<5)|(1<<4)|(1<<3)|(1<<2));
			}
			else	{
				static uint8_t matrix_count = 0;

				LED_MUX_PORT &= ~((1<<5)|(1<<4)|(1<<3)|(1<<2)); //switch off all LED-Matrixes

				if(matrix_count==0)	{
					//second digit of second number
					LED_MATRIX_PORT = matrix[points2%10];
				}
				else if(matrix_count==1)	{
					//first digit of second number
					if(points2/10 < 1)	{
						LED_MATRIX_PORT = 0x00; // show  not '0' but all leds dark!
					}
					else	{
						LED_MATRIX_PORT = matrix[points2/10];
					}
				}
				else if(matrix_count==2)	{
					//second digit of first number
					LED_MATRIX_PORT = matrix[points1%10];
				}
				else if(matrix_count==3)	{
					//first digit of first number
					if(points1/10 < 1)	{
						LED_MATRIX_PORT = 0x00; // show  not '0' but all leds dark!
					}
					else	{
						LED_MATRIX_PORT = matrix[points1/10];
					}
				}

				LED_MUX_PORT |= (1<<matrix_count+2);

				matrix_count++;
				if(matrix_count>3)	{
					matrix_count=0;
				}
			}
#endif


}

ISR(TIMER0_OVF_vect)	{
	TCNT0=PRELOAD_TIMER0;

	if(gamestatus==RUNNING)	{
		//don't increment in pause!!
		countdown_count++;
	}
	if(countdown_count>COUNTDOWN_COUNT_MAX)	{
		//executed with frequency of about 1Hz!
#ifdef CHAR_TESTER
		ch2++;
#endif
		if(gamestatus==RUNNING)	{
			if(countdown>0)	{
				countdown--;
			}
			if(countdown==0)	{
				countdown_show=0;
			}
		}
		countdown_count=0;
	}

	if(gamestatus==RUNNING)	{

	//read ADC values from Poti (change poti each time!
		if(adc_c==0)	{
			pl_old=pl;
			pl=ADCH;
			pl=(pl-POTI_OFF)*POTI_MULT;
#ifdef  ENABLE_EASTEREGG1
			if((easteregg_c1==1||easteregg_c11==1)&&posx<OFF_X_L+110)	{
				//set paddle to position of ball (3/4 so that the ball does not always hit the middle of the paddle)
				//pl = posy+incy-PAD_SIZE*3/4; // so that it is also incremented to next position

				int16_t delta = pl-PAD_SIZE/2 - posy;
				delta = delta * (-OFF_X_L + posx)/110;
				pl = posy + delta;
			}
#endif
			if(pl<0)	{
				pl=0;
			}
			if(pl>=(255-PAD_SIZE))	{
				pl=255-PAD_SIZE;
			}

			ADMUX|=(1<<MUX0);//switch to ADC1 (poti for right)
			//adc_c=1;
		}
		if(adc_c==1)	{
			pr_old=pr;
			pr=ADCH;
			pr=(pr-POTI_OFF)*POTI_MULT;
#ifdef  ENABLE_EASTEREGG1
			if((easteregg_c2==1||easteregg_c22==1)&&posx>OFF_X_R-100)	{
				//set paddle to position of ball (3/4 so that the ball does not always hit the middle of the paddle)
				//pr = posy+incy-PAD_SIZE*3/4; // so that it is also incremented to next position
				int16_t delta = pr-PAD_SIZE/2 - posy;
				delta = delta * (OFF_X_R - (posx+10))/110;
				pr = posy + delta;
			}
#endif
			if(pr<0)	{
				pr=0;
			}
			if(pr>=(255-PAD_SIZE))	{
				pr=255-PAD_SIZE;
			}

			ADMUX&=~(1<<MUX0);//switch to ADC0 (poti for left)
			//adc_c=0;
		}
		adc_c^=0x01;

		int8_t spl=pl_old-pl; //speed paddle left
		int8_t spr=pr_old-pr; //speed paddle right




		//change ball's position
			if(countdown==0)	{
				posx=posx+incx;
				posy=posy+incy;
			}


		//check if ball has bumped

			//bumped into right paddle
			//== so that only one position is checked (after it it is impossible to change the ball's direction)
			//ball is out then!
			//if(posx>=OFF_X_R-BALL_SIZE && posx<=OFF_X_R)	{
				//if(pr
			//}
			if(posx==OFF_X_R-BALL_SIZE)	{
				if(posy<(pr+PAD_SIZE) && (posy+BALL_SIZE)>pr)	{
					incy-=spr/3.0;//3.0 for floating conversion (does it work?)  //change speed of paddle in y-direction depending on speed of paddle
					if(incy==0)	{
					//so that y-direction is not 0 -> set to 1 or -1
						incy=-spr/abs(spr);
					}
					incx*=-INCX_STD;
					BEEP_ON();
					beep_c=BEEP_TIME;
				}
			}

			//bumped into left paddle
			//== so that only one position is checked (after it it is impossible to change the ball's direction)
			//ball is out then!
			else if(posx==OFF_X_L)	{
				if(posy<(pl+PAD_SIZE) && (posy+BALL_SIZE)>pl)	{
					incy-=spl/3.0;//3.0 for floating conversion (does it work?) //change speed in y-direction depending on speed of paddle
					if(incy==0)	{
					//so that y-direction is not 0 -> set to 1 or -1
						incy=-spr/abs(spr);
					}
					incx*=-INCX_STD;
					BEEP_ON();
					beep_c=BEEP_TIME;
				}
			}

			//ball run outside
			else if(posx<=0 || posx>=(255-BALL_SIZE))	{
				//player get points:
				if(posx<=0)	{
					//ball left out -> right player gets point
					points2+=1;
					if(points2>=POINTS_TO_WIN)	{
						//player2 has won!!!
						playerwon=2;
						gamestatus=WON;
					}
				}
				else	{
					//ball right out -> left player gets point
					points1+=1;
					if(points1>=POINTS_TO_WIN)	{
						//player1 has won!!!!!
						playerwon=1;
						gamestatus=WON;
					}
				}
				BEEP_ON();
				beep_c=2*BEEP_TIME;//longer beep
				//new ball
				TCNT1H=(PRELOAD_TIMER1>>8);
				TCNT1L=PRELOAD_TIMER1;
				//reset timer -> after 2 seconds the timer will set ballstart to 1
				//here countdown is only set to 1, just wait one second but don't show a countdown!
				countdown_count=0;
				countdown=1;

				c=pr;
				t=0; // so that ball is plotted newly (position has changed dramatically -> you
				//would see a unaesthetic line for a short time!

				posx=BALL_INIT_X;
				posy=BALL_INIT_Y;
				//change dir of incx
				incx*=-1;

				//change dir of incy and set it to 1
				//set it random with use of the adc values
				incy=2*(((int16_t)pr*(int16_t)pl)%2)-1;

			}

			//bumped into bottom or top
			else if(posy>=OFF_Y_U-BALL_SIZE||posy<=OFF_Y_D)	{
				incy*=-1;
				BEEP_ON();
				beep_c=2*BEEP_TIME;//longer beep
			}

			//so that incy doesn't get too big!
			if(incy>INCY_MAX)	{
				incy=INCY_MAX;
			}
			if(incy<-INCY_MAX)	{
				incy=-INCY_MAX;
			}

			if(posy>=OFF_Y_U-BALL_SIZE)	{
				posy=OFF_Y_U-BALL_SIZE;
			}
			else if(posy<=OFF_Y_D)	{
				posy=OFF_Y_D;
			}
			if(posx>=255-BALL_SIZE)	{
				posx=255-BALL_SIZE;
			}
			else if(posx<=0)	{
				posx=0;
			}
	}
}




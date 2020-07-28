#define  F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <asf.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <math.h>


#define FOSC 16000000// Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
#define ilosc_zapamietanych 300
#define rezerwa_zapamietanych 200
#define PORT_ECHO PORTC
#define PIN_ECHO PINC
#define PORT_TRIG PORTB
#define kierunek_przod 1
#define kierunek_tyl 0
#define dlugoscMiejscaProstopadle 50
#define dlugocsMiejscaRownolegle 100
#define glebokoscMiejscaProstopadle 40
#define glebokoscMiejscaRownolegle 40
#define TRUE 1
#define FALSE 0
#define pi 3.1415
#define bezpieczna_odleglosc 5
#define szerokoszRobota 36
#define dlugoscRobota 36

uint8_t kierunek_jazdy= kierunek_przod; 
uint8_t czujnik_przod[ilosc_zapamietanych];
uint8_t czujnik_lewa[ilosc_zapamietanych];
uint8_t czujnik_prawa[ilosc_zapamietanych];
uint8_t czujnik_tyl[ilosc_zapamietanych];
uint8_t pwm_lewa[ilosc_zapamietanych];
uint8_t pwm_prawa[ilosc_zapamietanych];
uint8_t kierunek[ilosc_zapamietanych];

volatile int iloscIteracji = ilosc_zapamietanych - rezerwa_zapamietanych;
volatile uint16_t czas;
volatile int i=0;
uint16_t dane_z_ADC;
unsigned char dataFormBT;
unsigned char lastDataFormBT;

int startLewa = 0;
int stopLewa = 0;
uint8_t zmianaLewa = FALSE;
int startPrawa = 0;
int stopPrawa = 0;
uint8_t zmianaPrawa = FALSE;

void prg_init(void);
void init_pwmA(void);
void init_timer0(void);
void stop_timer0(void);
void init_timer1(void);
void stop_timer1(void);
void initUART(unsigned int);
void USART_Transmit( unsigned char data );
void initADC(void);
unsigned char ReadADCL(void);
unsigned char ReadADCH(void);
unsigned char USART_Receive(void);
unsigned char mierz(uint8_t nr_czujnika);
void srednia (uint8_t* tablica, int start, int stop);
int rozmiar (uint8_t* tablica, int start, int stop);
void obliczenia(uint8_t* tab_pomiarow, uint8_t* tab_predkosc, uint8_t nr_czujnika , int* start, int* stop, uint8_t* zmiana);
void parkujProstopadle(uint8_t nr_czujnika);
void parkujRownolegle(uint8_t nr_czujnika);



int main(void)
{
	sei();
	board_init();
	prg_init();
	init_pwmA();
	init_timer1();
	initUART(MYUBRR);
	init_timer0();
	//initADC();
	
	while(1)
	{
		
	}
}

void prg_init(void)
{
	//kierunek przód
	DDRA=0xff;
	PORTA&=~(1<<4);
	PORTA&=~(1<<6);
	PORTA=(1<<3)|(1<<5);
	
	//Ledy wyjscie
	DDRD=0xff;


	DDRG|=(1<<3);
	PORTG|=(1<<3);
	DDRE=0xff;
	PORTE=(1<<5)|(1<<6);

	//trigery czujnikow jako wyjscie
	DDRB = 0xff;
	//echo czujnikow jako wejscie z rezystorami do vcc
	DDRC = 0x00;
	PORT_ECHO = 0xff;
	
	//ustawienie braku przerwañ timerów, ka¿dy bedzie odblokwywal swoje bity
	TIMSK=0x00;
}

void initADC() // init rejestru przetwornika A/C
{
	ADMUX=(1<<REFS0)| 0b01001;//ADC1
	//ADMUX = (1<<REFS0)|(1<<MUX3)|(1<<MUX2)|(1<<MUX0);//ADC3
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	
}

unsigned char ReadADCL()
{	
	ADCSRA|=(1<<ADSC); // start conversison
	while(ADCSRA & (1<<ADSC));
	return ADCL;
}
unsigned char ReadADCH()
{
	ADCSRA|=(1<<ADSC); // start conversison
	while(ADCSRA & (1<<ADSC));
	return ADCH;
}
unsigned char mierz(uint8_t nr_czujnika) // pomiar odleg³oœci
{
	czas=0;
	PORT_TRIG=(1<<nr_czujnika);
	_delay_us(10);
	PORT_TRIG&=~(1<<nr_czujnika);
	while(!(PIN_ECHO & (1<<nr_czujnika)));
	init_timer0();
	while((PIN_ECHO & (1<<nr_czujnika))){}
	stop_timer0();
	czas= czas + TCNT0/2;
	uint16_t odleglosc = (czas)/58;
	return (unsigned char)odleglosc;
}
void initUART(unsigned int ubrr)  // init USART do obs³ugi BlueTooth
{
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;

	UCSR0B=(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
	UCSR0C=(1<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);

}

void USART_Transmit( unsigned char data ) //Transmisja 8-bitów przez BlueTooth
{
		while ( !( UCSR0A & (1<<UDRE)) );
		UDR0 = data;
}
 unsigned char  USART_Receive()	//Odbiór 8-bitów przez BlueTooth
 {
	while( !(UCSR0A & (1<<RXC)));
	return UDR0;
 }



ISR(USART0_RX_vect) //Przerwanie w razie odebrania danych przez BlueTooth
{		
		
		
		dataFormBT = UDR0;
		
		if(lastDataFormBT == 3)
		{
			OCR3B = dataFormBT;
		}
		else if (lastDataFormBT == 4)
		{
			OCR3A = dataFormBT;
		}

		if(dataFormBT==15) //stop
		{	
				OCR3A=0;
				OCR3B=0;
		}
		else if(dataFormBT==14)
		{
				OCR3B+=50;
				OCR3A+=50;
		}
		else if(dataFormBT==13)
		{
				OCR3B=255;
				OCR3A=255;
		}
		else if(dataFormBT==50) //kierunek przód
		{
				PORTA&=~(1<<4);
				PORTA&=~(1<<6);
				PORTA=(1<<3)|(1<<5);
				kierunek_jazdy=kierunek_przod;									
		}
		else if(dataFormBT == 51)
		{
			PORTA&=~(1<<4);
			PORTA&=~(1<<5);
			PORTA=(1<<3);
			PORTA|=(1<<6);
		}
		else if(dataFormBT==60) //kierunek tyl
		{
				PORTA&=~(1<<3);
				PORTA&=~(1<<5);
				PORTA=(1<<4)|(1<<6);
				kierunek_jazdy=kierunek_tyl;	
		}
		else if(dataFormBT==17) //stop zapisu do tablicy i pomiary
		{
			stop_timer1();
		}
		
		else if(dataFormBT==18) //wznow zapisu do tablicy i pomiary
		{
		    i=0;
			init_timer1();
		}
	//USART_Transmit(mierz(0));
	//USART_Transmit(mierz(1));
	/*int sumaL=0;
	int sumaH=0;
	for(int i =0 ;i<40; i++)
	{
	_delay_ms(10);
	sumaL += ReadADC();
	_delay_ms(10);
	sumaH += ReadADCH();
	}
	USART_Transmit(OCR3B);
	USART_Transmit((unsigned char)(sumaL/40));
	USART_Transmit((unsigned char)(sumaH/40));*/
	lastDataFormBT = dataFormBT;
		
}

void init_timer0() //init timera do mierzenia czasów stanu wysokiego na pinie echo czujnika f=2Mhz
{
	TCCR0=(1<<CS01);
	TIMSK|=1<<TOIE0;
	TCNT0=0;
}

void stop_timer0()
{
TIMSK&=~(1<<TOIE0);
}

 void init_timer1() //init timera który wyzwala pomiary f = 3,85Hz, czyli pomiary co 0,26s
 {
	 TCCR1B=(1<<CS10)|(1<<CS11);
	 //TCCR1B=(1<<CS12)|(1<<CS10);
	 TIMSK|=(1<<TOIE1);
 }

 void stop_timer1()
 {
	TIMSK&=~(1<<TOIE1);
 }

void init_pwmA() //init timera odpowiedzialnego za sterowanie PWM
{
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1)|(1<<COM3A0)|(1<<COM3B0)|(1<<WGM30);
	TCCR3B |= (1<<CS31)|(1<<WGM32);
	OCR3B=0;
	OCR3A=0;
	//f= 7812Hz, a najlepiej 10kHz
}

ISR(TIMER0_OVF_vect) //obsluga przerwania timera 0
{
czas = czas +128;
}

ISR(TIMER1_OVF_vect)  //obsluga przerwania timera 1 g³ówny blok obliczeniowy
{
	sei();
	pwm_lewa[i]=OCR3B;
	pwm_prawa[i]=OCR3A;
	kierunek[i]= kierunek_jazdy;

	
	uint8_t pomiarL = mierz(0);
	czujnik_lewa[i]=pomiarL;
	USART_Transmit(czujnik_lewa[i]);
	
	uint8_t pomiarP = mierz(1);
	czujnik_prawa[i]=pomiarP;
	USART_Transmit(czujnik_prawa[i]);
	
	//ATOMIC_BLOCK(ATOMIC_FORCEON)
	//{
	obliczenia(czujnik_lewa, pwm_lewa, 0, &startLewa, &stopLewa , &zmianaLewa);
	obliczenia(czujnik_prawa, pwm_prawa, 1 , &startPrawa ,  &stopPrawa , &zmianaPrawa);
	//};

	i++;
	if(i==ilosc_zapamietanych-1) 
	{i=0;
	startLewa = 0;
	stopLewa = 0;
	zmianaLewa=FALSE;
	startPrawa = 0;
	startPrawa = 0;
	zmianaPrawa=FALSE;
	}
}

void obliczenia(uint8_t* tab_pomiarow, uint8_t* tab_predkosc, uint8_t nr_czujnika, int* start, int* stop, uint8_t* zmiana)
{
	
	if (*zmiana == TRUE && i>1)
	{
		if (abs(tab_pomiarow[i - 2] - tab_pomiarow[i]) < 10)

		{
			*stop = i;
			tab_pomiarow[i - 1] = tab_pomiarow[i - 2];
		}

		else
		{
			if ((abs(tab_pomiarow[i] - tab_pomiarow[i - 1]) < 10))
			{
				*stop = i - 2;
				srednia(tab_pomiarow, *start, *stop);
				*start = i-1;
				
			}
			else
			{
				*stop = i - 2;
				srednia(tab_pomiarow, *start, *stop);
				tab_pomiarow[i - 1] = tab_pomiarow[i];
				*start = i - 1;
			}

			if (tab_pomiarow[i - 2] - tab_pomiarow[i - 1] > glebokoscMiejscaRownolegle)
			{
				
				for (int j = i - 2; j > 0; j--)
				{
					
					if (tab_pomiarow[j] - tab_pomiarow[j - 1] > glebokoscMiejscaRownolegle && rozmiar(tab_predkosc, j, i - 2) > dlugocsMiejscaRownolegle)
					{
						parkujRownolegle(nr_czujnika);
						return;
					}
					else if (tab_pomiarow[j] != tab_pomiarow[j - 1])
					{
						break;
					}
				}
			}

			if (tab_pomiarow[i - 2] - tab_pomiarow[i - 1] > glebokoscMiejscaProstopadle)
			{
				for (int j = i - 2; j > 0; j--)
				{
					if (tab_pomiarow[j] - tab_pomiarow[j - 1] > glebokoscMiejscaProstopadle && rozmiar(tab_predkosc, j, i-2) > dlugoscMiejscaProstopadle)
					{

						parkujProstopadle(nr_czujnika);
						return;
					}
					else if (tab_pomiarow[j] != tab_pomiarow[j - 1])
					{
						break;
					}
				}
			}
			else if (tab_pomiarow[i - 2] - tab_pomiarow[i - 1] > (glebokoscMiejscaProstopadle * (-1)) || tab_pomiarow[i - 2] - tab_pomiarow[i - 1] > (glebokoscMiejscaRownolegle * (-1)))
			{
				//napotkano na poczatek miejsca parkingowego zwiêksz rozmiar tablicy pomiarów
				iloscIteracji = ilosc_zapamietanych;

			}
		}
		
		
		*zmiana = FALSE;
	}
	else if (abs(tab_pomiarow[i] - tab_pomiarow[i - 1]) < 10)
	{
		*stop = i + 1;
		PORTD^=(1<<7);
		
	}


	else
	{
		*zmiana = TRUE;
	}
}

void srednia (uint8_t* tablica, int start, int stop)
{
	int suma = 0;
	for (int i = start; i <= stop; i++)
	{
		suma = tablica[i] + suma;
	}
	for (int i = start; i <= stop; i++)
	{
		tablica[i] = (suma / (stop - start + 1));
	}
}

int rozmiar(uint8_t* tablica, int start, int stop)
 {
	 double suma = 0;
	 for (int i = start; i <= stop; i++)
	 {
		 suma = tablica[i] + suma;
	 }
	 double predkosc = (suma / (stop - start +1 ));
	 predkosc = predkosc * 0.19 - 20;
	 return (int)(predkosc * (0.26 * (stop - start)));
 }


 //Do dyspozycji masz tablice predkoci pwm_lewa i pwm_prawa, tablice pomiarów- pomiary pomiary_lewa i pomiary_prawa gdzie i to indeks ostatniego pomiaru przed zatrzymaniem
 //samochód parkuje jesli spe³nia zaleznosci
 //dlugoscMiejscaProstopadle >50
 //dlugocsMiejscaRownolegle >80
 //lub
 //glebokoscMiejscaProstopadle > 40
 //glebokoscMiejscaRownolegle > 40
 //nr_czujnika na wejœciu pokazuje, który czujnik wykryl ze mozna zaparkowaæ 0 =lewy, 1= prawy
 //OCR3A - rejestr prawego ko³a
 //OCR3B - rejestr lewego ko³a


 void parkujProstopadle(uint8_t nr_czujnika)
 {/*
 stop_timer1();
 if(nr_czujnika==1)
 PORTA&=~(1<<3);
 PORTA&=~(1<<5);
 PORTA=(1<<4)|(1<<6);
 while(mierz(1)<czujnik_prawa[i-2]-5)
 {
	OCR3A=200;
	OCR3B=200;
	_delay_ms(90);
 }
 _delay_ms(700);
 
 PORTA&=~(1<<3);
 PORTA&=~(1<<6);
 PORTA=(1<<4);

	PORTA|=(1<<5);
	OCR3A=220;
	OCR3B=220;
	
	_delay_ms(2700);
 
 OCR3A=0;
 OCR3B=0;
 _delay_ms(500);
 PORTA&=~(1<<3);
 PORTA&=~(1<<5);
 PORTA=(1<<4)|(1<<6);
 OCR3A=200;
 OCR3B=200;

  while(mierz(2)>20)
  {
  USART_Transmit(mierz((2)));
  _delay_ms(10);
  }
   OCR3A=0;
   OCR3B=0;
   return;*/
 
 if(nr_czujnika == 0) //lewa
	{
	//zatrzymaj_pomiary
	stop_timer1();
	//zatrzymaj robota
	OCR3A=0;
	OCR3B=0;
	//zapal led sygnalizujacy ze znaleziono miejsce
	PORTD&=~(1<<7);
	_delay_ms(50);
	 USART_Transmit(254);
	 _delay_ms(50);
	 dataFormBT = 0;
	 /*while(dataFormBT != 255 && dataFormBT != 254)
	 {
		_delay_ms(10);
		PORTD^=(1<<7);
		 USART_Transmit(dataFormBT);
	 }
	 if(dataFormBT == 254) return;
	 */
	 PORTA&=~(1<<4);
	 PORTA&=~(1<<6);
	 PORTA=(1<<3)|(1<<5);
	 kierunek_jazdy=kierunek_przod;

	 OCR3A=200;
	 OCR3B=200;

	 _delay_ms(400);
	 
	 OCR3A=0;
	 OCR3B=0;
	
	PORTA&=~(1<<3);
	PORTA&=~(1<<5);
	PORTA=(1<<4)|(1<<6);
	kierunek_jazdy=kierunek_tyl;

	_delay_ms(500);

	OCR3A=70;
	OCR3B=250;

	_delay_ms(3500);

	OCR3A=0;
	OCR3B=0;

	PORTA&=~(1<<4);
	PORTA&=~(1<<6);
	PORTA=(1<<3)|(1<<5);
	kierunek_jazdy=kierunek_przod;

	_delay_ms(1000);

	OCR3A=250;
	OCR3B=70;

	_delay_ms(3500);

	OCR3A=0;
	OCR3B=0;

	_delay_ms(1000);

	while(mierz(3) > 10)
	{

	OCR3A=200;
	OCR3B=200;
	_delay_ms(10);

	}

	OCR3A=0;
	OCR3B=0;
	}

	if(nr_czujnika == 1) //prawa
	{
		//zatrzymaj_pomiary
		stop_timer1();
		//zatrzymaj robota
		OCR3A=0;
		OCR3B=0;
		//zapal led sygnalizujacy ze znaleziono miejsce
		PORTD&=~(1<<7);
		_delay_ms(50);
		USART_Transmit(254);
		_delay_ms(50);
		dataFormBT = 0;
		/*while(dataFormBT != 255 && dataFormBT != 254);
		{
			_delay_ms(10);
			PORTD^=(1<<7);
			USART_Transmit(dataFormBT);
		}
		if(dataFormBT == 254) return;
		*/
		PORTA&=~(1<<4);
		PORTA&=~(1<<6);
		PORTA=(1<<3)|(1<<5);
		kierunek_jazdy=kierunek_przod;

		OCR3A=200;
		OCR3B=200;

		_delay_ms(400);
		
		OCR3A=0;
		OCR3B=0;
		
		PORTA&=~(1<<3);
		PORTA&=~(1<<5);
		PORTA=(1<<4)|(1<<6);
		kierunek_jazdy=kierunek_tyl;

		_delay_ms(500);

		OCR3A=250;
		OCR3B=70;

		_delay_ms(3500);

		OCR3A=0;
		OCR3B=0;

		PORTA&=~(1<<4);
		PORTA&=~(1<<6);
		PORTA=(1<<3)|(1<<5);
		kierunek_jazdy=kierunek_przod;

		_delay_ms(1000);

		OCR3A=70;
		OCR3B=250;

		_delay_ms(3500);

		OCR3A=0;
		OCR3B=0;

		_delay_ms(1000);

		while(mierz(3) > 10)
		{

			OCR3A=200;
			OCR3B=200;
			_delay_ms(10);

		}

		OCR3A=0;
		OCR3B=0;
	}


	/*_delay_ms(1000);
	//dataFormBT = UDR0;
	PORTA&=~(1<<3);
	PORTA&=~(1<<5);
	PORTA=(1<<4)|(1<<6);
	kierunek_jazdy=kierunek_tyl;

	int Z = 8;
	int b = 150; //roztaw kol
	int promien_kol = 6; //promien kol
	int dt = 1;
	int R = 40;

	double x;
	double y;
	double fi;
	double xn_1 = 0;
	double yn_1 = 0;
	int sgn;
	double fin_1 = 0;
	

	for (int n = 0; n < R / 2 * (dt * Z)  ; n ++)
	{
		x = Z * n * dt;
		y = sqrt(pow(R, 2) - pow(n * Z - 40, 2));
		double vdx = (x - xn_1) / dt;
		double vdy = (y - yn_1) / dt;
		double v = (sqrt(pow(vdx, 2)+ pow(vdy, 2)));

		if (v > 0)
		{
			sgn = 1;
		}
		else if (v < 0)
		{
			sgn = -1;
		}
		else
		{
			sgn = 0;
		}
		

		fi = atan2(sgn * vdy, sgn * vdx);
		double wCalkowita = (fi - fin_1)/dt;
		double wp = (v + (b / 2) * wCalkowita) / promien_kol;
		double wl = (v - (b / 2) * wCalkowita) / promien_kol;
		double wObrNaMinp = (wp * 30) / pi;
		double wObrNaMinl = (wl * 30) / pi;
		double PWMp = (wObrNaMinp + 35) / 0.34;
		double PWMl = (wObrNaMinl + 35) / 0.34;
		//Console.WriteLine("Czas: "+ n*dt + ";  PWM prawy: " + Math.Round(PWMp,0) +";  PWM lewy: " + Math.Round(PWMl,0)+ ";  Pozycja X: "+ x );
		PWMp = (int)round(PWMp + 0.5);
		PWMl = (int)round(PWMl + 0.5);

		uint8_t pl = (uint8_t)PWMl;
		uint8_t pp = (uint8_t)PWMp;

		OCR3A=pl;
		OCR3B=pp;
		
		USART_Transmit(pl);
		USART_Transmit(pp);

		if(n > 1)
		{
			_delay_ms(950);
		}
		
		xn_1 = x;
		yn_1 = y;
		fin_1 = fi;
		
	}

	OCR3A=0;
	OCR3B=0;
	*/
 
 }

 void parkujRownolegle(uint8_t nr_czujnika)
 {
	if(nr_czujnika == 0) //lewa
	{
	//zatrzymaj_pomiary
	stop_timer1();
	//zatrzymaj robota
	OCR3A=0;
	OCR3B=0;
	//zapal led sygnalizujacy ze znaleziono miejsce
	 PORTD&=~(1<<6);
	 _delay_ms(50);
	 USART_Transmit(255);
	 _delay_ms(50);
	 dataFormBT = 0;
	 while(dataFormBT != 255 && dataFormBT != 254)
	 {
	 _delay_ms(10);
	 }
	 if(dataFormBT == 254) return;
	 

	 
	_delay_ms(1000);
	//dataFormBT = UDR0;
	PORTA&=~(1<<3);
	PORTA&=~(1<<5);
	PORTA=(1<<4)|(1<<6);
	kierunek_jazdy=kierunek_tyl;
	 uint8_t po_zatrzymaniu_lewa = czujnik_lewa[i];
	 USART_Transmit(po_zatrzymaniu_lewa);
	 uint16_t A = po_zatrzymaniu_lewa + szerokoszRobota + bezpieczna_odleglosc;
	 uint16_t R = dlugocsMiejscaRownolegle;
	 
	 _delay_ms(1000);

	 OCR3A=200;
	 OCR3B=200;

	 _delay_ms(400);	 

	 int Z = 8;
	 int b = 475; //roztaw kol
	 int promien_kol = 10; //promien kol
	 int dt = 1;

	 double x;
	 double y;
	 double fi;
	 double xn_1 = Z * -1 * dt;
	 double yn_1 = (A / 2) * cos((2 * pi * -1 * Z) / (2 * R)) + A / 2;
	 x = Z * 0 * dt;
	 y = (A / 2) * cos((2 * pi * 0 * Z) / (2 * R)) + A / 2;
	 double vdx = (x - xn_1) / dt;
	 double vdy = (y - yn_1) / dt;
	 double v = (sqrt(pow(vdx, 2)+ pow(vdy, 2)));
	 int sgn;
	 if (v > 0)
	 {
		 sgn = 1;
	 }
	 else if (v < 0)
	 {
		 sgn = -1;
	 }
	 else
	 {
		 sgn = 0;
	 }
	 double fin_1 = atan2(sgn * vdy, sgn * vdx);
	  
	 for (int n = 0; n <= R / (dt * Z); n++)
	 {
		 x = Z * n * dt;
		 y = (A / 2) * cos((2 * pi * n * Z) / (2 * R)) + A / 2;
		 double vdx = (x - xn_1) / dt;
		 double vdy = (y - yn_1) / dt;
		 double v = (sqrt(pow(vdx, 2)+ pow(vdy, 2)));
		 int sgn;

		 if (v > 0)
			{
			 sgn = 1;
			}
		 else if (v < 0)
			{
			 sgn = -1;
			}
		 else
			{
			 sgn = 0;
			}
	

		 fi = atan2(sgn * vdy, sgn * vdx);
		 double wCalkowita = (fi - fin_1)/dt;
		 double wp = (v + (b / 2) * wCalkowita) / promien_kol;
		 double wl = (v - (b / 2) * wCalkowita) / promien_kol;
		 double wObrNaMinp = (wp * 30) / pi;
		 double wObrNaMinl = (wl * 30) / pi;
		 double PWMp = (wObrNaMinp + 35) / 0.34;
		 double PWMl = (wObrNaMinl + 35) / 0.34;
		 //Console.WriteLine("Czas: "+ n*dt + ";  PWM prawy: " + Math.Round(PWMp,0) +";  PWM lewy: " + Math.Round(PWMl,0)+ ";  Pozycja X: "+ x );
		 PWMp = (int)round(PWMp + 0.5);
		 PWMl = (int)round(PWMl + 0.5);

		 uint8_t pl = (uint8_t)PWMl;
		 uint8_t pp = (uint8_t)PWMp;

		 OCR3A=pl;
		 OCR3B=pp;
		 
		 USART_Transmit(pl);
		 USART_Transmit(pp);

		 if(n > 1)
		 {
			_delay_ms(1000);
		 }
		
		 xn_1 = x;
		 yn_1 = y;
		 fin_1 = fi;
		 
	}

	OCR3A=0;
	OCR3B=0;
	}

	else if(nr_czujnika == 1) //prawa
	{
	//zatrzymaj_pomiary
	stop_timer1();
	//zatrzymaj robota
	OCR3A=0;
	OCR3B=0;
	//zapal led sygnalizujacy ze znaleziono miejsce
	PORTD&=~(1<<6);
	_delay_ms(50);
	USART_Transmit(255);
	_delay_ms(50);
	dataFormBT = 0;
	while(dataFormBT != 255 && dataFormBT != 254)
	{
		_delay_ms(10);
	}
	if(dataFormBT == 254) return;
	

	
	_delay_ms(1000);
	//dataFormBT = UDR0;
	PORTA&=~(1<<3);
	PORTA&=~(1<<5);
	PORTA=(1<<4)|(1<<6);
	kierunek_jazdy=kierunek_tyl;
	uint8_t po_zatrzymaniu_prawa = czujnik_prawa[i];
	USART_Transmit(po_zatrzymaniu_prawa);
	uint16_t A = po_zatrzymaniu_prawa + szerokoszRobota + bezpieczna_odleglosc;
	uint16_t R = dlugocsMiejscaRownolegle;
	
	_delay_ms(1000);

	OCR3A=200;
	OCR3B=200;

	_delay_ms(400);
	

	int Z = 8;
	int b = 475; //roztaw kol
	int promien_kol = 10; //promien kol
	int dt = 1;

	double x;
	double y;
	double fi;
	double xn_1 = Z * -1 * dt;
	double yn_1 = (A / 2) * cos((2 * pi * -1 * Z) / (2 * R)) + A / 2;
	x = Z * 0 * dt;
	y = (A / 2) * cos((2 * pi * 0 * Z) / (2 * R)) + A / 2;
	double vdx = (x - xn_1) / dt;
	double vdy = (y - yn_1) / dt;
	double v = (sqrt(pow(vdx, 2)+ pow(vdy, 2)));
	int sgn;
	if (v > 0)
	{
		sgn = 1;
	}
	else if (v < 0)
	{
		sgn = -1;
	}
	else
	{
		sgn = 0;
	}
	double fin_1 = atan2(sgn * vdy, sgn * vdx);
	
	for (int n = 0; n <= R / (dt * Z); n++)
	{
		x = Z * n * dt;
		y = (A / 2) * cos((2 * pi * n * Z) / (2 * R)) + A / 2;
		double vdx = (x - xn_1) / dt;
		double vdy = (y - yn_1) / dt;
		double v = (sqrt(pow(vdx, 2)+ pow(vdy, 2)));
		int sgn;

		if (v > 0)
		{
			sgn = 1;
		}
		else if (v < 0)
		{
			sgn = -1;
		}
		else
		{
			sgn = 0;
		}
		

		fi = atan2(sgn * vdy, sgn * vdx);
		double wCalkowita = (fi - fin_1)/dt;
		double wp = (v + (b / 2) * wCalkowita) / promien_kol;
		double wl = (v - (b / 2) * wCalkowita) / promien_kol;
		double wObrNaMinp = (wp * 30) / pi;
		double wObrNaMinl = (wl * 30) / pi;
		double PWMp = (wObrNaMinp + 35) / 0.34;
		double PWMl = (wObrNaMinl + 35) / 0.34;
		PWMp = (int)round(PWMp + 0.5);
		PWMl = (int)round(PWMl + 0.5);
		if(PWMp > 255)
		PWMp = 254;
		else if (PWMp < 0)
		PWMp = 0;
		if(PWMl > 255)
		PWMl = 254;
		else if(PWMl < 0)
		PWMl = 0;
		uint8_t pl = (uint8_t)PWMl;
		uint8_t pp = (uint8_t)PWMp;

		OCR3A=pp;
		OCR3B=pl;
		
		USART_Transmit(pl);
		USART_Transmit(pp);

		if(n > 1)
		{
			_delay_ms(1000);
		}
		
		xn_1 = x;
		yn_1 = y;
		fin_1 = fi;
		
	}

	OCR3A=0;
	OCR3B=0;
		}
	
 }
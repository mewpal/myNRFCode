/*
 * RF_Tranceiver.c
 *
 * Created: 2012-08-10 15:24:35
 *  Author: Kalle
 *  Atmega88
 */ 

#include <avr/io.h>
#include <stdio.h>
#define F_CPU 8000000UL  // 8 MHz - with no prescaler
#include <util/delay.h>
#include <avr/interrupt.h>

#include "nRF24L01.h"

uint8_t *data;


void clockprescale(void)	
{
	CLKPR = 0b10000000;	//Prepare the chip for a change of clock prescale (CLKPCE=1 and the rest zeros)
	CLKPR = 0b00000000;	//Wanted clock prescale (CLKPCE=0 and the four first bits CLKPS0-3 sets division factor = 1)
	//See page 38 in datasheet
}
////////////////////////////////////////////////////


/*****************SPI*****************************/
void InitSPI(void)
{	//Set SCK (PB2), MISO of AVR (PB1) as output
	//THIS HAS TO BE SET BEFORE SPI-ENABLE BELOW
	DDRB |= (1<<PB1) | (1<<PB2) | (1<<PB3); //added PB3 as output because we're adding CE back in - no 3 pin setup

	// Set MOSI (PB0) as input
	DDRB &= ~(1<<PB0);
	PORTB |= (1<<PB0);

	USICR |= (1<<USIWM0)|(1<<USICS1)|(1<<USICLK);

	//omitting this because we are using these pins for IO interaction.
	// CE is now tied high and CSN is multiplexed with SCLK
	//SETBIT(PORTB, 4);	//CSN IR_High to start with, nothing to be sent to the nRF yet
	//CLEARBIT(PORTB, 3);	//CE low to start with, nothing to send/receive yet!
}

uint8_t WriteByteSPI(uint8_t cData)
{
	//Load byte to Data register
	USIDR = cData;

	USISR |= (1<<USIOIF);//clear flag to be able to receive new data

	/* Wait for transmission complete */
	while((USISR & (1<<USIOIF)) == 0)
	{
		USICR |= (1<<USITC);
	}

	return USIDR;
}
////////////////////////////////////////////////////


/*****************in/out***************************/
//Here is where you set up your input/output ports for the ATtiny.
void ioinit(void)
{
	DDRB |= (1<<PB4); //led AFTER DO THE 4 PIN SETUP
	PORTB &=~(1<<PB4);//initalize port low
}
////////////////////////////////////////////////////


//Get a register (I think)
uint8_t GetReg(uint8_t reg)
{
	_delay_us(10);
	CLEARBIT(PORTB, 2);	//CSN low - nRF starts to listen for a command (this is with the 5 pin setup)
//WHAT WE NEED TO DO IS SET THIS TO CLEAR AND WRITE TO PB2, WHICH IS THE SCLK PIN
//SO WE GET A REGISTER WHEN CSN IS HIGH????? SO ITS ALWAYS GOING TO BE HIGH?
	_delay_ms(10);//increased delay to account for multiplexing [i think this can be microseconds]
	WriteByteSPI(R_REGISTER + reg);	//R_Register = set the nRF to reading mode, reg = this registry will be read back
	_delay_us(10);
	reg = WriteByteSPI(NOP);	//Send NOP (dummy byte) once to receive back the first byte in the "reg" register
	_delay_us(10);
	SETBIT(PORTB, 2);	//set SCLK to high 
	_delay_ms(64); //increased delay to account for multiplexing [i think this can be microseconds]
	return reg;	// Return the read registry
}


/*****************nrf-setup***************************/
uint8_t *WriteToNrf(uint8_t ReadWrite, uint8_t reg, uint8_t *val, uint8_t antVal)
{
	if (ReadWrite == W)	//if "W" then you want to write to the nRF (read mode "R" = 0x00)
	{
		reg = W_REGISTER + reg;	//add the "write" bit to the "reg"
	}

	//Static uint8_t is needed to be able to return an array, this array returned at the end
	static uint8_t ret[32];

	_delay_us(10);		//make sure last command has finished
	CLEARBIT(PORTB, 2);	//CSN low = nrf starts to listen for a command
	_delay_ms(10);
	WriteByteSPI(reg);	//set the nRF to Write or Read mode of "reg"
	_delay_ms(10);

	int i;
	for(i=0; i<antVal; i++)
	{
		if (ReadWrite == R && reg != W_TX_PAYLOAD)// did you want to read a registry?
		{//Note: when writing to the W_TX_PAYLOAD you cannot add the "W" because its on the same level as the registry - not sure what this means)
			ret[i]=WriteByteSPI(NOP);	//Send dummy bytes to read out the data
			_delay_us(10);
		}
		else
		{
			WriteByteSPI(val[i]);	//sends the commands to the nRF one at a time
			_delay_us(10);
		}
	}
	SETBIT(PORTB, 2);	//CSN High = nrf goes back to sleep
	_delay_ms(5);
	return ret;	//return the array
}

//initierar nrf'en (obs nrfen måste vala i vila när detta sker CE-låg)
void nrf24L01_init(void)
{
	_delay_ms(100);	//allow radio to reach power down if shut down

	uint8_t val[5];	//an array of integers to send to the *WriteToNrf function

	//EN_AA - (enable auto-acknowledgements) - Transmitter gets automatic response from receiver when successful tranmission
	//Only works if transmitter has identical RF _Address on its channel ex: RX_ADDR_P0 = TX_ADDR
	val[0]=0x01;//set the number value (this case is one) 
	WriteToNrf(W, EN_AA, val, 1);	//W=write mode, EN_AA=register to write to, val=data to write, 1=number of data bytes

	//0b0010 00011 "2" sets it up to 750uS delay between every retry (at least 500us at 250kbps and if payload >5bytes in 1Mbps, and if payload >15byte in 2Mbps) "F" is number of retries (1-15, now 15)
	val[0]=0x2F;
	WriteToNrf(W, SETUP_RETR, val, 1);

	//choose number of enabled data pipes (1-5)
	val[0]=0x01;
	WriteToNrf(W, EN_RXADDR, val, 1); //enable data pipe 0

	//RF_Adress width setup (how many bytes is the receiver address? 1-5 bytes)
	val[0]=0x03;
	WriteToNrf(W, SETUP_AW, val, 1); //0b0000 00011 = 5byte RF_Address (wait isn't that three?)

	//RF channel setup - choose frequency 2,400-2,527GHz 1MHz/step
	val[0]=0x01;
	WriteToNrf(W, RF_CH, val, 1); //RF channel registry 0b0000 0001 = 2,401GHz

	//RF setup - choose power mode and data speed.
	val[0]=0x07;
	WriteToNrf(W, RF_SETUP, val, 1); //00000111 bit 3="0" 1Mbps=longer range, bit 2-1 power mode ("11" = -0dB ; "00" = -18dB)

	//RX RF_Address setup 5 byte
	int i;
	for(i=0; i<5; i++)
	{
		val[i]=0x12;	//RF channel registry 0b10101011 x 5 - long and secure
	}
	WriteToNrf(W, RX_ADDR_P0, val, 5); //since we chose pipe 0 on EN_RXADDR we give this address to that channel
	//Here you can give different addresses to different channels (if they are enabled in EN_RXADDR) to listen on several different transmitters)


	//TX RF_Address setup 5 byte - not used in receiver but doesn't hurt to set it
	for(i=0; i<5; i++)
	{
		val[i]=0x12;	//RF channel registry 0b10111100 x 5 - make this the same as RX-RF_Address to use EN_AA
	}
	WriteToNrf(W, TX_ADDR, val, 5);

	// payload width setup - 1-32bytes (how many bytes per transmission)
	val[0]=5;		//send 5 bytes per package (same on receiver and trasmitter)
	WriteToNrf(W, RX_PW_P0, val, 1);

	//CONFIG reg setup - now its time to boot up the nRF and choose tx or rx
	val[0]=0x1F;  //0b0001 1111 CONFIG registry <--- CHANGED TO RECEIVER 10/17/14
	//bit "1":1=power up,  bit "0":0=transmitter (bit "0":1=Reciver) (bit "4":1=mask_Max_RT) 
	WriteToNrf(W, CONFIG, val, 1);

//device need 1.5ms to reach standby mode
	_delay_ms(100);
}

void ChangeAddress(uint8_t adress)
{
	_delay_ms(100);
	uint8_t val[5];
	//RX RF_Adress setup 5 byte - väljer RF_Adressen på Recivern (Måste ges samma RF_Adress om Transmittern har EN_AA påslaget!!!)
	int i;
	for(i=0; i<5; i++)
	{
		val[i]=adress;	//RF channel registry 0b10101011 x 5 - skriver samma RF_Adress 5ggr för att få en lätt och säker RF_Adress (samma på transmitterns chip!!!)
	}
WriteToNrf(W, RX_ADDR_P0, val, 5); //0b0010 1010 write registry - eftersom vi valde pipe 0 i "EN_RXADDR" ovan, ger vi RF_Adressen till denna pipe. (kan ge olika RF_Adresser till olika pipes och därmed lyssna på olika transmittrar)
	
	//TX RF_Adress setup 5 byte -  väljer RF_Adressen på Transmittern (kan kommenteras bort på en "ren" Reciver)
	//int i; //återanvänder föregående i...
	for(i=0; i<5; i++)
	{
		val[i]=adress;	//RF channel registry 0b10111100 x 5 - skriver samma RF_Adress 5ggr för att få en lätt och säker RF_Adress (samma på Reciverns chip och på RX-RF_Adressen ovan om EN_AA enablats!!!)
	}
	WriteToNrf(W, TX_ADDR, val, 5);
	_delay_ms(100);
}
/////////////////////////////////////////////////////

/*****************Functions***************************/
//Reset the IRQs
void reset(void)
{
	_delay_us(10);
	CLEARBIT(PORTB, 2);	//CSN low
	_delay_ms(10);
	WriteByteSPI(W_REGISTER + STATUS);	//write to STATUS registry
	_delay_us(10);
	WriteByteSPI(0x70);	//reset all IRQ in STATUS registry
	_delay_us(10);
	SETBIT(PORTB, 2);	//CSN IR_High
	_delay_ms(5);
}


/*********************Receive function******************************/
void receive_payload(void)
{
	SETBIT(PORTB, 1);	//CE IR_High = listen for data
	_delay_ms(1000);	//listen for 1 second at a time
	CLEARBIT(PORTB, 1); //ce low again - stop listening
}

//Send data
void transmit_payload(uint8_t * W_buff)
{
	//Send 0xE1 to flush the registry of old data. W_buff is only there because the method needs an array to be called
	WriteToNrf(R, FLUSH_TX, W_buff, 0);

	//Send the data in W_buff to the nrf
	//Why FLUSH_TX os sent woth an "R" instead of a "W" is because they are on the highest byte level of the nRF (see datasheet)
	WriteToNrf(R, W_TX_PAYLOAD, W_buff, 5);

	_delay_ms(10);		//needs a 10ms delay to work after loading the nRF with the payload
	SETBIT(PORTB, 1);	//CE high= transmit data
	_delay_us(20);		//wait 10 us!
	CLEARBIT(PORTB, 1);	//CE low = stop transmitting
	_delay_ms(10);		//delay again..
}


/////////////////////////////////////////////////////

int main(void)
{
//	clockprescale();
	InitSPI();
	ioinit();
	nrf24L01_init();

	/*SETBIT(PORTB,4);		//see if the LED (on PB4) is working and the chip is on
	_delay_ms(1000);
	CLEARBIT(PORTB,4);
	_delay_ms(1000);
	SETBIT(PORTB,4);*/
	int i;
	uint8_t firstByteReceived;

	while(1)
	{
		//FOR RECEIVER

		reset();
		//receive_payload(); -- no longer need this because CE tied high
		firstByteReceived = WriteToNrf(R, R_RX_PAYLOAD, 1, 1);//Read the first byte in the payload. 1 and 1 are just placeholders required by the method for tx.
		if (firstByteReceived == 0x03)
		{
			for (i=0;i<100;i++)
			{
				SETBIT(PORTB, 4);
				_delay_ms(200);
				CLEARBIT(PORTB, 4);
				_delay_ms(200);
			}
		}

	}
	return 0;
}

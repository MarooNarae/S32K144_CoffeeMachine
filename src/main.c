#include "device_registers.h" /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
#include "LPUART.h"
#include "S32K144.h"
#include <stdio.h>
#include <string.h>
int lpit0_ch0_flag_counter = 0; /*< LPIT0 timeout counter */
int lpit0_ch1_flag_counter = 0; /*< LPIT0 timeout counter */
int lpit0_ch3_flag_counter = 0;
char data=0;
char receive = ' ';

#define PTE5 5
#define PTE6 6
#define N_STEP 1500
#define NUM_OF_STATES 8
#define DELAY_MAX 150

unsigned int num,num0,num1,num2,num3 =0;
unsigned int i=0;
unsigned int j=0; /*FND select pin index */
int a,b;
unsigned int FND_DATA[10]={0x13E, 0x00C,0x236,0x21E,0x30C,0x31A,0x33A,0x10E,0x33E,0x30E};// 0~9 number
unsigned int Delaytime = 0; /* Delay Time Setting Variable*/
unsigned int FND_SEL[4]={0x0400,0x0800,0x1000,0x2000};
char state_array[NUM_OF_STATES] = {0x06, 0x02, 0x0A, 0x08, 0x09, 0x01, 0x05, 0x04};
int steps_to_move;
int next_state;

void NVIC_init_IRQs(void){
   /*LPIT ch0 overflow set*/

   /*LPIT ch1 overflow set*/
   S32_NVIC->ICPR[1] = 1 << (49 % 32);
   S32_NVIC->ISER[1] = 1 << (49 % 32);
   S32_NVIC->IP[49] = 0x0B;
   // LPIT0 CH3
   S32_NVIC -> ICPR[1] |= 1 << (51 % 32);
   S32_NVIC -> ISER[1] |= 1 << (51 % 32);
   S32_NVIC -> IP[51] = 0x00;
}

void PORT_init (void)
{
    //PORT_INIT FOR 7-SEGMENT => START
     PCC-> PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORT D */

     PTB->PDDR |= (1<<1);
     PTB->PDDR |= (1<<2);
     PTB->PDDR |= (1<<3);
     PTB->PDDR |= (1<<4);
     PTB->PDDR |= (1<<5);
     PTB->PDDR |= (1<<8);
     PTB->PDDR |= (1<<9);

     PORTB->PCR[1] = PORT_PCR_MUX(1);
     PORTB->PCR[2] = PORT_PCR_MUX(1);
     PORTB->PCR[3] = PORT_PCR_MUX(1);
     PORTB->PCR[4] = PORT_PCR_MUX(1);
     PORTB->PCR[5] = PORT_PCR_MUX(1);
     PORTB->PCR[8] = PORT_PCR_MUX(1);
     PORTB->PCR[9] = PORT_PCR_MUX(1);

     PTB->PDDR |= 1<<10|1<<11|1<<12|1<<13;
     PORTB->PCR[10] = PORT_PCR_MUX(1);
     PORTB->PCR[11] = PORT_PCR_MUX(1);
     PORTB->PCR[12] = PORT_PCR_MUX(1);
     PORTB->PCR[13] = PORT_PCR_MUX(1);
// PORT_INIT FOR 7_SEGMENT => END

// PORT_INIT FOR LPUART => START
     PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
     PORTC->PCR[6] |= PORT_PCR_MUX(2);
     PORTC->PCR[7] |= PORT_PCR_MUX(2);
// PORT_INIT FOR LPUART => END

//PORT_INIT FOR DC_MOTOR => START
     PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
     PTE->PDDR |= 1<<PTE5|1<<PTE6;
     PORTE->PCR[5] = PORT_PCR_MUX(1);
     PORTE->PCR[6] = PORT_PCR_MUX(1);
//PORT_INIT FOR DC_MOTOR => END

//PORT_INIT FOR STEP_MOTOR =>START
     PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;
     PTA->PDDR|=0x0F;
     PORTA->PCR[0] = PORT_PCR_MUX(1);
     PORTA->PCR[1] = PORT_PCR_MUX(1);
     PORTA->PCR[2] = PORT_PCR_MUX(1);
     PORTA->PCR[3] = PORT_PCR_MUX(1);
//PORT_INIT FOR STEP_MOTOR => END

//PORT_INIT FOR LCD => START
     PTD->PDDR |= 1<<9 | 1<<10 | 1<<11 | 1<<12 | 1<<13 | 1<<14 | 1<<15;
     PCC->PCCn[PCC_PORTD_INDEX] &= ~PCC_PCCn_CGC_MASK;
     PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_PCS(0x001);
     PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
     PCC->PCCn[PCC_FTM2_INDEX]  &= ~PCC_PCCn_CGC_MASK;
     PCC->PCCn[PCC_FTM2_INDEX]  |= (PCC_PCCn_PCS(1)| PCC_PCCn_CGC_MASK);		//Clock = 80MHz
         //Pin mux
     PORTD->PCR[9]= PORT_PCR_MUX(1);
     PORTD->PCR[10]= PORT_PCR_MUX(1);
     PORTD->PCR[11]= PORT_PCR_MUX(1);
     PORTD->PCR[12]= PORT_PCR_MUX(1);
     PORTD->PCR[13]= PORT_PCR_MUX(1);
     PORTD->PCR[14]= PORT_PCR_MUX(1);
     PORTD->PCR[15]= PORT_PCR_MUX(1);
//PORT_INIT FOR LCD => END
}

void PortOut(char Phase)
{
	if (Phase & 0x01 )	{
		PTA->PSOR |= 1;
	}
	else	{
		PTA->PCOR |= 1;
	}

	if (Phase & 0x02 )	{
		PTA->PSOR |= 2;
	}
	else	{
		PTA->PCOR |= 2;
	}

	if (Phase & 0x04 )	{
		PTA->PSOR |= 4;
	}
	else	{
		PTA->PCOR |= 4;
	}

	if (Phase & 0x08 )	{
		PTA->PSOR |= 8;
	}
	else	{
		PTA->PCOR |= 8;
	}
}

void StepForward(void)
{

		if (next_state > (NUM_OF_STATES - 1)) //If next_state is greater than the highest
												//available state, 7, then cycle back to 0
		{
			next_state = 0;
		}
		// PTU = state_array[next_state]; //Place new value in Port U. Rotation may be observed

		//StepPhase = PTA->PDIR;
		//StepPhase &= ~(0x0F);
		//PTA->PDOR =  StepPhase | (state_array[next_state] & 0x0F);
		//PTA->PDOR =  state_array[next_state];
		PortOut(state_array[next_state]);

		MotorDelay();

		next_state++; //Increment next_state. Cycling though the states causes rotation
						//in one direction. Decrementing states causes opposite rotation.
		steps_to_move--; //Subtract 1 from the total # of steps remaining to be moved.

	steps_to_move = N_STEP;
}

void StepBackward(void)
{

		if (next_state < 0 ) //If next_state is greater than the highest
												//available state, 7, then cycle back to 0
		{
			next_state = (NUM_OF_STATES - 1);
		}
		//PTU = state_array[next_state]; //Place new value in Port U. Rotation may be observed
		// PTA->PDOR = (PTA->PDOR & ~(0x0F)) | (state_array[next_state] & 0x0F);
		//StepPhase = (PTA->PDIR & ~(0x0F));
		//StepPhase = PTA->PDIR;
		//StepPhase &= ~(0x0F);
		//PTA->PDOR =  StepPhase | (state_array[next_state] & 0x0F);
		//PTA->PDOR =  state_array[next_state];
		PortOut(state_array[next_state]);

		MotorDelay();

		next_state--;   //Decrement next_state. Cycling though the states causes rotation
						//in one direction. Decrementing states causes opposite rotation.
		steps_to_move--; //Subtract 1 from the total # of steps remaining to be moved.

	steps_to_move = N_STEP;
}

void WDOG_disable (void)
{
  WDOG->CNT=0xD928C520;     /* Unlock watchdog       */
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value    */
  WDOG->CS = 0x00002100;    /* Disable watchdog       */
}



void LPIT0_init (uint32_t Dtime)
{
   /*!
    * LPIT Clocking:
    * ==============================
    */
   PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
   PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs       */
     /*!
      * LPIT Initialization:
      */
   LPIT0->MCR = 0x00000001;  /* DBG_EN-0: Timer chans stop in Debug mode */
                                           /* DOZE_EN=0: Timer chans are stopped in DOZE mode */
                                           /* SW_RST=0: SW reset does not reset timer chans, regs */
                                           /* M_CEN=1: enable module clk (allows writing other LPIT0 regs) */
   LPIT0->MIER = 0x0E;  /* TIE0=1: Timer Interrupt Enabled fot Chan 0,1,2 */

   LPIT0->TMR[3].TVAL = 40000000;      /* Chan 0 Timeout period: 40M clocks */
    LPIT0->TMR[3].TCTRL = 0x00000001;
                                        /* T_EN=1: Timer channel is enabled */
                                 /* CHAIN=0: channel chaining is disabled */
                                 /* MODE=0: 32 periodic counter mode */
                                 /* TSOT=0: Timer decrements immediately based on restart */
                                 /* TSOI=0: Timer does not stop after timeout */
                                 /* TROT=0 Timer will not reload on trigger */
                                 /* TRG_SRC=0: External trigger soruce */
                                 /* TRG_SEL=0: Timer chan 0 trigger source is selected*/

   LPIT0->TMR[1].TVAL = Dtime* 40000;      /* Chan 1 Timeout period: 40M clocks */
    LPIT0->TMR[1].TCTRL = 0x00000001;
                                        /* T_EN=1: Timer channel is enabled */
                                 /* CHAIN=0: channel chaining is disabled */
                                 /* MODE=0: 32 periodic counter mode */
                                 /* TSOT=0: Timer decrements immediately based on restart */
                                 /* TSOI=0: Timer does not stop after timeout */
                                 /* TROT=0 Timer will not reload on trigger */
                                 /* TRG_SRC=0: External trigger soruce */
                                 /* TRG_SEL=0: Timer chan 0 trigger source is selected*/

   LPIT0->TMR[2].TVAL = Dtime* 40000;      /* Chan 1 Timeout period: 40M clocks */
    LPIT0->TMR[2].TCTRL = 0x00000001;

    uint32_t timeout;
    timeout = Dtime*10000;
    LPIT0->TMR[0].TVAL = timeout;
    LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
}

LPIT0_init_delay(uint32_t Delay_time)
{
    LPIT0->TMR[2].TVAL = Delay_time* 40000;      /* Chan 1 Timeout period: 40M clocks */
    LPIT0->TMR[2].TCTRL = 0x00000001;
}

void delay_ms (volatile int us){
   LPIT0_init_delay(us);           /* Initialize PIT0 for 1 second timeout  */
   while (0 == (LPIT0->MSR &   LPIT_MSR_TIF2_MASK))
               lpit0_ch1_flag_counter++;         /* Increment LPIT0 timeout counter */
               LPIT0->MSR |= LPIT_MSR_TIF2_MASK;;//............LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
}


void Seg_out(int number){

    Delaytime = 1;

   num3=(number/1000)%10;
   num2=(number/100)%10;
   num1=(number/10)%10;
   num0= number%10;


   // 1000?�리??출력
   PTB->PSOR = FND_SEL[j];
   PTB->PSOR = FND_DATA[num3];
   delay_ms(Delaytime);
   PTB->PCOR = 0x3fff;
   j++;

   // 100?�리??출력
   PTB->PSOR = FND_SEL[j];
   PTB->PSOR = FND_DATA[num2];
   delay_ms(Delaytime);
   PTB->PCOR = 0x3fff;
   j++;

   // 10?�리??출력
   PTB->PSOR = FND_SEL[j];
   PTB->PSOR = FND_DATA[num1];
   delay_ms(Delaytime);
   PTB->PCOR = 0x3fff;
   j++;

   // 1?�리??출력
   PTB->PSOR = FND_SEL[j];
   PTB->PSOR = FND_DATA[num0];
   delay_ms(Delaytime);
   PTB->PCOR = 0x3fff;
   j=0;

}

void LPIT0_Ch1_IRQHandler (void){     /* delay counter */
   lpit0_ch1_flag_counter++;         /* Increment LPIT1 timeout counter */
   LPIT0->MSR |= LPIT_MSR_TIF1_MASK;  /* Clear LPIT0 timer flag 0 */
}

void LPIT0_Ch3_IRQHandler (void){
   lpit0_ch3_flag_counter++;         /* Increment LPIT0 timeout counter */
   num++;
   LPIT0->MSR |= LPIT_MSR_TIF3_MASK;  /* Clear LPIT0 timer flag 0 */
}

void MotorDelay()
{
	 for(a = 0; a < DELAY_MAX; a++)
	 {
		 for(b = 0; b < DELAY_MAX; b++) ;
			 //Wait here for a while.
	 }
}
void text_lcd(char *mess){
   while(mess[i] != '\0'){
            if(i%16 == 0){
                lcdinput(0x80+0x40);
                if(i%32==0){
                    lcdinit();
                }
            	delay_ms(500);
            }
            lcdcharinput(mess[i]); // 1(first) row text-char send to LCD module
            delay_ms(200);
			i++;
		}
}

void Select(void){
	LPUART1_transmit_string("Machine is Ready\n\r");
	LPUART1_transmit_string("Select Menu(1:HOT Coffee, 2:ICE Coffee, 3:HOT Tea)\n\r");
	LPUART1_transmit_string("(4:ICE Tea, 5:Hot Milk, 6:Coke with Ice)\n\r");
	receive = LPUART1_receive_char();
	num = 0;
	 //for HOT coffee
	if(receive == '1'){
		LPUART1_transmit_string("Take the HOT Coffee after 1 minute.\n\r");
		char msg[50] = "I'm Making HOT Coffee ^o^";
		text_lcd(msg);
		for(;;){
			PTE->PSOR |= 1<<PTE5;
			Seg_out(num);
			StepForward();
			if(num == 20){
				break;
			}
		}
		PTE->PCOR |= 1<<PTE5;
		LPUART1_transmit_string("Your HOT Coffee is ready. Come and take it.\n\r");
		LPUART1_transmit_string("Don't Forget to bring 500won ^^. \n\r");
		num = 0;
		lcdinput(0x01);	//Clear display
		i=0;
		char msg1[50] = "500WON Before you take Coffee^o^";
		text_lcd(msg1);
		for(;;){
			Seg_out(num);
			if(num < 60){
			}
			if(num == 60){
				break;
		}

		}
		PTE->PSOR |= 1<<PTE6;
		LPUART1_transmit_string("PLEASE TAKE THE COFFEE!");

	}
	//for ICE Coffee
	else if(receive == '2'){
		LPUART1_transmit_string("Take the ICE Coffee after 1 minute.\n\r");
		char msg2[50] = "I'm Making ICE Coffee ^o^";
		text_lcd(msg2);
		for(;;){
					PTE->PSOR |= 1<<PTE5;
					Seg_out(num);
					StepBackward();
					if(num == 20){
						break;
					}
				}
		PTE->PCOR |= 1<<PTE5;
		LPUART1_transmit_string("Your ICE Coffee is ready. Come and take it.\n\r");
		LPUART1_transmit_string("Don't Forget to bring 500won ^^. \n\r");
		num = 0;
		lcdinput(0x01);	//Clear display
		i=0;
		char msg3[50] = "500WON Before you take Coffee^o^";
		text_lcd(msg3);
		for(;;){
			Seg_out(num);
			if(num < 60){
				}
			if(num == 60){
				break;
				}
				}
		PTE->PSOR |= 1<<PTE6;
		LPUART1_transmit_string("PLEASE TAKE THE COFFEE!");
	}
	else if(receive == '3'){
		LPUART1_transmit_string("Take the HOT TEA after 1 minute.\n\r");
				char msg3[50] = "I'm Making HOT TEA ^o^";
				text_lcd(msg3);
				for(;;){
					PTE->PSOR |= 1<<PTE5;
					Seg_out(num);
					StepForward();
					if(num == 20){
						break;
					}
				}
				PTE->PCOR |= 1<<PTE5;
				LPUART1_transmit_string("Your HOT TEA is ready. Come and take it.\n\r");
				LPUART1_transmit_string("Don't Forget to bring 500won ^^. \n\r");
				num = 0;
				char msg4[50] = "500WON Before you take TEA^o^";
				lcdinput(0x01);	//Clear display
				i=0;
				text_lcd(msg4);
				for(;;){
					Seg_out(num);
					if(num < 60){
					}
					if(num == 60){
						break;
				}
				}
				PTE->PSOR |= 1<<PTE6;
				LPUART1_transmit_string("PLEASE TAKE THE TEA!");
	}
	else if(receive == '4'){
		LPUART1_transmit_string("Take the ICE TEA after 1 minute.\n\r");
				char msg5[50] = "I'm Making ICE TEA ^o^";
				text_lcd(msg5);
				for(;;){
					PTE->PSOR |= 1<<PTE5;
					Seg_out(num);
					StepBackward();
					if(num == 20){
						break;
					}
				}
				PTE->PCOR |= 1<<PTE5;
				LPUART1_transmit_string("Your ICE TEA is ready. Come and take it.\n\r");
				LPUART1_transmit_string("Don't Forget to bring 500won ^^. \n\r");
				num = 0;
				lcdinput(0x01);	//Clear display
				i=0;
				char msg6[50] = "500WON Before you take TEA^o^";
				text_lcd(msg6);
				for(;;){
					Seg_out(num);
					if(num < 60){
					}
					if(num == 60){
						break;
				}
				}
				PTE->PSOR |= 1<<PTE6;
				LPUART1_transmit_string("PLEASE TAKE THE TEA!");
	}
	else if(receive == '5'){
		LPUART1_transmit_string("Take the HOT MILK after 1 minute.\n\r");
				char msg7[50] = "I'm Making HOT MILK ^o^";
				text_lcd(msg7);
				for(;;){
					PTE->PSOR |= 1<<PTE5;
					Seg_out(num);
					StepForward();
					if(num == 20){
						break;
					}
				}
				PTE->PCOR |= 1<<PTE5;
				LPUART1_transmit_string("Your HOT MILK is ready. Come and take it.\n\r");
				LPUART1_transmit_string("Don't Forget to bring 500won ^^. \n\r");
				num = 0;
				lcdinput(0x01);	//Clear display
				i=0;
				char msg8[50] = "500WON Before you take MILK ^o^";
				text_lcd(msg8);
				for(;;){
					Seg_out(num);
					if(num < 60){
					}
					if(num == 60){
						break;
				}
				}
				PTE->PSOR |= 1<<PTE6;
				LPUART1_transmit_string("PLEASE TAKE THE MILK!");
	}
	else if(receive == '6'){
		LPUART1_transmit_string("Take the COKE after 1 minute.\n\r");
		char msg9[50] = "I'm Making COKE ^o^";
		text_lcd(msg9);
				for(;;){
					PTE->PSOR |= 1<<PTE5;
					Seg_out(num);
					StepBackward();
					if(num == 20){
						break;
					}
				}
		PTE->PCOR |= 1<<PTE5;
		LPUART1_transmit_string("Your COKE is ready. Come and take it.\n\r");
		LPUART1_transmit_string("Don't Forget to bring 500won ^^. \n\r");
		num = 0;
		lcdinput(0x01);	//Clear display
				i=0;
		char msg10[50] = "500WON Before you take COKE^o^";
		text_lcd(msg10);
		for(;;){
			Seg_out(num);
			if(num < 60){
			}
					if(num == 60){
						break;
				}
				}
				PTE->PSOR |= 1<<PTE6;
								LPUART1_transmit_string("PLEASE TAKE THE COKE!");
	}
	else {
		LPUART1_transmit_string("PLEASE ENTER THE CORRECT MENU \n\r");
	}
}

int main(void)
{
   WDOG_disable();/* Disable Watchdog in case it is not done in startup code */
   PORT_init();            /* Configure ports */
   SOSC_init_8MHz();       /* Initialize system oscilator for 8 MHz xtal */
   SPLL_init_160MHz();     /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
   NormalRUNmode_80MHz();  /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
   NVIC_init_IRQs();       /* Enable desired interrupts and priorities */
   LPIT0_init(1);
   LPUART1_init();
   steps_to_move = N_STEP;
   next_state = 0;
   for(;;){
	   Select();
   }
}

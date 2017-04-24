#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include "LinkedQueue.h"

volatile char STATE;

// define the global variables that can be used in every function ==========
volatile unsigned int ADC_result;
volatile unsigned char ADCH_result;
volatile unsigned char ADCL_result;
volatile unsigned int ADC_lowest;
volatile unsigned int material_check;

// stepper
volatile int gv = 0;
volatile char step[4] = {0b00110110, 0b00101110, 0b00101101, 0b00110101};
volatile char initial_stepper_flag = 0;

// pause and ramp down
volatile int b_n = 0;
volatile int al_n = 0;
volatile int w_n = 0;
volatile int st_n = 0;
volatile int onbelt = 0;
volatile int dd = 1000;
volatile int ramp_flag = 0;
volatile int pause_flag = 0;

// pieces
volatile int al_min = 0b0000000001;
volatile int al_max = 0b0101000010;//
volatile int st_min = 0b0101000011;
volatile int st_max = 0b1100100010;//
volatile int  w_min = 0b1100100011;
volatile int  w_max = 0b1101111000;//
volatile int  b_min = 0b1101111001;
volatile int  b_max = 0b1111111111;

volatile int current_plate = 0;
volatile int pre_plate = 0;

//volatile int i = 0;

volatile int step90[50]   = {19,19,18,18,17,16,15,14,13,12,11,10,9,8,7,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,8,9,10,11,12,13,14,15,16,17,17,18,18};

volatile int step180[100] = {19,19,18,18,17,16,15,14,13,12,11,10,9,8,7,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,8,9,10,11,12,13,14,15,16,17,17,18,18};

// initialize the link list for main task
link *head;			/* The ptr to the head of the queue */
link *tail;			/* The ptr to the tail of the queue */
link *newLink = NULL;		/* A ptr to a link aggregate data type (struct) */
link *rtnLink = NULL;		/* same as the above */

// initial the link list for material check
link *head_m;			/* The ptr to the head of the queue */
link *tail_m;			/* The ptr to the tail of the queue */
link *newLink_m = NULL;		/* A ptr to a link aggregate data type (struct) */
link *rtnLink_m = NULL;		/* same as the above */

// Functions
void mTimer(int count);// Timer
void rTimer();
void pwm();// Speed for motor
volatile int speed = 0x40;
void dcForward();
void dcBrake();// Motor stop

void determine(int r, int r_m);

void initialstepper();// Initialize the stepper motor
void stepper(int n, char d);// Running the stepper motor
void stepperpro(int n, char d);

// link list
void setup(link **h,link **t);// 
void enqueue(link **h, link **t, link **nL);
void dequeue(link **h, link **t, link **deQueuedLink);
element firstValue(link **h);
void clearQueue(link **h, link **t);
char isEmpty(link **h);
int size(link **h, link **t);

int main(){

	STATE = 0;

	//ADC_lowest = 0xFFFF;// Store the lowest reflective number

	TCCR1B |= _BV(CS11);//create the timer
	

	cli(); // disable all of the interrupt ==========================
	
	//system clock
	CLKPR = 0b10000000;
	CLKPR = 0b00000000;

	// config the external interrupt ======================================
	EIMSK |= _BV(INT0);// OI
	EIMSK |= _BV(INT1);// IN
	EIMSK |= _BV(INT2);// OR
	EIMSK |= _BV(INT3);// Button 1 pause
	EIMSK |= _BV(INT4);// EX
	EIMSK |= _BV(INT5);// HE
	EIMSK |= _BV(INT6);// Button 2 ramp

	EICRA |= _BV(ISC01);
	EICRA |= _BV(ISC11);
	EICRA |= _BV(ISC21) | _BV(ISC20);
	EICRA |= _BV(ISC31);// | _BV(ISC30);
	EICRB |= _BV(ISC41); 
	EICRB |= _BV(ISC51);
	EICRB |= _BV(ISC61);// | _BV(ISC60);

 	// config ADC =========================================================
	// by default, the ADC input (analog input is set to be ADC0 / PORTF0
	ADCSRA |= _BV(ADEN); // enable ADC
	ADCSRA |= _BV(ADIE); // enable interrupt of ADC
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1);// prescaler
	ADMUX |= _BV(MUX0) | _BV(REFS0);
 	
	// PORT
	DDRA = 0b11111111;// Stepper [0-5]
	DDRB = 0b11111111;// DC[0-3] pwm[7]
	DDRC = 0b11111111;// LEDs
	DDRD = 0b11110000;// PF2 OP2; 
	DDRE = 0b10001111;// 
	DDRF = 0b11111001;// PF1 reflective sensor

	// sets the Global Enable for all interrupts ==========================
	sei();

	setup(&head, &tail);// set up linked list
	setup(&head_m, &tail_m);

	// initialize pwm
	TCCR0B |= _BV(CS01) | _BV(CS00);
	TCCR0A |= _BV(WGM01);
	TCCR0A |= _BV(WGM00);
	//TIMSK0 = TIMSK0 |0b00000010;
	TCCR0A |= _BV(COM0A1);	
	TCNT0 = 0x00;
	
	// initialize stepper
	initialstepper();

	// initialize the ADC, start one conversion at the beginning ==========]	
	//ADCSRA |= _BV(ADSC);
	PORTC = 0x00;
	goto POLLING_STAGE;

	// POLLING STATE
	POLLING_STAGE:
		//if(ramp_flag == 1 && size(&head, &tail) == 0){
		//	STATE = 5;
		//}

		if(pause_flag == 1){
			STATE = 4;
		}

		pwm();
		dcForward();	// Indicates this state is active
		switch(STATE){
			case (0) :
				goto POLLING_STAGE;
				break;	//not needed but syntax is correct
			case (1) :
				goto MAGNETIC_STAGE;
				break; 
			case (2) :
				goto REFLECTIVE_STAGE;
				break;
			case (3) :
				goto BUCKET_STAGE;
				break;
			case (4) :
				goto PAUSE;
				break;
			case (5) :
				goto RAMP;
				break;
			case (6) :
				goto END;
			default :
				goto POLLING_STAGE;
		}//switch STATE
		

	MAGNETIC_STAGE:
		initLink(&newLink_m);
		newLink_m->e.itemCode = material_check;
		enqueue(&head_m, &tail_m, &newLink_m);

		//Reset the state variable
		STATE = 0;
		goto POLLING_STAGE;

	REFLECTIVE_STAGE:
		initLink(&newLink);
		newLink->e.itemCode = ADC_lowest;
		enqueue(&head, &tail, &newLink);
		//if(size(&head, &tail) == 1){
		//	determine(head->e.itemCode, head_m->e.itemCode);
		//}

		STATE = 0;
		goto POLLING_STAGE;
	
	BUCKET_STAGE:
		if(pause_flag != 1){
			dcBrake();
			dequeue(&head, &tail, &rtnLink);
			dequeue(&head_m, &tail_m, &rtnLink_m);
			determine(rtnLink->e.itemCode, rtnLink_m->e.itemCode);
			free(rtnLink);
			free(rtnLink_m);
			//sei();
			//Reset the state variable
			STATE = 0;
			goto POLLING_STAGE;
		}else{
			STATE = 4;
			goto PAUSE;
		}
		

	PAUSE:
		//mTimer(80);
		dcBrake();
		// display
		
		onbelt = size(&head, &tail);
		
		while(pause_flag == 1){
			PORTC = 0b00010000 + b_n;
			mTimer(dd);
			if(pause_flag == 0) break;
			PORTC = 0b00100000 + al_n;
			mTimer(dd);
			if(pause_flag == 0) break;
			PORTC = 0b01000000 + w_n;
			mTimer(dd);
			if(pause_flag == 0) break;
			PORTC = 0b10000000 + st_n;
			mTimer(dd);
			if(pause_flag == 0) break;
			PORTC = 0b11110000 + onbelt;
			mTimer(dd);
			if(pause_flag == 0) break;
		}
		
		dcForward();
		STATE = 0;
		goto POLLING_STAGE;

	RAMP:
		PORTB = 0b00000110;
		//onbelt = size(&head, &tail);
		cli();
		while(1){
			PORTC = 0b00010000 + b_n;
			mTimer(dd);
			PORTC = 0b00100000 + al_n;
			mTimer(dd);
			PORTC = 0b01000000 + w_n;
			mTimer(dd);
			PORTC = 0b10000000 + st_n;
			mTimer(dd);
			//PORTC = 0b11110000 + onbelt;
			//mTimer(dd);
		}

	END:

	return(0);
}

// OI
ISR(INT0_vect){
	if(speed == 0x40){
		speed = 0x60;
	}
	STATE = 1;
	//initLink(&newLink_m);
	//newLink_m->e.itemCode = 0;
	//enqueue(&head_m, &tail_m, &newLink_m);
}

// IN
ISR(INT1_vect){
	tail_m->e.itemCode = 1;
	PORTC = 0b11111111;
}

// OR
ISR(INT2_vect){
	// when there is a rising edge, we need to do ADC =====================
	ADC_lowest = 0xFFFF;
	
	ADCSRA |= _BV(ADSC);
}

// pause
ISR(INT3_vect){
	mTimer(20);

	if(pause_flag == 0){
		pause_flag = 1;
		STATE = 4;
	}else{
		pause_flag = 0;
	}
}

//EX
ISR(INT4_vect){
	//cli();
	//PORTC ++;
	//mTimer(20);
	STATE = 3;
}

// HE
ISR(INT5_vect){
	initial_stepper_flag =1;
}

//ramp
ISR(INT6_vect){
	mTimer(20);
	rTimer();
	//ramp_flag = 1;
	
}

ISR(TIMER3_COMPA_vect){
	STATE = 5;
}

// the interrupt will be trigured if the ADC is done ========================
ISR(ADC_vect){
	if(ADC < ADC_lowest){
		ADCL_result = ADCL;
		ADCH_result = ADCH;
		ADC_result = ADC;
		ADC_lowest = ADC;
	}
	
	if((PIND & 0b00000100) == 0b00000100){
		ADCSRA |= _BV(ADSC);

	}else{
		PORTC = ADCL_result;
		PORTD = ADCH_result << 5;
		STATE = 2;
	}
	

}

void determine(int r, int r_m){
	if(r_m == 1){
		if(r > al_max){
			current_plate = 1; // St
			st_n ++;
		}else{
			current_plate = 3; // Al
			al_n ++;
		}
	}else if(r > w_max){
		current_plate = 0; // Black
		b_n ++;
	}else if (r > st_max){
		current_plate = 2; // White
		w_n ++;
	}else if(r > al_max){
		current_plate = 1; // St
		st_n ++;
	}else if(r > al_min){
		current_plate = 3; // Al
		al_n ++;
	}else{
		//PORTC = 0b11111111;
	}

/*	if(r_m == 1){
		if(r > al_max){
			current_plate = 1; // St
			st_n ++;
		}else{
			current_plate = 3; // Al
			al_n ++;
		}
	}else{
	    if(r > w_max){
			current_plate = 0; // Black
			b_n ++;
		}else {
			current_plate = 2; // White
			w_n ++;
		}
	}
*/
	int rotate = pre_plate - current_plate;

	if(rotate == 0){
	}else if(rotate == 1 || rotate == -3){
		stepperpro(50, 1);
		mTimer(50);
	}else if(rotate == 2 || rotate == -2){
		stepperpro(100, 0);
		mTimer(50);
	}else if(rotate == -1 || rotate == 3){
		stepperpro(50, 0);
		mTimer(50);
	}

	pre_plate = current_plate;
}

void dcForward(){
	PORTB = 0b00000001;
}

void dcBrake(){
	PORTB = 0b00000000;	
}

void pwm(){
	OCR0A = speed;
}

void mTimer(int count){
	int i = 0;
	// Set the Waveform Generation mode bit description to Clear Timer on Compare Math mode(CTC) only

	TCCR1B |= _BV(WGM12);
	// This will set the WGM bits to 0100, 142
	// Note WGM is spread over two register

	//Set Output Compare Register for 1000 cycles = 1ms
	OCR1A = 0x03e8;
	//OCR1A = 0x0001;

	//Set the initial value of the Timer Counter to 0x0000
	TCNT1 = 0x0000;

	//Enable the output compare interrupt enable
	//TIMSK1 = TIMSK1 |0b00000010;

	//Clear the timer interrupt flag and begin timer
	//If the following statement is confusing please ask for clarification
	TIFR1 |= _BV(OCF1A);

	//Poll the timer to determine when the timer has reached 0x03e8
	while(i < count){
		if((TIFR1 & 0x02) == 0x02){
			//clear the interrupt flag by wrinting a one to the bit

			TIFR1 |=_BV(OCF1A);

			i++;
			//Note: the timer resets on its own from our WGM setting above
		}
	}
	return;

}

void rTimer(){
	TCCR3B |= _BV(CS32) | _BV(CS30);
	TCCR3B |= _BV(WGM32);
	// This will set the WGM bits to 0100, 142
	// Note WGM is spread over two register

	//Set Output Compare Register for 1000 cycles = 1ms
	OCR3A = 0x6ace;
	//OCR1A = 0x0001;

	//Set the initial value of the Timer Counter to 0x0000
	TCNT3 = 0x0000;

	//Enable the output compare interrupt enable
	TIMSK3 = TIMSK3 |0b00000010;

	return;

}

void initialstepper(){
	int delay_time = 16;
	while(initial_stepper_flag == 0){
		gv++;
		if(gv > 3){
			gv = 0;
		}

		PORTA = step[gv];
		mTimer(delay_time);
	}

}

void stepperpro(int n, char d){
	int i;

	if(d == 1){

		for(i = 0; i < n; i++){
			gv++;
			if(gv > 3){
				gv = 0;
			}

			PORTA = step[gv];
			if(n == 50){
				mTimer(step90[i]);
			}else{
				mTimer(step180[i]);
			}
			
		}
	}else{

		for(i = 0; i < n; i++){
			gv--;
			if(gv < 0){
				gv = 3;
			}

			PORTA = step[gv];
			if(n == 50){
				mTimer(step90[i]);
			}else{
				mTimer(step180[i]);
			}
		}
	}
}

void stepper(int n, char d){
	int i;
	int a_step = 12;
	int delay_time = 18;
	int limitspeed = 6;
	int initialspeed = delay_time;

	if(d == 1){

		for(i = 0; i < n; i++){
			gv++;
			if(gv > 3){
				gv = 0;
			}

			PORTA = step[gv];
			mTimer(delay_time);

			if(i < a_step){
				//delay_time --;
				delay_time = delay_time - (initialspeed - limitspeed)/a_step;
			}
			
			if(i >= (n - a_step)){
				//delay_time ++;
				delay_time = delay_time + (initialspeed - limitspeed)/a_step;
			}
		}
	}else{

		for(i = 0; i < n; i++){
			gv--;
			if(gv < 0){
				gv = 3;
			}

			PORTA = step[gv];
			mTimer(delay_time);

			if(i < a_step){
				//delay_time --;
				delay_time = delay_time - (initialspeed - limitspeed)/a_step;
			}
			
			if(i >= (n - a_step)){
				//delay_time ++;
				delay_time = delay_time + (initialspeed - limitspeed)/a_step;
			}
		}
	}
}

/**************************************************************************************/
/***************************** Linked List ********************************************/
/**************************************************************************************/
/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/

void setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}/*setup*/




/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}/*initLink*/




/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail		
*  of the queue accordingly				
*  INPUT: the head and tail pointers, and a pointer to the new link that was created 
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		//(*h)->next = *nL;
		//should be this
		*h = *nL;
		*t = *nL;
	}/* else */
	return;
}/*enqueue*/




/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink' 
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t, link **deQueuedLink){
	/* ENTER YOUR CODE HERE */
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
	if (*h != NULL){
		*h = (*h)->next;

		if (*h == NULL){
			*t = NULL;
		}
	}/*if*/
	
	return;
}/*dequeue*/




/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/





/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
	
	/* Last but not least set the tail to NULL */
	*t = NULL;		

	return;
}/*clearQueue*/





/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	/* ENTER YOUR CODE HERE */
	return(*h == NULL);
}/*isEmpty*/





/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
int size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	int 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}/*while*/
	
	return(numElements);
}/*size*/



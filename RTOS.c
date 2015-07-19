// Target uC:       33FJ128MC802
// Devices used:    LEDs and PBs

// Hardware description:
// Red LED
//   anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
// Green LED
//   anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
// Yellow LED
//   anode connected through 100ohm resistor to RB3 (pin 7), cathode grounded
// Orange LED
//   anode connected through 100ohm resistor to RB2 (pin 6), cathode grounded
// Push Buttons
//   push button 0 connected between RB12 (pin 23) and ground
//   push button 1 connected between RB13 (pin 24) and ground
//   push button 2 connected between RB14 (pin 25) and ground
//   push button 3 connected between RB15 (pin 26) and ground
// SP3232E RS-232 Interface
//  T1IN connected to RP10 (pin 21)
//  R1OUT connected to RP11 (pin 22)

//-----------------------------------------------------------------------------
// Device includes and assembler directives             
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <p33FJ128MC802.h>
#include<string.h>
#define FCY 40000000UL                       //Instruction cycle rate
#include <libpic30.h>                        
                                             

#define PIN_YELLOW LATAbits.LATA4            //Definition i/os
#define PIN_ORANGE LATBbits.LATB3
#define PIN_GREEN LATBbits.LATB4
#define PIN_RED LATBbits.LATB5
#define PIN_PB0 PORTBbits.RB12
#define PIN_PB1 PORTBbits.RB13
#define PIN_PB2 PORTBbits.RB14
#define PIN_PB3 PORTBbits.RB15

#define BAUD_57600 42                        //fclock: 40 MHz

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables                
//-----------------------------------------------------------------------------

//Function pointer for calling process function 
typedef void (*_fn)();

#define TRUE  1
#define FALSE 0

// semaphore
#define MAX_QUEUE_SIZE 10
struct semaphore{
  unsigned int count;
  unsigned int queue_size;
  unsigned int process_queue[MAX_QUEUE_SIZE]; // store task index here
  char name[15];
} *s, key_pressed, key_released, flash_req,char_received;

// task 
#define STATE_INVALID    0 	// no task
#define STATE_READY      1 	// ready to run
#define STATE_BLOCKED    2 	// has run, but now blocked by semaphore
#define STATE_DELAYED    3 	// has run, but now awaiting timer

#define MAX_TASKS 10       	// maximum number of valid tasks
int task_current = 0;      	// index of last dispatched task
int task_count = 0;        	// total number of valid tasks

int task_delayed = 0;		//Flag for task delayed

int *Z = (unsigned int*) 0x1e;	

int buttons,rx,rxc;

int rtos_mode;             	// Mode: Co-operative or Preemptive 
#define MODE_COOPERATIVE 0
#define MODE_PREEMPTIVE  1


//Task control block
struct _tcb{
  unsigned int state;            // STATE_ values (0, 1, 2, 3) 
  unsigned int pid;              // Unique Process ID 
  unsigned int sp;               // location of stack pointer for process
  unsigned int priority;         // 0=lowest, 7=highest
  unsigned int current_priority;
  int ticks;            	 // time counter value for sleep()  
  unsigned int semaphore;        // Relative semaphore struct address
  char task_name[11];
  char task_state[10];  	 //"DELAYED", "READY  ", "BLOCKED", "INVALID" 
} tcb[MAX_TASKS], *t;

//New stack for each process
unsigned int stack[MAX_TASKS][256];


//initialization for prioritization
int pritester1;			//Priority scheduling testers
int pritester2;
int pri1;
int pri7;
int pri0;
int pri2;
int pri3;
int pri4;
int pri5;
int pri6;
int a,b,c,d,e,f,K,L;
int pri[MAX_TASKS];
int global_ticks;


void serial_init(int baud_rate){
  // set baud rate
  U1BRG = baud_rate;
  // enable uarts, 8N1, low speed brg
  U1MODE = 0x8000;
  // enable tx and rx
  U1STA = 0x0400;
  IEC0bits.U1RXIE = 1;
}


//-----------------------------------------------------------------------------
// RTOS Kernel                
//-----------------------------------------------------------------------------

void rtos_init(int mode){
  int i;
  rtos_mode = mode;

  //No tasks running
  task_count = 0;

  //Clear out tcb records
  for (i = 0; i < MAX_TASKS; i++){
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0x00000000;
    tcb[i].sp = 0x00000000;
    strcpy(tcb[i].task_state,"INVALID  ");
    strcpy(tcb[i].task_name,"INVALID   ");
  }

  //timer 3 for 1ms system timer
  //used by delay..every ms it'll look to see any TCB entry that is valid, 
  //that is blocked, that is delayed and will dec delay count till it gets to 0.
  T3CONbits.TCS=0;
  TMR3=0;
  PR3=40000;
  T3CONbits.TCKPS=0x00;
  //when it goes to 0, it'll set the state.
}

int rtos_scheduler(){
  // prioritization to 8 levels
  static int ok;
  static int task = 0xFF;
  
  int i;

  int flag; 
  ok = FALSE;

  while(!ok){
	flag=1;
    	task++;
	for(i=0;i<MAX_TASKS-1;i++){
		if(pri[i]!=pri[i+1]){
			flag=0;
			break;
		}
	}
	if(flag==1&&pri[0]==-1){
		for(i=0;i<MAX_TASKS;i++)
		pri[i]=tcb[i].priority;
	}

    	if (task >= MAX_TASKS)
	    	task = 0;

	if(pri[task]==-1)
		task++;
		
	if(task>=MAX_TASKS)
		task=0;

	if(pri[task]>=0){
    		ok = (tcb[task].state == STATE_READY);
		pri[task]--;
	}
	
  }
  return task;
}




char serial_getc(){
  // clear out any overflow error condition
  if (U1STAbits.OERR == 1)
    U1STAbits.OERR = 0;
  // wait until character is ready
  while(!U1STAbits.URXDA);
  return U1RXREG;
}


void serial_puts(char str[]){
    unsigned int i;
     for (i = 0; i <= strlen(str); i++){
    // make sure buffer is empty
        while(U1STAbits.UTXBF);
    // write character
        U1TXREG = str[i];
    }

}


// current PC for all tasks in the TCB every 3 seconds   task name, task pid, task state, task sp, task number 
void serial_tx(){
  int i,j=0;
  char  string[64];
  
  serial_puts("PROCESS   PID   STATE    SP    SEMAPHORE  \n\r");  
  for(i=0;i<64;i++) 
      string[i]=32;
  while(1){ 
	for(i=0;i<task_count;i++){  
     		strcpy(string,tcb[i].task_name);       //name 10
     		snprintf(string+10,5, "%d", tcb[i].pid);        //pid 4
     		strcpy(string+14,"  ");
     		strcpy(string+16, tcb[i].task_state);  // state 9
     		sprintf(string+25, "%d", tcb[i].sp);           // sp 4
     		strcpy(string+29,"  ");
     		//for semaphore
     		if(tcb[i].semaphore != 0){
     			s = tcb[i].semaphore;
     			strcpy(string+31,s->name); 
  	 	}
     		else{    
			strcpy(string+31, "INVALID   ");
     		}
 
     		strcpy(string+41," \n\r");
     
     		for (j = 0; j < 45; j++){
    			// make sure buffer is empty
        		while(U1STAbits.UTXBF);
    			// write character
        		U1TXREG = string[j];
     		}
     

    	}
    	serial_puts("\n\n\r");
  	sleep(3000);
  	serial_puts("\n\n\rPROCESS   PID   STATE    SP    SEMAPHORE  \n\r");
  }

}

int create_process(_fn fn, int priority, char name[], void* p){
  int ok = FALSE;
  int i = 0, k = 0, flag=0;
  int *W,t,u;
  W = 0x1C;
  s = p; 
  int found = FALSE;
  if(rtos_mode == 0){
  
  //IEC1bits.T3IE = 0; 
  //Save starting address if room in task list
  if (task_count < MAX_TASKS){
    
   	//Make sure fn not already in list (prevent reentrancy)    
    	while (!found && (i < MAX_TASKS)){
      		found = (tcb[i++].pid == (unsigned int) fn);
    	}
	
    	if(!found){
      		//Find first available tcb record
      		i = 0;
      		while (tcb[i].state != STATE_INVALID) {i++;}
      		if(i>task_count)
      			flag=1;

      		//Stack filled with initial values to look like task has already run
      		tcb[i].state = STATE_READY;
      		tcb[i].pid = (unsigned int) fn;

      		//The original SP pointer location (depends on init above)
	  	stack[i][0] = (unsigned int)fn;
	  	stack[i][1] = 0;
	  	stack[i][2] = 0;
      		stack[i][3] = &stack[i][3];
	  	stack[i][4] = 0;
	  	stack[i][5] = 0;
	  	stack[i][6] = 0;
      		stack[i][7] = 0;
	  	stack[i][8] = 0;
	  	stack[i][9] = 0;
      
      		tcb[i].sp = &stack[i][10]; 
      		tcb[i].priority = priority;    
      		tcb[i].current_priority = priority;
      		tcb[i].semaphore = s;
      		strcpy(tcb[i].task_name,name);  
      		strcpy(tcb[i].task_state,"READY    ");  
      		
		// increment task count
      		if(flag==1)
      		task_count++;
      		ok = TRUE;
    	}
  }
  }
  else
  {
  	if (task_count < MAX_TASKS){
    		//Make sure fn not already in list (prevent reentrancy)
    		while (!found && (i < MAX_TASKS)){
      			found = (tcb[i++].pid == (unsigned int) fn);
    		}
    		if (!found){
      			// find first available tcb record
      			i = 0;
      			while (tcb[i].state != STATE_INVALID) {i++;}
      			//Seed the stack with initial values to look like task has already run
      			tcb[i].state = STATE_READY;
      			tcb[i].pid = (unsigned int) fn;
      			t = SR;
      			t = (t <<8);
      			u = CORCONbits.IPL3;
      			u = (0x0080 & (u<<7));
      			t = (t|u); 
      
      			//Set the original SP pointer location (depends on init above)
	  		stack[i][0] = (unsigned int)fn;
      			stack[i][1] = t;
	  		stack[i][2] = 0;           //rcount
      			stack[i][3] = 0;           //W0
	  		stack[i][4] = 0;           //W1
	  		stack[i][5] = 0;           //W2
	  		stack[i][6] = 0;           //W3
      			stack[i][7] = 0;           //W4
	  		stack[i][8] = 0;           //W5
	  		stack[i][9] = 0;           //W6
      			stack[i][10] = 0;          //W7
      			stack[i][11] = 0;          //dummy variable
      			stack[i][12] = &stack[i][12];     //W14
      			stack[i][13] = 0;          //TBLPAG 
      			stack[i][14] = 0;          //PSVPAG
      			stack[i][15] = 0;          //DOENDH
 	  		stack[i][16] = 0;          //DOENDL
      			stack[i][17] = 0;          //DOSTARTH
      			stack[i][18] = 0;          //DOSTARTL
      			stack[i][19] = 0;          //DCOUNT 
      			stack[i][20] = 0;          //W13
      			stack[i][21] = 0;          //W12
 	  		stack[i][22] = 0;          //W11
      			stack[i][23] = 0;          //W10
      			stack[i][24] = 0;          //W9
      			stack[i][25] = 0;          //W8
      
      
      			tcb[i].sp = &stack[i][26]; 
      			tcb[i].priority = priority;    
      			tcb[i].current_priority = priority;
      			strcpy(tcb[i].task_name,name);  
      			strcpy(tcb[i].task_state,"READY    ");
      			tcb[i].semaphore = s;  
      			// increment task count
      			task_count++;
      			ok = TRUE;
    		}
  	}
  }
  return ok;
}


void rtos_start(){
  int *SP = (int*)0x1E, *FP = (int*)0x1C, *W = (int*)0x10, j = 0, k = 8;
  task_current = rtos_scheduler();
  if(rtos_mode == MODE_COOPERATIVE){
  	*SP = tcb[task_current].sp;
  	asm("POP W8");
  	asm("POP W9");
  	asm("POP W10");
  	asm("POP W11");
  	asm("POP W12");
  	asm("POP W13");
  	asm("POP W14");
  }
  else
  {
	T5CONbits.TCS=0;
  	TMR5 = 0;
  	PR5 = 40000;
  	T5CONbits.TCKPS = 0;
  	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 1;
  	T5CONbits.TON = 1;
  	*SP = tcb[task_current].sp;
  	asm("POP TBLPAG");
  	asm("POP PSVPAG");
  	asm("POP DOENDH");
  	asm("POP DOENDL");
  	asm("POP DOSTARTH");
  	asm("POP DOSTARTL");
  	asm("POP DCOUNT");
  	asm("POP W8");
  	asm("POP W9");
  	asm("POP W10");
  	asm("POP W11");
  	asm("POP W12");
  	asm("POP W13");
  	asm("POP W14");
  	asm("ulnk");
  	asm("mov.d [--W15], W6");
  	asm("mov.d [--W15], W4");
  	asm("mov.d [--W15], W2");
  	asm("mov.d [--W15], W0");
  	asm("pop RCOUNT");
  	asm("retfie");
  }
}


void init(void* p, int count, char str[]){
  s = p;
  s->count = count;  
  s->queue_size = 0;
  //Name  of the semaphore stored in the name character array
  strcpy(s->name,str);            
}

//Dispatcher: Function to yield execution back to scheduler
//Push registers, call scheduler, pop registers, return to new function
void yield(){
  if(rtos_mode == 0){
  	asm("PUSH W14");
  	asm("PUSH W13");
 	asm("PUSH W12");
  	asm("PUSH W11");
  	asm("PUSH W10");
  	asm("PUSH W9");
  	asm("PUSH W8");
  	tcb[task_current].sp=*Z;
  	task_current=rtos_scheduler();
  	*Z=tcb[task_current].sp;
  	asm("POP W8");
  	asm("POP W9");
  	asm("POP W10");
  	asm("POP W11");
  	asm("POP W12");
  	asm("POP W13");
  	asm("POP W14");
  	asm("ulnk");
  	asm("return");
  }
  else{ 
  	IFS1bits.T5IF = 0;
   	IEC1bits.T5IE = 0;
  	T5CONbits.TON = 0;
	asm("mov SR, W0");
  	asm("sl W0,#8,W0");
  	asm("mov CORCON,W1");
  	asm("LSR W1,W1");
  	asm("LSR W1,W1");
  	asm("LSR W1,W1");
  	asm("mov #01,W3");
  	asm("AND W1,W3,W1");
  	asm("sl W1,#7,W1");
  	asm("IOR W1,W0,W0"); 
  	asm("mov W14,W15");
  	asm("dec W15,W15");
  	asm("dec W15,W15");
  	asm("mov W0,[W15++]");
  	asm("push RCOUNT");
  	asm("mov.d  W0,[W15++]");
  	asm("mov.d  W2,[W15++]");
  	asm("mov.d  W4,[W15++]");
  	asm("mov.d  W6,[W15++]");
  	asm("mov #0x0,W8"); 
  	asm("mov W8,[W15++]");
  	asm("mov W15,[W15++]");
 	 asm("PUSH W13");
  	asm("PUSH W12");
  	asm("PUSH W11");
  	asm("PUSH W10");
  	asm("PUSH W9");
  	asm("PUSH W8");
  	asm("PUSH DCOUNT");
  	asm("PUSH DOSTARTL");
  	asm("PUSH DOSTARTH");
  	asm("PUSH DOENDL");
  	asm("PUSH DOENDH");
  	asm("PUSH PSVPAG");
  	asm("PUSH TBLPAG");
  	Z =  0x1E;
  	tcb[task_current].sp = *Z;
  	task_current = rtos_scheduler();
  	*Z = tcb[task_current].sp;
  	asm("POP TBLPAG");
  	asm("POP PSVPAG");
  	asm("POP DOENDH");
  	asm("POP DOENDL");
  	asm("POP DOSTARTH");
  	asm("POP DOSTARTL");
  	asm("POP DCOUNT");
  	asm("POP W8");
  	asm("POP W9");
  	asm("POP W10");
  	asm("POP W11");
  	asm("POP W12");
  	asm("POP W13");
  	asm("POP W14");
  	asm("ulnk");
  	asm("mov.d [--W15], W6");
  	asm("mov.d [--W15], W4");
  	asm("mov.d [--W15], W2");
  	asm("mov.d [--W15], W0"); 
  	asm("pop RCOUNT");
  	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 1;
  	T5CONbits.TON = 1;
  	asm("retfie"); 
  }
	
}

//Function to support 1ms system timer
//Execution yielded back to scheduler until time elapses
//Push registers, set state to delayed, store timeout, call scheduler, pop registers, 
//return to new function

void sleep(global_ticks){
  if(rtos_mode == 0){
  	asm("PUSH W14");
  	asm("PUSH W13");
  	asm("PUSH W12");
  	asm("PUSH W11");
  	asm("PUSH W10");
  	asm("PUSH W9");
  	asm("PUSH W8");
  	tcb[task_current].sp=*Z;
  	tcb[task_current].state = STATE_DELAYED;
  	tcb[task_current].ticks=global_ticks;
  	strcpy(tcb[task_current].task_state,"DELAYED  ");  
  	IFS0bits.T3IF=0;
  	IEC0bits.T3IE = 1;
  	T3CONbits.TON=1;  

  	task_current=rtos_scheduler();
 
  	*Z=tcb[task_current].sp;
  	asm("POP W8");
  	asm("POP W9");
  	asm("POP W10");
  	asm("POP W11");
  	asm("POP W12");
  	asm("POP W13");
  	asm("POP W14");
  	asm("ulnk");
  	asm("return");
  }
  else{
  	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 0;
  	T5CONbits.TON = 0;
   	asm("mov SR, W0");
  	asm("sl W0,#8,W0");
  	asm("mov CORCON,W1");
  	asm("LSR W1,W1");
  	asm("LSR W1,W1");
  	asm("LSR W1,W1");
  	asm("mov #01,W3");
  	asm("AND W1,W3,W1");
  	asm("mov 0xff00,W3");
  	asm("AND W0,W3,W0");
  	asm("sl W1,#7,W1");
  	asm("IOR W1,W0,W0");
        asm("mov W14,W15");
  	asm("dec W15,W15");
  	asm("dec W15,W15");
 
  	asm("mov W0,[W15++]");
  	asm("push RCOUNT");
  	asm("mov.d  W0,[W15++]");
  	asm("mov.d  W2,[W15++]");
  	asm("mov.d  W4,[W15++]");
  	asm("mov.d  W6,[W15++]");
  	asm("mov #0x0,W8"); 
  	asm("mov W8,[W15++]");
  	asm("mov W15,[W15++]");

  	asm("PUSH W13");
  	asm("PUSH W12");
  	asm("PUSH W11");
  	asm("PUSH W10");
  	asm("PUSH W9");
  	asm("PUSH W8");
  	asm("PUSH DCOUNT");
  	asm("PUSH DOSTARTL");
  	asm("PUSH DOSTARTH");
  	asm("PUSH DOENDL");
  	asm("PUSH DOENDH");
  	asm("PUSH PSVPAG");
  	asm("PUSH TBLPAG");
  	tcb[task_current].sp = *Z;
  	tcb[task_current].state = STATE_DELAYED;
  	tcb[task_current].ticks = global_ticks;
  	strcpy(tcb[task_current].task_state,"DELAYED  ");  
  	task_current = rtos_scheduler();
  	*Z=tcb[task_current].sp;
  	asm("POP TBLPAG");
  	asm("POP PSVPAG");
  	asm("POP DOENDH");
  	asm("POP DOENDL");
  	asm("POP DOSTARTH");
  	asm("POP DOSTARTL");
  	asm("POP DCOUNT");
  	asm("POP W8");
  	asm("POP W9");
  	asm("POP W10");
  	asm("POP W11");
  	asm("POP W12");
  	asm("POP W13");
  	asm("POP W14");
  	asm("ulnk");
  	asm("mov.d [--W15], W6");
  	asm("mov.d [--W15], W4");
  	asm("mov.d [--W15], W2");
  	asm("mov.d [--W15], W0");
  	asm("pop RCOUNT");
  
  	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 1;
  	T5CONbits.TON = 1;
  	asm("retfie");
  }
  
}

//Manage blocked processes awaiting sleep() completion

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void){
  T3CONbits.TON=0;
  IFS0bits.T3IF=0;
  IEC0bits.T3IE = 0;
  int i=0,flag=0;
  for(i=0;i<MAX_TASKS;i++){
  	if(tcb[i].state == STATE_DELAYED){
     		tcb[i].ticks--;
     		flag++; 
     		if(tcb[i].ticks==0){
         		flag--;
         		tcb[i].state = STATE_READY;
         		strcpy(tcb[i].task_state,"READY    ");
    		}
   	}
  } 
  
   if(flag!=0){
      TMR3=0;
      PR3=40000;      
      IFS0bits.T3IF=0;
      IEC0bits.T3IE = 1;
      T3CONbits.TON=1;
     
   }
   else{
     T3CONbits.TON=0;
     IFS0bits.T3IF=0;
     IEC0bits.T3IE = 0;
   }
 
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void)
{	
    rxc = U1RXREG;
    signal(&char_received);
    rx = 0;
    IFS0bits.U1RXIF=0; //clear flag
}

//Function to wait a semaphore with priority inheritance 
//Return if avail, else yield to scheduler

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt (void){


  IFS1bits.T5IF = 0;
  IEC1bits.T5IE = 0;
  T5CONbits.TON = 0;
  
  TMR5 = 0;
  PR5 = 40000;
  asm("PUSH W14");
  asm("PUSH W13");
  asm("PUSH W12");
  asm("PUSH W11");
  asm("PUSH W10");
  asm("PUSH W9");
  asm("PUSH W8");
  asm("PUSH DCOUNT");
  asm("PUSH DOSTARTL");
  asm("PUSH DOSTARTH");
  asm("PUSH DOENDL");
  asm("PUSH DOENDH");
  asm("PUSH PSVPAG");
  asm("PUSH TBLPAG");
  Z = 0x1E;
  tcb[task_current].sp=*Z;

  for(L=0;L<task_count;L++){
   	if(tcb[L].state == STATE_DELAYED){
     		tcb[L].ticks--;
      		if(tcb[L].ticks == 0){
         	        tcb[L].state = STATE_READY;
         		strcpy(tcb[L].task_state,"READY    ");
    		}
   	}
  }

  task_current=rtos_scheduler();

  *Z=tcb[task_current].sp;
  asm("POP TBLPAG");
  asm("POP PSVPAG");
  asm("POP DOENDH");
  asm("POP DOENDL");
  asm("POP DOSTARTH");
  asm("POP DOSTARTL");
  asm("POP DCOUNT");
  asm("POP W8");
  asm("POP W9");
  asm("POP W10");
  asm("POP W11");
  asm("POP W12");
  asm("POP W13");
  asm("POP W14");
  IFS1bits.T5IF = 0;
  IEC1bits.T5IE = 1;
  T5CONbits.TON = 1;
 }


void wait(void* p){
  s = p;
  if(rtos_mode == 0){
  	asm("PUSH W14");
  	asm("PUSH W13");
  	asm("PUSH W12");
  	asm("PUSH W11");
  	asm("PUSH W10");
  	asm("PUSH W9");
  	asm("PUSH W8");
  	tcb[task_current].sp = *Z;
  	if( s->count > 0 ){
      		s->count--;
      		*Z = tcb[task_current].sp;
      		asm("POP W8");
  	  	asm("POP W9");
  	  	asm("POP W10");
      		asm("POP W11");
      		asm("POP W12");
      		asm("POP W13");
      		asm("POP W14");
      		asm("ulnk");
      		asm("return");
   	}
   	else{   
      		tcb[task_current].state = STATE_BLOCKED;
      		strcpy(tcb[task_current].task_state,"BLOCKED  ");
      		s->process_queue[s->queue_size] = tcb[task_current].pid;
      		s->queue_size++;             //post increment
      		task_current = rtos_scheduler();
     		*Z = tcb[task_current].sp;
      		asm("POP W8");
      		asm("POP W9");
      		asm("POP W10");
      		asm("POP W11");
      		asm("POP W12");
      		asm("POP W13");
      		asm("POP W14");
      		asm("ulnk");
      		asm("return");
	
  	}
  }
  else{
	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 0;
  	T5CONbits.TON = 0;
   	asm("mov SR, W0");
  	asm("sl W0,#8,W0");
  	asm("mov CORCON,W1");
  	asm("LSR W1,W1");
  	asm("LSR W1,W1");
  	asm("LSR W1,W1");
  	asm("mov #01,W3");
  	asm("AND W1,W3,W1");
  	asm("sl W1,#7,W1");
  	asm("IOR W1,W0,W0");
  	asm("mov W14,W15");
  	asm("dec W15,W15");
  	asm("dec W15,W15");
  	asm("mov W0,[W15++]");
  	asm("push RCOUNT");
  	asm("mov.d  W0,[W15++]");
  	asm("mov.d  W2,[W15++]");
  	asm("mov.d  W4,[W15++]");
  	asm("mov.d  W6,[W15++]");
  	asm("mov #0x0,W8"); 
  	asm("mov W8,[W15++]");
  	asm("mov W15,[W15++]");
	asm("PUSH W13");
  	asm("PUSH W12");
  	asm("PUSH W11");
  	asm("PUSH W10");
  	asm("PUSH W9");
  	asm("PUSH W8");
  	asm("PUSH DCOUNT");
  	asm("PUSH DOSTARTL");
  	asm("PUSH DOSTARTH");
  	asm("PUSH DOENDL");
  	asm("PUSH DOENDH");
  	asm("PUSH PSVPAG");
  	asm("PUSH TBLPAG");
  	tcb[task_current].sp = *Z;
  	if( s->count > 0 ){
      		s->count--;
      		*Z = tcb[task_current].sp;
       		asm("POP TBLPAG");
       		asm("POP PSVPAG");
       		asm("POP DOENDH");
       		asm("POP DOENDL");
       		asm("POP DOSTARTH");
       		asm("POP DOSTARTL");
       		asm("POP DCOUNT");
       		asm("POP W8");
       		asm("POP W9");
       		asm("POP W10");
  		asm("POP W11");
  		asm("POP W12");
  		asm("POP W13");
  		asm("POP W14");
  		asm("ulnk");
  		asm("mov.d [--W15], W6");
  		asm("mov.d [--W15], W4");
  		asm("mov.d [--W15], W2");
  		asm("mov.d [--W15], W0");
  		asm("pop RCOUNT");
  		IFS1bits.T5IF = 0;
  		IEC1bits.T5IE = 1;
  		T5CONbits.TON = 1;
  		asm("retfie");
       	}
   	else{   
      		tcb[task_current].state = STATE_BLOCKED;
      		strcpy(tcb[task_current].task_state,"BLOCKED  ");
      		s->process_queue[s->queue_size] = tcb[task_current].pid;
      		s->queue_size++;             //post increment
      		task_current = rtos_scheduler();
     		*Z = tcb[task_current].sp;
       		asm("POP TBLPAG");
  		asm("POP PSVPAG");
  		asm("POP DOENDH");
  		asm("POP DOENDL");
  		asm("POP DOSTARTH");
  		asm("POP DOSTARTL");
  		asm("POP DCOUNT");
  		asm("POP W8");
  		asm("POP W9");
  		asm("POP W10");
  		asm("POP W11");
  		asm("POP W12");
  		asm("POP W13");
 		asm("POP W14");
  		asm("ulnk");
  		asm("mov.d [--W15], W6");
  		asm("mov.d [--W15], W4");
  		asm("mov.d [--W15], W2");
  		asm("mov.d [--W15], W0");
  		asm("pop RCOUNT");
  		IFS1bits.T5IF = 0;
  		IEC1bits.T5IE = 1;
  		T5CONbits.TON = 1;
  		asm("retfie");
	}
  }
}

//Function to signal a semaphore is available
void signal(void* p){
  unsigned int v;
  int i;
  s=p;
 
  if(rtos_mode == 0){
  	s->count++;
  	if(s->count==1){
      		s->queue_size--;                 //predecrement
   		v=s->process_queue[s->queue_size];
    		s->process_queue[s->queue_size] = 0; // for safety in case of delete process 
           	for(i=0;i<10;i++){
   			if(tcb[i].pid == v)
			break;
    		}        
		tcb[i].state = STATE_READY;
    		strcpy(tcb[i].task_state,"READY    ");
       	}  
  }
  else{
	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 0;
  	T5CONbits.TON = 0;
  	IEC0bits.U1RXIE = 0;
  	IFS0bits.U1RXIF=0;
  
  	int i;
  	s=p;
  	s->count++;
  	if(s->count==1){
      		s->queue_size--;
   		K=s->process_queue[s->queue_size];
           	for(i=0;i<10;i++){
   			if(tcb[i].pid == K) 
				break;
		      
    		}        
			
    		tcb[i].state = STATE_READY;
    		strcpy(tcb[i].task_state,"READY    ");
    	}

   	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 1;
  	T5CONbits.TON = 1;
  	IEC0bits.U1RXIE = 1;
  	IFS0bits.U1RXIF=0;  
  }
}

void delete_process(_fn fn){
  int i,v;
  if(rtos_mode == 0){
  	//switching timer 3 off temporarily, to avoid the process to be deleted from entering sleep 
  	T3CONbits.TON = 0;
  	IFS0bits.T3IF = 0;
  	IEC0bits.T3IE = 0;
 
	//the following loop will search for all the possible entries of the task to be deleted and delete them
 	for(i = 0;i < task_count;i++ ){
      		if(tcb[i].pid == (unsigned int)fn){
          		tcb[i].state = STATE_INVALID;     //marking it invalid, incase it is delayed or blocked 
          		tcb[i].ticks = 0;                 //ticks are -1 to avoid the process from being scheduled via sleep
          		tcb[i].pid = 0; 
          		tcb[i].priority = 0; 
          		tcb[i].sp = 0;                     //pid and sp are marked zero   
          		strcpy(tcb[i].task_state,"INVALID  "); //state_name and task_name as they wil appear in thread dumping
          		strcpy(tcb[i].task_name,"INVALID   ");     
          		s = tcb[i].semaphore;
          		if(s != 0){
          			s->count = 0; 
          			v = s->queue_size;
          			for(i=0;i < v;i++){
          				s->queue_size--; 
          				s->process_queue[s->queue_size] = 0;
          			}
          		}
          		tcb[i].semaphore = 0;
          		break; 
	  	}
  	}
  	IFS0bits.T3IF = 0;
  	IEC0bits.T3IE = 1;
  	T3CONbits.TON = 1;
  }
  else{
	//switching timer 3 off temporarily, to avoid the process to be deleted from entering sleep 
  	T3CONbits.TON = 0;
  	IFS0bits.T3IF = 0;
  	IEC0bits.T3IE = 0;
 
	//the following loop will search for all the possible entries of the task to be deleted and delete them
 	for(i = 0;i < task_count;i++ ){
      		if(tcb[i].pid == (unsigned int)fn){
          		tcb[i].state = STATE_INVALID;     //marking it invalid, incase it is delayed or blocked 
         	 	tcb[i].ticks = 0;                 //ticks are -1 to avoid the process from being scheduled via sleep
          		tcb[i].pid = 0; 
          		tcb[i].priority = 0; 
          		tcb[i].sp = 0;                     //pid and sp are marked zero   
          		strcpy(tcb[i].task_state,"INVALID  "); //state_name and task_name as they wil appear in thread dumping
          		strcpy(tcb[i].task_name,"INVALID   ");     
          		s = tcb[i].semaphore;
          		if(s != 0){
          			s->count = 0; 
          			v = s->queue_size;
          			for(i=0;i < v;i++){
          				s->queue_size--; 
          				s->process_queue[s->queue_size] = 0;
          			}
          		}
          		tcb[i].semaphore = 0;
          		break; 
	  	}
          
  	}
  	IFS1bits.T5IF = 0;
  	IEC1bits.T5IE = 1;
  	T5CONbits.TON = 1;
  }

          
}

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void init_hw(){
  AD1PCFGLbits.PCFG4 = 1;                    // make selected pins digital
  AD1PCFGLbits.PCFG5 = 1;
  LATAbits.LATA4 = 0;                        // write 0 into output latches
  LATBbits.LATB3 = 0;
  LATBbits.LATB4 = 0;
  LATBbits.LATB5 = 0;
  TRISAbits.TRISA4 = 0;                      // led pins outputs
  TRISBbits.TRISB3 = 0;
  TRISBbits.TRISB4 = 0;
  TRISBbits.TRISB5 = 0;
  CNPU1bits.CN11PUE = 1;                     // enable pull-ups for push buttons
  CNPU1bits.CN12PUE = 1;
  CNPU1bits.CN13PUE = 1;
  CNPU1bits.CN14PUE = 1;
  PLLFBDbits.PLLDIV = 38;                    // pll feedback divider = 40;
  CLKDIVbits.PLLPRE = 0;                     // pll pre divider = 2
  CLKDIVbits.PLLPOST = 0;
  RPINR18bits.U1RXR = 11;                    // assign U1RX to RP11
  RPOR5bits.RP10R = 3;                       // assign U1TX to RP10
                    
}





// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose

void idle(){
  while(TRUE) 
  { 

    PIN_ORANGE = 1;  
    __delay_ms(1);
    PIN_ORANGE = 0;
    yield();
  }
}


void one_shot()
{
  while(TRUE)
  {
    wait(&flash_req);
    PIN_YELLOW = 1;
    sleep(1000);
    PIN_YELLOW = 0;
  }
}

void flash_4hz(){
  while(TRUE){
    PIN_GREEN ^= 1;
    sleep(125);
  }
}

void rx_process()
{	
	while(1){	
		wait(&char_received); //wait for character
		if( rx == 0){ 
        		serial_puts("\n\rCharacter received\n\r");
        		rx = 1;
        	}
        	yield();
	}
}


void part_of_lengthy_fn(){
  // represent some lengthy operation
  __delay_ms(1);
  // give another process a chance
  yield();
}

void lengthy_fn()
{
  long i;
  while(TRUE){
    for (i = 0; i < 4000; i++){
      	part_of_lengthy_fn();
    }
    PIN_RED ^= 1;
  }
}


int read_pbs()
{
  return (~PORTB >> 12);
}


void read_keys(){
  int buttons;
  while(TRUE){
  	wait(&key_released);      //key released intialized by 1
    	buttons = 0;   
        while (buttons == 0){
      		buttons = read_pbs();
      		yield();
   	}
    	signal(&key_pressed);
    	if ((buttons & 1) != 0){          //push button 23
      		PIN_YELLOW ^= 1;
      		PIN_RED = 1;
    	}
    	if ((buttons & 2) != 0){          //push button 24
    	    	signal(&flash_req);
      		PIN_RED = 0;
    	}
    	if ((buttons & 4) != 0){           //push button 25
    	    create_process(flash_4hz, 7,"FLASH_4   ",0);
    	}
    	if((buttons & 8) != 0){         //push button 26
    	    delete_process(flash_4hz);
	}
	yield();
  }
}

void debounce(){
  int count;
  while(TRUE){
    	wait(&key_pressed);
    	count = 10;
    	while (count != 0){  
      		sleep(10);
      		if (read_pbs() == 0)
        		count--;
      		else
        		count = 10;
    	}
    	signal(&key_released);
  }
}

void uncooperative(){
  while(TRUE){
    while (read_pbs() == 8){}
    yield();
  }
}







int main(void){
  int ok ;
  int pb,i;
 

  //Initialize hardware
  init_hw();  
  serial_init(BAUD_57600);                   // configure uart

 //Power-up flash
  PIN_RED = 1;
  __delay32(10000000);
  PIN_RED = 0;
  __delay32(10000000);  
 serial_puts("Ready..!\n\r");

  //Init semaphores
  init(&key_pressed, 0, "KEY_PR    ");
  init(&key_released, 1, "KEY_RL    ");
  init(&flash_req, 5, "FLASH_REQ ");
  init(&char_received, 0 ,"CHAR_RX   ");
  
  // initialize selected RTOS
  ok = FALSE;
  while (!ok){
   	pb = read_pbs();
  	if (pb & 4){
	      ok = TRUE;
      		rtos_init(MODE_COOPERATIVE);
  	}
  	if (pb & 8){
     		ok = TRUE;
     		rtos_init(MODE_PREEMPTIVE);
  	}
  }
 

  ok =  create_process(idle, 0, "IDLE      ",0) >= 0;
  ok &= create_process(flash_4hz, 7, "FLASH_4   ",0) >= 0;
  ok &= create_process(lengthy_fn, 1, "LNGTHY_F  ",0) >= 0;
  ok &= create_process(one_shot, 4, "1SHOT     ",&flash_req) >= 0;
  ok &= create_process(read_keys, 6,"READ_KEY  ",&key_released) >= 0;
  ok &= create_process(debounce, 4, "DEBOUNCE  ", &key_pressed) >= 0;
  ok &= create_process(uncooperative, 2, "UNCO-OP   ",0) >= 0;
  ok &= create_process(rx_process, 0, "CHAR_RX   ",&char_received) >= 0;
  ok &=  create_process(serial_tx, 4, "SERL_TX   ",0) >= 0;
 
  // start up rtos
  if (ok) 
    rtos_start(); // never returns
  else
    PIN_RED = 1;

  return 0;
  // unreachable code
  // if a function is only called once in your code, it will be
  // accessed with two goto instructions instead of call-return,
  // so any stack-based code will not function correctly
  yield(); sleep(0); wait(0); signal(0);
}

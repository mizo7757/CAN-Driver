//0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000//

                                           /*Moaz Ayman Mokhtar
                                         This is my first driver
                              This driver is for TM4C123GH6PM micro controller
                                Under the supervision of Dr. Youssif Awny*/

//0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000//


#include "CAN Header.h"

void PortB_CAN0_Init(uint8_t Mode) {

	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_CAN0;                 //CAN0 clock enable
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;                //GPIOB clock enable

	/*Initializing PB4 and PB5 pins for CAN0 module*/
	/* PB4=Rx and PB5=Tx */

	GPIO_PORTB_LOCK_R = magicKey;                        //unlocking port
	GPIO_PORTB_AFSEL_R |= 0x30;                          //using alternating function for B4 AND B5
	GPIO_PORTB_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFF00FFFF) + 0x00880000;     //Choosing CAN from PCTL table
	GPIO_PORTB_AMSEL_R &= 0xCF;                          //Disabling analog function for B4 and B5
	GPIO_PORTB_DEN_R |= 0x30;                            //Enabling digital port

	CAN0_CTL_R |= CAN_CTL_INIT;                          //Initialize CAN0 module
	while (CAN0_IF1CRQ_R & CAN_IF1CRQ_BUSY);             //Wait till CAN1 module is ready , this will set a Flag

	if (Mode == Test_Mode) {
		CAN0_CTL_R |= CAN_CTL_TEST;                      //Enabling Test Mode
		CAN0_TST_R |= CAN_TST_LBACK;                     //Enabling Loopback Mode
	}
	else {
		CAN0_CTL_R &= (~(CAN_CTL_TEST));                 //Disabling Test Mode
		CAN0_TST_R &= (~(CAN_TST_LBACK));                //Disabling Loopback Mode
	}
}


void PortE_CAN0_Init(uint8_t Mode) {

		SYSCTL_RCGC0_R |= SYSCTL_RCGC0_CAN0;                 //CAN0 clock enable
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;                //GPIOE clock enable

		/*Initializing PE4 and PE5 pins for CAN0 module*/
		/* PE4=Rx and PE5=Tx */

		GPIO_PORTE_LOCK_R = magicKey;                        //unlocking port
		GPIO_PORTE_AFSEL_R |= 0x30;                          //using alternating function for E4 AND E5
		GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & 0xFF00FFFF) + 0x00880000;     //Choosing CAN from PCTL table
		GPIO_PORTE_AMSEL_R &= 0xCF;                          //Disabling analog function for E4 and E5
		GPIO_PORTE_DEN_R |= 0x03;                            //Enabling digital port

		CAN0_CTL_R |= CAN_CTL_INIT;                          //Initialize CAN0 module
		while (CAN0_IF1CRQ_R & CAN_IF1CRQ_BUSY);             //Wait till CAN1 module is ready , this will set a Flag

		if (Mode == Test_Mode) {
			CAN0_CTL_R |= CAN_CTL_TEST;                      //Enabling Test Mode
			CAN0_TST_R |= CAN_TST_LBACK;                     //Enabling Loopback Mode
		}
		else {
			CAN0_CTL_R &= (~(CAN_CTL_TEST));                 //Disabling Test Mode
			CAN0_TST_R &= (~(CAN_TST_LBACK));                //Disabling Loopback Mode
		}
}


void PortF_CAN0_Init(uint8_t Mode) {

	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_CAN0;                 //CAN0 clock enable
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;                //GPIOF clock enable

	/*Initializing PF0 and PF3 pins for CAN0 module*/
	/* PF0=Rx and PF3=Tx */

	GPIO_PORTF_LOCK_R = magicKey;                        //unlocking port
	GPIO_PORTF_AFSEL_R |= 0x09;                          //using alternating function for F0 AND F3
	GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R & 0xFFFF0FF0) + 0x00003003;     //Choosing CAN from PCTL table
	GPIO_PORTF_AMSEL_R &= 0xF6;                          //Disabling analog function for F0 and F3
	GPIO_PORTF_DEN_R |= 0x03;                            //Enabling digital port

	CAN0_CTL_R |= CAN_CTL_INIT;                          //Initialize CAN0 module
	while (CAN0_IF1CRQ_R & CAN_IF1CRQ_BUSY);             //Wait till CAN1 module is ready , this will set a Flag

	if (Mode == Test_Mode) {
		CAN0_CTL_R |= CAN_CTL_TEST;                      //Enabling Test Mode
		CAN0_TST_R |= CAN_TST_LBACK;                     //Enabling Loopback Mode
	}
	else {
		CAN0_CTL_R &= (~(CAN_CTL_TEST));                 //Disabling Test Mode
		CAN0_TST_R &= (~(CAN_TST_LBACK));                //Disabling Loopback Mode
	}
}


void CAN1_Init(uint8_t Mode) {

	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_CAN1;                 //CAN1 clock enable
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;                //GPIOA clock enable

	 /*Initializing PA0 and PA1 pins for CAN1 module*/
	 /* PA0=Rx and PA1=Tx */

	GPIO_PORTA_LOCK_R = magicKey;                        //unlocking port
	GPIO_PORTA_AFSEL_R |= 0x03;                          //using alternating function for A0 and A1
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) + 0x00000088;     //Choosing CAN from PCTL table
	GPIO_PORTA_AMSEL_R &= 0xFC;                          //Disabling analog function for A0 and A1
	GPIO_PORTA_DIR_R |= 0x02;                            //Setting the pin direction
	GPIO_PORTA_DEN_R |= 0x03;                            //Enabling digital port

	CAN1_CTL_R |= CAN_CTL_INIT;                          //Initialize CAN1 module
	while (CAN1_IF1CRQ_R & CAN_IF1CRQ_BUSY);             //Wait till CAN1 module is ready , this will set a Flag

	if (Mode == Test_Mode) {
		CAN1_CTL_R |= CAN_CTL_TEST;                      //Enabling Test Mode
		CAN1_TST_R |= CAN_TST_LBACK;                     //Enabling Loopback Mode
	}
	else {
		CAN1_CTL_R &= (~(CAN_CTL_TEST));                 //Disabling Test Mode
		CAN1_TST_R &= (~(CAN_TST_LBACK));                //Disabling Loopback Mode
	}
}


void EnableCAN0() {

	CAN0_CTL_R &= (~(CAN_CTL_INIT));                    //Exiting initialization mode in CAN0 module for transmit and recieve starting
}


void EnableCAN1() {

	CAN1_CTL_R &= (~(CAN_CTL_INIT));                    //Exiting initialization mode in CAN1 module for transmit and recieve starting
}


void DisableCAN0() {

	SYSCTL_RCGC0_R &= (~(SYSCTL_RCGC0_CAN0));                 //CAN0 clock disable
}


void DisableCAN1() {

	SYSCTL_RCGC0_R &= (~(SYSCTL_RCGC0_CAN1));                 //CAN1 clock disable
}


void SetCAN0_BitTime(CANBitParms* Parms) {

	CAN0_CTL_R |= CAN_CTL_CCE;                               //Enabling configuration change
	CAN0_BIT_R &= 0x0000;                                    //Clearing CANBIT register

	CAN0_BIT_R |= (Parms -> prescaler - 1);                  //Setting Baud Rate Prescaler
	CAN0_BIT_R |= ((Parms -> SJW - 1) << 6);                 //Setting Synchronization Jump Width
    CAN0_BIT_R |= ((Parms -> PropPhase1Seg - 1) << 8);       //Phase Buffer Segment 1 + Propagation Segment
	CAN0_BIT_R |= ((Parms->Phase2Seg - 1) << 12);            //Setting Phase Buffer Segment 2

	CAN0_CTL_R &= (~(CAN_CTL_CCE));                          //Disabling configuration change
}


void SetCAN1_BitTime(CANBitParms* Parms) {

	CAN1_CTL_R |= CAN_CTL_CCE;                               //Enabling configuration change
	CAN1_BIT_R &= 0x0000;                                    //Clearing CANBIT register

	CAN1_BIT_R |= (Parms->prescaler - 1);                    //Setting Baud Rate Prescaler
	CAN1_BIT_R |= ((Parms->SJW - 1) << 6);                   //Setting Synchronization Jump Width
	CAN1_BIT_R |= ((Parms->PropPhase1Seg - 1) << 8);         //Phase Buffer Segment 1 + Propagation Segment
	CAN1_BIT_R |= ((Parms->Phase2Seg - 1) << 12);            //Setting Phase Buffer Segment 2

	CAN1_CTL_R &= (~(CAN_CTL_CCE));                          //Disabling configuration change
}


void EnableCAN0_Int(uint32_t flags) {

	NVIC_EN1_R |= 0x0080;                                     //Enabling interrupt in NVIC
	CAN0_CTL_R |= flags;                                      //Setting CAN0_CTL_R register flags
}


void EnableCAN1_Int(uint32_t flags) {

	NVIC_EN1_R |= 0x0100;                                     //Enabling interrupt in NVIC
	CAN1_CTL_R |= flags;                                      //Setting CAN1_CTL_R register flags
}


void DisableCAN0_Int(uint32_t flags) {

	CAN0_CTL_R &= (~(flags));                                 //Clearing CAN0_CTL_R register flags
}


void DisableCAN1_Int(uint32_t flags) {

	CAN1_CTL_R &= (~(flags));                                 //Clearing CAN1_CTL_R register flags
}


void ClearCAN0_Int( uint32_t Int_Clear) {

	CAN0_INT_R &= (~(Int_Clear));                            //Clearing INTID bit field for CAN0
}


void CANIntClear(uint32_t Int_Clear) {

	CAN1_INT_R &= (~(Int_Clear));                            //Clearing INTID bit field for CAN1
}


uint32_t ReadCAN0_IntSTS(uint8_t choice) {

	uint32_t status;

	if (choice == 0x01) {
		status = CAN0_INT_R;                                 //Reading the highest priority pending interrupt 
	}
	else {
		status = (CAN0_MSG1INT_R & 0xFFFF);
		status |= (CAN0_MSG2INT_R & 0xFFFF) << 16 );         //Reading all messege objects pending interrupts
	}
	return status;
}


uint32_t ReadCAN1_IntSTS(uint8_t choice) {

	uint32_t status;

	if (choice == 0x01) {
		status = CAN1_INT_R;                                 //Reading the highest priority pending interrupt 
	}
	else {
		status = (CAN1_MSG1INT_R & 0xFFFF);
		status |= (CAN1_MSG2INT_R & 0xFFFF) << 16 );         //Reading all messege objects pending interrupts
	}
	return status;
}


void CAN0_msgObj_transmit_config(uint32_t Msg_ID, uint32_t Ex_flag, MsgObj_ID Obj_ID) {

	CANFrame_Type type = Standard_Frame;                     //Setting standard frame type as default

	CAN0_IF1ARB2_R &= (~(CAN_IF1ARB2_MSGVAL));               //Ignoring msg obj

	if ((Msg_ID < CAN_MAX_EXTENDED_ID) && (Msg_ID > CAN_MAX_STANDARD_ID) && (Ex_flag & CAN_IF1ARB2_XTD)) {  //Checking the frame type

		type = Extended_Frame;                               //Setting Extended frame as CAN frame type
	}
	else if (Msg_ID > CAN_MAX_EXTENDED_ID) {

		return;                                              //Invalid messege ID
	}

	CAN0_IF1CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_ARB;                       //Enable transfere arbitration bits to interface registers
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2

	CAN0_IF1MSK2_R |= CAN_IF2MSK2_MDIR;                      //Enabling the effect of DIR bit for acceptance filtering
	CAN0_IF1ARB2_R |= CAN_IF1ARB2_DIR;                       //Setting the direction as transmit
	CAN0_IF1ARB2_R |= CAN_IF1ARB2_MSGVAL;                    //Setting as valid messege

	if (type == Standard_Frame) {                            //For standard Identifier

		CAN0_IF1ARB2_R &= (~(CAN_IF1ARB2_XTD));              //Indicating a standard frame
		CAN0_IF1MSK2_R &= (~(CAN_IF1MSK2_MXTD));             //Disabling the effect of extended frame for acceptance filtering
		CAN0_IF1ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field of ARB2
		CAN0_IF1ARB2_R |= (Msg_ID << 2);                     //Writing the msg ID in bits [12:2] in the ID bit field
	}

	else if (type == Extended_Frame) {                       //For Extended Identifier

		CAN0_IF1ARB2_R |= CAN_IF1ARB2_XTD;                   //Indicating an extended frame
		CAN0_IF1MSK2_R |= CAN_IF1MSK2_MXTD                   //Enabling the effect of extended frame for acceptance filtering
		CAN0_IF1ARB1_R &= (~(0xFFFF));                       //Clearing the ID bit field of ARB1
		CAN0_IF1ARB1_R |= (Msg_ID & 0x0000FFFF);             //Writing the lower 16 bits of messege in ID bit field of ARB1
		CAN0_IF1ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field ARB2
		CAN0_IF1ARB2_R |= ((Msg_ID & 0x1FFF0000) >> 16);     //Writing the upper 13 bits of messege in ID bit field of ARB2
	}

	if (Ex_flag & CAN_IF1MCTL_TXIE) {

		CAN0_IF1MCTL_R |= CAN_IF1MCTL_TXIE;				     //Enabling interrupt after a successful transmission
	}

		CAN0_IF1MCTL_R |= CAN_IF1MCTL_EOB;                   //Setting End of buffer bit field

		CAN0_IF1CRQ_R = Obj_ID;                              //Writing the messege object ID in the MNUM bit field
}


void CAN1_msgObj_transmit_config(uint32_t Msg_ID, uint32_t Ex_flag, MsgObj_ID Obj_ID) {

	CANFrame_Type type = Standard_Frame;                     //Setting standard frame type as default

	CAN1_IF1ARB2_R &= (~(CAN_IF1ARB2_MSGVAL));               //Ignoring msg obj

	if ((Msg_ID < CAN_MAX_EXTENDED_ID) && (Msg_ID > CAN_MAX_STANDARD_ID) && (Ex_flag & CAN_IF1ARB2_XTD)) {  //Checking the frame type
	
		type = Extended_Frame;                               //Setting Extended frame as CAN frame type
	}
	else if (Msg_ID > CAN_MAX_EXTENDED_ID){

		return;                                              //Invalid messege ID
	}

	CAN1_IF1CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_ARB;                       //Enable transfere arbitration bits to interface registers
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2

	CAN1_IF1MSK2_R |= CAN_IF2MSK2_MDIR;                      //Enabling the effect of DIR bit for acceptance filtering
	CAN1_IF1ARB2_R |= CAN_IF1ARB2_DIR;                       //Setting the direction as transmit
	CAN1_IF1ARB2_R |= CAN_IF1ARB2_MSGVAL;                    //Setting as valid messege

	if (type == Standard_Frame) {
	
		CAN1_IF1ARB2_R &= (~(CAN_IF1ARB2_XTD));              //Indicating a standard frame
		CAN1_IF1MSK2_R &= (~(CAN_IF1MSK2_MXTD));             //Disabling the effect of extended frame for acceptance filtering
		CAN1_IF1ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field of ARB2
		CAN1_IF1ARB2_R |= ( Msg_ID << 2);                    //Writing the msg ID in bits [12:2] in the ID bit field
	}

	else if (type == Extended_Frame) {

		CAN1_IF1ARB2_R |= CAN_IF1ARB2_XTD;                   //Indicating an extended frame
		CAN1_IF1MSK2_R |= CAN_IF1MSK2_MXTD                   //Enabling the effect of extended frame for acceptance filtering
		CAN1_IF1ARB1_R &= (~(0xFFFF));                       //Clearing the ID bit field of ARB1
		CAN1_IF1ARB1_R |= (Msg_ID & 0x0000FFFF);             //Writing the lower 16 bits of messege in ID bit field of ARB1
		CAN1_IF1ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field ARB2
		CAN1_IF1ARB2_R |= ((Msg_ID & 0x1FFF0000) >> 16);     //Writing the upper 13 bits of messege in ID bit field of ARB2
	}

	if (Ex_flag & CAN_IF1MCTL_TXIE) {

		CAN1_IF1MCTL_R |= CAN_IF1MCTL_TXIE;				     //Enabling interrupt after a successful transmission
	}
		CAN1_IF1MCTL_R |= CAN_IF1MCTL_EOB;                   //Setting End of buffer bit field

		CAN1_IF1CRQ_R = Obj_ID;                              //Writing the messege object ID in the MNUM bit field
}



void CAN0_msgObj_recieve_config(uint32_t Msg_ID, uint32_t Ex_flag, uint32_t ID_MSK, MsgObj_ID Obj_ID) {

	CANFrame_Type type = Standard_Frame;                     //Setting standard frame type as default

	CAN0_IF2ARB2_R &= (~(CAN_IF2ARB2_MSGVAL));               //Ignoring msg obj

	if ((Msg_ID < CAN_MAX_EXTENDED_ID) && (Msg_ID > CAN_MAX_STANDARD_ID) && (Ex_flag & CAN_IF1ARB2_XTD)) {  //Checking the frame type

		type = Extended_Frame;                               //Setting Extended frame as CAN frame type
	}
	else if (Msg_ID > CAN_MAX_EXTENDED_ID) {

		return;                                              //Invalid messege ID
	}

	if (Ex_flag & CAN_IF1MCTL_UMASK) {

		CAN0_IF2MCTL_R |= CAN_IF1MCTL_UMASK;                 //Using acceptance mask
		CAN0_IF2CMSK_R |= CAN_IF2CMSK_MASK;			         //Accessing mask bits
	}

	CAN0_IF2CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN0_IF2CMSK_R |= CAN_IF1CMSK_ARB;                       //Enable transfere arbitration bits to interface registers
	CAN0_IF2CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN0_IF2CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN0_IF2CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2

	CAN0_IF2MSK2_R |= CAN_IF2MSK2_MDIR;                      //Enabling the effect of DIR bit for acceptance filtering
	CAN0_IF2ARB2_R &= (~(CAN_IF1ARB2_DIR));                  //Setting the direction as recieve
	CAN0_IF2ARB2_R |= CAN_IF1ARB2_MSGVAL;                    //Setting as valid messege

	if (type == Standard_Frame) {

		CAN0_IF2ARB2_R &= (~(CAN_IF1ARB2_XTD));              //Indicating a standard frame
		CAN0_IF2MSK2_R &= (~(CAN_IF1MSK2_MXTD));             //Disabling the effect of extended frame for acceptance filtering

		if (Ex_flag & CAN_IF1MCTL_UMASK) {

			CAN0_IF2MCTL_R |= CAN_IF1MCTL_UMASK;             //Using CANIF2MSK2 mask bits for acceptance filtering
			CAN0_IF2MSK2_R &= ~(0x1FFF);                     //Clearing ID Mask bit field
			CAN0_IF2MSK2_R |= ((ID_MSK) << 2);               //Writing the mask ID in bits [12:2] in the MSK bit field          
			CAN0_IF2MSK2_R |= CAN_IF2MSK2_MDIR;              //DIR bit is used for acceptance filtering
		}

		CAN0_IF2ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field of ARB2
		CAN0_IF2ARB2_R |= (Msg_ID << 2);                     //Writing the msg ID in bits [12:2] in the ID bit field
	}
	else if (type == Extended_Frame) {
	
		CAN0_IF2ARB2_R |= CAN_IF1ARB2_XTD;                   //Indicating a extended frame

		if (Ex_flag & CAN_IF1MCTL_UMASK) {

			CAN0_IF2MCTL_R |= CAN_IF1MCTL_UMASK;             //Using CANIF2MSK2 mask bits for acceptance filtering
			CAN0_IF2MSK2_R |= CAN_IF1MSK2_MXTD;              //Enabling the effect of extended frame for acceptance filtering
			CAN0_IF2MSK1_R &= ~(0xFFFF);                     //Clearing MSK bit field (1)
			CAN0_IF2MSK1_R |= (ID_MSK & 0x0000FFFF);	     //Writing the lower part in the ID_MSK
			CAN0_IF2MSK2_R &= ~(0x1FFF);			         //Clearing MSK bit field (2)
			CAN0_IF2MSK2_R |= ((ID_MSK & 0x1FFF0000) >> 16); //Writing the lower part in the ID_MSK
		}
		CAN0_IF2ARB1_R &= (~(0xFFFF));                       //Clearing the ID bit field of ARB1
		CAN0_IF2ARB1_R |= (Msg_ID & 0x0000FFFF);             //Writing the lower 16 bits of messege in ID bit field of ARB1
		CAN0_IF2ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field ARB2
		CAN0_IF2ARB2_R |= ((Msg_ID & 0x1FFF0000) >> 16);     //Writing the upper 13 bits of messege in ID bit field of ARB2
	}

	if (Ex_flag & CAN_IF1MCTL_RXIE) {

		CAN0_IF2MCTL_R |= CAN_IF1MCTL_RXIE;				     //Enabling interrupt after a successful reception
	}

		CAN0_IF2MCTL_R |= CAN_IF1MCTL_EOB;                   //Setting End of buffer bit field
		CAN0_IF2CRQ_R = Obj_ID;                              //Writing the messege object ID in the MNUM bit field
	}



void CAN1_msgObj_recieve_config(uint32_t Msg_ID, uint32_t Ex_flag, uint32_t ID_MSK, MsgObj_ID Obj_ID) {

	CANFrame_Type type = Standard_Frame;                     //Setting standard frame type as default

	CAN1_IF2ARB2_R &= (~(CAN_IF2ARB2_MSGVAL));               //Ignoring msg obj

	if ((Msg_ID < CAN_MAX_EXTENDED_ID) && (Msg_ID > CAN_MAX_STANDARD_ID) && (Ex_flag & CAN_IF1ARB2_XTD)) {  //Checking the frame type

		type = Extended_Frame;                               //Setting Extended frame as CAN frame type
	}
	else if (Msg_ID > CAN_MAX_EXTENDED_ID) {

		return;                                              //Invalid messege ID
	}

	if (Ex_flag & CAN_IF1MCTL_UMASK) {

		CAN1_IF2MCTL_R |= CAN_IF1MCTL_UMASK;                 //Using acceptance mask
		CAN1_IF2CMSK_R |= CAN_IF2CMSK_MASK;			         //Accessing mask bits
	}

	CAN1_IF2CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN1_IF2CMSK_R |= CAN_IF1CMSK_ARB;                       //Enable transfere arbitration bits to interface registers
	CAN1_IF2CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN1_IF2CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN1_IF2CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2

	CAN1_IF2MSK2_R |= CAN_IF2MSK2_MDIR;                      //Enabling the effect of DIR bit for acceptance filtering
	CAN1_IF2ARB2_R &= (~(CAN_IF1ARB2_DIR));                  //Setting the direction as recieve
	CAN1_IF2ARB2_R |= CAN_IF1ARB2_MSGVAL;                    //Setting as valid messege

	if (type == Standard_Frame) {
	
		CAN1_IF2ARB2_R &= (~(CAN_IF1ARB2_XTD));              //Indicating a standard frame
		CAN1_IF2MSK2_R &= (~(CAN_IF1MSK2_MXTD));             //Disabling the effect of extended frame for acceptance filtering
	
		if (Ex_flag & CAN_IF1MCTL_UMASK) {
		
			CAN1_IF2MCTL_R |= CAN_IF1MCTL_UMASK;             //Using CANIF2MSK2 mask bits for acceptance filtering
			CAN1_IF2MSK2_R &= ~(0x1FFF);                     //Clearing ID Mask bit field
			CAN1_IF2MSK2_R |= ((ID_MSK) << 2);               //Writing the mask ID in bits [12:2] in the MSK bit field
			CAN1_IF2MSK2_R |= CAN_IF2MSK2_MDIR;              //DIR bit is used for acceptance filtering

		}

		CAN1_IF2ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field of ARB2
		CAN1_IF2ARB2_R |= (Msg_ID << 2);                     //Writing the msg ID in bits [12:2] in the ID bit field
	}
	else if (type == Extended_Frame) {

		CAN1_IF2ARB2_R |= CAN_IF1ARB2_XTD;                   //Indicating a extended frame

		if (Ex_flag & CAN_IF1MCTL_UMASK) {

			CAN1_IF2MCTL_R |= CAN_IF1MCTL_UMASK;             //Using CANIF2MSK2 mask bits for acceptance filtering
			CAN1_IF2MSK2_R |= CAN_IF1MSK2_MXTD;              //Enabling the effect of extended frame for acceptance filtering
			CAN1_IF2MSK1_R &= ~(0xFFFF);                     //Clearing MSK bit field (1)
			CAN1_IF2MSK1_R |= (ID_MSK & 0x0000FFFF);	     //Writing the lower part in the ID_MSK
			CAN1_IF2MSK2_R &= ~(0x1FFF);			         //Clearing MSK bit field (2)
			CAN1_IF2MSK2_R |= ((ID_MSK & 0x1FFF0000) >> 16); //Writing the lower part in the ID_MSK
		}
		CAN1_IF2ARB1_R &= (~(0xFFFF));                       //Clearing the ID bit field of ARB1
		CAN1_IF2ARB1_R |= (Msg_ID & 0x0000FFFF);             //Writing the lower 16 bits of messege in ID bit field of ARB1
		CAN1_IF2ARB2_R &= (~(0x1FFF));                       //Clearing the ID bit field ARB2
		CAN1_IF2ARB2_R |= ((Msg_ID & 0x1FFF0000) >> 16);     //Writing the upper 13 bits of messege in ID bit field of ARB2
	}

	if (Ex_flag & CAN_IF1MCTL_RXIE) {

		CAN1_IF2MCTL_R |= CAN_IF1MCTL_RXIE;				     //Enabling interrupt after a successful reception
	}

		CAN1_IF2MCTL_R |= CAN_IF1MCTL_EOB;                   //Setting End of buffer bit field
		CAN1_IF2CRQ_R = Obj_ID;                              //Writing the messege object ID in the MNUM bit field
}


void CAN0_Recieved_Handler(MsgObj_ID Obj_ID, Data *MsgObj, uint8_t INTCLR) {

	CAN0_IF2CMSK_R &= (~(CAN_IF2CMSK_WRNRD));                 //Clearing the WRNRD bit field
	CAN0_IF2CRQ_R = Obj_ID;                                   //Writing the messege object ID in the MNUM bit field  

	MsgObj->Msg_Data[0] = (CAN0_IF2DA1_R & 0x000000FF);       //Putting the data byte no. 0
	MsgObj->Msg_Data[1] = (CAN0_IF2DA1_R & 0x0000FF00) >> 8); //Putting the data byte no. 1
	MsgObj->Msg_Data[2] = (CAN0_IF2DA2_R & 0x000000FF);       //Putting the data byte no. 2
	MsgObj->Msg_Data[3] = (CAN0_IF2DA2_R & 0x0000FF00) >> 8); //Putting the data byte no. 3
	MsgObj->Msg_Data[4] = (CAN0_IF2DB1_R & 0x000000FF);       //Putting the data byte no. 4
	MsgObj->Msg_Data[5] = (CAN0_IF2DB1_R & 0x0000FF00) >> 8); //Putting the data byte no. 5
	MsgObj->Msg_Data[6] = (CAN0_IF2DB2_R & 0x000000FF);       //Putting the data byte no. 6
	MsgObj->Msg_Data[7] = (CAN0_IF2DB2_R & 0x0000FF00) >> 8); //Putting the data byte no. 7

	MsgObj->Message_Length = (CAN0_IF2MCTL_R & 0xF);          //Length of the messege

	if (INTCLR == True) {
		CAN0_IF2MCTL_R &= (~(CAN_IF2MCTL_INTPND));            //Clearing INTPND bit field
	}
}


void CAN1_Recieved_Handler(MsgObj_ID Obj_ID, Data *MsgObj, uint8_t INTCLR) {

	CAN1_IF2CMSK_R &= (~(CAN_IF2CMSK_WRNRD));                 //Clearing the WRNRD bit field
	CAN1_IF2CRQ_R = Obj_ID;                                   //Writing the messege object ID in the MNUM bit field

	MsgObj->Msg_Data[0] = (CAN1_IF2DA1_R & 0x000000FF);       //Putting the data byte no. 0
	MsgObj->Msg_Data[1] = (CAN1_IF2DA1_R & 0x0000FF00) >> 8); //Putting the data byte no. 1
	MsgObj->Msg_Data[2] = (CAN1_IF2DA2_R & 0x000000FF);       //Putting the data byte no. 2
	MsgObj->Msg_Data[3] = (CAN1_IF2DA2_R & 0x0000FF00) >> 8); //Putting the data byte no. 3
	MsgObj->Msg_Data[4] = (CAN1_IF2DB1_R & 0x000000FF);       //Putting the data byte no. 4
	MsgObj->Msg_Data[5] = (CAN1_IF2DB1_R & 0x0000FF00) >> 8); //Putting the data byte no. 5
	MsgObj->Msg_Data[6] = (CAN1_IF2DB2_R & 0x000000FF);       //Putting the data byte no. 6
	MsgObj->Msg_Data[7] = (CAN1_IF2DB2_R & 0x0000FF00) >> 8); //Putting the data byte no. 7

	MsgObj->Message_Length = (CAN1_IF2MCTL_R & 0xF);          //Length of the messege

	if (INTCLR == True) {
		CAN1_IF2MCTL_R &= (~(CAN_IF2MCTL_INTPND));            //Clearing INTPND bit field
	}
}


uint8_t CAN0_ReGet() {

	return (!(CAN0_CTL_R & CAN_CTL_DAR));                   //Retrying getting messege
}


uint8_t CAN1_ReGet() {

	return (!(CAN1_CTL_R & CAN_CTL_DAR));                   //Retrying getting messege
}


uint32_t Check_CAN0_STS(CANSts STS) {

	uint32_t status;

	if (STS == CONTROL) {

		status = (CAN0_STS_R & 0xFF);                                               //Reading the controller status
	}
	else if (STS == TXREQUEST) {

		status = ((CAN0_TXRQ1_R & 0xFFFF) | ((CAN0_TXRQ2_R & 0xFFFF) << 16));       //Checking for pending transmission request
	}
	else if (STS == NEWDATA) {

		status = ((CAN0_NWDA1_R & 0xFFFF) | ((CAN0_NWDA2_R & 0xFFFF) << 16));       //Checking for new data
	}
	else if (STS == MSGVAL) {

		status = ((CAN0_MSG1VAL_R & 0xFFFF) | ((CAN0_MSG2VAL_R & 0xFFFF) << 16));   //Checking for messege validity
	}
	return status;
}


uint32_t Check_CAN1_STS(CANSts STS) {

	uint32_t status;

	if (STS == CONTROL) {

		status = (CAN1_STS_R & 0xFF);                                               //Reading the controller status
	}
	else if (STS == TXREQUEST) {

		status = ((CAN1_TXRQ1_R & 0xFFFF) | ((CAN1_TXRQ2_R & 0xFFFF) << 16));       //Checking for pending transmission request
	}
	else if (STS == NEWDATA) {

		status = ((CAN1_NWDA1_R & 0xFFFF) | ((CAN1_NWDA2_R & 0xFFFF) << 16));       //Checking for new data
	}
	else if (STS == MSGVAL) {

		status = ((CAN1_MSG1VAL_R & 0xFFFF) | ((CAN1_MSG2VAL_R & 0xFFFF) << 16));   //Checking for messege validity
	}
	return status;
}


void CLR_CAN0_Msg() {

	CAN0_IF1CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_ARB;                       //Enable transfere arbitration bits to interface registers       
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_CLRINTPND;                 //Clearing pending interrupts
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2   

	CAN0_IF1MSK1_R &= ~(0xFFFF);					         //Clearing Msk1 bit Field
	CAN0_IF1MSK2_R &= ~(0x1FFF);					         //Clearing Msk2 bit Field
	CAN0_IF1ARB1_R &= ~(0xFFFF);				             //Clearing ARB1 bit Field
	CAN0_IF1ARB2_R &= ~(0x1FFF);					         //Clearing ARB2 bit Field

	CAN0_IF1MCTL_R &= ~(0xF);			             	     //Clearing DLC bit Field

	CAN0_IF2DA1_R = 0;                                       
	CAN0_IF2DA2_R = 0;
	CAN0_IF2DB1_R = 0;                                      //Clearing all data registers
	CAN0_IF2DB2_R = 0;

	CAN0_IF1ARB2_R &= ~(CAN_IF1ARB2_MSGVAL);                //Messege Invalid
}


void CLR_CAN1_Msg() {

	CAN1_IF1CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_ARB;                       //Enable transfere arbitration bits to interface registers       
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_CLRINTPND;                 //Clearing pending interrupts
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2   

	CAN1_IF1MSK1_R &= ~(0xFFFF);					         //Clearing Msk1 bit Field
	CAN1_IF1MSK2_R &= ~(0x1FFF);					         //Clearing Msk2 bit Field
	CAN1_IF1ARB1_R &= ~(0xFFFF);				             //Clearing ARB1 bit Field
	CAN1_IF1ARB2_R &= ~(0x1FFF);					         //Clearing ARB2 bit Field

	CAN1_IF1MCTL_R &= ~(0xF);			             	     //Clearing DLC bit Field

	CAN1_IF2DA1_R = 0;
	CAN1_IF2DA2_R = 0;
	CAN1_IF2DB1_R = 0;                                      //Clearing all data registers
	CAN1_IF2DB2_R = 0;

	CAN1_IF1ARB2_R &= ~(CAN_IF1ARB2_MSGVAL);                //Messege Invalid
}


void CAN0_Write(MsgObj_ID Obj_ID , uint32_t Msg_ID , uint32_t Ex_flag , DataTX *MsgObj) {

	CAN0_IF1CMSK_R &= (~(CAN_IF2CMSK_WRNRD));                //Clearing the WRNRD bit field
	CAN0_IF1CRQ_R = Obj_ID;                                  //Writing the messege object ID in the MNUM bit field 

	CAN0_IF1CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN0_IF1CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2 

	CAN0_IF1MCTL_R &= ~(0xF);			             	     //Clearing DLC bit Field
	CAN0_IF1MCTL_R |= MsgObj->Message_Length;	             //Writing data length code

	CAN0_IF1DA1_R = (MsgObj->Msg_Data[0]);                   //Writing the data byte no. 0
	CAN0_IF1DA1_R |= ((MsgObj->Msg_Data[1]) << 8);           //Writing the data byte no. 1
	CAN0_IF1DA2_R = (MsgObj->Msg_Data[2]);                   //Writing the data byte no. 2
	CAN0_IF1DA2_R |= ((MsgObj->Msg_Data[3]) << 8);           //Writing the data byte no. 3
	CAN0_IF1DB1_R = (MsgObj->Msg_Data[4]);                   //Writing the data byte no. 4
	CAN0_IF1DB1_R |= ((MsgObj->Msg_Data[5]) << 8);           //Writing the data byte no. 5
	CAN0_IF1DB2_R = (MsgObj->Msg_Data[6]);                   //Writing the data byte no. 6
	CAN0_IF1DB2_R |= ((MsgObj->Msg_Data[7]) << 8);           //Writing the data byte no. 7

	CAN0_IF1MCTL_R |= CAN_IF1MCTL_TXRQST;                    //Requesting for this transmission
	CAN0_IF1MCTL_R |= CAN_IF1MCTL_NEWDAT;                    //Setting newdat bit for new data available flag

	CAN0_IF1CRQ_R = Obj_ID;                                  //Writing the messege object ID in the MNUM bit field
}


void CAN1_Write(MsgObj_ID Obj_ID , uint32_t Msg_ID , uint32_t Ex_flag , DataTX *MsgObj) {

	CAN1_IF1CMSK_R &= (~(CAN_IF2CMSK_WRNRD));                //Clearing the WRNRD bit field
	CAN1_IF1CRQ_R = Obj_ID;                                  //Writing the messege object ID in the MNUM bit field  

	CAN1_IF1CMSK_R |= CAN_IF1CMSK_WRNRD;                     //Setting write not read bit
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_CONTROL;                   //Enable transfere control bits to interface registers
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_DATAA;                     //Enable transfere 0-3 bits from msg obj to CANIFnDA1 and CANIFnDA2
	CAN1_IF1CMSK_R |= CAN_IF1CMSK_DATAB;                     //Enable transfere 4-7 bits from msg obj to CANIFnDA1 and CANIFnDA2 

	CAN1_IF1MCTL_R &= ~(0xF);			             	     //Clearing DLC bit Field
	CAN1_IF1MCTL_R |= MsgObj->Message_Length;	             //Write data length code

	CAN1_IF1DA1_R = (MsgObj->Msg_Data[0]);                   //Writing the data byte no. 0
	CAN1_IF1DA1_R |= ((MsgObj->Msg_Data[1]) << 8);           //Writing the data byte no. 1
	CAN1_IF1DA2_R = (MsgObj->Msg_Data[2]);                   //Writing the data byte no. 2
	CAN1_IF1DA2_R |= ((MsgObj->Msg_Data[3]) << 8);           //Writing the data byte no. 3
	CAN1_IF1DB1_R = (MsgObj->Msg_Data[4]);                   //Writing the data byte no. 4
	CAN1_IF1DB1_R |= ((MsgObj->Msg_Data[5]) << 8);           //Writing the data byte no. 5
	CAN1_IF1DB2_R = (MsgObj->Msg_Data[6]);                   //Writing the data byte no. 6
	CAN1_IF1DB2_R |= ((MsgObj->Msg_Data[7]) << 8);           //Writing the data byte no. 7

	CAN1_IF1MCTL_R |= CAN_IF1MCTL_TXRQST;                    //Requesting for this transmission
	CAN1_IF1MCTL_R |= CAN_IF1MCTL_NEWDAT;                    //Setting newdat bit for new data available flag

	CAN1_IF1CRQ_R = Obj_ID;                                  //Writing the messege object ID in the MNUM bit field
}


void CAN0_Resend(uint8_t choice) {

	if (choice == True) {
		CAN0_CTL_R &= ~(CAN_CTL_DAR); 	                    //Enabling Retrans. mode
	}
	else if (choice == False) {
		CAN0_CTL_R |= CAN_CTL_DAR;			                //Disabling Retrans. mode
	}
}


void CAN1_Resend(uint8_t choice) {

	if (choice == True) {
		CAN1_CTL_R &= ~(CAN_CTL_DAR); 	                    //Enabling Auto Retrans. mode
	}
	else if (choice == False) {
		CAN1_CTL_R |= CAN_CTL_DAR;			                //Disabling Auto Retrans. mode
	}
}
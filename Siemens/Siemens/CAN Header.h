#define magicKey 0x4C4F434B
#define  TRUE 			1
#define  FALSE			0
#define CAN_MAX_DATA_LENGTH		8
#define CAN_MAX_STANDARD_ID		0x7FF
#define	CAN_MAX_EXTENDED_ID		0x1FFFFFFF
typedef unsigned char uint8_t;
typedef unsigned long uint32_t;
#define Test_Mode 1
#define Real_Mode 0
#define True 1
#define False 0

typedef struct {
    uint32_t prescaler;
    uint32_t PropPhase1Seg;
    uint32_t Phase2Seg;
    uint32_t SJW;
}CANBitParms;

typedef enum {
    Standard_Frame = 0,
    Extended_Frame = 1
}CANFrame_Type;

typedef enum {
	ZERO_BYTE = 0,
	ONE_BYTE = 1,
	TWO_BYTE = 2,
	THREE_BYTE = 3,
	FOUR_BYTE = 4,
	FIVE_BYTE = 5,
	SIX_BYTE = 6,
	SEVEN_BYTE = 7,
	EIGHT_BYTE = 8,
}CAN_MSG_LENGTH;

typedef enum {
	MsgObj1 = 0x01,
	MsgObj2 = 0x02,
	MsgObj3 = 0x03,
	MsgObj4 = 0x04,
	MsgObj5 = 0x05,
	MsgObj6 = 0x06,
	MsgObj7 = 0x07,
	MsgObj8 = 0x08,
	MsgObj9 = 0x09,
	MsgObj10 = 0x0A,
	MsgObj11 = 0x0B,
	MsgObj12 = 0x0C,
	MsgObj13 = 0x0D,
	MsgObj14 = 0x0E,
	MsgObj15 = 0x0F,
	MsgObj16 = 0x10,
	MsgObj17 = 0x11,
	MsgObj18 = 0x12,
	MsgObj19 = 0x13,
	MsgObj20 = 0x14,
	MsgObj21 = 0x15,
	MsgObj22 = 0x16,
	MsgObj23 = 0x17,
	MsgObj24 = 0x18,
	MsgObj25 = 0x19,
	MsgObj26 = 0x1A,
	MsgObj27 = 0x1B,
	MsgObj28 = 0x1C,
	MsgObj29 = 0x1D,
	MsgObj30 = 0x1E,
	MsgObj31 = 0x1F,
	MsgObj32 = 0x20,
}MsgObj_ID;

typedef struct {

	CAN_MSG_LENGTH Message_Length;
	uint8_t Msg_Data[CAN_MAX_DATA_LENGTH];

}Data;

typedef struct {

	CAN_MSG_LENGTH Message_Length;
	uint8_t Msg_Data[CAN_MAX_DATA_LENGTH];

}DataTX;

typedef enum {
	CONTROL = 0,		           //the main controller status
	TXREQUEST = 1,	               //bit mask of objects pending transmission
	NEWDATA = 2,			       //bit mask of objects with new data
	MSGVAL = 3,			           //bit mask of objects with valid configuration
}CANSts;

//======================================================================//
                         /*System ctrl registers*/
//======================================================================//

#define SYSCTL_RCGC0_R          (*((volatile uint32_t *)0x400FE100))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108))

//======================================================================//
                           /*CAN0 registers*/
//======================================================================//

#define CAN0_CTL_R              (*((volatile uint32_t *)0x40040000))
#define CAN0_BIT_R              (*((volatile uint32_t *)0x4004000C))
#define CAN0_IF1CRQ_R           (*((volatile uint32_t *)0x40040020))
#define CAN0_IF2CRQ_R           (*((volatile uint32_t *)0x40040080))
#define CAN0_TST_R              (*((volatile uint32_t *)0x40040014))
#define NVIC_EN1_R              (*((volatile uint32_t *)0xE000E104))
#define CAN0_INT_R              (*((volatile uint32_t *)0x40040010))
#define CAN0_MSG1INT_R          (*((volatile uint32_t *)0x40040140))
#define CAN0_MSG2INT_R          (*((volatile uint32_t *)0x40040144))
#define CAN0_IF1CMSK_R          (*((volatile uint32_t *)0x40040024))
#define CAN0_IF2CMSK_R          (*((volatile uint32_t *)0x40040084))
#define CAN0_IF1ARB1_R          (*((volatile uint32_t *)0x40040030))
#define CAN0_IF1ARB2_R          (*((volatile uint32_t *)0x40040034))
#define CAN0_IF1MSK1_R          (*((volatile uint32_t *)0x40040028))
#define CAN0_IF1MSK2_R          (*((volatile uint32_t *)0x4004002C))
#define CAN0_IF2MSK1_R          (*((volatile uint32_t *)0x40040088))
#define CAN0_IF2MSK2_R          (*((volatile uint32_t *)0x4004008C))
#define CAN0_IF1MCTL_R          (*((volatile uint32_t *)0x40040038))
#define CAN0_IF2MCTL_R          (*((volatile uint32_t *)0x40040098))
#define CAN0_IF2ARB1_R          (*((volatile uint32_t *)0x40040090))
#define CAN0_IF2ARB2_R          (*((volatile uint32_t *)0x40040094))
#define CAN0_IF2DA1_R           (*((volatile uint32_t *)0x4004009C))
#define CAN0_IF2DA2_R           (*((volatile uint32_t *)0x400400A0))
#define CAN0_IF2DB1_R           (*((volatile uint32_t *)0x400400A4))
#define CAN0_IF2DB2_R           (*((volatile uint32_t *)0x400400A8))
#define CAN0_STS_R              (*((volatile uint32_t *)0x40040004))
#define CAN0_TXRQ1_R            (*((volatile uint32_t *)0x40040100))
#define CAN0_TXRQ2_R            (*((volatile uint32_t *)0x40040104))
#define CAN0_NWDA1_R            (*((volatile uint32_t *)0x40040120))
#define CAN0_NWDA2_R            (*((volatile uint32_t *)0x40040124))
#define CAN0_MSG1VAL_R          (*((volatile uint32_t *)0x40040160))
#define CAN0_MSG2VAL_R          (*((volatile uint32_t *)0x40040164))
#define CAN0_IF1DA1_R           (*((volatile uint32_t *)0x4004003C))
#define CAN0_IF1DA2_R           (*((volatile uint32_t *)0x40040040))
#define CAN0_IF1DB1_R           (*((volatile uint32_t *)0x40040044))
#define CAN0_IF1DB2_R           (*((volatile uint32_t *)0x40040048))

//======================================================================//
                           /*CAN1 registers*/
//======================================================================//

#define CAN1_CTL_R              (*((volatile uint32_t *)0x40041000))
#define CAN1_BIT_R              (*((volatile uint32_t *)0x4004100C))
#define CAN1_IF1CRQ_R           (*((volatile uint32_t *)0x40041020))
#define CAN1_IF2CRQ_R           (*((volatile uint32_t *)0x40041080))
#define CAN1_TST_R              (*((volatile uint32_t *)0x40041014))
#define CAN1_INT_R              (*((volatile uint32_t *)0x40041010))
#define CAN1_MSG1INT_R          (*((volatile uint32_t *)0x40041140))
#define CAN1_MSG2INT_R          (*((volatile uint32_t *)0x40041144))
#define CAN1_IF1CMSK_R          (*((volatile uint32_t *)0x40041024))
#define CAN1_IF2CMSK_R          (*((volatile uint32_t *)0x40041084))
#define CAN1_IF1ARB1_R          (*((volatile uint32_t *)0x40041030))
#define CAN1_IF1ARB2_R          (*((volatile uint32_t *)0x40041034))
#define CAN1_IF1MSK1_R          (*((volatile uint32_t *)0x40041028))
#define CAN1_IF1MSK2_R          (*((volatile uint32_t *)0x4004102C))
#define CAN1_IF2MSK1_R          (*((volatile uint32_t *)0x40041088))
#define CAN1_IF2MSK2_R          (*((volatile uint32_t *)0x4004108C))
#define CAN1_IF1MCTL_R          (*((volatile uint32_t *)0x40041038))
#define CAN1_IF2MCTL_R          (*((volatile uint32_t *)0x40041098))
#define CAN1_IF2ARB1_R          (*((volatile uint32_t *)0x40041090))
#define CAN1_IF2ARB2_R          (*((volatile uint32_t *)0x40041094))
#define CAN1_IF2DA1_R           (*((volatile uint32_t *)0x4004109C))
#define CAN1_IF2DA2_R           (*((volatile uint32_t *)0x400410A0))
#define CAN1_IF2DB1_R           (*((volatile uint32_t *)0x400410A4))
#define CAN1_IF2DB2_R           (*((volatile uint32_t *)0x400410A8))
#define CAN1_STS_R              (*((volatile uint32_t *)0x40041004))
#define CAN1_TXRQ1_R            (*((volatile uint32_t *)0x40041100))
#define CAN1_TXRQ2_R            (*((volatile uint32_t *)0x40041104))
#define CAN1_NWDA1_R            (*((volatile uint32_t *)0x40041120))
#define CAN1_NWDA2_R            (*((volatile uint32_t *)0x40041124))
#define CAN1_MSG1VAL_R          (*((volatile uint32_t *)0x40041160))
#define CAN1_MSG2VAL_R          (*((volatile uint32_t *)0x40041164))
#define CAN1_IF1DA1_R           (*((volatile uint32_t *)0x4004103C))
#define CAN1_IF1DA2_R           (*((volatile uint32_t *)0x40041040))
#define CAN1_IF1DB1_R           (*((volatile uint32_t *)0x40041044))
#define CAN1_IF1DB2_R           (*((volatile uint32_t *)0x40041048))

//======================================================================//
          /*Definition of bit fields for SYSCTL_RCGCGPIO register.*/
//======================================================================//

#define SYSCTL_RCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control
#define SYSCTL_RCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control
#define SYSCTL_RCGCGPIO_R0      0x00000001  // GPIO Port A Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R1      0x00000002  // GPIO Port B Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R4      0x00000010  // GPIO Port E Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R5      0x00000020  // GPIO Port F Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // Port A Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOF      0x00000020  // Port F Clock Gating Control

//======================================================================//
                         /*PortA GPIO registers*/
//======================================================================//

#define GPIO_PORTA_LOCK_R       (*((volatile uint32_t *)0x40004520))
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40004420))
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40004400))
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4000452C))

//======================================================================//
                         /*PortB GPIO registers*/
//======================================================================//

#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile uint32_t *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_LOCK_R       (*((volatile uint32_t *)0x40005520))
#define GPIO_PORTB_AMSEL_R      (*((volatile uint32_t *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile uint32_t *)0x4000552C))

//======================================================================//
                        /*PortE GPIO registers*/
//======================================================================//

#define GPIO_PORTE_LOCK_R       (*((volatile uint32_t *)0x40024520))
#define GPIO_PORTE_DEN_R        (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile uint32_t *)0x4002452C))
#define GPIO_PORTE_DIR_R        (*((volatile uint32_t *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile uint32_t *)0x40024420))

//======================================================================//
                        /*PortE GPIO registers*/
//======================================================================//

#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile uint32_t *)0x40025520))
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C))

//======================================================================//
          /*Definition of bit fields for CAN_CTL register.*/
//======================================================================//

#define CAN_CTL_INIT            0x00000001  // Initialization
#define CAN_BIT_BRP_M           0x0000003F  // Baud Rate Prescaler
#define CAN_CTL_CCE             0x00000040  // Configuration Change Enable
#define CAN_CTL_TEST            0x00000080  // Test Mode Enable
#define CAN_CTL_DAR             0x00000020  // Disable Automatic-Retransmission

//======================================================================//
          /*Definition of bit fields for CAN_IF1CRQ register.*/
//======================================================================//

#define CAN_IF1CRQ_BUSY         0x00008000  // Busy Flag

//======================================================================//
          /*Definition of bit fields for CAN_TST register.*/
//======================================================================//

#define CAN_TST_LBACK           0x00000010  // Loopback Mode

//======================================================================//
          /*Definition of bit fields for CANn_IF1CMSK register.*/
//======================================================================//

#define CAN_IF1CMSK_WRNRD       0x00000080  // Write, Not Read
#define CAN_IF1CMSK_ARB         0x00000020  // Access Arbitration Bits
#define CAN_IF1CMSK_CONTROL     0x00000010  // Access Control Bits
#define CAN_IF1CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3
#define CAN_IF1CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7
#define CAN_IF1CMSK_CLRINTPND   0x00000008  // Clear Interrupt Pending Bit

//======================================================================//
		  /*Definition of bit fields for CANn_IF2CMSK register.*/
//======================================================================//

#define CAN_IF2CMSK_WRNRD       0x00000080  // Write, Not Read
#define CAN_IF2CMSK_MASK        0x00000040  // Access Mask Bits
#define CAN_IF2CMSK_ARB         0x00000020  // Access Arbitration Bits
#define CAN_IF2CMSK_CONTROL     0x00000010  // Access Control Bits
#define CAN_IF2CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3
#define CAN_IF2CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7

//======================================================================//
          /*Definition of bit fields for CANn_IF1ARB2 register.*/
//======================================================================//

#define CAN_IF1ARB2_MSGVAL      0x00008000  // Message Valid
#define CAN_IF1ARB2_XTD         0x00004000  // Extended Identifier
#define CAN_IF1ARB2_DIR         0x00002000  // Message Direction

//======================================================================//
		  /*Definition of bit fields for CAN_IF2ARB2 register.*/
//======================================================================//

#define CAN_IF2ARB2_MSGVAL      0x00008000  // Message Valid

//======================================================================//
          /*Definition of bit fields for CANn_IF1MSK2 register.*/
//======================================================================//

#define CAN_IF1MSK2_MXTD        0x00008000  // Mask Extended Identifier
#define CAN_IF2MSK2_MDIR        0x00004000  // Mask Message Direction

//======================================================================//
          /*Definition of bit fields for CAN_IF1MCTL register.*/
//======================================================================//

#define CAN_IF1MCTL_TXIE        0x00000800  // Transmit Interrupt Enable
#define CAN_IF1MCTL_RXIE        0x00000400  // Receive Interrupt Enable
#define CAN_IF1MCTL_EOB         0x00000080  // End of Buffer
#define CAN_IF1MCTL_UMASK       0x00001000  // Use Acceptance Mask
#define CAN_IF2MCTL_INTPND      0x00002000  // Interrupt Pending
#define CAN_IF1MCTL_TXRQST      0x00000100  // Transmit Request
#define CAN_IF1MCTL_NEWDAT      0x00008000  // New Data

//======================================================================//
		                     /*Prototypes*/
//======================================================================//

void PortB_CAN0_Init(uint8_t Mode);
void PortE_CAN0_Init(uint8_t Mode);
void PortF_CAN0_Init(uint8_t Mode);
void CAN1_Init(uint8_t Mode);
void EnableCAN0();
void EnableCAN1();
void DisableCAN0();
void DisableCAN1();
void SetCAN0_BitTime(CANBitParms* Parms);
void SetCAN1_BitTime(CANBitParms* Parms);
void EnableCAN0_Int(uint32_t flags);
void EnableCAN1_Int(uint32_t flags);
void DisableCAN0_Int(uint32_t flags);
void DisableCAN1_Int(uint32_t flags);
void ClearCAN0_Int(uint32_t Int_Clear);
void CANIntClear(uint32_t Int_Clear);
uint32_t ReadCAN0_IntSTS(uint8_t choice);
uint32_t ReadCAN1_IntSTS(uint8_t choice);
void CAN0_msgObj_transmit_config(uint32_t Msg_ID, uint32_t Ex_flag, MsgObj_ID Obj_ID);
void CAN1_msgObj_transmit_config(uint32_t Msg_ID, uint32_t Ex_flag, MsgObj_ID Obj_ID);
void CAN0_msgObj_recieve_config(uint32_t Msg_ID, uint32_t Ex_flag, uint32_t ID_MSK, MsgObj_ID Obj_ID);
void CAN1_msgObj_recieve_config(uint32_t Msg_ID, uint32_t Ex_flag, uint32_t ID_MSK, MsgObj_ID Obj_ID);
void CAN0_Recieved_Handler(MsgObj_ID Obj_ID, Data* MsgObj, uint8_t INTCLR);
void CAN1_Recieved_Handler(MsgObj_ID Obj_ID, Data* MsgObj, uint8_t INTCLR);
void CLR_CAN0_Msg();
void CLR_CAN1_Msg();
void CAN0_Resend(uint8_t choice);
void CAN1_Resend(uint8_t choice);
uint8_t CAN0_ReGet();
uint8_t CAN1_ReGet();
uint32_t Check_CAN0_STS(CANSts STS);
uint32_t Check_CAN1_STS(CANSts STS);
void CAN0_Write(MsgObj_ID Obj_ID, uint32_t Msg_ID, uint32_t Ex_flag, DataTX *MsgObj);
void CAN1_Write(MsgObj_ID Obj_ID, uint32_t Msg_ID, uint32_t Ex_flag, DataTX *MsgObj);
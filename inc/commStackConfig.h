//Use this file to control the amount of RAM used by flexsea-comm

#ifndef INC_FLEXSEA_COMM_STACK_CONFIG_H
#define INC_FLEXSEA_COMM_STACK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE



#else

	//Default: everything enabled

	#define NUMBER_OF_PORTS					7		//Has to match enum below!

	//Communication port/interface:
	typedef enum {
		//Slave:
		PORT_RS485_1 = 0,
		PORT_SUB1  = PORT_RS485_1,
		PORT_RS485_2 = 1,
		PORT_SUB2 = PORT_RS485_2,
		//Master:
		PORT_USB = 2,
		PORT_SPI = 3,
		PORT_WIRELESS = 4,
		PORT_EXP = 5,
		PORT_BWC = 6,
		//None
		PORT_NONE	//PORT_NONE always has to be the last item
	}Port;


#endif

#ifdef __cplusplus
}
#endif

#endif	//INC_FLEXSEA_COMM_STACK_CONFIG_H

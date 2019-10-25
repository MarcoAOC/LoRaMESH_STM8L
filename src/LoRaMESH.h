/* ---------------------------------------------------
            Marco Antônio de Oliveira Costa
   ---------------------------------------------------
    This library contains a set of functions to configure
    and operate the EndDevice LoRaMESH Radioenge from 
    Radioenge Equipamentos de Telecomunicações using 
    STM8L152C6T6 MCU
  
  Date: 14/10/19
*/

#include "stm8l15x.h"
#include "stm8_eval.h"
#include "stdio.h" 
#include "stdbool.h"
#include "string.h"

/* ----- Defines ------ */

#define MAX_PAYLOAD_SIZE 232
#define MAX_BUFFER_SIZE 237

/* -----  Default Comands List ----- */
typedef enum
{
  CMD_LORAPARAMETER   = 0xD6,   /* Gets or Sets the LoRa modulation parameters */
  CMD_LOCALREAD       = 0xE2,   /* Gets the ID, NET and UNIQUE ID info from the local device */
  CMD_REMOTEREAD      = 0xD4,   /* Gets the ID, NET and UNIQUE ID info from a remote device */
  CMD_WRITECONFIG     = 0xCA,   /* Writes configuration info to the device, i.e. NET and ID */
  CMD_GPIOCONFIG      = 0xC2,   /* Configures a given GPIO pin to a desired mode, gets or sets its value */
  CMD_DIAGNOSIS       = 0xE7,   /* Gets diagnosis information from the device */
  CMD_READNOISE       = 0xD8,   /* Reads the noise floor on the current channel */
  CMD_READRSSI        = 0xD5,   /* Reads the RSSI between the device and a neighbour */
  CMD_TRACEROUTE      = 0xD2,   /* Traces the hops from the device to the master */
  CMD_SENDTRANSP      = 0x28,    /* Sends a packet to the device's transparent serial port */
  CMD_SENDDATA        = 0x15    /* Sends a packet to the device's command serial port */
} Cmd_Typedef;


/* GPIO Enum */
typedef enum
{
  GPIO0,
  GPIO1,
  GPIO2,
  GPIO3,
  GPIO4,
  GPIO5,
  GPIO6,
  GPIO7
} GPIO_Typedef;

/* GPIO mode enum */
typedef enum
{
  DIGITAL_IN,
  DIGITAL_OUT,
  ANALOG_IN = 3
} Mode_Typedef;

/* Pull resistor enum */
typedef enum
{
  PULL_OFF,
  PULLUP,
  PULLDOWN
} Pull_Typedef;

typedef enum
{
  MESH_OK,
  MESH_ERROR
} MeshStatus_Typedef;

/* ----- Public global variables ----- */

/* ----- Public Functions Prototype ----- */

/**
  * @brief Initializes and Enable USART port and Rx/Tx interrupts
  * @param USART_BaudRate: Serial baudrate, in bps
  * @param USART_WordLength: Size of data in bits
  * @param USART_StopBits: Amount of bits to be transmitted at the end of frame
  * @param USART_Parity: Type of parity
  * @retval None
  */
void SerialCommandsInit(uint32_t USART_BaudRate,USART_WordLength_TypeDef USART_WordLength,USART_StopBits_TypeDef USART_StopBits,USART_Parity_TypeDef USART_Parity);

/**
  * @brief Initializes and Enable USART port and Rx/Tx interrupts with defaults values of Arduino Serial Library
  * @param USART_BaudRate: Serial baudrate, in bps
  * @retval None
  */
void ArduinoSerialCommandsInit(uint32_t USART_BaudRate);


/**
  * @brief Initializes and Enable USART port and Rx/Tx interrupts
  * @param USART_BaudRate: Serial baudrate, in bps
  * @param USART_WordLength: Size of data in bits
  * @param USART_StopBits: Amount of bits to be transmitted at the end of frame
  * @param USART_Parity: Type of parity
  * @retval None
  */
void SerialTranspInit(uint32_t USART_BaudRate,USART_WordLength_TypeDef USART_WordLength,USART_StopBits_TypeDef USART_StopBits,USART_Parity_TypeDef USART_Parity);



/**
  * @brief Prepares a frame to transmission via commands interface
  * @param id: Target device's ID
  * @param command: Byte that indicates the command
  * @param payload: pointer to the array holding the payload
  * @param payloadSize: payload size
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef PrepareFrameCommand(uint16_t id, uint8_t command, uint8_t* payload, uint8_t payloadSize);




/**
  * @brief Prepares a frame to transmission via transparent interface
  * @param id: Target device's ID
  * @param payload: pointer to the array holding the payload
  * @param payloadSize: payload size
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef PrepareFrameTransp(uint16_t id, uint8_t* payload, uint8_t payloadSize);




/**
  * @brief Sends a frame previously prepared by PrepareFrame
  * @param None
  * @retval None
  */
MeshStatus_Typedef SendPacket(void);




/**
  * @brief Receives a packet from the commands interface
  * @param id[out]: ID from the received message
  * @param command[out]: received command
  * @param payload[out]: buffer where the received payload will be copied to
  * @param payloadSize[out]: received payload size
  * @param timeout: reception timeout, in ms
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef ReceivePacketCommand(uint16_t* id, uint8_t* command, uint8_t* payload, uint8_t* payloadSize, uint32_t timeout);




/**
  * @brief Receives a packet from the transparent interface
  * @param id[out]: ID from the received message
  * @param payload[out]: buffer where the received payload will be copied to
  * @param payloadSize[out]: received payload size
  * @param timeout: reception timeout, in ms
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef ReceivePacketTransp(uint16_t* id, uint8_t* payload, uint8_t* payloadSize, uint32_t timeout);





/**
  * @brief Configures a GPIO of a device
  * @param id: Target device's ID
  * @param pin: pin to be configured (GPIO0 ... GPIO7)
  * @param mode: operation mode (DIGITAL_IN, DIGITAL_OUT, ANALOG_IN)
  * @param pull: internal pull resistor mode (PULL_OFF, PULLUP, PULLDOWN)
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef GpioConfig(uint16_t id, GPIO_Typedef pin, Mode_Typedef mode, Pull_Typedef pull);



/**
  * @brief Sets the logic level of a given pin
  * @param id: Target device's ID
  * @param pin: pin to be configured (GPIO0 ... GPIO7)
  * @param value: 1 or 0
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef GpioWrite(uint16_t id, GPIO_Typedef pin, uint8_t value);



/**
  * @brief Reads the logic level (digital) or the analog value of a pin
  * @param id: Target device's ID
  * @param pin: pin to be configured (GPIO0 ... GPIO7)
  * @param value[out]: 1 or 0 (digital mode); 0 to 4095 (analog mode)
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef GpioRead(uint16_t id, GPIO_Typedef pin, uint16_t* value);



/**
  * @brief Gets the ID, NET and UNIQUE ID info from the local device
  * @param id[out]: Local device's id
  * @param net[out]: Configured NET on local device
  * @param uniqueId[out]: Device Unique ID 
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef LocalRead(uint16_t* id, uint16_t* net, uint32_t* uniqueId);

/**
  * @brief  Computes CRC16.
  * @param  data_in: Pointer to the input buffer.
  * @param  length: Buffer size
  * @retval CRC16
  */
uint16_t ComputeCRC(uint8_t* data_in, uint16_t length);

/**
  * @brief  Delay Function
  * @param  nCount: Time for delay
  * @retval None
  */
void delay(__IO uint32_t nCount);

/**
  * @brief  Clear RxCounter and enable Rx interrupt
  * @param  None
  * @retval None
  */
void ReStartRx(void);

/**
  * @brief  Get byte i from frame buffer
  * @param  i: position of required byte
  * @retval Byte from position i of frame buffer
  */
uint8_t getFrameBuffer_i(uint8_t i);

/**
  * @brief  Get size of frame buffer
  * @param  None
  * @retval Size of frame buffer
  */
uint8_t getFrameSize(void);

/**
  * @brief  Set byte of position i of frame buffer with value
  * @param  i: Position to be modified
  * @param  value: Value to put into position i of buffer
  * @retval None
  */
void setFrameBuffer_i(uint8_t i, uint8_t value);

/**
  * @brief  Set RxCounter with value
  * @param  value: Value to put in RxCounter
  * @retval None
  */
void setRxCounter(uint8_t value);

/**
  * @brief  Get RxCounter
  * @param  None
  * @retval Value of RxCounter
  */
uint8_t getRxCounter(void);

/**
  * @brief  Set TxCounter with value
  * @param  value: Value to put in TxCounter
  * @retval None
  */
void setTxCounter(uint8_t value);

/**
  * @brief  Get TxCounter
  * @param  None
  * @retval Value of TxCounter
  */
uint8_t getTxCounter(void);

/**
  * @brief  Get RxResetFlag
  * @param  None
  * @retval Value of RxResetFlag
  */
uint8_t getRxResetFlag(void);

/**
  * @brief  Get RxResetFlag
  * @param  value: Value to put in RxResetFlag
  * @retval None
  */
void setRxResetFlag(uint8_t value);

MeshStatus_Typedef SendData(uint16_t id, uint8_t value);
MeshStatus_Typedef RemoteRead(uint16_t id, uint16_t* net, uint32_t* uniqueId);
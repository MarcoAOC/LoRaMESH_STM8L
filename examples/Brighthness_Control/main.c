/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "LoRaMESH.h"

uint16_t localId;

uint16_t localNet;

uint32_t localUniqueId;

/* IDS of nodes LoRaMESH */
uint16_t LDRID = 3;
uint16_t LEDID = 2;

/* Value of analog input */
uint16_t AdcIn;

int ledstatus = 0;
uint16_t MAXBRIGHT = 3500;
int error = 0;
uint16_t result;
void main(void)
{
  /*High speed internal clock prescaler: 1*/
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);

  /*Initialize serial interface with master module*/
  ArduinoSerialCommandsInit((uint32_t)9600);
  delay(500);

  /* Checks if it is properly connected */
  error = 0;
  while(LocalRead(&localId, &localNet, &localUniqueId) != MESH_OK)
    error = 1;
  delay(500);

  /* Ports Config as Analog Input of LDR module and Digital Output of LED module */
  error = 0;
  if(GpioConfig(LDRID, GPIO6, ANALOG_IN, PULL_OFF) != MESH_OK)
    error = 1;
  error = 0;
  if(GpioConfig(LEDID, GPIO6, DIGITAL_OUT, PULL_OFF) != MESH_OK)
    error = 1;
  while (1)
  {

    delay(1000);
    /* Read of GPIO pin 6 from LDR module */
    error = 0;
    if(GpioRead(LDRID, GPIO6, &AdcIn) != MESH_OK)
      error = 1;
    else
    {
      result = AdcIn;
      if(result>MAXBRIGHT){ /* Max value of bright */
        if(ledstatus==1){
          /* Write 0 in GPIO pin 6 in LED module */
          error = 0;
          if(GpioWrite(LEDID, GPIO6,0) != MESH_OK)
            error = 1;
         else
            ledstatus = 0;
        }
      }
      else{
        if(ledstatus == 0){
          /* Write 1 in GPIO pin 6 in LED module */
          error = 0;
          if(GpioWrite(LEDID, GPIO6,1) != MESH_OK)
            error = 1;
          else
            ledstatus = 1;
        }
      }
    }
  }
}

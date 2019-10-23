<p align="center">
  <img  src="https://art.pixilart.com/2545581a1178b3c.gif" width="123" height="137" >
</p>

<h1 align="center">LoRaMESH STM8L Library</h1>

<p align="center">
  Library made to configure <a href="https://www.radioenge.com.br/solucoes/iot/34-modulo-loramesh.html">EndDevice LoRaMESH</a>'s GPIO pins and to require and send data from them. Was based in <a href="https://github.com/Radioenge/LoRaMESH">Radioenge's Library</a>
</p>

## How to use
Download all the files on <a href="https://github.com/MarcoAOC/LoRaMESH_STM8L/tree/master/src">this root</a> and include them on your project. Good to use with STM8L Discovery board and your <a href="https://www.st.com/en/embedded-software/stsw-stm8016.html">"standard peripheral library"</a>.

### Configuring USART Interface 
As STM8L152C6T6 has only one USART interface and the LoRaMESH module has two, the choice of which interface will be used will be made by the hardware connection.
You can choose two ways to configure USART, using Arduino default or using manual configurations. For manual configurations you may use the types defined in "stm8l15x_usart" provided by ST. Arduino's mode works well with the LoRa MESH module. 
``` c
//Easy way
uint32_t baudrate = 9600;
ArduinoSerialCommandsInit(baudrate);

//Manual
/* USART configured as follow:
     - BaudRate = 9600 baud  
     - Word Length = 8 Bits
     - One Stop Bit
     - No parity
*/
  SerialCommandsInit(9600, USART_WordLength_8b, USART_StopBits_1,USART_Parity_No);
``` 
### Main Features
The main features of the library are the functions to request and send data from pins of other network modules.
``` c
/* GpioConfig */
GpioConfig(NODEID, PIN, GPIOMODE, PULL_R_TYPE);
/* GpioWrite */
GpioWrite(NODEID, PIN, ValueForWrite);
/* GpioRead */
GpioRead(NODEID, PIN, &ReturnedValue);
``` 
### GPIO's modes
You can configure the GPIO pins with the following modes:
``` c
/* DIGITAL_IN */
GpioConfig(NODEID, PIN, DIGITAL_IN, PULL_R_TYPE);
/* DIGITAL_OUT */
GpioConfig(NODEID, PIN, DIGITAL_OUT, PULL_R_TYPE);
/* ANALOG_IN */
GpioConfig(NODEID, PIN, ANALOG_IN, PULL_R_TYPE);
``` 
### Pull Resistor Types
You can configure the pull resistors with the following types:
``` c
/* PULL_OFF */
GpioConfig(NODEID, PIN, DIGITAL_IN, PULL_OFF);
/* PULLUP */
GpioConfig(NODEID, PIN, DIGITAL_OUT, PULLUP);
/* PULLDOWN */
GpioConfig(NODEID, PIN, ANALOG_IN, PULLDOWN);
``` 
### Request Status
The return of the request is given by following status to indicate error or not:
``` c
/* MESH_OK */
while(LocalRead(&localId, &localNet, &localUniqueId) != MESH_OK);
/* MESH_ERROR */
if(GpioRead(NODEID, PIN, &ReturnedValue)  == MESH_ERROR){
  //Invalid value
}

``` 

### Running a example

You can run the examples provided in <a href="https://github.com/MarcoAOC/LoRaMESH_STM8L/tree/master/examples">this</a> root following the instructions.

## Documentation
You can read the full description of functions in <a href="https://github.com/MarcoAOC/LoRaMESH_STM8L/tree/master/sr/LoRaMESH.h">here</a>.
## How to contribute

Read [this](CONTRIBUTING.md) guide.

## Contributors

| [<img src="https://avatars1.githubusercontent.com/u/2962215?v=3&s=115" width="100" height="100"><br><sub>@vjpamorim</sub>](https://github.com/vjpamorim) | [<img src="https://avatars0.githubusercontent.com/u/46674511?v=3&s=115" width="100" height="100"><br><sub>@Radioenge</sub>](https://github.com/Radioenge) |
| :---: | :---: | 


## Author

| [<img src="https://avatars0.githubusercontent.com/u/27908235?v=3&s=115"><br><sub>@MarcoAOC</sub>](https://github.com/MarcoAOC) |
| :---: |

---

Do you like this library? Please [star this project on GitHub](https://github.com/MarcoAOC/LoRaMESH_STM8L/stargazers)!

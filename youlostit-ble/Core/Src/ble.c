/*
 * ble.c
 *
 *  Created on: 18 gen 2021
 *      Author: UTPM9 (modifications by ADS)
 */

#include "ble.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include "ble_commands.h"

extern SPI_HandleTypeDef hspi3;
extern int dataAvailable;

// Device name sent in BLE advertisement packets
uint8_t deviceName[]={'P','r','i','v','T','a','g'};

uint8_t buffer[255];

// NORDIC UART Service
uint8_t UUID_NORDIC_UART_SERVICE[]={0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
uint8_t NORDIC_UART_SERVICE_HANDLE[2];

// NORDIC TX UART Characteristic
uint8_t UUID_CHAR_WRITE[]={0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
uint8_t WRITE_CHAR_HANDLE[2];

// NORDIC RX UART Characteristic
uint8_t UUID_CHAR_READ[]={0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};
uint8_t READ_CHAR_HANDLE[2];

uint16_t stackInitCompleteFlag=0;
uint8_t* rxEvent;
int16_t connectionHandler[2] = {-1, -1}; // Little Endian Format for connection handler

/**
 * Initializes the BLE module with appropriate settings
 */
void ble_init(){
	//fetching the reset event
	rxEvent=(uint8_t*)malloc(EVENT_STARTUP_SIZE);
	int res;

	while(!dataAvailable);
	res=fetchBleEvent(rxEvent,EVENT_STARTUP_SIZE);

	if(res==BLE_OK){
	res=checkEventResp(rxEvent,EVENT_STATUP_DATA,EVENT_STARTUP_SIZE);
	if(res==BLE_OK){
	   stackInitCompleteFlag|=0x01;
	}
	}
	HAL_Delay(10);
	free(rxEvent);

	//INIT GATT
	if(BLE_command(ACI_GATT_INIT,sizeof(ACI_GATT_INIT),ACI_GATT_INIT_COMPLETE,sizeof(ACI_GATT_INIT_COMPLETE),0)==BLE_OK){
	   stackInitCompleteFlag|=0x02;
	}
	free(rxEvent);

	//INIT GAP, actually the handle that i get is a GATT handle of a service, will change the name later
	if(BLE_command(ACI_GAP_INIT,sizeof(ACI_GAP_INIT),ACI_GAP_INIT_COMPLETE,sizeof(ACI_GAP_INIT_COMPLETE),3)==BLE_OK){
	   stackInitCompleteFlag|=0x04;
	   memcpy(GAP_SERVICE_HANDLE,rxEvent+7,2);
	   memcpy(GAP_CHAR_NAME_HANDLE,rxEvent+9,2);
	   memcpy(GAP_CHAR_APP_HANDLE,rxEvent+11,2);
	}
	free(rxEvent);

	//SET THE NAME OF THE BOARD IN THE SERVICE CREATED AUTOMATICALLY
	updateCharValue(GAP_SERVICE_HANDLE,GAP_CHAR_NAME_HANDLE,0,sizeof(deviceName),deviceName);
	stackInitCompleteFlag|=0x08;
	free(rxEvent);

	//INIT AUTH
	if(BLE_command(ACI_GAP_SET_AUTH,sizeof(ACI_GAP_SET_AUTH),ACI_GAP_SET_AUTH_RESP,sizeof(ACI_GAP_SET_AUTH_RESP),0)==BLE_OK){
	   stackInitCompleteFlag|=0x10;
	}
	free(rxEvent);

	//SET_TX_LEVEL
	if(BLE_command(ACI_HAL_SET_TX_POWER_LEVEL,sizeof(ACI_HAL_SET_TX_POWER_LEVEL),ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE,sizeof(ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE),0)==BLE_OK){
	   stackInitCompleteFlag|=0x20;
	}
	free(rxEvent);

	//SET SCAN RESPONSE DATA
	if(BLE_command(HCI_LE_SET_SCAN_RESPONSE_DATA,sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA),HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE,sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE),0)==BLE_OK){
	   stackInitCompleteFlag|=0x40;
	}
	free(rxEvent);

	//This will start the advertisment,
	setConnectable();

	//add the nordic UART service
	addService(UUID_NORDIC_UART_SERVICE,NORDIC_UART_SERVICE_HANDLE,SET_ATTRIBUTES(7)); //SET_ATTRIBUTES(1+2+3*2+3+3));//1 atribute service +2 attribute char readable+3*(2 NOTIFYABLE READABLE charachteristics)

	//add the nordic UART charachteristics
	addCharacteristic(UUID_CHAR_READ,READ_CHAR_HANDLE,NORDIC_UART_SERVICE_HANDLE,SET_CONTENT_LENGTH(20),NOTIFIBLE);
	addCharacteristic(UUID_CHAR_WRITE,WRITE_CHAR_HANDLE,NORDIC_UART_SERVICE_HANDLE,SET_CONTENT_LENGTH(20),WRITABLE);

	if(stackInitCompleteFlag==255){
	  //turn on led blue if everything was fine
	//  HAL_GPIO_WritePin(CPU_LED_GPIO_Port,CPU_LED_Pin,GPIO_PIN_SET);
	}
	return;
}

void standbyBle() {
	 //STANDBY MODE
	 if(BLE_command(ACI_HAL_SET_STANDBY,sizeof(ACI_HAL_SET_STANDBY),ACI_HAL_SET_STANDBY_COMPLETE,sizeof(ACI_HAL_SET_STANDBY_COMPLETE),0)==BLE_OK){
	 }
	 free(rxEvent);
}

int fetchBleEvent(uint8_t *container, int size){

  uint8_t master_header[]={0x0b,0x00,0x00,0x00,0x00};
  uint8_t slave_header[5];

  //Wait until it is available an event coming from the BLE module (GPIO PIN COULD CHANGE ACCORDING TO THE BOARD)
  if(HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){

  HAL_Delay(5);
  //PIN_CS of SPI2 LOW
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

  //SPI2 in this case, it could change according to the board
  //we send a byte containing a request of reading followed by 4 dummy bytes
  HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

  HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);

  //let's get the size of data available
  int dataSize;
  dataSize=(slave_header[3]|slave_header[4]<<8);
  int i;
  char dummy=0xff;

  if(dataSize>size){
	  dataSize=size;
  }

  if(dataSize>0){
	    //let's fill the get the bytes availables and insert them into the container variable
  		for(i=0;i<dataSize;i++){
  		HAL_SPI_TransmitReceive(&hspi3,(uint8_t*)&dummy,container+i,1,1);

  		}
  		HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
  	}else{
  		HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
        return -1;
  	}

  //let's stop the SPI2
  dataAvailable=0;
  return BLE_OK;
  }else{
  return -2;
  }
}


int checkEventResp(uint8_t *event, uint8_t *reference, int size){
	int j=0;

	for(j=0;j<size;j++){

		if(event[j]!=reference[j]){
			return -1;
		}
	}

return BLE_OK;
}

void sendCommand(uint8_t *command,int size){

	  uint8_t master_header[]={0x0a,0x00,0x00,0x00,0x00};
	  uint8_t slave_header[5];

	  int result;

	do{
	  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,0);

	  //wait until it is possible to write
	  //while(!dataAvailable);
	  HAL_SPI_TransmitReceive(&hspi3,master_header,slave_header,5,1);
	  int bufferSize=(slave_header[2]<<8|slave_header[1]);
	  if(bufferSize>=size){
		HAL_SPI_Transmit(&hspi3,command,size,1);
		result=0;
	  }else{
		result=-1;
	  }
	  HAL_GPIO_WritePin(BLE_CS_GPIO_Port,BLE_CS_Pin,1);
	  dataAvailable=0;
	}while(result!=0);

}

void catchBLE(){
      int result=fetchBleEvent(buffer,127);
	  if(result==BLE_OK){
		  if(checkEventResp(buffer,EVENT_DISCONNECTED,3)==BLE_OK){
			  // This automatically sets your device to be discoverable
			  // as soon as it disconnects from a device
			  setConnectable();
		  }
		  if(checkEventResp(buffer, EVENT_CONNECTED, 5)==BLE_OK){
			  // Little Endian Format
			  *(connectionHandler) = buffer[5];
			  *(connectionHandler + 1) = buffer[6];
		  }
	  }else{
		  //something bad is happening if I am here
	  }
}

void setConnectable(){
	   uint8_t* rxEvent;
	   //Start advertising
	   uint8_t *localname;
	   int res;
	   localname=(uint8_t*)malloc(sizeof(deviceName)+5);//carattere di terminazione+listauid+slavetemp
	   memcpy(localname,deviceName,sizeof(deviceName));
	   localname[sizeof(deviceName)+1]=0x00;
	   localname[sizeof(deviceName)+2]=0x00;
	   localname[sizeof(deviceName)+3]=0x00;
	   localname[sizeof(deviceName)+4]=0x00;
	   localname[sizeof(deviceName)]=0x00;


	   ACI_GAP_SET_DISCOVERABLE[11]=sizeof(deviceName)+1;
	   ACI_GAP_SET_DISCOVERABLE[3]=sizeof(deviceName)+5+sizeof(ACI_GAP_SET_DISCOVERABLE)-4;

	   uint8_t *discoverableCommand;
	   discoverableCommand=(uint8_t*)malloc(sizeof(ACI_GAP_SET_DISCOVERABLE)+sizeof(deviceName)+5);
	   memcpy(discoverableCommand,ACI_GAP_SET_DISCOVERABLE,sizeof(ACI_GAP_SET_DISCOVERABLE));
	   memcpy(discoverableCommand+sizeof(ACI_GAP_SET_DISCOVERABLE),localname,sizeof(deviceName)+5);

	   sendCommand(discoverableCommand,sizeof(deviceName)+5+sizeof(ACI_GAP_SET_DISCOVERABLE));
	   rxEvent=(uint8_t*)malloc(7);
	   while(!dataAvailable);
	   res=fetchBleEvent(rxEvent,7);
	   if(res==BLE_OK){
	   res=checkEventResp(rxEvent,ACI_GAP_SET_DISCOVERABLE_COMPLETE,7);
	   if(res==BLE_OK){
		   stackInitCompleteFlag|=0x80;
	   }
	   }

	   free(rxEvent);
	   free(discoverableCommand);
	   free(localname);
	   HAL_Delay(10);
}

/**
 * @brief Sends a BLE command and processes the response event.
 *
 * This function transmits a command to the BLE module, waits for the BLE interrupt pin to
 * signal that a response is available, retrieves the event data, and then checks the event
 * response.
 *
 * @param command Pointer to the buffer containing the command to be sent.
 * @param size Size of the command buffer.
 * @param result Pointer to the buffer where the response result will be stored.
 * @param sizeRes Expected size of the response result.
 * @param returnHandles Number of handles expected in the response (each handle occupies 2 bytes).
 * @return int Returns BLE_OK if the command was successfully executed and the event response is valid,
 *             or an error code if something went wrong.
 */
int BLE_command(uint8_t* command, int size, uint8_t* result, int sizeRes, int returnHandles){
	   int response;

	   sendCommand(command,size);
	   rxEvent=(uint8_t*)malloc(sizeRes+2*returnHandles);

	   long contatore=0;
	   while(!HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
		   contatore++;
		   if(contatore>30000){
			   break;
		   }
	   }


	   response=fetchBleEvent(rxEvent,sizeRes+returnHandles*2);
	   if(response==BLE_OK){
		   response=checkEventResp(rxEvent,result,sizeRes);
	   }
	   HAL_Delay(10);


	return response;
}

void addService(uint8_t* UUID, uint8_t* handle, int attributes){


	//memcpy
	memcpy(ADD_PRIMARY_SERVICE+5,UUID,16);
    ADD_PRIMARY_SERVICE[22]=attributes;
	   if(BLE_command(ADD_PRIMARY_SERVICE,sizeof(ADD_PRIMARY_SERVICE),ADD_PRIMARY_SERVICE_COMPLETE,sizeof(ADD_PRIMARY_SERVICE_COMPLETE),1)==BLE_OK){
		   handle[0]=rxEvent[7];
		   handle[1]=rxEvent[8];
	    }
	   free(rxEvent);
}

void addCharacteristic(uint8_t* UUID,uint8_t* handleChar, uint8_t* handleService, uint8_t maxsize, uint8_t proprieties){
    memcpy(ADD_CUSTOM_CHAR+7,UUID,16);

	ADD_CUSTOM_CHAR[4]= handleService[0];
	ADD_CUSTOM_CHAR[5]= handleService[1];
	ADD_CUSTOM_CHAR[23]= maxsize;
	ADD_CUSTOM_CHAR[25]= proprieties;
	if(BLE_command(ADD_CUSTOM_CHAR,sizeof(ADD_CUSTOM_CHAR),ADD_CUSTOM_CHAR_COMPLETE,sizeof(ADD_CUSTOM_CHAR_COMPLETE),1)==BLE_OK){
        handleChar[0]=rxEvent[7];
	    handleChar[1]=rxEvent[8];
	}
	free(rxEvent);
}

void updateCharValue(uint8_t* handleService,uint8_t* handleChar, int offset, int size,uint8_t* data){
	UPDATE_CHAR[3]=size+6;
	UPDATE_CHAR[4]=handleService[0];
	UPDATE_CHAR[5]=handleService[1];
	UPDATE_CHAR[6]=handleChar[0];
	UPDATE_CHAR[7]=handleChar[1];
	UPDATE_CHAR[8]=offset;
	UPDATE_CHAR[9]=size;

	uint8_t* commandComplete;
	commandComplete=(uint8_t*)malloc(10+size);
	memcpy(commandComplete,UPDATE_CHAR,10);
	memcpy(commandComplete+10,data,size);

	BLE_command(commandComplete,10+size,ADD_CUSTOM_CHAR_COMPLETE,sizeof(ADD_CUSTOM_CHAR_COMPLETE),0);

	free(commandComplete);
	free(rxEvent);
}

/** 
 * @brief Disconnects the peripheral from the central
*/
void disconnectBLE(){
	 if (connectionHandler[0] == -1 && connectionHandler[1] == -1){
		// should not be -1. If -1, then no connection was made at the first place!
		return;
	 }
	 uint8_t command[7];
	 memcpy(command, DISCONNECT, 4);
	 command[4] = connectionHandler[0];
	 command[5] = connectionHandler[1];
	 command[6] = 0x13;
	 if(BLE_command(command,sizeof(command),EVENT_DISCONNECTED,3,0)==BLE_OK){
		connectionHandler[0] = -1;
		connectionHandler[1] = -1;
	 }
	 free(rxEvent);

}

/**
 * DO NOT CHANGE FUNCTION definition
 * @brief Sets the discoverability of the peripheral
 * @param mode 0 => Non Discoverable, 1 => Discoverable
 * */
void setDiscoverability(uint8_t mode){
	if (mode == 1){
		setConnectable();
	}
	else if (mode == 0){
		if(BLE_command(ACI_GAP_SET_NON_DISCOVERABLE,sizeof(ACI_GAP_SET_NON_DISCOVERABLE),
		ACI_GAP_SET_NON_DISCOVERABLE_COMPLETE,sizeof(ACI_GAP_SET_NON_DISCOVERABLE_COMPLETE),0)==BLE_OK){}
	}
	else{
		// Do nothing
	}
}
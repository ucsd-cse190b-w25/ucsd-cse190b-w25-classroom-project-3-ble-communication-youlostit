/*
 * ble.h
 *
 *  Created on: 18 gen 2021
 *      Author: UTPM9 (modificaiton by ADS)
 */

#ifndef BLE_H_
#define BLE_H_

#include "main.h"
//#include "ble_commands.h"

#define BLE_OK 0

#define EVENT_STARTUP_SIZE 6
#define ACI_GATT_INIT_COMPLETE_SIZE 7
#define SET_ATTRIBUTES(n) (n)
#define SET_CONTENT_LENGTH(n) (n)

extern uint8_t NORDIC_UART_SERVICE_HANDLE[2];

extern uint8_t READ_CHAR_HANDLE[];

extern uint8_t* rxEvent;

//function for starting the BLE protocol
void ble_init(void);

void standbyBle();

//function that gets a pending event and save the data in the pointer *container
int fetchBleEvent(uint8_t *container, int size);

//check if the event that was fetched is what I expected
int checkEventResp(uint8_t *event, uint8_t *reference, int size);

void sendCommand(uint8_t *command,int size);

void catchBLE();

void setConnectable();

int BLE_command(uint8_t* command, int size, uint8_t* result, int sizeRes, int returnHandles);

void addService(uint8_t* UUID, uint8_t* handle, int attributes);

void addCharacteristic(uint8_t* UUID,uint8_t* handleChar, uint8_t* handleService, uint8_t maxsize, uint8_t proprieties);

void updateCharValue(uint8_t* handleService,uint8_t* handleChar, int offset, int size,uint8_t* data);
#endif

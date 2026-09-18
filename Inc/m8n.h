/*
 * m8n.h
 *
 *  Created on: Nov 29, 2025
 *      Author: SAIED
 */

#ifndef INC_M8N_H_
#define INC_M8N_H_


typedef struct _M8N_UBX_NAV_PVT
{
	unsigned char CLASS;
	unsigned char ID;
	unsigned short length;

	unsigned int iTOW;
	signed int lon;
	signed int lat;
	signed int height;
	signed int hMSL;
	unsigned int hACC;
	unsigned int vACC;
	signed int velN;
	signed int velE;
	signed int velD;

	float lon_f64;
	float lat_f64;




	//for the ack-ack
	unsigned int clsID;
	unsigned int msgID;



}M8N_UBX_NAV_PVT;

extern M8N_UBX_NAV_PVT pvt;

void M8N_TransmitData(unsigned char* data, unsigned char len);
void M8N_UART4_Initialization(void);
void M8N_Initialization(void);

unsigned char M8N_UBX_CHKSUM_Check(unsigned char* data, unsigned char len);
void M8N_UBX_NAV_PVT_Parsing(unsigned char* data, M8N_UBX_NAV_PVT* pvt);

#endif /* INC_M8N_H_ */

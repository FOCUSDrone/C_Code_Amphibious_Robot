/*
 * SCSCL.c
 * 飞特SCSCL系列串行舵机应用层程序
 * 日期: 2024.12.2
 * 作者: txl
 */

#include <string.h>
#include "INST.h"
#include "SCS.h"
#include "SCSCL.h"

static uint8_t Mem[SCSCL_PRESENT_CURRENT_H - SCSCL_PRESENT_POSITION_L+1];
int SCSCL_WritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return genWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

int SCSCL_RegWritePos(uint8_t ID, uint16_t Position, uint16_t Time, uint16_t Speed)
{
	uint8_t bBuf[6];
	Host2SCS(bBuf+0, bBuf+1, Position);
	Host2SCS(bBuf+2, bBuf+3, Time);
	Host2SCS(bBuf+4, bBuf+5, Speed);
	
	return regWrite(ID, SCSCL_GOAL_POSITION_L, bBuf, 6);
}

void SCSCL_RegWriteAction()
{
	regAction(0xfe);
}

void SCSCL_SyncWritePos(uint8_t ID[], uint8_t IDN, uint16_t Position[], uint16_t Time[], uint16_t Speed[])
{
  uint8_t offbuf[32*6];
	uint8_t i;
  for(i = 0; i<IDN; i++){
		uint16_t T, V;
		if(Time){
			T = Time[i];
		}else{
			T = 0;
		}
		if(Speed){
			V = Speed[i];
		}else{
			V = 0;
		}
    Host2SCS(offbuf+i*6+0, offbuf+i*6+1, Position[i]);
    Host2SCS(offbuf+i*6+2, offbuf+i*6+3, T);
    Host2SCS(offbuf+i*6+4, offbuf+i*6+5, V);
  }
  syncWrite(ID, IDN, SCSCL_GOAL_POSITION_L, offbuf, 6);
}

int SCSCL_PWMMode(uint8_t ID)
{
	uint8_t bBuf[4];
	bBuf[0] = 0;
	bBuf[1] = 0;
	bBuf[2] = 0;
	bBuf[3] = 0;
	return genWrite(ID, SCSCL_MIN_ANGLE_LIMIT_L, bBuf, 4);	
}

int SCSCL_WritePWM(uint8_t ID, int16_t pwmOut)
{
	uint8_t bBuf[2];
	if(pwmOut<0){
		pwmOut = -pwmOut;
		pwmOut |= (1<<10);
	}
	Host2SCS(bBuf+0, bBuf+1, pwmOut);

	return genWrite(ID, SCSCL_GOAL_TIME_L, bBuf, 2);
}
int SCSCL_EnableTorque(uint8_t ID, uint8_t Enable)
{
	return writeByte(ID, SCSCL_TORQUE_ENABLE, Enable);
}

int SCSCL_unLockEprom(uint8_t ID)
{
	return writeByte(ID, SCSCL_LOCK, 0);
}

int SCSCL_LockEprom(uint8_t ID)
{
	return writeByte(ID, SCSCL_LOCK, 1);
}

int SCSCL_FeedBack(int ID)
{
	int nLen = Read(ID, SCSCL_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if(nLen!=sizeof(Mem)){
		return -1;
	}
	return nLen;
}
	
int SCSCL_ReadPos(int ID)
{
	int Pos = -1;
	if(ID==-1){
		Pos = Mem[SCSCL_PRESENT_POSITION_L-SCSCL_PRESENT_POSITION_L];
		Pos <<= 8;
		Pos |= Mem[SCSCL_PRESENT_POSITION_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Pos = readWord(ID, SCSCL_PRESENT_POSITION_L);
	}
	return Pos;
}

int SCSCL_ReadSpeed(int ID)
{
	int Speed = -1;
	if(ID==-1){
		Speed = Mem[SCSCL_PRESENT_SPEED_L-SCSCL_PRESENT_POSITION_L];
		Speed <<= 8;
		Speed |= Mem[SCSCL_PRESENT_SPEED_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Speed = readWord(ID, SCSCL_PRESENT_SPEED_L);
	}
	if(Speed&(1<<15)){
		Speed = -(Speed&~(1<<15));
	}	
	return Speed;
}

int SCSCL_ReadLoad(int ID)
{
	int Load = -1;
	if(ID==-1){
		Load = Mem[SCSCL_PRESENT_LOAD_L-SCSCL_PRESENT_POSITION_L];
		Load <<= 8;
		Load |= Mem[SCSCL_PRESENT_LOAD_H-SCSCL_PRESENT_POSITION_L];
	}else{
		Load = readWord(ID, SCSCL_PRESENT_LOAD_L);
	}
	if(Load&(1<<10)){
		Load = -(Load&~(1<<10));
	}	
	return Load;
}

int SCSCL_ReadVoltage(int ID)
{
	int Voltage = -1;
	if(ID==-1){
		Voltage = Mem[SCSCL_PRESENT_VOLTAGE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Voltage = readByte(ID, SCSCL_PRESENT_VOLTAGE);
	}
	return Voltage;
}

int SCSCL_ReadTemper(int ID)
{
	int Temper = -1;
	if(ID==-1){
		Temper = Mem[SCSCL_PRESENT_TEMPERATURE-SCSCL_PRESENT_POSITION_L];	
	}else{
		Temper = readByte(ID, SCSCL_PRESENT_TEMPERATURE);
	}
	return Temper;
}

int SCSCL_ReadMove(int ID)
{
	int Move = -1;
	if(ID==-1){
		Move = Mem[SCSCL_MOVING-SCSCL_PRESENT_POSITION_L];	
	}else{
		Move = readByte(ID, SCSCL_MOVING);
	}
	return Move;
}

int SCSCL_ReadCurrent(int ID)
{
	int Current = -1;
	if(ID==-1){
		Current = Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L];
		Current <<= 8;
		Current |= Mem[SCSCL_PRESENT_CURRENT_L-SCSCL_PRESENT_POSITION_L];
	}else{
		Current = readWord(ID, SCSCL_PRESENT_CURRENT_L);
	}
	if(Current&(1<<15)){
		Current = -(Current&~(1<<15));
	}	
	return Current;
}

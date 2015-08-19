#include "common.h"
#include "uart.h"
#include "assert.h"
#include "modbus.h"

void ModbusCompress(Modbusp Modp)
{
    switch(Modp->Stage)
    {
    case 0: MBAddrChk(Modp);break;
    case 1: MBFunChk(Modp);break;
    case 2: Modp->Stage += 1;break;
    case 3: MBSAChk(Modp);break;
    case 4: Modp->Stage += 1;break;
    case 5: MBQChk(Modp);break;
//    case 6: MBBChk(Modp);break;
//    case 7: MBRecv(Modp);break;
    case 8: Modp->Stage += 1;break;
    case 9: MBCCRC(Modp);break;
    }
}

void InitModbusCon(Modbusp Modp, u8 *Buff)
{
    Modp->Stage = 0;
    Modp->Buff = Buff;
    Modp->FunCode = 0;
    Modp->StartAddr = 0;
    Modp->Quantity = 0;
    Modp->BCount = 0;
    ReceiveCount = 0;
}

void FlushModbusCon(Modbusp Modp)
{
    Modp->Stage = 0;
    Modp->FunCode = 0;
    Modp->StartAddr = 0;
    Modp->Quantity = 0;
    Modp->BCount = 0;
    ReceiveCount = 0;
    FlushArray(Modp->Buff, BuffMax);
}

void FlushArray(u8 *Buff, u16 Num)
{
    int i;
    for(i = 0; i < Num; i++)
    {
        *(Buff + i) = 0;
    }
}

void MBAddrChk(Modbusp Modp)
{
    if((*(Modp->Buff) + AddrB) == LocalAddr)
    {
        Modp->Stage += 1;
    }
    else
    {
        FlushModbusCon(Modp);
    }
}

void MBFunChk(Modbusp Modp)
{
    if((*(Modp->Buff + FunCB) == ReadR) || (*(Modp->Buff + FunCB) == WriteR))
    {
        Modp->Stage += 1;
        Modp->FunCode = *(Modp->Buff + 1);
    }
    else
    {
        FlushModbusCon(Modp);
    }
}

void MBSAChk(Modbusp Modp)
{
    u16 Tem = 0;
    Tem = ((*(Modp->Buff + SAddrH)) << 8) | (*(Modp->Buff + SAddrL));
    if(Tem <= 0x0f)
    {
        Modp->StartAddr = Tem;
        Modp->Stage += 1;
    }
    else
    {
        FlushModbusCon(Modp);
    }
}
        
void MBQChk(Modbusp Modp)
{
    u16 Tem = 0;
    Tem = ((*(Modp->Buff + QuanH)) << 8) | (*(Modp->Buff + QuanL));
    if(Modp->FunCode == ReadR)
    {
        if((Tem + Modp->StartAddr - 1) <= 0x0f)
        {
            Modp->Quantity = Tem;
        }
        else
        {
            FlushModbusCon(Modp);
        }
    }   
    
    Modp->Stage = 8;
}
        
void MBBChk(Modbusp Modp)
{
    Modp->BCount = *(Modp->Buff + BContB);
    Modp->Stage = 7;   
}
        
/*void MBRecv(Modbusp Modp) 
{
    Modp->Count += 1;
    if(Modp->Count >= (Modp->BCount))
    {
        Modp->Stage = 8;
    }
}*/
       
void MBCCRC(Modbusp Modp)
{
    u16 RecCRC = GetRecCRC(Modp);
    u16 MyCRC = CalCRC(Modp->Buff,6);
    
    if(RecCRC == MyCRC)
    {
        MBResp(Modp);
    }
    else
    {
        FlushModbusCon(Modp);
    }
}
       
u16 GetRecCRC(Modbusp Modp)
{
    u16 CRC = 0;   
    CRC = (*(Modp->Buff + CRCL)) | ((*(Modp->Buff + CRCH)) << 8);   
    return CRC;
}
       
void MBResp(Modbusp Modp)
{
    u8 count;
    u8 tembuf[128];
    u16 CRC = 0;
    u16 len = 0;
    
    FlushArray(tembuf,128);
    
    tembuf[0] = LocalAddr;
    tembuf[1] = Modp->FunCode;
    
    if((Modp->FunCode) == ReadR)
    {
        tembuf[2] = 2 * (Modp->Quantity);
        for(count = 0; count <= (2 * (Modp->Quantity)); count++)
        {
            if((count%2) == 0)
            {
                tembuf[3 + count] = ((RegV[(Modp->StartAddr) + (count / 2)]) & 0xFF00) >> 8;
            }
            else
            {
                tembuf[3 + count] = ((RegV[(Modp->StartAddr) + (count / 2)]) & 0x00FF);
            }
        }
        CRC = CalCRC(tembuf,(Modp->Quantity) * 2 + 3);
        tembuf[3 + count - 1] = (CRC & 0xFF00) >> 8;
        tembuf[3 + count ] = CRC & 0x00FF;   
        len = 4 + count ;
    }
    else
    {
        MBRR(Modp);
        tembuf[2] = *(Modp->Buff + SAddrH);
        tembuf[3] = *(Modp->Buff + SAddrL);
        tembuf[4] = *(Modp->Buff + ValH);
        tembuf[5] = *(Modp->Buff + ValL);
        CRC = CalCRC(tembuf,6);
        tembuf[6] = (CRC & 0xFF00) >> 8;
        tembuf[7] = CRC & 0x00FF; 
        len = 8;
    }
    
    uart_sendN(UARTPort, tembuf, len);
    
    FlushModbusCon(Modp);
}

u16 CalCRC(u8 *puchMsg,u16 usDataLen)
{
    u8 uchCRCHi = 0xFF ;    // 高CRC字节初始化 
    u16 uchCRCLo = 0xFF ;    // 低CRC 字节初始化
    u16 uIndex ;                  // CRC循环中的索引
    while (usDataLen--)                // 传输消息缓冲区  
    {
        uIndex = uchCRCHi ^ *puchMsg++ ; // 计算CRC         
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCLo = auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo) ;
}

void MBRR(Modbusp Modp)
{
    u16 Val = 0;
    Val = ((*(Modp->Buff + ValH)) << 8) | (*(Modp->Buff + ValL));
    RegV[(Modp->StartAddr)] = Val;
}
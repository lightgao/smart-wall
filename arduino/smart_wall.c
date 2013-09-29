#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <SPI.h>
#define uchar   unsigned char
#define uint    unsigned int
#define ulong   unsigned long


//----------------------------------------------------------------------------------------------------------


//#define     DEBUG_MOD

void debugInfo(char *str,  uint len=0, uchar *data=NULL)
{
#ifdef DEBUG_MOD
    Serial.println(str);
    if (len==0) return;
    for (int i=0; i<len; i++)
    {
        Serial.print(data[i], HEX);
        Serial.print(' ');
    }
    Serial.println("");
#endif
}

void debugInfo(char *str, char *text)
{
#ifdef DEBUG_MOD
    Serial.print(str);
    Serial.println(text);
#endif
}

//----------------------------------------------------------------------------------------------------------
 
int iTemp;
uchar ucTemp;

//Buzzer
const int buzzerPin = A0;

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Keypad
const byte ROWS=4;
const byte COLS=4;
byte keypadRowPins[ROWS] = {9,8,7,6};
byte keypadColPins[COLS] = {5,4,3,2};
char keys[ROWS][COLS] = {
  {'1','2','3','A'}, 
  {'4','5','6','B'}, 
  {'7','8','9','C'},
  {'*','0','#','D'} 
};
Keypad keypad = Keypad( makeKeymap(keys), keypadRowPins, keypadColPins, ROWS, COLS );

//Mode
#define  OPT_UNKNOWN    0XFF
#define  OPT_CREATE     8
#define  OPT_QUERY       9
#define  OPT_MOVE_NEXT  10
uchar mode = OPT_UNKNOWN;

//story prefix
const uchar STORY_PREFIX_NUMBER = 2;
char story_prefixes[STORY_PREFIX_NUMBER][8] = {
  {'R', 'C', 'B', '-', '\0'},
  {'M', 'Y', 'R', 'C', 'A', '-', '\0'}
};
uchar curStoryPrefix = 0;

//story number
char storyNumberBuffer[6] = {0};

//Current Swiping Card Information
uchar currentCardCapacity;
uchar currentCardId[5]={0};        //4 bytest card id, and 1 byte checksum
//Last Swiping status
uchar lastSwipingCardId[4]={0};
ulong lastSwipingCardTime=0;

//Which block we use to store story number in card?
#define    BLOCKNUM        4
//How many bytes we used in this block
#define BYTES_USED_IN_BLOCK 16

//
#define    OK              0x01
#define    NOTOK           0x00

//Message
#define  MSG_LEN           19				//(1 + 1 + 16 + 1)
#define  MSG_START_FLAG     0x3E
#define  MSG_END_FLAG       0xE3
uchar msgBuf[MSG_LEN] = {0};





void setup() {  
    //buzzer
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
    
    //ttl
    Serial.begin(9600);
    
    //i2c-lcd    
    lcd.begin();
    lcd.backlight();
    refreshLCD();

    //spi & rc522
    SPI.begin();
    MFRC522_Init();  

    //
    mode = OPT_QUERY;
}


//----------------------------------------------------------------------------------------------------------
const int chipSelectPin = 10;//如果控制板为UNO,328,168
//const int chipSelectPin = 53; //如果控制板为mega 2560,1280
//const int NRSTPD = 5;

uchar sectorKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//uchar defaultControlBlock[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff,0x07,0x80,0x69, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//uchar sectorKeys[15][16] = {
//    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
//    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
//    {0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5},
//    {0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5},
//    {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff},
//    {0x4d, 0x3a, 0x99, 0xc3, 0x51, 0xdd},
//    {0x1a, 0x98, 0x2c, 0x7e, 0x45, 0x9a},
//    {0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7},
//    {0x71, 0x4c, 0x5c, 0x88, 0x6e, 0x97},
//    {0x58, 0x7e, 0xe5, 0xf9, 0x35, 0x0f},
//    {0xa0, 0x47, 0x8c, 0xc3, 0x90, 0x91},
//    {0x53, 0x3c, 0xb6, 0xc7, 0x23, 0xf6},
//    {0x8f, 0xd0, 0xa4, 0xf2, 0x56, 0xe9},
//    {0xFF, 0xzz, 0xzz, 0xzz, 0xzz, 0xzz},
//    {0xA0, 0xzz, 0xzz, 0xzz, 0xzz, 0xzz}
//};


//buffer used to talk with rc522
#define MAX_BYTES_RECV_FROM_RC522_FIFO 16    //the max response data buffer length, ref: MFRC522_ToCard function
uchar rcv_buf_from_rc522[MAX_BYTES_RECV_FROM_RC522_FIFO];
                               
//MF522命令字
#define PCD_IDLE              0x00               //NO action;取消当前命令
#define PCD_AUTHENT           0x0E               //验证密钥
#define PCD_RECEIVE           0x08               //接收数据
#define PCD_TRANSMIT          0x04               //发送数据
#define PCD_TRANSCEIVE        0x0C               //发送并接收数据
#define PCD_RESETPHASE        0x0F               //复位
#define PCD_CALCCRC           0x03               //CRC计算
//Mifare_One卡片命令字
#define PICC_REQIDL           0x26               //寻天线区内未进入休眠状态
#define PICC_REQALL           0x52               //寻天线区内全部卡
#define PICC_ANTICOLL         0x93               //防冲撞
#define PICC_SElECTTAG        0x93               //选卡
#define PICC_AUTHENT1A        0x60               //验证A密钥
#define PICC_AUTHENT1B        0x61               //验证B密钥
#define PICC_READ             0x30               //读块
#define PICC_WRITE            0xA0               //写块
#define PICC_DECREMENT        0xC0               
#define PICC_INCREMENT        0xC1               
#define PICC_RESTORE          0xC2               //调块数据到缓冲区
#define PICC_TRANSFER         0xB0               //保存缓冲区中数据
#define PICC_HALT             0x50               //休眠
//和MF522通讯时返回的错误代码
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2
//---MFRC522寄存器
//Page 0:Command and Status
#define     Reserved00            0x00    
#define     CommandReg            0x01    
#define     CommIEnReg            0x02    
#define     DivlEnReg             0x03    
#define     CommIrqReg            0x04    
#define     DivIrqReg             0x05
#define     ErrorReg              0x06    
#define     Status1Reg            0x07    
#define     Status2Reg            0x08    
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
//Page 1:Command     
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
//Page 2:CFG    
#define     Reserved20            0x20  
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg              0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//Page 3:TestRegister     
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   
#define     TestADCReg            0x3B   
#define     Reserved31            0x3C   
#define     Reserved32            0x3D   
#define     Reserved33            0x3E   
#define     Reserved34            0x3F
/*
 * 函 数 名：Write_MFRC5200
 * 功能描述：向MFRC522的某一寄存器写一个字节数据
 * 输入参数：addr--寄存器地址；val--要写入的值
 * 返 回 值：无
 */
void Write_MFRC522(uchar addr, uchar val)
{
    digitalWrite(chipSelectPin, LOW);
    //地址格式：0XXXXXX0
    SPI.transfer((addr<<1)&0x7E);   
    SPI.transfer(val);
    
    digitalWrite(chipSelectPin, HIGH);
}
/*
 * 函 数 名：Read_MFRC522
 * 功能描述：从MFRC522的某一寄存器读一个字节数据
 * 输入参数：addr--寄存器地址
 * 返 回 值：返回读取到的一个字节数据
 */
uchar Read_MFRC522(uchar addr)
{
    uchar val;
    digitalWrite(chipSelectPin, LOW);
    //地址格式：1XXXXXX0
    SPI.transfer(((addr<<1)&0x7E) | 0x80);  
    val =SPI.transfer(0x00);
    
    digitalWrite(chipSelectPin, HIGH);
    
    return val; 
}
/*
 * 函 数 名：SetBitMask
 * 功能描述：置RC522寄存器位
 * 输入参数：reg--寄存器地址;mask--置位值
 * 返 回 值：无
 */
void SetBitMask(uchar reg, uchar mask)  
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}
/*
 * 函 数 名：ClearBitMask
 * 功能描述：清RC522寄存器位
 * 输入参数：reg--寄存器地址;mask--清位值
 * 返 回 值：无
 */
void ClearBitMask(uchar reg, uchar mask)  
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}
/*
 * 函 数 名：AntennaOn
 * 功能描述：开启天线,每次启动或关闭天险发射之间应至少有1ms的间隔
 * 输入参数：无
 * 返 回 值：无
 */
void AntennaOn(void)
{
    uchar temp;
    temp = Read_MFRC522(TxControlReg);
    if (!(temp & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}
/*
 * 函 数 名：AntennaOff
 * 功能描述：关闭天线,每次启动或关闭天险发射之间应至少有1ms的间隔
 * 输入参数：无
 * 返 回 值：无
 */
void AntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}
/*
 * 函 数 名：ResetMFRC522
 * 功能描述：复位RC522
 * 输入参数：无
 * 返 回 值：无
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}
/*
 * 函 数 名：InitMFRC522
 * 功能描述：初始化RC522
 * 输入参数：无
 * 返 回 值：无
 */
void MFRC522_Init(void)
{
    pinMode(chipSelectPin,OUTPUT);             // Set digital pin 10 as OUTPUT to connect it to the RFID /ENABLE pin 
    digitalWrite(chipSelectPin, LOW);          // Activate the RFID reader
    
//    pinMode(NRSTPD,OUTPUT);               // Set digital pin 10 , Not Reset and Power-down
//    digitalWrite(NRSTPD, HIGH);
    
    MFRC522_Reset();
        
    //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_MFRC522(TModeReg, 0x8D);      //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_MFRC522(TPrescalerReg, 0x3E); //TModeReg[3..0] + TPrescalerReg
    Write_MFRC522(TReloadRegL, 30);           
    Write_MFRC522(TReloadRegH, 0);
    
    Write_MFRC522(TxAutoReg, 0x40);     //100%ASK
    Write_MFRC522(ModeReg, 0x3D);       //CRC初始值0x6363  ???
    //ClearBitMask(Status2Reg, 0x08);       //MFCrypto1On=0
    //Write_MFRC522(RxSelReg, 0x86);        //RxWait = RxSelReg[5..0]
    //Write_MFRC522(RFCfgReg, 0x7F);        //RxGain = 48dB
    AntennaOn();        //打开天线
}
/*
 * 函 数 名：CalulateCRC
 * 功能描述：用MF522计算CRC
 * 输入参数：pIndata--要读数CRC的数据，len--数据长度，pOutData--计算的CRC结果
 * 返 回 值：无
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;
    ClearBitMask(DivIrqReg, 0x04);          //CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);         //清FIFO指针
    //Write_MFRC522(CommandReg, PCD_IDLE);
    //向FIFO中写入数据    
    for (i=0; i<len; i++)
    {   
        Write_MFRC522(FIFODataReg, *(pIndata+i));   
    }
    Write_MFRC522(CommandReg, PCD_CALCCRC);
    //等待CRC计算完成
    i = 0xFF;
    do 
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));            //CRCIrq = 1
    //读取CRC计算结果
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}
/*
 * 函 数 名：MFRC522_ToCard
 * 功能描述：RC522和ISO14443卡通讯
 * 输入参数：command--MF522命令字，
 *           sendData--通过RC522发送到卡片的数据, 
 *           sendLen--发送的数据长度        
 *           backData--接收到的卡片返回数据，
 *           backLen--返回数据的位长度
 * 返 回 值：成功返回MI_OK
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    uint i;
    switch (command)
    {
        case PCD_AUTHENT:       //认证卡密
        {
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        }
        case PCD_TRANSCEIVE:    //发送FIFO中数据
        {
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        }
        default:
            break;
    }
   
    Write_MFRC522(CommIEnReg, irqEn|0x80);  //允许中断请求
    ClearBitMask(CommIrqReg, 0x80);         //清除所有中断请求位
    SetBitMask(FIFOLevelReg, 0x80);         //FlushBuffer=1, FIFO初始化
    
    Write_MFRC522(CommandReg, PCD_IDLE);    //NO action;取消当前命令  ???
    //向FIFO中写入数据
    for (i=0; i<sendLen; i++)
    {   
        Write_MFRC522(FIFODataReg, sendData[i]);    
    }
    //执行命令
    Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {    
        SetBitMask(BitFramingReg, 0x80);        //StartSend=1,transmission of data starts  
    }   
    
    //等待接收数据完成
    i = 2000;   //i根据时钟频率调整，操作M1卡最大等待时间25ms ???
    do 
    {
        //CommIrqReg[7..0]
        //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));
    ClearBitMask(BitFramingReg, 0x80);          //StartSend=0
    
    if (i != 0)
    {    
        if(!(Read_MFRC522(ErrorReg) & 0x1B))    //BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {   
                status = MI_NOTAGERR;           //??   
            }
            if (command == PCD_TRANSCEIVE)
            {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {   
                    *backLen = (n-1)*8 + lastBits;   
                }
                else
                {   
                    *backLen = n*8;   
                }
                if (n == 0)
                {   
                    n = 1;    
                }
                if (n > MAX_BYTES_RECV_FROM_RC522_FIFO)
                {   
                    n = MAX_BYTES_RECV_FROM_RC522_FIFO;   
                }
                
                //读取FIFO中接收到的数据
                for (i=0; i<n; i++)
                {   
                    backData[i] = Read_MFRC522(FIFODataReg);    
                }
            }
        }
        else
        {   
            status = MI_ERR;  
        }
        
    }
    
    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE);
    return status;
}
/*
 * 函 数 名：MFRC522_Request
 * 功能描述：寻卡，读取卡类型号
 * 输入参数：reqMode--寻卡方式，
 *           TagType--返回卡片类型
 *              0x4400 = Mifare_UltraLight
 *              0x0400 = Mifare_One(S50)
 *              0x0200 = Mifare_One(S70)
 *              0x0800 = Mifare_Pro(X)
 *              0x4403 = Mifare_DESFire
 * 返 回 值：成功返回MI_OK
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
    uchar status;  
    uint backBits;          //接收到的数据位数
    Write_MFRC522(BitFramingReg, 0x07);     //TxLastBists = BitFramingReg[2..0] ???
    
    TagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
    if ((status != MI_OK) || (backBits != 0x10))
    {    
        status = MI_ERR;
    }
   
    return status;
}
/*
 * 函 数 名：MFRC522_Anticoll
 * 功能描述：防冲突检测，读取选中卡片的卡序列号
 * 输入参数：serNum--返回4字节卡序列号,第5字节为校验字节
 * 返 回 值：成功返回MI_OK
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
    uchar serNumCheck=0;
    uint unLen;
   
    //ClearBitMask(Status2Reg, 0x08);       //TempSensclear
    //ClearBitMask(CollReg,0x80);           //ValuesAfterColl
    Write_MFRC522(BitFramingReg, 0x00);     //TxLastBists = BitFramingReg[2..0]
 
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
    if (status == MI_OK)
    {
        //校验卡序列号
        for (i=0; i<4; i++)
        {   
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i])
        {   
            status = MI_ERR;    
        }
    }
    //SetBitMask(CollReg, 0x80);        //ValuesAfterColl=1
    return status;
}
/*
 * 函 数 名：MFRC522_SelectTag
 * 功能描述：选卡，读取卡存储器容量
 * 输入参数：serNum--传入卡序列号
 * 返 回 值：成功返回卡容量
 */
uchar MFRC522_SelectTag(uchar *serNum)
{
    uchar i;
    uchar status;
    uchar size;
    uint recvBits;
    uchar buffer[9];
    //ClearBitMask(Status2Reg, 0x08);           //MFCrypto1On=0
    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
        buffer[i+2] = *(serNum+i);
    }
    CalulateCRC(buffer, 7, &buffer[7]);     //??
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    
    if ((status == MI_OK) && (recvBits == 0x18))
    {   
        size = buffer[0]; 
    }
    else
    {   
        size = 0;    
    }
    return size;
}
/*
 * 函 数 名：MFRC522_Auth
 * 功能描述：验证卡片密码
 * 输入参数：authMode--密码验证模式
                 0x60 = 验证A密钥
                 0x61 = 验证B密钥 
             BlockAddr--块地址
             Sectorkey--扇区密码
             serNum--卡片序列号，4字节
 * 返 回 值：成功返回MI_OK
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
{
    uchar status;
    uint recvBits;
    uchar i;
    uchar buff[12];
    //验证指令+块地址＋扇区密码＋卡序列号
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {    
        buff[i+2] = *(Sectorkey+i);   
    }
    for (i=0; i<4; i++)
    {    
        buff[i+8] = *(serNum+i);   
    }
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);
    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {   
        status = MI_ERR;   
    }
    
    return status;
}
/*
 * 函 数 名：MFRC522_Read
 * 功能描述：读块数据
 * 输入参数：blockAddr--块地址;recvData--读出的块数据
 * 返 回 值：成功返回MI_OK
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
{
    uchar status;
    uint unLen;
    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }
    
    return status;
}
/*
 * 函 数 名：MFRC522_Write
 * 功能描述：写块数据
 * 输入参数：blockAddr--块地址;writeData--向块写16字节数据
 * 返 回 值：成功返回MI_OK
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint recvBits;
    uchar i;
    uchar buff[18]; 
    
    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);
    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {   
        status = MI_ERR;   
    }
        
    if (status == MI_OK)
    {
        for (i=0; i<16; i++)        //向FIFO写16Byte数据
        {    
            buff[i] = *(writeData+i);   
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
        
        if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {   
            status = MI_ERR;   
        }
    }
    
    return status;
}
/*
 * 函 数 名：MFRC522_Halt
 * 功能描述：命令卡片进入休眠状态
 * 输入参数：无
 * 返 回 值：无
 */
void MFRC522_Halt(void)
{
    uchar status;
    uint unLen;
    uchar buff[4];
    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);
 
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}

//------------------------------------------------------------------------------------------------

#define BEEP(_time)                \
{                                  \
  digitalWrite(buzzerPin, HIGH);   \
  delay(_time);                   \
  digitalWrite(buzzerPin, LOW);    \
}

void refreshLCD() 
{
    lcd.clear();
    switch(mode) 
    {
        case OPT_CREATE:
            lcd.print(story_prefixes[curStoryPrefix]);
            lcd.print(storyNumberBuffer);
            break;
        case OPT_QUERY:
            lcd.print("Query ");
            break;
        case OPT_MOVE_NEXT:
            lcd.print("Move ");
            break;
        default:
            lcd.print("Smart Wall :)");
            break;
    }
}

uchar initCard(uchar blockAddr) {
    uchar flag = 0;
    if (MFRC522_Request(PICC_REQIDL, rcv_buf_from_rc522) == MI_OK)
    {
        debugInfo("request idle ok, type bytes:", 2, rcv_buf_from_rc522);
        if(rcv_buf_from_rc522[0] == 0x44 && rcv_buf_from_rc522[1]==0x00)
            debugInfo("Card Type: Mifare_UltraLight");
        else if (rcv_buf_from_rc522[0] == 0x04 && rcv_buf_from_rc522[1]==0x00)
            debugInfo("Card Type: Mifare_One_S50");
        else if (rcv_buf_from_rc522[0] == 0x04 && rcv_buf_from_rc522[1]==0x00)
            debugInfo("Card Type: Mifare_One_S70");
        else if (rcv_buf_from_rc522[0] == 0x04 && rcv_buf_from_rc522[1]==0x00)
            debugInfo("Card Type: Mifare_Pro_X");
        else if (rcv_buf_from_rc522[0] == 0x04 && rcv_buf_from_rc522[1]==0x00)
            debugInfo("Card Type: Mifare_DESFire");
        else
            debugInfo("Card Type: Unknown!!!");
        flag |= 0x01;
    }
    if (MFRC522_Anticoll(rcv_buf_from_rc522) == MI_OK) 
    {
        debugInfo("anticoll ok. card id: ", 5, rcv_buf_from_rc522);
        memcpy(currentCardId, rcv_buf_from_rc522, 5);
        flag |= 0x02;
    }
    currentCardCapacity = MFRC522_SelectTag(currentCardId);
    if (currentCardCapacity != 0)
    {
        debugInfo("select ok, card capacity [K bit] :", 1, &currentCardCapacity);
        flag |= 0x04;
    }
    if (MFRC522_Auth(PICC_AUTHENT1A, blockAddr, sectorKey, currentCardId) == MI_OK)
    {
        debugInfo("auth ok");
        flag |= 0x08;
    }
    
    if(flag == 0x0f) 
    {
        debugInfo("SELETC OK ");
        return OK;
    }
    return NOTOK;
}

uchar readCard(uchar blockAddr, uchar *data) {
    uchar status;
    status = MFRC522_Read(blockAddr, data);
    if (status == MI_OK)
    {
        debugInfo("read card ok:", 16, data);
        return OK;
    }
    return NOTOK;
}

uchar writeCard(uchar blockAddr, uchar *data) {
    uchar status;
    status = MFRC522_Write(blockAddr, data);
    if (status == MI_OK)
    {
        debugInfo("write card ok:", 16, data);
        return OK;
    }
    return NOTOK;
}

void processCard()
{
    uchar writeData[BYTES_USED_IN_BLOCK];
    char* storyNumber = (char*)writeData;

    //find and open card
    //  currentCardId & currentCardCapacity will be updated
    if(initCard(BLOCKNUM) == NOTOK)
    {
        MFRC522_Halt();
        return;
    } 
    
    //get current story number from card
    debugInfo("Get Current Story Number");
    memset(rcv_buf_from_rc522, 0, MAX_BYTES_RECV_FROM_RC522_FIFO);
    if(readCard(BLOCKNUM, rcv_buf_from_rc522)==NOTOK)
    {
        MFRC522_Halt();
        return;
    }
    memcpy(storyNumber, rcv_buf_from_rc522, BYTES_USED_IN_BLOCK);
    
    //detect query & move mode?
    unsigned long currentTime = millis();
    if( (mode==OPT_QUERY || mode==OPT_MOVE_NEXT) )
    {
//        if(lastSwipingCardTime > currentTime)          //when timer overflow
//            mode = OPT_QUERY;
//        else if( (memcmp(lastSwipingCardId, currentCardId, sizeof(lastSwipingCardId))==0)  && (currentTime-lastSwipingCardTime)<10000 )    //when swiping same card twice in then seconds
//            mode = OPT_MOVE_NEXT;
//        else                                            //swiping another card
//            mode = OPT_QUERY;
//        
//        lastSwipingCardTime = currentTime;
//        memcpy(lastSwipingCardId, currentCardId, sizeof(lastSwipingCardId));

        if(lastSwipingCardTime > currentTime)          //when timer overflow
            mode = OPT_QUERY;
        else if( mode==OPT_MOVE_NEXT )                //already move? back to query
            mode = OPT_QUERY;
        else if( (memcmp(lastSwipingCardId, currentCardId, sizeof(lastSwipingCardId))==0) && (currentTime-lastSwipingCardTime)<10000 )    //when swiping same card twice in then seconds
            mode = OPT_MOVE_NEXT;
        else                                            //swiping another card
            mode = OPT_QUERY;
        
        lastSwipingCardTime = currentTime;
        memcpy(lastSwipingCardId, currentCardId, sizeof(lastSwipingCardId));
    }
   
    //send request to proxy to operate story card       
    if(mode == OPT_CREATE) 
    {
        //assemble story id
        memset(writeData, 0, sizeof(writeData));
        strcpy((char*)writeData, story_prefixes[curStoryPrefix]);
        iTemp = strlen((char*)writeData);
        strcpy((char*)(writeData+iTemp), storyNumberBuffer);
        
        debugInfo("> Create Card : ", (char*)writeData);

        //save story id to card     
        writeCard(BLOCKNUM, writeData);    
        
        BEEP(50); delay(60); BEEP(50); delay(60); BEEP(50);

        //back to query mode
        memset(storyNumberBuffer, 0, sizeof(storyNumberBuffer));
        mode=OPT_QUERY;
        refreshLCD();
    } 
    else if(mode == OPT_QUERY) 
    {
        debugInfo("> Query Card : ", storyNumber);

        //generate and send out request
        memset(msgBuf, 0x00, MSG_LEN); msgBuf[0] = MSG_START_FLAG; msgBuf[MSG_LEN-1] = MSG_END_FLAG;
        msgBuf[1] = OPT_QUERY;
        memcpy(&msgBuf[2], storyNumber, BYTES_USED_IN_BLOCK);
        debugInfo("Send Msg: ", MSG_LEN, msgBuf);
        debugInfo("- Msg Start -");
        Serial.write(msgBuf, MSG_LEN);
        debugInfo("- Msg End -");
        
        BEEP(60);

        //output card number on lcd
        refreshLCD();
        lcd.print(storyNumber);
    } 
    else if(mode == OPT_MOVE_NEXT) 
    {
        debugInfo("> Move Card : ", storyNumber);

        //generate and send out request
        memset(msgBuf, 0x00, MSG_LEN); msgBuf[0] = MSG_START_FLAG; msgBuf[MSG_LEN-1] = MSG_END_FLAG;
        msgBuf[1] = OPT_MOVE_NEXT;
        memcpy(&msgBuf[2], storyNumber, BYTES_USED_IN_BLOCK);
        debugInfo("Send Msg: ", MSG_LEN, msgBuf);
        debugInfo("- Msg Start -");
        Serial.write(msgBuf, MSG_LEN);
        debugInfo("- Msg End -");
        
        BEEP(40); delay(50); BEEP(40);

        //output card number on lcd
        refreshLCD();
        lcd.print(storyNumber);
    }
    else
    {
        debugInfo("> Unknow Mode");
    }

    MFRC522_Halt();    
}

void gatherInputData()
{
    char key=keypad.getKey();
    if(key==NO_KEY)  return; 
    
    //detect whether change to create mode? OR change back to read mode
    if( (key=='#' || (key>='0' && key<='9'))
         && mode != OPT_CREATE )
    {
        mode=OPT_CREATE;
        memset(storyNumberBuffer, 0, sizeof(storyNumberBuffer));     
    }
    if( mode==OPT_CREATE && key=='*')
    {
        mode=OPT_QUERY;
        refreshLCD();
        BEEP(80);
    }
    
    //
    if(mode!=OPT_CREATE)  return;
    
    //change project prefix for create mode
    if(key=='#')
    {
        curStoryPrefix++;
        if(curStoryPrefix >= STORY_PREFIX_NUMBER)  curStoryPrefix = 0;

        BEEP(50); delay(100); BEEP(50);
    }
  
    //get story number in create mode
    else if(key>='0' && key<='9')
    {
        int iTemp = strlen(storyNumberBuffer);
        if( iTemp >= (sizeof(storyNumberBuffer)-1) )
        {
            debugInfo("too much data");
            BEEP(20); delay(30); BEEP(20);
        }
        else
        {
            storyNumberBuffer[iTemp] = key;
            BEEP(30);
        }
    }
    
    refreshLCD();
}

void loop()
{ 
    gatherInputData();
    
    processCard();
}

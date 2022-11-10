//
//  SEGA MDmini Converter2 
//  - MD/CyberStick to USB Converter -
//
//  MegaDrivemini2 Cyberstickモード対応版  
//  Copyright 2022 @v9938
//
//  メガドライブ対応コントローラ(3B/6B),
//	SHARP CyberStick(CZ-8NJ2),マイコンソフト XE-1AJを
//  DirectInput方式のUSBコントローラとして使える様にするコンバータです。  
//

// MicroChipのライセンス
/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

#ifndef USBJOYSTICK_C
#define USBJOYSTICK_C



/** INCLUDES *******************************************************/
#include "usb.h"
#include "usb_device_hid.h"
#include "system.h"
#include "stdint.h"
#include <xc.h>


#define _XTAL_FREQ 48000000 // delay用に必要(クロック12MHzを指定)

#define _delay(x) _delay((unsigned long)((x)))
#define _delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000UL)))
#define _delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000UL)))


#define TIMEOUTMAX 200
#define ERR_TIMEOUT 1
#define ERR_NONE 0

//スイッチ関係の設定
#define TRISMODE_EQMD		(TRISB == 0xD0)
#define DSUBPIN8_EQGND		(PORTBbits.RB6==0)
#define DSUBPIN9_EQGND		(PORTBbits.RB7==0)


#define MDMODE_LEDON        PORTCbits.RC7 = 1
#define MDMODE_LEDOFF       PORTCbits.RC7 = 0
#define MDMODE_EQON       	(PORTCbits.RC7 == 1)

#define CYBER_REQOFF        PORTBbits.RB6 = 1
#define CYBER_REQON         PORTBbits.RB6 = 0
#define CYBER_LH_EQHIGH   	(PORTBbits.RB4 == 1)
#define CYBER_ACK_EQHIGH  	(PORTBbits.RB5 == 1)
#define CYBER_LH_EQLOW    	(PORTBbits.RB4 == 0)
#define CYBER_ACK_EQLOW   	(PORTBbits.RB5 == 0)

#define CYBERMD_REQOFF        PORTBbits.RB5 = 1
#define CYBERMD_REQON         PORTBbits.RB5 = 0
#define CYBERMD_LH_EQHIGH   	(PORTBbits.RB4 == 1)
#define CYBERMD_ACK_EQHIGH  	(PORTBbits.RB7 == 1)
#define CYBERMD_LH_EQLOW    	(PORTBbits.RB4 == 0)
#define CYBERMMD_ACK_EQLOW   	(PORTBbits.RB7 == 0)

#define E2PROM_MAGICNUM		0xA3
//3Bモードの長押しLOOP数 8ms*400 =約3.2秒
#define MD3B_MODE_TIMEOUT	400
//連射ON/OFFのMODE/SELECTスイッチの長押しLOOP数 16ms*255 =約5秒
#define RAPIDSW_TIMEOUT 	313

#define HID_INTERVAL_1MS        1
#define HID_INTERVAL_2MS        2
#define HID_INTERVAL_4MS        3
#define HID_INTERVAL_8MS        4

#define FRAME_RATE_US			16685
#define USB_RATE_US				16000

///////////////////////////////////////////////////////////////////////
// Global Value
///////////////////////////////////////////////////////////////////////
unsigned char IncomingHIDCBSetReport =0;
unsigned char CyberData[14];			//
unsigned short Md3bModeSWCount;			// 3ボタンモードのスタートボタン長押しのカウント
unsigned short sDataModeSWCountOut;		// スタートボタン長押しのカウントのタイムアウト値(8ms x タイムアウト値)
unsigned short sFrameDiffCount;			// コントローラのアクセス時間と実機のスキャン間隔との差分値
unsigned char bDataRapidSpeedA;			//連射ボタンの分周設定値(STEP1)
unsigned char bDataRapidSpeedB;			//連射ボタンの分周設定値(STEP2)
unsigned char RapidSpeedCount;			//連射ボタンの分周カウント

unsigned char RapidCount;				// 連射テーブルのカウンター
unsigned char RapidCountOut;			// 連射テーブルの切り替えカウント値
unsigned char RapidTable;				// 連射テーブルの選択テーブル番号
unsigned char RapidEnable;				// 連射有効/無効スイッチ
unsigned char RapidStart;				// 連射Start flag
unsigned short RapidSWCount;			// 連射有効化/無効化までのSELECT長押しのカウント
unsigned short bRapidSWCountOut;		// 連射有効化/無効化までのSELECT長押しのカウント設定値


int CyberMode;							// CyberStickの入力状態 (0=正常[アナログモード] 1=TIMEOUT[デジタルモード])
int XE1APMode;							// XE1AP(メガドラモード）フラグ

uint8_t centerX,centerY,centerZ;		// CyberStickの入力の中心遊び値
uint8_t muxXY, muxZ;					// CyberStickの移動量(1=0.5x,2=1.0x,3=1.5x,4=2.0x)
unsigned short slewX,slewY,slewZ;		// 設定値から計算できる増分値

///////////////////////////////////////////////////////////////////////
// Global Value ボタンのマスクデータ
///////////////////////////////////////////////////////////////////////
unsigned char bDataCyber[20];			//CyberStickモード用
unsigned char bDataCyMDmode[20];		//CyberStick(MD6B)モード用
unsigned char bDataCyMDXY;				//CyberStick(MD6B)モード用XY格納位置番号
unsigned char bDataCyMDZ;				//CyberStick(MD6B)モード用Z格納位置番号
unsigned char bDataCyberXY;				//CyberStickモード用XY格納位置番号
unsigned char bDataCyberZ;				//CyberStickモード用Z格納位置番号
unsigned char bDataCyberRvXYZ;			//CyberStickモード用XYZ反転フラグ
unsigned char bDataMD6B[16];			//Fighting Pad 6Bモード用
unsigned char bDataMD6BXY ;				//Fighting Pad 6Bモード用XY格納位置番号

unsigned char bDataMD6BRapid[4];		//Fighting Pad 6Bモード用連射ボタン設定
unsigned char bDataCyberRapid[4];		//CyberStickモード用連射ボタン設定
unsigned char DataMD6BRapidMask[2];		//Fighting Pad 6Bモード用連射ボタンマスク値
unsigned char DataCyberRapidMask[2];	//CyberStickモード用連射ボタンマスク値



//Cyber Stick ボタンデータ配置
// val4/cal5
// 0/1: 		Aボタンの配置情報
// 2/3: 		Bボタンの配置情報
// 4/5: 		Cボタンの配置情報
// 6/7: 		Dボタンの配置情報
// 8/9: 		E1ボタンの配置情報
// 10/11: 		E2ボタンの配置情報
// 12/13: 		Startボタンの配置情報
// 14/15: 		Selectボタンの配置情報
// 16/17: 		A'ボタンの配置情報
// 18/19: 		B'ボタンの配置情報




//MD6B ボタンデータ配置
// val4/cal5
// 0/1: 		Aボタンの配置情報
// 2/3: 		Bボタンの配置情報
// 4/5: 		Cボタンの配置情報
// 6/7: 		Xボタンの配置情報
// 8/9: 		Yボタンの配置情報
// 10/11: 		Zボタンの配置情報
// 12/13: 		Startボタンの配置情報
// 14/15: 		MODEボタンの配置情報

#define MD6B_A						0
#define MD6B_B						2
#define MD6B_C						4
#define MD6B_X						6
#define MD6B_Y						8
#define MD6B_Z						10
#define MD6B_START					12
#define MD6B_MODE					14

#define CYBER_A						0
#define CYBER_B						2
#define CYBER_C						4
#define CYBER_D						6
#define CYBER_E1					8
#define CYBER_E2					10
#define CYBER_START					12
#define CYBER_SELECT				14
#define CYBER_Ad					16
#define CYBER_Bd					18

//XYZのIN PACKET位置
#define MD6B_XY_STICK				0x3
#define MD6B_Z_STICK				0x2

#define CYBER_XY_STICK				0x3
#define CYBER_Z_STICK				0x6
#define CYBER_HAT_STICK				0x2

//ボタンのIN PACKETフラグ位置
#define MD6B_BUTTON1_ON				0x10
#define MD6B_BUTTON2_ON				0x20
#define MD6B_BUTTON3_ON				0x40
#define MD6B_BUTTON4_ON				0x80
#define MD6B_BUTTON5_ON				0x01
#define MD6B_BUTTON6_ON				0x02
#define MD6B_BUTTON7_ON				0x04
#define MD6B_BUTTON8_ON				0x08
#define MD6B_BUTTON9_ON				0x10
#define MD6B_BUTTON10_ON			0x20

#define CYBER_BUTTON1_ON			0x01
#define CYBER_BUTTON2_ON			0x02
#define CYBER_BUTTON3_ON			0x04
#define CYBER_BUTTON4_ON			0x08
#define CYBER_BUTTON5_ON			0x10
#define CYBER_BUTTON6_ON			0x20
#define CYBER_BUTTON7_ON			0x40
#define CYBER_BUTTON8_ON			0x80

// Hut data
#define HAT_UP           0x00
#define HAT_UP_RIGHT     0x01
#define HAT_RIGHT        0x02
#define HAT_DOWN_RIGHT   0x03
#define HAT_DOWN         0x04
#define HAT_DOWN_LEFT    0x05
#define HAT_LEFT         0x06
#define HAT_UP_LEFT      0x07
#define HAT_CENTER       0x08

//CyberStickのXYZ軸を反転させるかのフラグ位置
#define CYBER_MD6B_RVX			0x01
#define CYBER_MD6B_RVY			0x02
#define CYBER_MD6B_RVZ			0x04
#define CYBER_RVX				0x10
#define CYBER_RVY				0x20
#define CYBER_RVZ				0x40
#define CYBER_HATRVZ			0x80


#define VAL_MD6BDATA(r)			{joystick_input.val[5] |= bDataMD6B[r]; 	joystick_input.val[6] |= bDataMD6B[r+1];	}
#define VAL_MD6BCYBERDATA(r)	{joystick_input.val[5] |= bDataCyMDmode[r]; joystick_input.val[6] |= bDataCyMDmode[r+1];}
#define VAL_CYBERDATA(r)		{joystick_input.val[0] |= bDataCyber[r]; 	joystick_input.val[1] |= bDataCyber[r+1];	}


/** TYPE DEFINITIONS ************************************************/
typedef union _INTPUT_CONTROLS_TYPEDEF
{
    uint8_t val[8];
} INPUT_CONTROLS;

typedef union _OUTTPUT_CONTROLS_TYPEDEF
{
    uint8_t val[64];
} EP2OUTPUT_CONTROLS;

typedef union _INT2PUT_CONTROLS_TYPEDEF
{
    uint8_t val[64];
} EP2INPUT_CONTROLS;


/** VARIABLES ******************************************************/
/* Some processors have a limited range of RAM addresses where the USB module
 * is able to access.  The following section is for those devices.  This section
 * assigns the buffers that need to be used by the USB module into those
 * specific areas.
 */
#if defined(FIXED_ADDRESS_MEMORY)
    #if defined(COMPILER_MPLAB_C18)
        #pragma udata JOYSTICK_DATA=JOYSTICK_DATA_ADDRESS
            INPUT_CONTROLS joystick_input;
        #pragma udata
    #elif defined(__XC8)
        INPUT_CONTROLS joystick_input @ JOYSTICK_DATA_ADDRESS;
    #endif
#else
    INPUT_CONTROLS joystick_input;
#endif

#if defined(FIXED_ADDRESS_MEMORY)
    EP2INPUT_CONTROLS ToSendDataBuffer @ HID_CUSTOM_IN_DATA_BUFFER_ADDRESS;
    EP2OUTPUT_CONTROLS ReceivedDataBuffer@ HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS;;
#else
    EP2INPUT_CONTROLS ToSendDataBuffer;
    EP2OUTPUT_CONTROLS ReceivedDataBuffer;
#endif

    
USB_VOLATILE USB_HANDLE lastTransmissionEP1 = 0;
USB_VOLATILE USB_HANDLE lastTransmissionEP2 = 0;
USB_VOLATILE USB_HANDLE lastReceive = 0;



/*********************************************************************
* Function: void USBHIDCBSetReportComplete(void)
* Overview: MLA側からHID SetReportの処理完了時にCallされるフック
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
static void USBHIDCBSetReportComplete(void)
{
    // MLA USB Frame WorkからCallされる。
    // SetReportが来た事をが分かる様にフラグを立てておく
    IncomingHIDCBSetReport = true;
}

/*********************************************************************
* Function: USBHIDCBSetReportHandler(void);
* Overview: MLA側からHID SetReportの実処理をしてCallされるフック
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void USBHIDCBSetReportHandler(void)
{
    /* Prepare to receive the HID data through a SET_REPORT
     * control transfer on endpoint 0.  The host should only send 1 byte,
     * since this is all that the report descriptor allows it to send. */

    // SetReportでOUTパケットが来る場合は、EP0側にパケットが来るので
    // EP2側のバッファーに転送してやり、通常のOUTパケットと同様に処理できる様にする。 
    USBEP0Receive((uint8_t*)&ReceivedDataBuffer, USB_EP0_BUFF_SIZE, USBHIDCBSetReportComplete);

}

/*********************************************************************
* Function: void calcSlew(void);
* Overview: CyberStickのXYZのSlew値を計算して設定する
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void calcSlew(void)
{
	// Slewの割り算は毎回やっていたらペナルティーが大きいので
	// あらかじめ計算してGlobal変数に突っ込んでおく
    slewX = 0x2000/(128-centerX);
    slewY = 0x2000/(128-centerY);
    slewZ = 0x2000/(128-centerZ);
}

/*********************************************************************
* Function: void calcRapidCountOut(void);
* Overview: USBの転送時間設定からRapidCountの処理をする固定値を計算
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void calcRapidCountOut(void)
{
	// RapidCountOutは固定値なので事前に代入しておく
	RapidCountOut = 2-1; //HID_INTERVAL_8MSの設定値
    if (HID_Interval == HID_INTERVAL_1MS) RapidCountOut = 16-1;
    if (HID_Interval == HID_INTERVAL_2MS) RapidCountOut = 8-1;
    if (HID_Interval == HID_INTERVAL_4MS) RapidCountOut = 4-1;

	//マスク値計算
    DataMD6BRapidMask[0] = 0xff ^ (bDataMD6BRapid[0] | bDataMD6BRapid[1]);
    DataMD6BRapidMask[1] = 0xff ^ (bDataMD6BRapid[2] | bDataMD6BRapid[3]);
    DataCyberRapidMask[0] = 0xff ^ (bDataCyberRapid[0] | bDataCyberRapid[1]);
    DataCyberRapidMask[1] = 0xff ^ (bDataCyberRapid[2] | bDataCyberRapid[3]);

    //ついでに必要な変数も初期化
	sFrameDiffCount = 0;
	RapidSpeedCount = 0;
	RapidCount = 0;
	RapidTable = 0;

}

/*********************************************************************
* Function: void calcRapidTable(unsigned char,unsigned char);
* Overview: シンクロ連射のTable番号の更新処理を行う
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void calcRapidTable(unsigned char stepAmask,unsigned char stepBmask)
{
   //16ms経過したかチェック
    if (RapidCount >= RapidCountOut) {
		RapidCount = 0;														//カウンター初期化
		sFrameDiffCount = sFrameDiffCount +(FRAME_RATE_US - USB_RATE_US);	//フレームレートと送信差を積算する

		if (sFrameDiffCount > USB_RATE_US)	{								//フレームレート差が送信間隔を超えた場合
			sFrameDiffCount = sFrameDiffCount - USB_RATE_US;				//フレームレート値から送信間隔をさっ引く
																			//テーブル値の更新はスキップ
		} else {
			if ((RapidTable == 0) && (RapidSpeedCount >= bDataRapidSpeedA))	{						//分周値と比較
				RapidSpeedCount = 0;										//カウント値リセット
				RapidTable = 0x01;											//ABテーブルの切り替え
			}else if ((RapidTable == 1) && (RapidSpeedCount >= bDataRapidSpeedB))	{						//分周値と比較
				RapidSpeedCount = 0;										//カウント値リセット
				RapidTable = 0x00;											//ABテーブルの切り替え
			}else{
				RapidSpeedCount++;
			}
		}
	}else{
		RapidCount++;														//16ms未満なのでカウンター値のみ更新
	}

	if ((stepAmask != 0) && (RapidStart)){
		RapidSpeedCount = 0;										//カウント値リセット
		RapidTable = 0x00;											//Aテーブルの切り替え
	}
	
	if ((stepBmask != 0) && (RapidStart)){
		RapidSpeedCount = 0;										//カウント値リセット
		RapidTable = 0x01;											//Bテーブルの切り替え
	}

	if ((stepAmask != 0) || (stepBmask != 0))	RapidStart = false;
	else	RapidStart = true;

}


/*********************************************************************
* Function: calcRapidEnableSwitch(unsigned char)
* Overview: シンクロ連射の有効無効キー処理を行う
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void calcRapidEnableSwitch(unsigned char swFlag)
{
	//連射スイッチの有効無効スイッチの処理
    if (!swFlag){
		if (RapidCount == 0) {													// 16ms毎に0になる
			if (RapidSWCount < bRapidSWCountOut) RapidSWCount++;				// 押しっぱなしの間カウントアップ
		}
	}else{
		if (RapidSWCount == bRapidSWCountOut) RapidEnable = RapidEnable ^ 0x1;		//modeボタンを放した時にフラグ逆転
		RapidSWCount = 0;
	}
}


/*********************************************************************
* Function: unsigned char calFixedVal(unsigned char,unsigned char,unsigned char,unsigned short);
* Overview: CyberStickの出力値からUSB用のデータを補正して生成する。
* PreCondition: None
* Input: Cyber生値,センター遊び値,移動率(1x,1.5x,2x),移動量
* Output: 補正値
*
********************************************************************/

unsigned char calFixedVal(unsigned char rawVal,unsigned char centerVal,unsigned char muxVal,unsigned short Slew)
{
    unsigned short calc,subRaw;
    
    // 遊び値範囲内の場合はセンター値を返す
    if ((rawVal <= (0x80+centerVal)) && (rawVal >= (0x80-centerVal))){
        return 0x80;
    }

	// 以下の計算は精度と速度を狙って、128倍した値で補正計算を行う
	// センタからマイナス側の計算
    if (rawVal<0x80){
		//差分量計算 (センター位置からのオフセット量計算）
        subRaw = 0x80 -(unsigned short) centerVal - (unsigned short)rawVal;
        calc = subRaw * Slew * (unsigned short) muxVal;
		//桁あふれ対策
        if (calc >= 0x4000) calc = 0x4000;
        calc = 0x4000 - calc;
    }

	// センタからプラス側の計算
   if (rawVal>0x80){
		//差分量計算 (センター位置からのオフセット量計算）
        subRaw = (unsigned short)rawVal-0x7f-(unsigned short) centerVal;
        calc = subRaw * Slew * (unsigned short) muxVal;
		//桁あふれ対策
        if (calc >= 0x4000) calc = 0x4000;
        calc = 0x3fff + calc;
    }
    return (unsigned char) (0xff & (calc /128));		// 1/128して8bitで値を返す
}

/*********************************************************************
* Function: unsigned char eepromReadByte(unsigned char)
* Overview: E2PROM領域からのRead
* PreCondition: None
* Input: E2PROMのAddress
* Output: 読み出し値
*
********************************************************************/
unsigned char eepromReadByte(unsigned char address)
{
    EEADR = address;            						// Address Set
    EECON1bits.EEPGD = 0;       						// Select E2PROM BANK
    EECON1bits.CFGS = 0;        
    EECON1bits.RD = 1;          						// E2PROM Read
    while(EECON1bits.RD == 1);  						// Wait for E2PROM Read
    return EEDATA;              						// Return E2PROM Data
}


/*********************************************************************
* Function: void eepromWriteByte(unsigned char, unsigned char)
* Overview: E2PROM領域からのRead
* PreCondition: None
* Input: E2PROMのAddress、書き込み値
* Output: 無し
*
********************************************************************/
void eepromWriteByte(unsigned char address, unsigned char data)
{
	char GIE_on;
    EEADR = address;                					// Address Set
    EEDATA = data;                  					// Select E2PROM BANK
    EECON1bits.EEPGD = 0;           
    EECON1bits.CFGS = 0;            
    EECON1bits.WREN = 1;            					// Enable E2PROM write
    GIE_on = INTCONbits.GIE;        					// Push interrupt setting
    INTCONbits.GIE = 0;             					// Disable interrupt
    EECON2 = 0x55;                  					// E2ROM Write Magic Number1
    EECON2 = 0xAA;                  					// E2ROM Write Magic Number2
    EECON1bits.WR = 1;              					// Write Start
    if(GIE_on)INTCONbits.GIE = 1;   					// Pop interrupt setting
    while(PIR2bits.EEIF == 0);      					// Wait for E2PROM Write
    PIR2bits.EEIF = 0;              					// Clear EEIF bit
    EECON1bits.WREN = 0;            					// Disable E2PROM write
}

/*********************************************************************
* Function: void eepromConfigSave(void)
* Overview: E2PROMに設定初期値を保存する
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void eepromConfigSave(void)
{  
    eepromWriteByte(0,E2PROM_MAGICNUM);
    eepromWriteByte(1, centerX);
    eepromWriteByte(2, centerY);
    eepromWriteByte(3, centerZ);
    eepromWriteByte(4, muxXY);
    eepromWriteByte(5, muxZ);
}

/*********************************************************************
* Function: void eepromBDataSave(void)
* Overview: E2PROMにボタン設定データを保存する
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void eepromBDataSave(void)
{  
	int i;
	for (i=0;i<20;i++){
	    eepromWriteByte(32+i, bDataCyber[i]);
	}
	for (i=0;i<20;i++){
	    eepromWriteByte(32+20+i, bDataCyMDmode[i]);
	}
	for (i=0;i<16;i++){
	    eepromWriteByte(32+20+20+i, bDataMD6B[i]);
	}
	eepromWriteByte(32+20+20+16+0, bDataCyMDXY);
	eepromWriteByte(32+20+20+16+1, bDataCyMDZ);
	eepromWriteByte(32+20+20+16+2, bDataMD6BXY);
	eepromWriteByte(32+20+20+16+3, bDataCyberXY);
	eepromWriteByte(32+20+20+16+4, bDataCyberZ);
	eepromWriteByte(32+20+20+16+5, bDataCyberRvXYZ);
	eepromWriteByte(32+20+20+16+6, sDataModeSWCountOut & 0xff );
	eepromWriteByte(32+20+20+16+7, ((sDataModeSWCountOut & 0xff00) >> 8) );
//	eepromWriteByte(32+20+20+16+8, HID_Interval);
}
/*********************************************************************
* Function: void eepromIntervalSave(unsigned char)
* Overview: E2PROMにボタン設定データを保存する
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void eepromIntervalSave(unsigned char Interval )
{  
	eepromWriteByte(32+20+20+16+8, Interval);
}
/*********************************************************************
* Function: void eepromRapidSave(void)
* Overview: E2PROMに連射ボタン設定データを保存する
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void eepromRapidSave(void)
{  
	eepromWriteByte(32+20+20+16+9,  bDataMD6BRapid[0]);
	eepromWriteByte(32+20+20+16+10, bDataMD6BRapid[1]);
	eepromWriteByte(32+20+20+16+11,  bDataMD6BRapid[2]);
	eepromWriteByte(32+20+20+16+12, bDataMD6BRapid[3]);
	eepromWriteByte(32+20+20+16+13, bDataCyberRapid[0]);
	eepromWriteByte(32+20+20+16+14, bDataCyberRapid[1]);
	eepromWriteByte(32+20+20+16+15, bDataCyberRapid[2]);
	eepromWriteByte(32+20+20+16+16, bDataCyberRapid[3]);
	eepromWriteByte(32+20+20+16+17, bDataRapidSpeedA);
	eepromWriteByte(32+20+20+16+18, bDataRapidSpeedB);
	eepromWriteByte(32+20+20+16+19, bRapidSWCountOut & 0xff );
	eepromWriteByte(32+20+20+16+20, ((bRapidSWCountOut & 0xff00) >> 8) );
}


/*********************************************************************
* Function: void eepromConfigLoad(void)
* Overview: E2PROMにボタン設定データをロードする
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void eepromConfigLoad(void)
{  
    unsigned char tmpbuf[8];
    unsigned char sum;
    
    unsigned char i;
    
    //Stick補正値のロード
    centerX = eepromReadByte(1);
    centerY = eepromReadByte(2);
    centerZ = eepromReadByte(3);
    muxXY =   eepromReadByte(4);
    muxZ = eepromReadByte(5);
    
	// ボタン配置データのロード

	for (i=0;i<20;i++){
	    bDataCyber[i] = eepromReadByte(32+i);
	}
	for (i=0;i<20;i++){
	    bDataCyMDmode[i] = eepromReadByte(32+20+i);
	}
	for (i=0;i<16;i++){
	    bDataMD6B[i] = eepromReadByte(32+20+20+i);
	}
    
    bDataCyMDXY = eepromReadByte(32+20+20+16+0);
    bDataCyMDZ = eepromReadByte(32+20+20+16+1);
    bDataMD6BXY = eepromReadByte(32+20+20+16+2);
    bDataCyberXY = eepromReadByte(32+20+20+16+3);
    bDataCyberZ = eepromReadByte(32+20+20+16+4);
	bDataCyberRvXYZ = eepromReadByte(32+20+20+16+5);
	sDataModeSWCountOut = ((eepromReadByte(32+20+20+16+7)*0x100)|eepromReadByte(32+20+20+16+6));
    HID_Interval = eepromReadByte(32+20+20+16+8);

    bDataMD6BRapid[0]  = eepromReadByte(32+20+20+16+9);
	bDataMD6BRapid[1]  = eepromReadByte(32+20+20+16+10);
    bDataMD6BRapid[2]  = eepromReadByte(32+20+20+16+11);
	bDataMD6BRapid[3]  = eepromReadByte(32+20+20+16+12);
	bDataCyberRapid[0] = eepromReadByte(32+20+20+16+13);
	bDataCyberRapid[1] = eepromReadByte(32+20+20+16+14);
	bDataCyberRapid[2] = eepromReadByte(32+20+20+16+15);
	bDataCyberRapid[3] = eepromReadByte(32+20+20+16+16);
	bDataRapidSpeedA = eepromReadByte(32+20+20+16+17);
	bDataRapidSpeedB = eepromReadByte(32+20+20+16+18);
	bRapidSWCountOut = ((eepromReadByte(32+20+20+16+20)*0x100)|eepromReadByte(32+20+20+16+19));

    
}

/*********************************************************************
* Function: void eepromConfigMake(void)
* Overview: E2PROMに設定初期値を作成して保存する
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void eepromConfigMake(void)
{  
	unsigned char i;
	
	centerX = 0; 			                                                    // CyberStick X軸の遊び値
    centerY = 0;            			                                        // CyberStick Y軸の遊び値
    centerZ = 0;                        			                            // CyberStick Z軸の遊び値
    muxXY = 2;                                      			                // XYスティックの移動倍率(x1)
    muxZ = 2;                                                   			    // スロットルレバーの移動倍率(x1)
    eepromConfigSave();                                             			// E2PROMに記録
    
	// ボタン配置データを一旦初期化
	for (i=0;i<20;i++){
	    bDataCyber[i] = 0x00;
	}
	for (i=0;i<20;i++){
	    bDataCyMDmode[i] = 0x00;
	}
	for (i=0;i<16;i++){
	    bDataMD6B[i] = 0x00;
	}
    
	//ボタン配置初期データ

	//メガドラパッドボタン配置初期値
	//ボタン4までのval5のデータは偶数、
	//ボタン5からのVal6のデータは奇数に設定
	
    bDataMD6B[MD6B_A]				= MD6B_BUTTON3_ON;	// Aボタン [0-1]
    bDataMD6B[MD6B_B]				= MD6B_BUTTON2_ON;	// Bボタン [2-3]
    bDataMD6B[MD6B_C +1]			= MD6B_BUTTON6_ON;	// Cボタン [4-5]
    bDataMD6B[MD6B_X]				= MD6B_BUTTON4_ON;	// Xボタン [6-7]
    bDataMD6B[MD6B_Y] 				= MD6B_BUTTON1_ON;	// Yボタン [8-9]
    bDataMD6B[MD6B_Z +1]			= MD6B_BUTTON5_ON;	// Zボタン [10-11]
    bDataMD6B[MD6B_START +1] 		= MD6B_BUTTON10_ON;	// STARTボタン [12-13]
    bDataMD6B[MD6B_MODE +1 ] 		= MD6B_BUTTON9_ON;	// MODEボタン  [14-15]
    
	//CyberStickモードのボタン初期値(MD6Bモード時)
	//ボタン4までのval5のデータは偶数、
	//ボタン5からのVal6のデータは奇数に設定
    bDataCyMDmode[CYBER_A]			= MD6B_BUTTON3_ON;	// Aボタン [0-1]
    bDataCyMDmode[CYBER_B]			= MD6B_BUTTON2_ON;	// Bボタン [2-3]
    bDataCyMDmode[CYBER_C +1]		= MD6B_BUTTON6_ON;	// Cボタン [4-5]
    bDataCyMDmode[CYBER_D]			= MD6B_BUTTON4_ON;	// Dボタン [6-7]
    bDataCyMDmode[CYBER_E1]			= MD6B_BUTTON1_ON;	// E1ボタン [8-9]
    bDataCyMDmode[CYBER_E2 +1]		= MD6B_BUTTON5_ON;	// E2ボタン [10-11]
    bDataCyMDmode[CYBER_START +1]	= MD6B_BUTTON10_ON;	// STARTボタン [12-13]
    bDataCyMDmode[CYBER_SELECT +1]	= MD6B_BUTTON9_ON;	// SELECTボタン  [14-15]
    bDataCyMDmode[CYBER_Ad]			= MD6B_BUTTON3_ON;	// A'ボタン [16-17]
    bDataCyMDmode[CYBER_Bd]			= MD6B_BUTTON2_ON;	// B'ボタン  [18-19]

	//CyberStickモードのボタン初期値(XE1AJ-USBモード時)
	//ボタン4までのval5のデータは偶数、
	//ボタン5からのVal6のデータは奇数に設定
    bDataCyber[CYBER_A]  			= CYBER_BUTTON1_ON;	// Aボタン [0-1]
    bDataCyber[CYBER_B]  			= CYBER_BUTTON2_ON;	// Bボタン [2-3]
    bDataCyber[CYBER_C]  			= CYBER_BUTTON3_ON;	// Cボタン [4-5]
    bDataCyber[CYBER_D]  			= CYBER_BUTTON4_ON;	// Dボタン [6-7]
    bDataCyber[CYBER_E1]			= CYBER_BUTTON5_ON;	// E1ボタン [8-9]
    bDataCyber[CYBER_E2]			= CYBER_BUTTON6_ON;	// E2ボタン [10-11]
    bDataCyber[CYBER_START] 		= CYBER_BUTTON8_ON;	// STARTボタン [12-13]
    bDataCyber[CYBER_SELECT]		= CYBER_BUTTON7_ON;	// SELECTボタン  [14-15]
    bDataCyber[CYBER_Ad] 			= CYBER_BUTTON1_ON;	// A'ボタン [16-17]
    bDataCyber[CYBER_Bd] 			= CYBER_BUTTON2_ON;	// B'ボタン  [18-19]

    bDataCyMDXY = MD6B_XY_STICK; 	// CyberStick MD6Bモード時のXYデータ位置
    bDataCyMDZ = MD6B_Z_STICK;		// CyberStick MD6Bモード時のZデータ位置
    bDataMD6BXY = MD6B_XY_STICK;	// MD6Bモード時のXYデータ位置
    bDataCyberXY = CYBER_XY_STICK; 	// CyberStick モード時のXYデータ位置
    bDataCyberZ = CYBER_Z_STICK;	// CyberStick モード時のXYデータ位置
	bDataCyberRvXYZ = CYBER_RVZ | CYBER_MD6B_RVZ ;	//XYZのデータ反転フラグ
	sDataModeSWCountOut = MD3B_MODE_TIMEOUT;	// スタートボタン長押しのカウントのタイムアウト値
	eepromBDataSave();															// ボタン設定値をE2PROMの保存
	
    HID_Interval = HID_INTERVAL_1MS;
	eepromIntervalSave(HID_INTERVAL_1MS);	// interval設定値をE2PROMの保存(eepromBDataSave()ではワザと保存していない）

	bDataMD6BRapid[0] = 0x00;				//SEGA 6Bパッドモードの連射ボタンマスク値(表[5])
	bDataMD6BRapid[1] = 0x00;				//SEGA 6Bパッドモードの連射ボタンマスク値(裏[5])
	bDataMD6BRapid[2] = 0x00;				//SEGA 6Bパッドモードの連射ボタンマスク値(表[6])
	bDataMD6BRapid[3] = 0x00;				//SEGA 6Bパッドモードの連射ボタンマスク値(裏[6])

	bDataCyberRapid[0] = 0x00;				//XE1AJ-USBモードの連射ボタンマスク値(表[0])
	bDataCyberRapid[1] = 0x00;				//XE1AJ-USBモードの連射ボタンマスク値(裏[0])
	bDataCyberRapid[2] = 0x00;				//XE1AJ-USBモードの連射ボタンマスク値(表[1])
	bDataCyberRapid[3] = 0x00;				//XE1AJ-USBモードの連射ボタンマスク値(裏[1])
	bDataRapidSpeedA = 0;					//連射速度 (0=16発/1=8発/2=4発/3=2発/4=1発....)
	bDataRapidSpeedB = 0;					//連射速度 (0=16発/1=8発/2=4発/3=2発/4=1発....)
	bRapidSWCountOut = RAPIDSW_TIMEOUT;		//連射スイッチ有効化までの時間
	eepromRapidSave();						//Rapid設定値をE2PROMの保存

}
    

/*********************************************************************
* Function: void usbSerialInit(void);
* Overview: Set usb serial number data
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void usbSerialInit(void) 
{
    SN_update[0] = 22; // number of bytes
    SN_update[1] = USB_DESCRIPTOR_STRING;
    SN_update[2] = 'S';
    SN_update[3] = 0;
    SN_update[4] = 'N';
    SN_update[5] = 0;
    SN_update[6] = '-';
    SN_update[7] = 0;
    SN_update[8] = '-';
    SN_update[9] = 0;
    SN_update[10] = '-';
    SN_update[11] = 0;
    SN_update[12] = '-';
    SN_update[13] = 0;
    SN_update[14] = '-';
    SN_update[15] = 0;
    SN_update[16] = '-';
    SN_update[17] = 0;
    SN_update[18] = '-';
    SN_update[19] = 0;
    SN_update[20] = '-';
    SN_update[21] = 0;
};


/*********************************************************************
* Function: void sendDataBufferInitialize(int)
* Overview: 送信データのZero Fill
* PreCondition: None
* Input: 先頭位置
* Output: None
*
********************************************************************/
void sendDataBufferInitialize(int j)
{  
    int i;
    for (i=j;i<64;i++){
        ToSendDataBuffer.val[i] = 0;	//送信用Buffer64Byteを初期化
    }
}


/*********************************************************************
* Function: int cyberAtariInput(unsigned char *)
* Overview: ATARIジョイスティック/CyberStickの通信ルーチン
* PreCondition: None
* Input: 読み取りデータの格納先
* Output: TIMEOUT
*
********************************************************************/
int cyberAtariInput(unsigned char *ptr) {
// ボタン配置データ
// [0][1]のデータはTIMEOUT時のみ有効、デジタルモードはこちらを参照
// CyberData[0] = 0b00[PIN7][PIN6]_[PIN4][PIN3][PIN2][PIN1] (PIN8=Hのデータ)
// CyberData[1] = 0b00[PIN7][PIN6]_[PIN4][PIN3][PIN2][PIN1] (PIN8=Lのデータ)

// デジタルモードデータ
// CyberData[0] = 0b00[E2][E1]_[D] [C] [TH DOWN][TH UP]
// CyberData[1] = 0b00[B][A]_[RIGHT] [LEFT] [DOWN][UP]

// アナログモードデータ
// CyberData[2] = 0b0000_[A] [B] [C][D]
// CyberData[3] = 0b0000_[E1][E2][F][G]
// CyberData[4] = (未使用)
// CyberData[5] = 0b0000_Yデータ[7:4]
// CyberData[6] = 0b0000_Xデータ[7:4]
// CyberData[7] = 0b0000_Zデータ[7:4] 
// CyberData[8] = (未使用)
// CyberData[9] = 0b0000_Yデータ[3:0]
// CyberData[10]= 0b0000_Xデータ[3:0]
// CyberData[11]= 0b0000_Zデータ[3:0]
// CyberData[12]= 0b0000_[A] [B] [A'][B']

  int loopcnt;
  int timeout;
  unsigned char *temp;

  CYBER_REQOFF;
  temp = ptr;
  _delay_us(5);
  *temp = PORTB & 0x30 | PORTC & 0x0f;		//Data0: Digtal HIGH
  temp++;

  CYBER_REQON;
  _delay_us(5);
  *temp = PORTB & 0x30 | PORTC & 0x0f;		//Data1: Digtal LOW
  temp++;

  CYBER_REQOFF;

  for (loopcnt = 0; loopcnt <= 5; loopcnt++) {
    for (timeout = 0;; timeout++)
    {
      if ((CYBER_ACK_EQLOW)&(CYBER_LH_EQLOW)) break;
      if (timeout == TIMEOUTMAX) {
        return ERR_TIMEOUT;
      }
    }
    *temp = PORTC & 0x0f;					//Data2,4,6,8,10,12 ACK1,3,5,7,9,11
    temp++;

	if (loopcnt == 5 ) return ERR_NONE;
	
    for (timeout = 0;; timeout++)
    {
      if ((CYBER_ACK_EQLOW)&(CYBER_LH_EQHIGH)) break;
      if (timeout == TIMEOUTMAX) {
        return ERR_TIMEOUT;
      }
    }
    *temp = PORTC & 0x0f;					//Data3,5,7,9,11 ACK2,4,6,8,10
    temp++;
  }
  return ERR_TIMEOUT;

}

/*********************************************************************
* Function: int cyberInput(unsigned char *)
* Overview: MD modeでのCyberStickの通信ルーチン
* PreCondition: None
* Input: 読み取りデータの格納先
* Output: TIMEOUT
*
********************************************************************/
int cyberMdInput(unsigned char *ptr) {
// ボタン配置データ
// [0][1]のデータはTIMEOUT時のみ有効、デジタルモードはこちらを参照
// CyberData[0] = 0b[PIN9]0[SEL][PIN6]_[PIN4][PIN3][PIN2][PIN1] (PIN8=Hのデータ)
// CyberData[1] = 0b[PIN9]0[SEL][PIN6]_[PIN4][PIN3][PIN2][PIN1] (PIN8=Lのデータ)
// つまりPIN9のデータをBIT5に持っていく必要がある。

// デジタルモードデータ
// CyberData[0] = 0b001  1  _1       [C]    [TH DOWN][TH UP]
// CyberData[1] = 0b00[B][A]_[RIGHT] [LEFT] [DOWN]   [UP]

// アナログモードデータ (出力順ががXE1-AJと逆、このサブルーチン内でPCと同様になるようにしている)
// CyberData[2] = 0b0000_[E1][E2][F][G]
// CyberData[3] = 0b0000_[A] [B] [C][D]
// CyberData[4] = 0b0000_Yデータ[7:4]
// CyberData[5] = (未使用)
// CyberData[6] = 0b0000_Zデータ[7:4] 
// CyberData[7] = 0b0000_Xデータ[7:4]
// CyberData[8] = 0b0000_Yデータ[3:0]
// CyberData[9] = (未使用)
// CyberData[10]= 0b0000_Zデータ[3:0]
// CyberData[11]= 0b0000_Xデータ[3:0]
// CyberData[12] = (未使用)
// CyberData[13]= 0b0000_[A] [B] [A'][B']

  int loopcnt;
  int timeout;
  unsigned char *temp;
  unsigned char data_h,data_l;

  CYBERMD_REQOFF;
  temp = ptr;
  _delay_us(5);
  data_h = PORTB & 0xf0 | PORTC & 0x0f;		//Data0: Digtal HIGH

  CYBERMD_REQON;
  _delay_us(5);
  data_l = PORTB & 0xf0 | PORTC & 0x0f;		//Data1: Digtal LOW

  //データ構造がメガドラパッド仕様なので辻褄をあわせる  
  //つまり全部のデータは取得できない
  
  *temp = ((data_h & 0x80) >>5)|(data_l & 0x3)| 0xf8;
  temp++;
  *temp = ((data_h & 0x10)<<1)|(data_l & 0x10)|(data_h & 0xf);
  if ((data_l & 0x80)==0x00) *temp = *temp & 0xf3;
  temp++;
  

  CYBERMD_REQOFF;

  for (loopcnt = 0; loopcnt <= 5; loopcnt++) {
    for (timeout = 0;; timeout++)
    {
      if ((CYBERMMD_ACK_EQLOW)&(CYBERMD_LH_EQLOW)) break;
      if (timeout == TIMEOUTMAX) {
        return ERR_TIMEOUT;
      }
    }
    data_l = PORTC & 0x0f;					//Data2,4,6,8,10,12 ACK1,3,5,7,9,11

    for (timeout = 0;; timeout++)
    {
      if ((CYBERMMD_ACK_EQLOW)&(CYBERMD_LH_EQHIGH)) break;
      if (timeout == TIMEOUTMAX) {
        return ERR_TIMEOUT;
      }
    }
    data_h = PORTC & 0x0f;					//Data2,4,6,8,10,12 ACK1,3,5,7,9,11

    //データ位置がPCモードと逆なので辻褄をあわせる  
    *temp = data_h;
    temp++;
    *temp = data_l;
    temp++;

	if (loopcnt == 5 ) return ERR_NONE;
  }
  return ERR_TIMEOUT;

}
/*********************************************************************
* Function: int cyberInput(unsigned char *)
* Overview: CyberStickの通信ルーチン (MD mode対応)
* PreCondition: None
* Input: 読み取りデータの格納先
* Output: TIMEOUT
*
********************************************************************/
int cyberInput(unsigned char *ptr) {
	if (TRISMODE_EQMD) 	return cyberMdInput(ptr);				//XE1AP MDmodeのCyberStickのデータ処理
	else return cyberAtariInput(ptr);
}



/*********************************************************************
* Function: void makeMD6BDataForSEGAPAD(void)
* Overview: MDプロトコルを実行してFightingPAD 6BのUSBデータ生成します(SEGA MDコントローラ)
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeMD6BDataForSEGAPAD(void)
{  
	//SEGA6ボタンパッドの制約から
	//このルーチンの再実行は1.8ms以上間隔を空ける必要がある
	//そのため、HID intervalは1ms設定をしてはいけない
	unsigned char startFlag;
	unsigned char modeFlag;
	
	startFlag = true;
	modeFlag = true;
	
    //(読み捨て) 0State目 (L)
    PORTBbits.RB5 = 0;
    _delay_us(100);

    //(読み捨て) 1State目 (H)
    PORTBbits.RB5 = 1;
    _delay_us(100);
    
    // 2State目(L)	3ボタンPADのデータ(L側)はここで読み込む
    PORTBbits.RB5 = 0;
    _delay_us(100);
    if (PORTBbits.RB4 == 0 ) VAL_MD6BDATA(MD6B_A);		// Button3 (A:RB4)
//  if (PORTBbits.RB7 == 0 ) VAL_MD6BDATA(MD6B_START);	// Button8 (START:RB7)
    if (PORTBbits.RB7 == 0 ) startFlag = false;			// Button8 (START:RB7) ※後で処理する

    // 3State目(H)	3ボタンPADのデータ(H側)はここで読み込む
    PORTBbits.RB5 = 1;
    _delay_us(100);

    if (PORTCbits.RC0 == 0 ) joystick_input.val[bDataMD6BXY+1] = 0x00;						// UP     :RC0
    if (PORTCbits.RC1 == 0 ) joystick_input.val[bDataMD6BXY+1] = 0xff;						// DOWN   :RC1
    if (PORTCbits.RC2 == 0 ) joystick_input.val[bDataMD6BXY] = 0x00;						// LEFT   :RC2
    if (PORTCbits.RC3 == 0 ) joystick_input.val[bDataMD6BXY] = 0xff;						// RIGH   :RC3
    // Button (3B)
    if (PORTBbits.RB4 == 0 ) VAL_MD6BDATA(MD6B_B);								// Button2 (B:RC6)
    if (PORTBbits.RB7 == 0 ) VAL_MD6BDATA(MD6B_C);								// Button6 (C:RC7)

    // 4State(L)	PIN1-4の状態を確認して、6ボタンPADの判別を実施 
    PORTBbits.RB5 = 0;
    _delay_us(100);

    if ((PORTCbits.RC0 == 0 ) && (PORTCbits.RC1 == 0 )){        				//6B有り
		// 5State (H)	6ボタンパッドのXYZ,MODEはここで読み込み
        PORTBbits.RB5 = 1;              
         _delay_us(100);
        if (PORTCbits.RC0 == 0 ) VAL_MD6BDATA(MD6B_Z);							// Button5 (Z:RC0)
        if (PORTCbits.RC1 == 0 ) VAL_MD6BDATA(MD6B_Y);							// Button1 (Y:RC1)
        if (PORTCbits.RC2 == 0 ) VAL_MD6BDATA(MD6B_X);							// Button4 (X:RC2)
        if (PORTCbits.RC3 == 0 ) modeFlag = false;								// Button7 (MODE:RC3)
        
		if (startFlag == false ) VAL_MD6BDATA(MD6B_START);						// Start Button

		// DUMMY 6State (L)
        PORTBbits.RB5 = 0;
        _delay_us(100);
    }else{

	//START長押しのMODEスイッチ代用処理

		if (Md3bModeSWCount != sDataModeSWCountOut) {
			if (!startFlag) {
				VAL_MD6BDATA(MD6B_START);										// 普通はStart Button
				if (sDataModeSWCountOut != 0xffff)	Md3bModeSWCount ++;			// ffffの時は当該機能無効
			} else	Md3bModeSWCount = 0; 
		} else {
			modeFlag = false;													//TimeOutなのでMODEを代わりに押す
			if (startFlag) {
				Md3bModeSWCount = 0;
			}
		}
	}
	//連射スイッチの有効無効スイッチの処理
    calcRapidEnableSwitch(modeFlag);
	if (!modeFlag) 	VAL_MD6BDATA(MD6B_MODE);					// mode Button

    // 5State(3B PAD)/7State(6B PAD) (H)
    PORTBbits.RB5 = 1;

}

/*********************************************************************
* Function: void makeMD6BDataForATARI(void)
* Overview: FightingPAD 6BのUSBデータ生成します(ATARIコントローラ)
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeMD6BDataForAtari(void)
{  
	unsigned char x,y,z;
	unsigned char selectFlag;

	selectFlag = true;
	x = 0x80;
	y = 0x80;
	z = 0x80;
    //Input Data
    CyberMode = cyberInput(CyberData);											// CyberStickのプロトコル処理
    																			// CyberModeは他でも参照しているので一度変数に入れる
    if(CyberMode == ERR_TIMEOUT){												// TIMEOUT=Cyber Stick Digital modeかATARI Stickを意味する
        //Cyber Stick Digital mode
        if ((CyberData[1] & 0x03) == 0  ){   	selectFlag = false;				// Button7 (SELECT:MODE) ※XE1-APのみ対応
        }else{
            if ((CyberData[1] & 0x01) == 0  ) 	y = 0x00;  	// UP
            if ((CyberData[1] & 0x02) == 0  ) 	y = 0xFF;  	// DOWN
            if ((CyberData[0] & 0x01) == 0  ) 	z = 0x00;  	// Throt UP
            if ((CyberData[0] & 0x02) == 0  ) 	z = 0xFF;  	// Throt DOWN
        }

        if ((CyberData[1] & 0x0C) == 0  ){   VAL_MD6BCYBERDATA(CYBER_START);		// Button8 (START) ※XE1-APのみ対応
        }else{
            if ((CyberData[1] & 0x04) == 0  ) 	x = 0x00;  	// LEFT
            if ((CyberData[1] & 0x08) == 0  ) 	x = 0xFF;  	// RIGHT
            if ((CyberData[0] & 0x04) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_C);		// Button6 (C)
            if ((CyberData[0] & 0x08) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_D);		// Button4 (D:X)
        }

        if ((CyberData[1] & 0x10) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_A);			// Button3 (A)
        if ((CyberData[1] & 0x20) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_B);			// Button2 (B)
        if ((CyberData[0] & 0x10) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_E1);		// Button1 (E1:Y)
        if ((CyberData[0] & 0x20) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_E2); 		// Button5 (E2:Z)
    }else{
        // Cyber Stick Analog mode
        // 生値から8bitのXYZデータを生成する
        y = CyberData[4] << 4 | CyberData[8];
        x = CyberData[5] << 4 | CyberData[9];
        z = CyberData[6] << 4 | CyberData[10];

		// xyzデータの補正計算
        z = calFixedVal(z,centerZ,muxZ,slewZ);
        y = calFixedVal(y,centerY,muxXY,slewY);
        x = calFixedVal(x,centerX,muxXY,slewX);


		// ボタンデータの作成
        if ((CyberData[12] & 0x02) == 0 ) 	VAL_MD6BCYBERDATA(CYBER_Ad);  		// Button3  (A')
        if ((CyberData[12] & 0x01) == 0 ) 	VAL_MD6BCYBERDATA(CYBER_Bd);		// Button2  (B')
        if ((CyberData[12] & 0x08) == 0 ) 	VAL_MD6BCYBERDATA(CYBER_A);		  	// Button3  (A)
        if ((CyberData[12] & 0x04) == 0 ) 	VAL_MD6BCYBERDATA(CYBER_B);			// Button2  (B)
        if ((CyberData[2] & 0x02) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_C);			// Button6  (C)
        if ((CyberData[2] & 0x01) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_D);		  	// Button4  (D:X)
        if ((CyberData[3] & 0x08) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_E1);	  	// Button1  (E1:Y)
        if ((CyberData[3] & 0x04) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_E2);		// Button5  (E2:Z)
        if ((CyberData[3] & 0x01) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_SELECT);  	// Button7  (SELECT)
        if ((CyberData[3] & 0x02) == 0  ) 	VAL_MD6BCYBERDATA(CYBER_START);		// Button8  (START)

    }
	//xyzの反転処理
	if ((bDataCyberRvXYZ & CYBER_MD6B_RVX) != 0x00)	x = 0xff - x;
	if ((bDataCyberRvXYZ & CYBER_MD6B_RVY) != 0x00)	y = 0xff - y;
	if ((bDataCyberRvXYZ & CYBER_MD6B_RVZ) != 0x00)	z = 0xff - z;

    joystick_input.val[bDataCyMDXY] = x;
    joystick_input.val[bDataCyMDXY+1] = y;
    joystick_input.val[bDataCyMDZ] = z;

	//連射スイッチの有効無効スイッチの処理+セレクトスイッチの処理
    calcRapidEnableSwitch(selectFlag);
	if (!selectFlag) 	VAL_MD6BCYBERDATA(CYBER_SELECT);					// select Button

}

/*********************************************************************
* Function: unsigned char makeCyberHatData(unsigned char,unsigned char)
* Overview: HUT Dataを作成
* PreCondition: None
* Input: X,Yのアナログ値
* Output: Hut data
*
********************************************************************/
unsigned char makeCyberHatData(unsigned char x,unsigned char y)
{  
	unsigned char hat_data;
	

	// Left side
	if (x < 0x40) {
		// UP
		if		(y < 0x40) hat_data = HAT_UP_LEFT;
		// DOWN
		else if	(y > 0xc0) hat_data = HAT_DOWN_LEFT;
		// LEFT
		else hat_data = HAT_LEFT;
	// Right Side
	}else if (x > 0xc0) {
		// UP
		if		(y < 0x40) hat_data = HAT_UP_RIGHT;
		// DOWN
		else if	(y > 0xc0) hat_data = HAT_DOWN_RIGHT;
		// RIGHT
		else hat_data = HAT_RIGHT;
	// Center Side
	}else{
		// UP
		if		(y < 0x40) hat_data = HAT_UP;
		// DOWN
		else if	(y > 0xc0) hat_data = HAT_DOWN;
		// CENTER
		else hat_data = HAT_CENTER;
	}
	return hat_data;
}

/*********************************************************************
* Function: void makeCyberData(void)
* Overview: XE1AJ-USBのUSBデータ生成します(ATARI/MDモード)
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeCyberData(void)
{  
	unsigned char x,y,z;
	unsigned char selectFlag;

	selectFlag = true;

	x = 0x80;
	y = 0x80;
	z = 0x80;

    //Input Data
    CyberMode = cyberInput(CyberData);
    if(CyberMode == ERR_TIMEOUT){
        //Cyber Stick Digital mode

		//PIN1,2
       	if ((CyberData[1] & 0x03) == 0  ){   selectFlag = false;		// Button7 (SELECT:MODE) ※XE1-APのみ対応
        }else {
            if ((CyberData[1] & 0x01) == 0  ) 	y = 0x00;  	// UP
            if ((CyberData[1] & 0x02) == 0  ) 	y = 0xFF;  	// DOWN
            if ((CyberData[0] & 0x01) == 0  ) 	z = 0x00;  	// Throt UP
            if ((CyberData[0] & 0x02) == 0  ) 	z = 0xFF;  	// Throt DOWN
        }

		//PIN3,4
        if ((CyberData[1] & 0x0C) == 0  ){   VAL_CYBERDATA(CYBER_START);			// Button8 (START) ※XE1-APのみ対応
		}else{
            if ((CyberData[1] & 0x04) == 0  ) 	x = 0x00;  	// LEFT
            if ((CyberData[1] & 0x08) == 0  ) 	x = 0xFF;  	// RIGHT
            if ((CyberData[0] & 0x04) == 0  ) 	VAL_CYBERDATA(CYBER_C);		// Button6 (C)
            if ((CyberData[0] & 0x08) == 0  ) 	VAL_CYBERDATA(CYBER_D);		// Button4 (D:X)
        }
        
		//PIN6,7
        if ((CyberData[1] & 0x10) == 0  ) 	VAL_CYBERDATA(CYBER_A);				// Button1  (A+A')
        if ((CyberData[1] & 0x20) == 0  ) 	VAL_CYBERDATA(CYBER_B);				// Button2  (B+B')
        if ((CyberData[0] & 0x10) == 0  ) 	VAL_CYBERDATA(CYBER_E1);			// Button5  (E1)
        if ((CyberData[0] & 0x20) == 0  ) 	VAL_CYBERDATA(CYBER_E2);			// Button6  (E2)

    }else{
        //Cyber Stick Analog mode
        // 生値から8bitのXYZデータを生成する
        y = CyberData[4] << 4 | CyberData[8];
        x = CyberData[5] << 4 | CyberData[9];
        z = CyberData[6] << 4 | CyberData[10];

		// xyzデータの補正計算
        y = calFixedVal(y,centerY,muxXY,slewY);
        x = calFixedVal(x,centerX,muxXY,slewX);
        z = calFixedVal(z,centerZ,muxZ,slewZ);

        if ((CyberData[12] & 0x02) == 0 ) 	VAL_CYBERDATA(CYBER_Ad);  			// Button1  (A')
        if ((CyberData[12] & 0x01) == 0 ) 	VAL_CYBERDATA(CYBER_Bd);			// Button2  (B')
        if ((CyberData[12] & 0x08) == 0 ) 	VAL_CYBERDATA(CYBER_A);		  		// Button1  (A)
        if ((CyberData[12] & 0x04) == 0 ) 	VAL_CYBERDATA(CYBER_B);				// Button2  (B)
        if ((CyberData[2] & 0x02) == 0  ) 	VAL_CYBERDATA(CYBER_C);				// Button3  (C)
        if ((CyberData[2] & 0x01) == 0  ) 	VAL_CYBERDATA(CYBER_D);		  		// Button4  (D:X)
        if ((CyberData[3] & 0x08) == 0  ) 	VAL_CYBERDATA(CYBER_E1);	  		// Button5  (E1:Y)
        if ((CyberData[3] & 0x04) == 0  ) 	VAL_CYBERDATA(CYBER_E2);			// Button6  (E2:Z)
        if ((CyberData[3] & 0x01) == 0  ) 	selectFlag = false;  				// Button7  (SELECT)
        if ((CyberData[3] & 0x02) == 0  ) 	VAL_CYBERDATA(CYBER_START);			// Button8  (START)
    }

	//xyzの反転処理
	if ((bDataCyberRvXYZ & CYBER_RVX) != 0x00) x = 0xff - x;
	if ((bDataCyberRvXYZ & CYBER_RVY) != 0x00) y = 0xff - y;
	if ((bDataCyberRvXYZ & CYBER_RVZ) != 0x00) z = 0xff - z;
	
    // Hut Dataの処理 
    if (bDataCyberXY == CYBER_HAT_STICK ){
		joystick_input.val[bDataCyberXY]   = makeCyberHatData(x,y);
	}else{
		joystick_input.val[bDataCyberXY]   = x;
		joystick_input.val[bDataCyberXY+1] = y;
	}

    if (bDataCyberZ == CYBER_HAT_STICK ){
		if ((bDataCyberRvXYZ & CYBER_HATRVZ) != 0x00) joystick_input.val[bDataCyberZ]    = makeCyberHatData(z,0x80);
		else joystick_input.val[bDataCyberZ]    = makeCyberHatData(0x80,z);
	}else{
		joystick_input.val[bDataCyberZ]    = z;
	}

	//連射スイッチの有効無効スイッチの処理+セレクトスイッチの処理
    calcRapidEnableSwitch(selectFlag);
	if (!selectFlag) 	VAL_CYBERDATA(CYBER_SELECT);					// select Button

}

/*********************************************************************
* Function: void makeConfigDataFillZero(void)
* Overview: 送信用バッファーの余剰領域をZeroFill
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: Start point
* Output: None
*
********************************************************************/
void makeConfigDataFillZero(int start)
{  
	int i;
    for(i=start;i<63;i++){   
        ToSendDataBuffer.val[i] = 0x00;
    }
}

/*********************************************************************
* Function: void makeConfigDataSendMd16Setting(void)
* Overview: MD16PADのボタン設定値を返す
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeConfigDataSendMd16Setting(void)
{  
	int i;

	for (i=0;i<16;i++){
		ToSendDataBuffer.val[1+i]			= bDataMD6B[i];
	}
	ToSendDataBuffer.val[1+16+0]			= bDataMD6BXY;
	ToSendDataBuffer.val[1+16+1]			= sDataModeSWCountOut & 0xff;
	ToSendDataBuffer.val[1+16+2]			= ((sDataModeSWCountOut & 0xff00) >> 8);
	makeConfigDataFillZero(20);

}


/*********************************************************************
* Function: void makeConfigDataSendCyberSetting(void)
* Overview: CyberStickのボタン設定値を返す
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeConfigDataSendCyberSetting(void)
{  
	int i;

	for (i=0;i<20;i++){
	    ToSendDataBuffer.val[1+i]			= bDataCyber[i];
	}
	for (i=0;i<20;i++){
		ToSendDataBuffer.val[1+20+i]		= bDataCyMDmode[i];
	}
	ToSendDataBuffer.val[1+20+20+0]		= bDataCyMDXY;
	ToSendDataBuffer.val[1+20+20+1]		= bDataCyMDZ ;
    ToSendDataBuffer.val[1+20+20+2]		= bDataCyberXY ;
    ToSendDataBuffer.val[1+20+20+3]		= bDataCyberZ ;
    ToSendDataBuffer.val[1+20+20+4]		= bDataCyberRvXYZ ;

	makeConfigDataFillZero(46);

}


/*********************************************************************
* Function: void makeConfigDataFirmwareVer(void)
* Overview: 内部Versionを返すHIDデータを作成
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeConfigDataFirmwareVer(void)
{  
    ToSendDataBuffer.val[1] = 0x00;
    ToSendDataBuffer.val[2] = 0x00;

 
    ToSendDataBuffer.val[3] = (VersionWord) & 0xff;   							//アプリVersion
    ToSendDataBuffer.val[4] = (VersionWord>>8) & 0xff;
	makeConfigDataFillZero(5);
}

/*********************************************************************
* Function: void makeConfigDataSendRawData(void)
* Overview: 内部変数の生値を返す(Debug用)
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeConfigDataSendRawData(void)
{  
	int i;

    ToSendDataBuffer.val[1] = (uint8_t)CyberMode;        //Cyber Input mode (0:Analog 1:Digital)
    for(i=0;i<13;i++){   
        ToSendDataBuffer.val[i+2] = CyberData[i];
    }
    for(i=0;i<8;i++){   
        ToSendDataBuffer.val[i+15] = joystick_input.val[i];
    }
}

/*********************************************************************
* Function: void makeConfigDataSendSettingData(void)
* Overview: 設定値を返す
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeConfigDataSendSettingData(void)
{  
    ToSendDataBuffer.val[1] = muxXY;            								//2=1x mode 3=1.5x mode 4=2xmode
    ToSendDataBuffer.val[2] = muxZ;            									//2=1x mode 3=1.5x mode 4=2xmode
    ToSendDataBuffer.val[3] = centerX;          								//センターの無視する値(X)
    ToSendDataBuffer.val[4] = centerY;          								//センターの無視する値(Y)
    ToSendDataBuffer.val[5] = centerZ;          								//センターの無視する値(Z)
    ToSendDataBuffer.val[6] = HID_Interval;          							//HID Intervalの設定値(1=1ms,2=2ms,3=4ms,4=8ms)
    ToSendDataBuffer.val[7]  = bDataMD6BRapid[0];       						//Fighting Pad 6Bモード用連射ボタン設定（表[5]）
    ToSendDataBuffer.val[8]  = bDataMD6BRapid[1];       						//Fighting Pad 6Bモード用連射ボタン設定（裏[5]）
    ToSendDataBuffer.val[9]  = bDataMD6BRapid[2];       						//Fighting Pad 6Bモード用連射ボタン設定（表[6]）
    ToSendDataBuffer.val[10] = bDataMD6BRapid[3];       						//Fighting Pad 6Bモード用連射ボタン設定（裏[6]）
    ToSendDataBuffer.val[11] = bDataCyberRapid[0];          					//CyberStickモード用連射ボタン設定（表[0]）
    ToSendDataBuffer.val[12] = bDataCyberRapid[1];         						//CyberStickモード用連射ボタン設定（裏[0]）
    ToSendDataBuffer.val[13] = bDataCyberRapid[2];          					//CyberStickモード用連射ボタン設定（表[1]）
    ToSendDataBuffer.val[14] = bDataCyberRapid[3];         						//CyberStickモード用連射ボタン設定（裏[1]）
    ToSendDataBuffer.val[15] = bDataRapidSpeedA;         						//連射ボタンの継続時間（表）
    ToSendDataBuffer.val[16] = bDataRapidSpeedB;         						//連射ボタンの継続時間（裏）
   	ToSendDataBuffer.val[17] = bRapidSWCountOut & 0xff;							//連射有効化スイッチの時間
	ToSendDataBuffer.val[18] = ((bRapidSWCountOut & 0xff00) >> 8);

	makeConfigDataFillZero(19);
}

/*********************************************************************
* Function: void makeConfigDataOK(void)
* Overview: 戻り値OKを生成
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeConfigDataOK(void)
{  
	ToSendDataBuffer.val[1] = 'O';				//OK
	ToSendDataBuffer.val[2] = 'K';
	makeConfigDataFillZero(3);

}

/*********************************************************************
* Function: void makeConfigDataOK(void)
* Overview: 戻り値OKを生成
* PreCondition: Call from APP_DeviceJoystickTasks
* Input: None
* Output: None
*
********************************************************************/
void makeConfigDataERR(void)
{  
	ToSendDataBuffer.val[1] = 'E';				//ERR
	ToSendDataBuffer.val[2] = 'R';
	ToSendDataBuffer.val[3] = 'R';
	makeConfigDataFillZero(4);
}



/*********************************************************************
* Function: void APP_DeviceJoystickInitialize(void);
* Overview: Initializes Endpoint1/2
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/

void APP_DeviceJoystickInitialize(void)
{  
    int i;
    //initialize the variable holding the handle for the last
    // transmission
    lastTransmissionEP1 = 0;
    lastTransmissionEP2 = 0;
    lastReceive = 0;
    IncomingHIDCBSetReport = false;
    sendDataBufferInitialize(0);

    //enable the HID endpoint
    USBEnableEndpoint(JOYSTICK_EP,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Cyber Stick mode use EP1 IN/OUT
    if (!MDMODE_EQON){
        USBEnableEndpoint(CYBERCONFIG_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
        lastReceive = HIDRxPacket(CYBERCONFIG_EP,(uint8_t*)&ReceivedDataBuffer,64);
    }
  
}

/*********************************************************************
* Function: void APP_DeviceJoystickCheckConnect(void);
* Overview: Initializes check Connect Controller
* PreCondition: None
* Input: None
* Output: None
*
********************************************************************/
void APP_DeviceJoystickCheckConnect(void)
{  
	unsigned char initE2PROM;

    //USBが起動前に設定しておきたい変数はここで設定すること

    MDMODE_LEDOFF;																//初期値設定
    initE2PROM = false;															//設定値強制初期化フラグ
	XE1APMode = false;															//メガドラモードでのXE1APの有効化フラグ
	Md3bModeSWCount = 0;														// 3BパッドのMODEスイッチのカウンター
	RapidSWCount = 0;
	RapidStart = true;

    //  CyberStickとの通信ができる様になるには500ms程度時間が必要なので
    //  起動まで念の為１秒間程度待つ
    _delay_ms(1000); //Bootwait
    

	// TRISBを見てDSUB9PにATARI/MDコントローラーの
	// どちらが接続しているかを判別しています。(TRISの設定自体はSYSTEMINIT)

    if (TRISB != 0xD0){
        //スイッチがATARI側の場合の初期チェック
      
        cyberAtariInput(CyberData);     											//初回は不正データが出る場合があるので、読み捨て
        _delay_ms(500); 													//Cyber側のData更新時間を待つ

        //Original DEMPAXE1AJ-USB Compatible mode 
        //ATARI mode
        //Check Cyber Stick A Button
        //Yes,SEGA MD6 Controller Compatible mode 
        if(cyberAtariInput(CyberData) == ERR_TIMEOUT){
			// TimeOutなのでCyberStickデジタルモードとして取り扱う
            //ATARI 2B mode
            if ((CyberData[1] & 0x10) == 0  ) 	MDMODE_LEDON;			// Aが押されたのでMDモードで起動
            //Cyber Stick Digital mode
            if ((CyberData[1] & 0x20) == 0  ) 	initE2PROM = true;       // Bが押されたのでE2PROM初期化を実行
        }else{
            //Cyber Stick Analog mode
            if ((CyberData[12] & 0x0A) != 0x0A ) 	MDMODE_LEDON;       	// A+A'をチェック、押されている場合はMDモード

	        if ((CyberData[12] & 0x01) == 0 ) 	initE2PROM = true;			// Button2  (B')
	        if ((CyberData[12] & 0x04) == 0 ) 	initE2PROM = true;			// Button2  (B)

        }
    }else{
        cyberMdInput(CyberData);     										//初回は不正データが出る場合があるので、読み捨て
        _delay_ms(500); 													//Cyber側のData更新時間を待つ

        if(cyberMdInput(CyberData) == ERR_TIMEOUT){
	        //メガドラコントローラ時の初期チェック
	        //メガドラモードでのXE1APのデジタル時もここにくるが、判別ができないので仕様扱い
	        MDMODE_LEDON;                                                   // MDモードにセット
			// CyberDataのデジタルデータだとWait値が少ないので遅いコントローラでは取りこぼす
			// そのため改めてここで入力をやり直します。
	        PORTBbits.RB5 = 1;
	        _delay_us(100);
	        if (PORTBbits.RB4 == 0 ) initE2PROM = true;  	       		    // Bが押されているのでE2PROM初期化を実行

        }else{
			XE1APMode = true;												//メガドラモードでのXE1APの有効化フラグ

            //Cyber Stick Analog mode (MDモードではCyberData[12]は無効データかも）
	        if ((CyberData[12] & 0x01) == 0 ) 	initE2PROM = true;			// Button2  (B')
	        if ((CyberData[12] & 0x04) == 0 ) 	initE2PROM = true;			// Button2  (B)

        }

    }

    usbSerialInit();                                                      // USBSerial番号設定
    if ((eepromReadByte(0) != E2PROM_MAGICNUM)|(initE2PROM)) {                       // E2PROM Magic Numberのチェック
        //E2ROMに設定値が記録されていないので初期値を設定
		centerX = 0; 			                                                    // CyberStick X軸の遊び値
	    centerY = 0;            			                                        // CyberStick Y軸の遊び値
	    centerZ = 0;                        			                            // CyberStick Z軸の遊び値
	    muxXY = 2;                                      			                // XYスティックの移動倍率(x1)
	    muxZ = 2;                                                   			    // スロットルレバーの移動倍率(x1)
	    eepromConfigSave();                                             			// E2PROMに記録
		eepromConfigMake();															// ボタン配置データを作成保存
    }
    eepromConfigLoad();              		                            // 再ロード
   	if (MDMODE_EQON)	HID_Interval = HID_INTERVAL_8MS;				// SEGA FP6Bの場合はIntervalは8ms固定なので変数を修正
    calcSlew();															// StickのSlew値を事前に計算
   	RapidEnable = true;													// 連射Default有効
    calcRapidCountOut();												// 連射の設定値をロード

}


/*********************************************************************
* Function: void APP_DeviceJoystickTasks(void);
* Overview: USBコントローラのメインタスクです。
* PreCondition: APP_DeviceJoystickInitialize()とAPP_DeviceJoystickStart()の事前実行
* Input: None
* Output: None
*
********************************************************************/
void APP_DeviceJoystickTasks(void)
{  
	unsigned char x,y,z;
    int i;
	unsigned char sum;

    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) 
    {
		//メガドラミニでゲームを選択すると再CONFIGされる事があるので、再度同期を取り直す。
		sFrameDiffCount = 0;
		RapidCount = 0;
		RapidTable = 0;
		return;
	}

    //If the last transmision is complete
    if(!HIDTxHandleBusy(lastTransmissionEP1))    {

    // 基本的にこの場所にはUSB DescriptorのbInterval間隔で実行されます。
    // ただし、実際はHOSTがIN Packetを送信する時間に依存するので設定値通りにならない場合があります。
    // 実際、ファイティングパッド6Bはintervalは10ms設定ですが、MDmini/WindowsPCでは8ms間隔で実行されます。
    // XE1AJ-USBはintervalは1ms設定で1ms間隔で実行されます。
	// https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/usbspec/ns-usbspec-_usb_endpoint_descriptor

       	if (MDMODE_EQON){
			////////////////////////////////////////////////
			// SEGA FightingPAD 6B モード
			////////////////////////////////////////////////

            //Default Value
            joystick_input.val[0] = 0x01;										// ID
            joystick_input.val[1] = 0x7f;										// [Not Use] 0x7f
            joystick_input.val[2] = 0x7f;										// [Not Use] 0x7f
            joystick_input.val[3] = 0x7f;										// LEFT   0x00, RIGHT  0xFF
            joystick_input.val[4] = 0x7f;										// UP     0x00, DOWN   0xFF
            joystick_input.val[5] = 0x0f;										// X A B Y 1b 1b 1b 1b
            joystick_input.val[6] = 0x00;										// 0b 0b START MODE 0b 0b C Z
            joystick_input.val[7] = 0x00;										// [Not Use] 0x00


			// TRISBを見てDSUB9PにATARI/MDコントローラーの
			// どちらが接続しているかを判別しています。(TRISの設定自体はSYSTEMINIT)

            if (TRISMODE_EQMD){													//出力設定がメガドラ (DSUB7PIN[RB5]=OUTPUT)
				if (DSUBPIN8_EQGND){											//モードスイッチのチェック
					makeMD6BDataForSEGAPAD();									//SEGA MDパッドのデータ処理
                }
            }else{
                //Switch check
                if (DSUBPIN9_EQGND){											//モードスイッチのチェック
					makeMD6BDataForAtari();										//ATARI+CyberStickのデータ処理
                }
            }
			calcRapidTable( ((joystick_input.val[5] & bDataMD6BRapid[0]) | (joystick_input.val[6] & bDataMD6BRapid[2])),
							((joystick_input.val[5] & bDataMD6BRapid[1]) | (joystick_input.val[6] & bDataMD6BRapid[3])) );		//16ms経過したかチェック＋補正

            //シンクロ連射の処理
			if (RapidEnable){
	            joystick_input.val[5] = (joystick_input.val[5] & DataMD6BRapidMask[0]) | (joystick_input.val[5] & bDataMD6BRapid[RapidTable]);
   			    joystick_input.val[6] = (joystick_input.val[6] & DataMD6BRapidMask[1]) | (joystick_input.val[6] & bDataMD6BRapid[RapidTable+2]);
			}
    	    //Send the packet over USB to the host.
	        lastTransmissionEP1 = HIDTxPacket(JOYSTICK_EP, (uint8_t*)&joystick_input, sizeof(joystick_input));
        }else{
			////////////////////////////////////////////////
            // DEMPA CyberStick mode
			////////////////////////////////////////////////

            //Default Value (DirectInputフォーマット)
	        joystick_input.val[0] = 0x00;										// Button8-1
	        joystick_input.val[1] = 0x00;										// Button13-9 (上位バイトはダミー)
	        joystick_input.val[2] = 0x08;										// Hat
	        joystick_input.val[3] = 0x80;										// LX
	        joystick_input.val[4] = 0x80;										// LY
	        joystick_input.val[5] = 0x80;										// RX
	        joystick_input.val[6] = 0x80;										// RY
	        joystick_input.val[7] = 0x00;										// VendorSpec (Not Use)送ってはダメ


            if (TRISMODE_EQMD){													//出力設定がメガドラ (DSUB7PIN[RB5]=OUTPUT)
				if (DSUBPIN8_EQGND){											//モードスイッチのチェック (DSUB8PIN=GND)
					if (XE1APMode == true) 	makeCyberData();					//XE1AP MDmodeのCyberStickのデータ処理
                }
            }else{
	            //Switch check (MD or Cyber)
                if (DSUBPIN9_EQGND){											//モードスイッチのチェック(DSUB9PIN=GND)
					makeCyberData();											//ATARI+CyberStickのデータ処理
                }
            }

			calcRapidTable(((joystick_input.val[0] & bDataCyberRapid[0])|(joystick_input.val[1] & bDataCyberRapid[2])) ,
						   ((joystick_input.val[0] & bDataCyberRapid[1])|(joystick_input.val[1] & bDataCyberRapid[3])));		//16ms経過したかチェック＋補正

			if (RapidEnable){
	            //シンクロ連射の処理
	            joystick_input.val[0] = (joystick_input.val[0] & DataCyberRapidMask[0]) | (joystick_input.val[0] & bDataCyberRapid[RapidTable]);
	            joystick_input.val[1] = (joystick_input.val[1] & DataCyberRapidMask[1]) | (joystick_input.val[1] & bDataCyberRapid[RapidTable+2]);
			}
	        //Send the packet over USB to the host.
	        //XE1AJ-USBは7Byte送信、メガドライブミニは8バイトでもOKだがWindowsはダメ
    	    lastTransmissionEP1 = HIDTxPacket(JOYSTICK_EP, (uint8_t*)&joystick_input, sizeof(joystick_input)-1);
        }

    }
    
    // USB Outパケットの処理(Cyberモードのみ対応)
    // Outパケットは、SetReportでもHOSTから送信されるのでRxHandleとはSerReportのフラグもチェックする。
    
    
    if (!MDMODE_EQON){															// CyberStick Modeかどうかのチェック
        // CyberStick mode Config Data RCV/TX
        if((HIDRxHandleBusy(lastReceive) == false)||(IncomingHIDCBSetReport==true)){	// Check if data was received from the host.
            
            if (ReceivedDataBuffer.val[0] != 0){								// 1Byte目が実行するCMD
                ToSendDataBuffer.val[0] = ReceivedDataBuffer.val[0];			// とりあえずエコーバックさせておく(暫定実装)
            }

			//実際のCMD処理
            switch(ReceivedDataBuffer.val[0]){
                case 0x01:	//Firmware Version
	                makeConfigDataFirmwareVer();
                break;

                case 0x02:  //Send RAW DATA (for debug)
	                makeConfigDataSendRawData();
                break;

                case 0x03:  //Send Setting Data
	                makeConfigDataSendSettingData();
                break;

                case 0x04:  //Set Setting Data

	                // HOSTから送信された各補正値をセットして再計算を実施する
                    muxXY = ReceivedDataBuffer.val[1];
                    muxZ  = ReceivedDataBuffer.val[2];
                    centerX = ReceivedDataBuffer.val[3];
                    centerY = ReceivedDataBuffer.val[4];
                    centerZ = ReceivedDataBuffer.val[5];
                    calcSlew();

	                makeConfigDataOK();

                break;

                case 0x80:  //Set HID interval setting
                    eepromIntervalSave(ReceivedDataBuffer.val[1]);
	                makeConfigDataOK();
                break;

                case 0x81:  //Set MD6B mode  KeyAssign
                	//データの配置とボタンフォーマットはDefinitionを参照
                    for (i=0;i<16;i++){
						bDataMD6B[i] = ReceivedDataBuffer.val[i+1];
					}
					bDataMD6BXY = ReceivedDataBuffer.val[16+1+0];
					sDataModeSWCountOut = ((ReceivedDataBuffer.val[16+1+2] * 0x100) | ReceivedDataBuffer.val[16+1+1]);
	                makeConfigDataOK();
                break;

                case 0x82:  //Set MD6B mode CyberStick KeyAssign
                	//データの配置とボタンフォーマットはDefinitionを参照
                    for (i=0;i<20;i++){
						bDataCyMDmode[i] = ReceivedDataBuffer.val[i+1];
					}
					bDataCyMDXY = ReceivedDataBuffer.val[20+1+0];
					bDataCyMDZ = ReceivedDataBuffer.val[20+1+1];
					bDataCyberRvXYZ = (bDataCyberRvXYZ & 0xf0) | (ReceivedDataBuffer.val[20+1+2] & 0x0f);
	                makeConfigDataOK();
                break;

                case 0x83:  //Set CyberStick mode CyberStick KeyAssign
                	//データの配置とボタンフォーマットはDefinitionを参照
                    for (i=0;i<20;i++){
						bDataCyber[i] = ReceivedDataBuffer.val[i+1];
					}
					bDataCyberXY = ReceivedDataBuffer.val[20+1+0];
					bDataCyberZ = ReceivedDataBuffer.val[20+1+1];
					bDataCyberRvXYZ = (bDataCyberRvXYZ & 0x0f) | (ReceivedDataBuffer.val[20+1+2] & 0xf0);
	                makeConfigDataOK();
                break;

                case 0x84:  //Init Setting
                	//ボタン配置を初期状態に戻します。
                    eepromConfigMake(); 
                    makeConfigDataOK();
                break;
                
	            case 0x85:  //Send Botton Setting (MD6B)
	            	// ファイティングパッド6Bモード時のSEGAパッドのボタン設定情報を送信
	                makeConfigDataSendMd16Setting();
	            break;

	            case 0x86:  //Send Botton Setting (Cyberstick)
	            	// CyberStick時のSEGAパッド/XE1AJ-USBのボタン設定情報を送信
	                makeConfigDataSendCyberSetting();
	            break;

	            case 0x87:  //シンクロ連射設定 (MD6B/Cyber)
	            	// SEGA FB6B/XE1AJ-USBモード時のシンクロ連射のボタン設定情報を送信
                    bDataMD6BRapid[0]  = ReceivedDataBuffer.val[1];
                    bDataMD6BRapid[1]  = ReceivedDataBuffer.val[2];
                    bDataMD6BRapid[2]  = ReceivedDataBuffer.val[3];
                    bDataMD6BRapid[3]  = ReceivedDataBuffer.val[4];
                    bDataCyberRapid[0] = ReceivedDataBuffer.val[5];
                    bDataCyberRapid[1] = ReceivedDataBuffer.val[6];
                    bDataCyberRapid[2] = ReceivedDataBuffer.val[7];
                    bDataCyberRapid[3] = ReceivedDataBuffer.val[8];
                    bDataRapidSpeedA   = ReceivedDataBuffer.val[9];
                    bDataRapidSpeedB   = ReceivedDataBuffer.val[10];
                    bRapidSWCountOut   = (ReceivedDataBuffer.val[12]*0x100)|(ReceivedDataBuffer.val[11]);
                    calcRapidCountOut();
                    eepromRapidSave();
	                makeConfigDataOK();
	                //makeConfigDataSendCyberSetting();
	            break;

                case 0x05:  //EEPROM Setting 
					// E2PROMに設定値を保存する。
                    eepromConfigSave();
                    eepromBDataSave();
	                makeConfigDataOK();
                break;


                
                default:
                    if (ReceivedDataBuffer.val[0] != 0){
						makeConfigDataERR();									//非対応CMD
                    }
                break;
            }

            if(!HIDTxHandleBusy(lastTransmissionEP2))
                lastTransmissionEP2 = HIDTxPacket(CYBERCONFIG_EP,(uint8_t*)&ToSendDataBuffer,64);

            IncomingHIDCBSetReport = false;
            //Re-arm the OUT endpoint for the next packet
            lastReceive = HIDRxPacket(CYBERCONFIG_EP,(uint8_t*)&ReceivedDataBuffer,64);
        }
    }
    
}//end ProcessIO



#endif

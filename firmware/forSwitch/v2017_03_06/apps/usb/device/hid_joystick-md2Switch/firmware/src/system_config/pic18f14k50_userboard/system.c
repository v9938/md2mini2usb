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

#include "system.h"

/** CONFIGURATION Bits **********************************************/

#pragma config  CPUDIV	= NOCLKDIV	// CPU システムクロック選択ビット
#pragma config  USBDIV	= OFF		// USB クロック選択ビット
#pragma config  FOSC	= HS		// オシレータ選択ビット
#pragma config  PLLEN	= ON		// 4 X PLL イネーブルビット
#pragma config  PCLKEN	= OFF		// プライマリ クロック イネーブルビット
#pragma config  FCMEN	= OFF		// フェイルセーフ クロック モニタ イネーブルビット
#pragma config  IESO	= OFF		// 内部/外部オシレータ切り換えビット
#pragma config  PWRTEN	= OFF		// パワーアップ タイマ イネーブルビット (OFFで有効)
#pragma config  BOREN	= ON		// ブラウンアウト リセット イネーブルビット
#pragma config  BORV	= 30		// ブラウンアウト リセット電圧ビット
#pragma config  WDTEN	= OFF		// ウォッチドッグ タイマ イネーブルビット
#pragma config  WDTPS	= 1024		// ウォッチドッグ タイマ ポストスケーラ選択ビット
#pragma config  HFOFST	= OFF		// HFINTOSC 高速起動ビット
#pragma config  MCLRE	= OFF		// MCLR ピン イネーブルビット
#pragma config  STVREN	= ON		// スタックフル/ アンダーフロー リセット イネーブルビット
#pragma config  LVP		= OFF		// 単電源ICSP イネーブルビット
#pragma config  BBSIZ	= ON		// ブートブロック サイズ選択ビット(2kW boot block size)
#pragma config  XINST	= OFF		// 拡張命令セットイネーブルビット (ENHCPU)
#pragma config  CP0		= OFF		// コード保護ビット
#pragma config  CP1		= OFF		// コード保護ビット
#pragma config  CPB		= OFF		// ブートブロック コード保護ビット
#pragma config  CPD		= OFF		// EEPROM コード保護ビット
#pragma config  WRT0	= OFF		// 書き込み保護ビット
#pragma config  WRT1	= OFF		// 書き込み保護ビット
#pragma config  WRTC	= OFF		// コンフィグレーション レジスタ書き込み保護ビット
#pragma config  WRTB	= OFF		// ブートブロック書き込み保護ビット
#pragma config  WRTD	= OFF		// EEPROM 書き込み保護ビット
#pragma config  EBTR0	= OFF		// テーブル読み出し保護ビット
#pragma config  EBTR1	= OFF		// テーブル読み出し保護ビット
#pragma config  EBTRB	= OFF		// ブートブロック テーブル読み出し保護ビット

/*********************************************************************
* Function: void SYSTEM_Initialize( SYSTEM_STATE state )
*
* Overview: Initializes the system.
*
* PreCondition: None
*
* Input:  SYSTEM_STATE - the state to initialize the system into
*
* Output: None
*
********************************************************************/
void SYSTEM_Initialize( SYSTEM_STATE state )
{
    switch(state)
    {
        case SYSTEM_STATE_USB_START:
        	//IO Port Configrations
        	//I want a simple structure. Therefore,BSP setting was not used.
            PORTB = 0x0;
            PORTC = 0x0;
    		LATB = 0x0;
    		LATC = 0x0;
    		ANSEL = 0x00;
    		ANSELH = 0x00;

#if defined(OLD_MD2USB)
            // Old MD2USB Board (Development Only)
            //ATARI
    		TRISB = 0xD0;				//PORTB In:RB4/5 Out:RB6
            //MD
//    		TRISB = 0xB0;				//PORTB Input RB4/6 Out:RB5
   
    		TRISC = 0x0f;				//PORTC Input RC0-3
    		INTCON2bits.nRABPU = 0;		//WPU enable
    		WPUB = 0xf0;				//PORTB PULL UP Enable
										//PORTC Not support internal Pullup
#else
    		TRISB = 0xf0;				//PORTB In:RB4/5/6/7
   
    		TRISC = 0x0f;				//PORTC Input RC0-3
    		TRISC = 0x0f;				//PORTC Input RC0-4 (BUG FIX)
    		INTCON2bits.nRABPU = 0;		//WPU enable
    		WPUB = 0xf0;				//PORTB PULL UP Enable
            
            if (PORTBbits.RB6 == 0){    //DSUB PIN8 LOW
                //MD MODE
        		TRISB = 0xD0;			//PORTB Input RB4/6/7 Out:RB5
            }else{
                //ATARI MODE
        		TRISB = 0xB0;			//PORTB Input RB4/5/7 Out:RB6
            }
#endif
   

            break;
            
        case SYSTEM_STATE_USB_SUSPEND: 
            break;
            
        case SYSTEM_STATE_USB_RESUME:
            break;
    }
}

			
			
#if defined(__XC8)
void interrupt SYS_InterruptHigh(void)
{
    #if defined(USB_INTERRUPT)
        USBDeviceTasks();
    #endif
}
#else
        void YourHighPriorityISRCode();
        void YourLowPriorityISRCode();
        
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER

	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif

	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif

        #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}

	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.

	//To fix this situation, we should always deliberately place a
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code


	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
        #if defined(USB_INTERRUPT)
	        USBDeviceTasks();
        #endif

	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.

	}	//This return will be a "retfie", since this is in a #pragma interruptlow section
#endif

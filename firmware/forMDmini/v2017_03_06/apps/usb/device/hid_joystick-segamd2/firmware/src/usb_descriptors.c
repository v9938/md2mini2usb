/********************************************************************
 FileName:     	usb_descriptors.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18 or PIC24 USB Microcontrollers
*********************************************************************
-usb_descriptors.c-
-------------------------------------------------------------------
Filling in the descriptor values in the usb_descriptors.c file:
-------------------------------------------------------------------
********************************************************************/
 
/*********************************************************************
 * Descriptor specific type definitions are defined in:
 * usb_device.h
 *
 * Configuration options are defined in:
 * usb_config.h
 ********************************************************************/

/** INCLUDES *******************************************************/
#include "usb.h"
#include "usb_device_hid.h"

/** CONSTANTS ******************************************************/
#if defined(COMPILER_MPLAB_C18)
#pragma romdata
#endif

/* Device Descriptor */
const USB_DEVICE_DESCRIPTOR device_dsc=
{
    0x12,    // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,                // DEVICE descriptor type
    0x0200,                 // USB Spec Release Number in BCD format
    0x00,                   // Class Code
    0x00,                   // Subclass code
    0x00,                   // Protocol code
    USB_EP0_BUFF_SIZE,      // Max packet size for EP0, see usb_config.h
    0x0CA3,                 // Vendor ID, see usb_config.h
    0x0024,                 // Product ID, see usb_config.h
    0x0100,                 // Device release number in BCD format
    0x01,                   // Manufacturer string index
    0x02,                   // Product string index
    0x03,                   // Device serial number string index
    0x01                    // Number of possible configurations
};

/* Configuration 1 Descriptor */
const uint8_t configDescriptor1[]={
    /* Configuration Descriptor */
    0x09,//sizeof(USB_CFG_DSC),    // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                // CONFIGURATION descriptor type
    DESC_CONFIG_WORD(0x0022),                   // Total length of data for this cfg
    1,                      // Number of interfaces in this cfg
    1,                      // Index value of this configuration
    0,                      // Configuration string index
    _DEFAULT | _RWU,               // Attributes, see usb_device.h
    50,                     // Max power consumption (2X mA)

    /* Interface Descriptor */
    0x09,//sizeof(USB_INTF_DSC),   // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,               // INTERFACE descriptor type
    0,                      // Interface Number
    0,                      // Alternate Setting Number
    1,                      // Number of endpoints in this intf
    HID_INTF,               // Class code
    0,     // Subclass code
    0,     // Protocol code
    0,                      // Interface string index

    /* HID Class-Specific Descriptor */
    0x09,//sizeof(USB_HID_DSC)+3,    // Size of this descriptor in bytes RRoj hack
    DSC_HID,                // HID descriptor type
    DESC_CONFIG_WORD(0x0111),                 // HID Spec Release Number in BCD format (1.11)
    0x00,                   // Country Code (0x00 for Not supported)
    HID_NUM_OF_DSC,         // Number of class descriptors, see usbcfg.h
    DSC_RPT,                // Report descriptor type
    DESC_CONFIG_WORD(HID_RPT01_SIZE),   //sizeof(hid_rpt01),      // Size of the report descriptor
    
    /* Endpoint Descriptor */
    0x07,/*sizeof(USB_EP_DSC)*/
    USB_DESCRIPTOR_ENDPOINT,    //Endpoint Descriptor
    JOYSTICK_EP | _EP_IN,            //EndpointAddress
    _INTERRUPT,                       //Attributes
    DESC_CONFIG_WORD(64),        //size
    0xA,                        //Interval
};


//Language code string descriptor
const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[1];}sd000={
sizeof(sd000),USB_DESCRIPTOR_STRING,{0x0409
}};

//Manufacturer string descriptor
const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[4];}sd001={
sizeof(sd001),USB_DESCRIPTOR_STRING,
{'S','E','G','A'
}};

//Product string descriptor
const struct{uint8_t bLength;uint8_t bDscType;uint16_t string[13];}sd002={
sizeof(sd002),USB_DESCRIPTOR_STRING,
{'6','B',' ','c','o','n','t','r','o','l','l','e','r'
}};


//Array of configuration descriptors
const uint8_t *const USB_CD_Ptr[]=
{
    (const uint8_t *const)&configDescriptor1
};

//Array of string descriptors
const uint8_t *const USB_SD_Ptr[]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002
};
//Reference document:
//   https://www.usb.org/document-library/hid-descriptor-tool

const struct{uint8_t report[HID_RPT01_SIZE];}hid_rpt01={{
0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
0x09, 0x04,        // Usage (Joystick)
0xA1, 0x01,        // Collection (Application)
0xA1, 0x02,        //   Collection (Logical)
0x75, 0x08,        //     Report Size (8)
0x95, 0x05,        //     Report Count (5)
0x15, 0x00,        //     Logical Minimum (0)
0x26, 0xFF, 0x00,  //     Logical Maximum (255)
0x35, 0x00,        //     Physical Minimum (0)
0x46, 0xFF, 0x00,  //     Physical Maximum (255)
0x09, 0x30,        //     Usage (X)
0x09, 0x30,        //     Usage (X)
0x09, 0x30,        //     Usage (X)
0x09, 0x30,        //     Usage (X)
0x09, 0x31,        //     Usage (Y)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x75, 0x04,        //     Report Size (4)
0x95, 0x01,        //     Report Count (1)
0x25, 0x07,        //     Logical Maximum (7)
0x46, 0x3B, 0x01,  //     Physical Maximum (315)
0x65, 0x14,        //     Unit (System: English Rotation, Length: Centimeter)
0x09, 0x00,        //     Usage (Undefined)
0x81, 0x42,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
0x65, 0x00,        //     Unit (None)
0x75, 0x01,        //     Report Size (1)
0x95, 0x0A,        //     Report Count (10)
0x25, 0x01,        //     Logical Maximum (1)
0x45, 0x01,        //     Physical Maximum (1)
0x05, 0x09,        //     Usage Page (Button)
0x19, 0x01,        //     Usage Minimum (0x01)
0x29, 0x0A,        //     Usage Maximum (0x0A)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x06, 0x00, 0xFF,  //     Usage Page (Vendor Defined 0xFF00)
0x75, 0x01,        //     Report Size (1)
0x95, 0x0A,        //     Report Count (10)
0x25, 0x01,        //     Logical Maximum (1)
0x45, 0x01,        //     Physical Maximum (1)
0x09, 0x01,        //     Usage (0x01)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              //   End Collection
0xA1, 0x02,        //   Collection (Logical)
0x75, 0x08,        //     Report Size (8)
0x95, 0x04,        //     Report Count (4)
0x46, 0xFF, 0x00,  //     Physical Maximum (255)
0x26, 0xFF, 0x00,  //     Logical Maximum (255)
0x09, 0x02,        //     Usage (0x02)
0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0xC0,              //   End Collection
0xC0               // End Collection

// 101 bytes

}
};
/** EOF usb_descriptors.c ***************************************************/


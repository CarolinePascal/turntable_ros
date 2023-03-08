#ifndef USBGPIB_H
#define USBGPIB_H

#include <linux/version.h>

#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a)<16)|((b)<8)|(c))
#endif

// Vendor specific request code for Anchor Upload/Download
//
// This one is implemented in the core
//
#define ANCHOR_LOAD_INTERNAL  0xA0
//
// EZ-USB Control and Status Register.  Bit 0 controls 8051 reset
//
#define CPUCS_REG_FX2      0xE600
#define VR_REG              0xA1
#define VR_EEPROM           0xA2 // loads (uploads) EEPROM
#define VR_RAM              0xA3 // loads (uploads) external ram
#define	VR_FPGA             0xA4 // configure FPGA
#define VR_LOADER_REBOOT    0xC0

extern unsigned char usbgpib_int[];
extern unsigned long usbgpib_int_size;
extern unsigned char usbgpib_ext[];
extern unsigned long usbgpib_ext_size;
/*
extern unsigned char usbgpib_bix[];
extern unsigned long usbgpib_bix_size;
extern unsigned char usbgpib_RBF[];
extern unsigned long usbgpib_RBF_size;
*/

ssize_t fx2_fpga_download(struct usb_device *	udev, char *fileName);
ssize_t Firmware_Download_Int(struct usb_device *	udev, unsigned char * fImage, size_t FileLen );
ssize_t Firmware_Download_Ext(struct usb_device *	udev, unsigned char * fImage, size_t FileLen );

#endif

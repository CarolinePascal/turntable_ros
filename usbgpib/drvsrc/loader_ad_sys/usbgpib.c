//
// Include files needed for WDM driver support
//
#include <wdm.h>
#include "stdarg.h"
#include "stdio.h"

//
// Include files needed for USB support
//
#include "usbdi.h"
#include "usbdlib.h"

//
// Include file for the Ezusb Device
//
#include "usbdaq.h"
#include "usbgpib.h"

NTSTATUS
Ezusb_StartDevice(
    IN  PDEVICE_OBJECT fdo
    )
/*++

Routine Description:
   Initializes a given instance of the Ezusb Device on the USB.

   Arguments:
      fdo - pointer to the device object for this instance of a
                      Ezusb Device

Return Value:
   NT status code
--*/
{
    CHAR data;
    USHORT id;
    USHORT bcdDevice_iic;
    PDEVICE_EXTENSION pdx;

    Ezusb_KdPrint (("enter Ezusb_StartDevice\n"));

    //UDServiceFire(fdo, ADLINK_GET_DB_ID, 0, 0, sizeof(id), (PUCHAR)&id, 1);
    //Ezusb_KdPrint (("Daughter Board ID = %04X\n", id));

    pdx = fdo->DeviceExtension;

    GetDeviceDescriptor(fdo);
    if (pdx->DeviceDescriptor) {
        bcdDevice_iic = *(USHORT*)(&loader_iic[5]);
        if (pdx->DeviceDescriptor->bcdDevice < bcdDevice_iic) {
            Firmware_Download_EEPROM(fdo, loader_iic, loader_iic_size);
            Firmware_Download_Int(fdo, loader_bix, loader_bix_size);
        }
        ExFreePool(pdx->DeviceDescriptor);
    }

    //FileName = L"\\SystemRoot\\System32\\drivers\\G2.RBF";
    FileName = L"\\SystemRoot\\System32\\drivers\\USB-GPIB.RBF";
    if (NT_SUCCESS(Ezusb_DownloadFirmware(fdo, DLFW_OP_FPGA))) {
        Ezusb_KdPrint (("FPGA download Success.\n"));
    }
    else {
        Ezusb_KdPrint (("FPGA download Fail\n"));
    }

    /*
    FileName = L"\\SystemRoot\\System32\\drivers\\USB-2010.RBF";
    if (!NT_SUCCESS(Ezusb_DownloadFirmware(fdo, DLFW_OP_FPGA)))
        FPGA_Download(fdo, agu23xx_RBF, agu23xx_RBF_size);
    */

    /*
    SetInterface(fdo, 0, 0);

    Firmware_Download_Ext_BulkOut(fdo, agu2781_bix, agu2781_bix_size);
    */

    Firmware_Download_Ext(fdo, usbgpib_ext, usbgpib_ext_size);
    Firmware_Download_Int(fdo, usbgpib_int, usbgpib_int_size);

    //Firmware_Download_Int(fdo, usbgpib_bix, usbgpib_bix_size);

    Ezusb_KdPrint (("exit Ezusb_StartDevice\n"));

    return STATUS_SUCCESS;
}

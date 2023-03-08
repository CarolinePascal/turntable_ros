//////////////////////////////////////////////////////////////////////
//
// File:      ezusbsys.c
// $Archive: /EzUsb/Drivers/ezusbdrv/ezusbsys.c $
//
// Purpose:
//    General purpose USB device driver
//
// Environment:
//    kernel mode
//
// $Author: Markm $
//
// $History: ezusbsys.c $           
//  
//  *****************  Version 37  *****************
//  User: Markm        Date: 5/18/99    Time: 3:37p
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  
//  *****************  Version 36  *****************
//  User: Markm        Date: 4/29/99    Time: 9:48a
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  Fixed IOCTL_EZUSB_VENDOR_OR_CLASS_REQUEST so that it correctly returns
//  the number of bytes transferred.
//  
//  *****************  Version 35  *****************
//  User: Markm        Date: 4/16/99    Time: 3:39p
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  more minor changes to get rid of compiler warnings.
//  
//  *****************  Version 34  *****************
//  User: Markm        Date: 4/12/99    Time: 10:26a
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  minor changes to get rid of compiler warnings.
//  
//  *****************  Version 33  *****************
//  User: Markm        Date: 3/25/99    Time: 4:16p
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  Fixed a bug in the surprise removal code I just added.  I was returning
//  from the PnP dispatch function without unlocking the device object.
//  
//  *****************  Version 32  *****************
//  User: Markm        Date: 3/25/99    Time: 2:03p
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  Added code to allow unplugs (surprise removal) under NT5 without
//  notifying the user.
//  
//  *****************  Version 31  *****************
//  User: Markm        Date: 2/23/99    Time: 9:50a
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  Driver now supports ISO IN streaming with a path to user mode.
//  
//  *****************  Version 30  *****************
//  User: Markm        Date: 2/10/99    Time: 3:31p
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  removed lots of unused code.  Added ring buffer support functions.
//  
//  *****************  Version 29  *****************
//  User: Markm        Date: 2/01/99    Time: 11:57a
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  Added preliminary support for ISO streaming.
//  
//  *****************  Version 28  *****************
//  User: Markm        Date: 6/12/98    Time: 4:23p
//  Updated in $/EzUsb/Drivers/ezusbdrv
//  modified pipe reset and abort code so that the driver will work
//  correctly under Windows 95 OSR2.1.  For Win98, MS changed the size of
//  the URB_PIPE_REQUEST structure.  They added an extra ulong to it.  So,
//  a driver compiled with the 98 DDk and run under 95 will send the wrong
//  URB size.  The solution is to check the USBDI version whenever I do a
//  pipe reset or abort.  If the USBDI version is pre-win98, I subtract 4
//  from the size of the urb.  
//  
//  *****************  Version 27  *****************
//  User: Markm        Date: 4/10/98    Time: 2:52p
//  Updated in $/EZUSB/ezmon
//  Modified Intel Hex download code to stop the 8051 before downloading to
//  internal RAM.
//  
//  *****************  Version 26  *****************
//  User: Markm        Date: 4/09/98    Time: 4:39p
//  Updated in $/EZUSB/ezusb driver
//  Was not returning status information for ISO transfers.  Now it does!
//  
//  *****************  Version 25  *****************
//  User: Markm        Date: 4/09/98    Time: 4:39p
//  Updated in $/EZUSB/ezmon
//  changes for monitor version of driver
//  
//  *****************  Version 24  *****************
//  User: Markm        Date: 4/09/98    Time: 3:09p
//  Updated in $/EZUSB/ezusb driver
//  nothing much
//  
//  *****************  Version 23  *****************
//  User: Markm        Date: 4/09/98    Time: 3:00p
//  Updated in $/EZUSB/ezusb driver
//  Added function to download Intel Hex File records to EZ-USB.  For now,
//  this function is only used by the conditional build of the driver that
//  automatically downloads the Keil monitor.
//  
//  *****************  Version 22  *****************
//  User: Markm        Date: 4/07/98    Time: 1:52p
//  Updated in $/EZUSB/ezusb driver
//  Added IOCTL_EZUSB_GET_DRIVER_VERSION
//  
//  *****************  Version 21  *****************
//  User: Markm        Date: 4/06/98    Time: 4:26p
//  Updated in $/EZUSB/ezusb driver
//  Modified ISO transfer code.
//  * Transfer descriptors for the ISO transfer are now sent up to the
//  caller along with the actual data, so the caller can get the status of
//  the transfer on a packet-by-packet basis. 
//  * Disabled default values.  Caller must specify all fields in the ISO
//  control structure.
//  * Corrected bug where the Stream and Transfer objects weren't being
//  freed.
//  
//  Added some code to measure the latency of a bulk transfer.
//  
//  *****************  Version 20  *****************
//  User: Markm        Date: 3/19/98    Time: 10:13a
//  Updated in $/EZUSB/ezusb driver
//  Added IOCTL_EZUSB_ANCHOR_DOWNLOAD to support A0 loads to a specific
//  memory offset.
//  
//  *****************  Version 19  *****************
//  User: Markm        Date: 2/26/98    Time: 4:04p
//  Updated in $/EZUSB/ezusb driver
//  Added functions to perform anchor downloads and 8051 reset.
//  Added conditionally compiled code to support a special version of the
//  driver that will automatically download the Keil 8051 monitor after
//  device attachment.
//  
//  *****************  Version 18  *****************
//  User: Markm        Date: 2/25/98    Time: 2:09p
//  Updated in $/EZUSB/ezusb driver
//  changes for adding version resource to the driver
//  
//  *****************  Version 17  *****************
//  User: Markm        Date: 2/11/98    Time: 9:51a
//  Updated in $/EZUSB/ezusb driver
//  1. Added code to handle IRP_MN_CLOSE.
//  2. Now maintain a count of open file handles in the device extension.
//  3. Added workaround in Ezusb_SelectInterfaces() to avoid system hangs
//  during device removal under OSR2.1.  See comments there.
//  
//  *****************  Version 16  *****************
//  User: Markm        Date: 2/02/98    Time: 3:35p
//  Updated in $/EZUSB/ezusb driver
//  
//  *****************  Version 15  *****************
//  User: Markm        Date: 1/27/98    Time: 11:36a
//  Updated in $/EZUSB/ezusb driver
//  Modified ISO transfer code so that the number of transfer buffers and
//  the number of ISO frames per buffer can be specified by the caller.
//  
//  *****************  Version 14  *****************
//  User: Markm        Date: 1/26/98    Time: 10:11a
//  Updated in $/EZUSB/ezusb driver
//  1) modified device removal code that was crashing OSR2.1.  MS sample
//  code says that during device removal, the remove IRP should be passed
//  down the stack, and then the deviceobject should be deleted.  This
//  causes a bugcheck under OSR2.1.  Reversing the order of these
//  operations fixes the problem.
//  2) hardcoded the initial alternate setting to 2.  It was set to 1, but
//  this was also causing system crashes under OSR 2.1 during device
//  removal.  Still under investigation.
//  
//  *****************  Version 13  *****************
//  User: Markm        Date: 1/22/98    Time: 11:50a
//  Updated in $/EZUSB/ezusb driver
//  Fixed a bug in the device removal handler.  It wasn't deleteting the
//  symbolic link for the deviceobject after removal.
//  Removed lots of unused code.
//  Rewrote ISO loopback test code.
//  Wrote new ISO read/write functions.
//  
//  *****************  Version 12  *****************
//  User: Markm        Date: 1/18/98    Time: 3:20p
//  Updated in $/EZUSB/ezusb driver
//  renamed variables.  DeviceExtension becomes pdx and DeviceObject
//  becomes fdo.
//  rewrote handlers for device removal.   Cleaned up power management
//  dispatch.  Added code to maintain a usage count for the device.
//  
//  *****************  Version 11  *****************
//  User: Markm        Date: 1/15/98    Time: 4:36p
//  Updated in $/EZUSB/ezusb driver
//  Added
//  IOCTL_EZUSB_GET_CURRENT_FRAME_NUMBER
//  IOCTL_EZUSB_VENDOR_OR_CLASS_REQUEST
//  IOCTL_EZUSB_GET_LAST_ERROR
//  preliminary code for handling device removal gracefully.
//  
//  *****************  Version 10  *****************
//  User: Markm        Date: 1/14/98    Time: 10:29a
//  Updated in $/EZUSB/ezusb driver
//  Cleanup.
//  New functions for handling bulk transfers.
//  
//  *****************  Version 9  *****************
//  User: Markm        Date: 1/08/98    Time: 5:14p
//  Updated in $/EZUSB/ezusb driver
//  major changes to PnP IRP handling.  lots of reformatting.
//  
//  *****************  Version 8  *****************
//  User: Markm        Date: 1/02/98    Time: 1:41p
//  Updated in $/EZUSB/ezusb driver
//  Added support for setting the interface, preliminary code for naming
//  pipes, get string descriptor
//  
//  *****************  Version 7  *****************
//  User: Markm        Date: 11/18/97   Time: 4:40p
//  Updated in $/EZUSB/ezusb driver
//  Added abort pipe IOCTL
//  Added function to dump a buffer to the debuger
//  Added experimental file I/O code (from Brad Carpenter)
//  
//  *****************  Version 6  *****************
//  User: Markm        Date: 11/17/97   Time: 9:37a
//  Updated in $/EZUSB/ezusb driver
//  fixed bug where I set the pipesize
//  
//  *****************  Version 5  *****************
//  User: Markm        Date: 11/14/97   Time: 4:53p
//  Updated in $/EZUSB/ezusb driver
//  started using USBD_CreateConfigurationRequestEx
//  
//  *****************  Version 4  *****************
//  User: Markm        Date: 11/14/97   Time: 4:31p
//  Updated in $/EZUSB/ezusb driver
//  added code to experiment wth different methods of switiching
//  interfaces.
//  
//  *****************  Version 3  *****************
//  User: Markm        Date: 11/07/97   Time: 1:23p
//  Updated in $/EZUSB/ezusb driver
//  added sourcesafe keywords
//  
// Copyright (c) 1997 Anchor Chips, Inc.  May not be reproduced without
// permission.  See the license agreement for more details.
//
//////////////////////////////////////////////////////////////////////

// #define DRIVER
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
#include "ezusbsys.h"
#include "AdlDrvCm.h"
//#include "Register.h"
//#include "Service.h"

//
// incude file containing driver version
//
//#include "version.h"

//yuan add 05/08/01
#define OV_W2K         0
#define OV_WME         1
#define OV_W98_SE      2 
#define OV_W98_GOLD    3

USHORT Ver;

void
DumpBuffer(PVOID pvBuffer, ULONG length);

extern void AdlSleepU
(
	ULONG				dwUS
);
/*===========================================================*/

VOID GetPlatform (IN PDRIVER_OBJECT DriverObject)
{
  if (IoIsWdmVersionAvailable(0x01,0x10))
  {
    Ver = OV_W2K;
  }
  else if (IoIsWdmVersionAvailable(0x01,0x05))
  {
    Ver = OV_WME;
  }
  else if (IoIsWdmVersionAvailable(0x01,0x00))
  {
    if (DriverObject->DriverExtension->ServiceKeyName.Length)
    {
      Ver = OV_W98_SE;
    }
    else
    {
      Ver = OV_W98_GOLD;
    }
  }
}

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
    PPCI_DEVEXT pdx;
    NTSTATUS ntStatus;
    PUSB_DEVICE_DESCRIPTOR deviceDescriptor = NULL;
    PURB urb;
    ULONG siz;

    //Ezusb_KdPrint (("enter Ezusb_StartDevice\n"));

    pdx = fdo->DeviceExtension;
    pdx->NeedCleanup = TRUE;

    /*
    // Get some memory from then non paged pool (fixed, locked system memory)
    // for use by the USB Request Block (urb) for the specific USB Request we
    // will be performing below (a USB device request).
    */
    urb = ExAllocatePool( NonPagedPool,
                          sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));

    if (urb) {

        siz = sizeof(USB_DEVICE_DESCRIPTOR);

        // Get some non paged memory for the device descriptor contents
        deviceDescriptor = ExAllocatePool(NonPagedPool,
                                          siz);

        if (deviceDescriptor) {

            // Use a macro in the standard USB header files to build the URB
            UsbBuildGetDescriptorRequest(urb,
                                         (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                         USB_DEVICE_DESCRIPTOR_TYPE,
                                         0,
                                         0,
                                         deviceDescriptor,
                                         NULL,
                                         siz,
                                         NULL);
//change 12/29/11
            // Get the device descriptor
            ntStatus = Ezusb_CallUSBD2(fdo, urb);
            //ntStatus = Ezusb_CallUSBD(fdo, urb);

            // Dump out the descriptor info to the debugger
            if (NT_SUCCESS(ntStatus)) {
                Ezusb_KdPrint (("Device Descriptor = %x, len %x\n",
                                deviceDescriptor,
                                urb->UrbControlDescriptorRequest.TransferBufferLength));

                Ezusb_KdPrint (("Ezusb Device Descriptor:\n"));
                Ezusb_KdPrint (("-------------------------\n"));
                Ezusb_KdPrint (("bLength %d\n", deviceDescriptor->bLength));
                Ezusb_KdPrint (("bDescriptorType 0x%x\n", deviceDescriptor->bDescriptorType));
                Ezusb_KdPrint (("bcdUSB 0x%x\n", deviceDescriptor->bcdUSB));
                Ezusb_KdPrint (("bDeviceClass 0x%x\n", deviceDescriptor->bDeviceClass));
                Ezusb_KdPrint (("bDeviceSubClass 0x%x\n", deviceDescriptor->bDeviceSubClass));
                Ezusb_KdPrint (("bDeviceProtocol 0x%x\n", deviceDescriptor->bDeviceProtocol));
                Ezusb_KdPrint (("bMaxPacketSize0 0x%x\n", deviceDescriptor->bMaxPacketSize0));
                Ezusb_KdPrint (("idVendor 0x%x\n", deviceDescriptor->idVendor));
                Ezusb_KdPrint (("idProduct 0x%x\n", deviceDescriptor->idProduct));
                Ezusb_KdPrint (("bcdDevice 0x%x\n", deviceDescriptor->bcdDevice));
                Ezusb_KdPrint (("iManufacturer 0x%x\n", deviceDescriptor->iManufacturer));
                Ezusb_KdPrint (("iProduct 0x%x\n", deviceDescriptor->iProduct));
                Ezusb_KdPrint (("iSerialNumber 0x%x\n", deviceDescriptor->iSerialNumber));
                Ezusb_KdPrint (("bNumConfigurations 0x%x\n", deviceDescriptor->bNumConfigurations));
            }
        } else {
            ntStatus = STATUS_NO_MEMORY;
        }

        if (NT_SUCCESS(ntStatus)) {
            /*
            // Put a ptr to the device descriptor in the device extension for easy
            // access (like a "cached" copy).  We will free this memory when the
            // device is removed.  See the "Ezusb_RemoveDevice" code.
            */
            pdx->DeviceDescriptor = deviceDescriptor;
            //...pdx->Stopped = FALSE;
        } else if (deviceDescriptor) {
            /*
            // If the bus transaction failed, then free up the memory created to hold
            // the device descriptor, since the device is probably non-functional
            */
            ExFreePool(deviceDescriptor);
            pdx->DeviceDescriptor = NULL;
        }

        ExFreePool(urb);

    } else {
        // Failed getting memory for the Urb 
        ntStatus = STATUS_NO_MEMORY;
    }

    // If the Get_Descriptor call was successful, then configure the device.
    if (NT_SUCCESS(ntStatus)) {
        ntStatus = Ezusb_ConfigureDevice(fdo);
    }

    Ezusb_KdPrint (("exit Ezusb_StartDevice (%x)\n", ntStatus));

    return ntStatus;
}

NTSTATUS
Ezusb_StopDevice(
   IN  PDEVICE_OBJECT fdo
   )
/*++
Routine Description:
   Stops a given instance of a Ezusb Device device on USB.

Arguments:
   fdo - pointer to the device object for this instance of a Ezusb Device

Return Value:
   NT status code

  --*/
{
   PPCI_DEVEXT pdx;
   NTSTATUS ntStatus = STATUS_SUCCESS;
   PURB urb;
   ULONG siz;

   Ezusb_KdPrint (("enter Ezusb_StopDevice\n"));

   pdx = fdo->DeviceExtension;

   //
   // Send the select configuration urb with a NULL pointer for the configuration
   // handle, this closes the configuration and puts the device in the 'unconfigured'
   // state.
   //

   siz = sizeof(struct _URB_SELECT_CONFIGURATION);

   urb = ExAllocatePool(NonPagedPool,
                      siz);

   if (urb)
   {
      NTSTATUS status;

      UsbBuildSelectConfigurationRequest(urb,
                                       (USHORT) siz,
                                       NULL);
//modify 12/29/11
      status = Ezusb_CallUSBD2(fdo, urb);
      //status = Ezusb_CallUSBD(fdo, urb);
      
      Ezusb_KdPrint (("Device Configuration Closed status = %x usb status = %x.\n",
                     status, urb->UrbHeader.Status));

      ExFreePool(urb);
   }
   else
   {
      ntStatus = STATUS_NO_MEMORY;
   }

   Ezusb_KdPrint (("exit Ezusb_StopDevice (%x)\n", ntStatus));

   return ntStatus;
}

NTSTATUS
Ezusb_CallUSBD(
    IN PDEVICE_OBJECT fdo,
    IN PURB Urb
    )
/*++

Routine Description:
   Passes a Usb Request Block (URB) to the USB class driver (USBD)

Arguments:
   fdo - pointer to the device object for this instance of an Ezusb Device
   Urb          - pointer to Urb request block

Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise

--*/
{
   NTSTATUS ntStatus, status = STATUS_SUCCESS;
   PPCI_DEVEXT pdx;
   PIRP irp = 0;
   KEVENT event;
   IO_STATUS_BLOCK ioStatus;
   PIO_STACK_LOCATION nextStack;
      LARGE_INTEGER  qwTickRef;
KIRQL kIrql;
   ///Ezusb_KdPrint (("enter Ezusb_CallUSBD\n"));

   pdx = fdo->DeviceExtension;
   //Ezusb_KdPrint(("enter usbd0\n"));
   //KeAcquireSpinLock( &pdx->usbd_lock, &kIrql);
   ExAcquireFastMutex (&pdx->usbd_mutex);
   //Ezusb_KdPrint(("enter usbd1\n"));
  /***if(pdx->dmai) {
   	qwTickRef.QuadPart = ((LONGLONG)((LONG)(-10)) * (LONGLONG)((ULONG)(pdx->usec_timeout)));
   	ntStatus = KeWaitForSingleObject((PRKEVENT) &pdx->evRequest, Executive, KernelMode, FALSE, &qwTickRef);
   	if(ntStatus != STATUS_SUCCESS) {
   		//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   		return ntStatus;
   	}
   }
   pdx->dmai = 1;
   KeClearEvent(&pdx->evRequest);****/
   // issue a synchronous request (see notes above)
   KeInitializeEvent(&event, NotificationEvent, FALSE);

   irp = IoBuildDeviceIoControlRequest(
             IOCTL_INTERNAL_USB_SUBMIT_URB,
             pdx->pStackTopDO,
             NULL,
             0,
             NULL,
             0,
             TRUE, /* INTERNAL */
             &event,
             &ioStatus);
if(!irp) {
	Ezusb_KdPrint (("BUILD irp error\n"));
	ExReleaseFastMutex(&pdx->usbd_mutex);
	return STATUS_INSUFFICIENT_RESOURCES;
	}
   // Prepare for calling the USB driver stack
   nextStack = IoGetNextIrpStackLocation(irp);
   ASSERT(nextStack != NULL);

   // Set up the URB ptr to pass to the USB driver stack
   nextStack->Parameters.Others.Argument1 = Urb;

   ////Ezusb_KdPrint (("Calling USB Driver Stack\n"));

   //
   // Call the USB class driver to perform the operation.  If the returned status
   // is PENDING, wait for the request to complete.
   //
//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   ntStatus = IoCallDriver(pdx->pStackTopDO,
                         irp);
//Ezusb_KdPrint(("enter usbd2\n"));                         
ExReleaseFastMutex(&pdx->usbd_mutex);
//Ezusb_KdPrint(("enter usbd3\n"));
   ////Ezusb_KdPrint (("return from IoCallDriver USBD %x\n", ntStatus));
//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   if (ntStatus == STATUS_PENDING)
   {
      //Ezusb_KdPrint (("Wait for single object\n"));
qwTickRef.QuadPart = ((LONGLONG)((LONG)(-10)) * (LONGLONG)((ULONG)(pdx->usec_timeout)));
      status = KeWaitForSingleObject(
                    &event,
                    Executive,//Suspended,
                    KernelMode,
                    FALSE,
                    NULL
                    );

      Ezusb_KdPrint (("Wait for single object, returned %x\n", status));
   }
   else
   {
      ioStatus.Status = ntStatus;
   }
//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   Ezusb_KdPrint (("URB status = %x status = %x irp status %x\n", Urb->UrbHeader.Status, status, ioStatus.Status));

   ntStatus = ioStatus.Status;
   //
   // If the URB status was not USBD_STATUS_SUCCESS, we save a copy of the
   // URB status in the device extension.  After a failure, another IOCTL,
   // IOCTL_EZUSB_GET_LAST_ERROR can be used to retrieve the URB status
   // for the most recently failed URB.  Of course, this status gets
   // overwritten by subsequent failures, but it's better than nothing.
   //
   if (!(USBD_SUCCESS(Urb->UrbHeader.Status)))
      pdx->LastFailedUrbStatus = Urb->UrbHeader.Status;

   //
   // if ioStatus.Status indicates an error (ie. the IRP failed) then return that.
   // If ioStatus.Status indicates success, it is still possible that what we were
   // trying to do failed.  For example, if the IRP is cancelled, the status returned
   // by the I/O manager for the IRP will not indicate an error.  In that case, we
   // should check the URB status.  If it indicates anything other than
   // USBD_SUCCESS, then we should return STATUS_UNSUCCESSFUL.
   //
   if (NT_SUCCESS(ntStatus))
   {
      if (!(USBD_SUCCESS(Urb->UrbHeader.Status)))
         ntStatus = STATUS_UNSUCCESSFUL;
   }
  //AdlSleepU(1);
//KeSetEvent(&pdx->evRequest, 0, FALSE);

   ////Ezusb_KdPrint(("exit Ezusb_CallUSBD (%x)\n", ntStatus));

   return ntStatus;
}

NTSTATUS
Ezusb_CallUSBD2(
    IN PDEVICE_OBJECT fdo,
    IN PURB Urb
    )
/*++

Routine Description:
   Passes a Usb Request Block (URB) to the USB class driver (USBD)

Arguments:
   fdo - pointer to the device object for this instance of an Ezusb Device
   Urb          - pointer to Urb request block

Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise

--*/
{
   NTSTATUS ntStatus, status = STATUS_SUCCESS;
   PPCI_DEVEXT pdx;
   PIRP irp = 0;
   KEVENT event;
   IO_STATUS_BLOCK ioStatus;
   PIO_STACK_LOCATION nextStack;
      LARGE_INTEGER  qwTickRef;
KIRQL kIrql;
   Ezusb_KdPrint (("enter Ezusb_CallUSBD2\n"));

   pdx = fdo->DeviceExtension;
   
   //KeAcquireSpinLock( &pdx->usbd_lock, &kIrql);
   /////ExAcquireFastMutex (&pdx->usbd_mutex);
  /***if(pdx->dmai) {
   	qwTickRef.QuadPart = ((LONGLONG)((LONG)(-10)) * (LONGLONG)((ULONG)(pdx->usec_timeout)));
   	ntStatus = KeWaitForSingleObject((PRKEVENT) &pdx->evRequest, Executive, KernelMode, FALSE, &qwTickRef);
   	if(ntStatus != STATUS_SUCCESS) {
   		//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   		return ntStatus;
   	}
   }
   pdx->dmai = 1;
   KeClearEvent(&pdx->evRequest);****/
   // issue a synchronous request (see notes above)
   KeInitializeEvent(&event, NotificationEvent, FALSE);

   irp = IoBuildDeviceIoControlRequest(
             IOCTL_INTERNAL_USB_SUBMIT_URB,
             pdx->pStackTopDO,
             NULL,
             0,
             NULL,
             0,
             TRUE, /* INTERNAL */
             &event,
             &ioStatus);
if(!irp) {
	Ezusb_KdPrint (("BUILD irp error\n"));
	/////ExReleaseFastMutex(&pdx->usbd_mutex);
	return STATUS_INSUFFICIENT_RESOURCES;
	}
   // Prepare for calling the USB driver stack
   nextStack = IoGetNextIrpStackLocation(irp);
   ASSERT(nextStack != NULL);

   // Set up the URB ptr to pass to the USB driver stack
   nextStack->Parameters.Others.Argument1 = Urb;

   ////Ezusb_KdPrint (("Calling USB Driver Stack\n"));

   //
   // Call the USB class driver to perform the operation.  If the returned status
   // is PENDING, wait for the request to complete.
   //
//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   ntStatus = IoCallDriver(pdx->pStackTopDO,
                         irp);
                         
//////ExReleaseFastMutex(&pdx->usbd_mutex);
   ////Ezusb_KdPrint (("return from IoCallDriver USBD %x\n", ntStatus));
//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   if (ntStatus == STATUS_PENDING)
   {
      //Ezusb_KdPrint (("Wait for single object\n"));
/////qwTickRef.QuadPart = ((LONGLONG)((LONG)(-10)) * (LONGLONG)((ULONG)(pdx->usec_timeout)));
      status = KeWaitForSingleObject(
                    &event,
                    Executive,//Suspended,
                    KernelMode,
                    FALSE,
                    NULL
                    );

      Ezusb_KdPrint (("Wait for single object, returned %x\n", status));
   }
   else
   {
      ioStatus.Status = ntStatus;
   }
//KeReleaseSpinLock(&pdx->usbd_lock, kIrql);
   Ezusb_KdPrint (("URB status2 = %x status = %x irp status %x\n", Urb->UrbHeader.Status, status, ioStatus.Status));

   ntStatus = ioStatus.Status;
   //
   // If the URB status was not USBD_STATUS_SUCCESS, we save a copy of the
   // URB status in the device extension.  After a failure, another IOCTL,
   // IOCTL_EZUSB_GET_LAST_ERROR can be used to retrieve the URB status
   // for the most recently failed URB.  Of course, this status gets
   // overwritten by subsequent failures, but it's better than nothing.
   //
   if (!(USBD_SUCCESS(Urb->UrbHeader.Status)))
      pdx->LastFailedUrbStatus = Urb->UrbHeader.Status;

   //
   // if ioStatus.Status indicates an error (ie. the IRP failed) then return that.
   // If ioStatus.Status indicates success, it is still possible that what we were
   // trying to do failed.  For example, if the IRP is cancelled, the status returned
   // by the I/O manager for the IRP will not indicate an error.  In that case, we
   // should check the URB status.  If it indicates anything other than
   // USBD_SUCCESS, then we should return STATUS_UNSUCCESSFUL.
   //
   if (NT_SUCCESS(ntStatus))
   {
      if (!(USBD_SUCCESS(Urb->UrbHeader.Status)))
         ntStatus = STATUS_UNSUCCESSFUL;
   }
  //AdlSleepU(1);
//KeSetEvent(&pdx->evRequest, 0, FALSE);

   Ezusb_KdPrint(("exit Ezusb_CallUSBD2 (%x)\n", ntStatus));

   return ntStatus;
}

NTSTATUS
UDServiceFire(
   IN PDEVICE_OBJECT fdo,
   //IN PPCI_DEVEXT  pdx,
   ULONG Request, 
   ULONG Value, 
   ULONG Index, 
   ULONG data_len, 
   PUCHAR data,
   ULONG Direction
   )
{
   PPCI_DEVEXT          pdx = fdo->DeviceExtension;
   //PDEVICE_OBJECT             fdo = pdx->pSelfFDO
   NTSTATUS                   ntStatus;
   //UCHAR                      _urb[sizeof(struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST)];
   //PURB                       urb = (PURB)_urb;
   PURB                       urb = NULL;
   ULONG                      urbSize;

   ULONG                      transferFlags;
   USHORT                     urbFunction = 0;
   LARGE_INTEGER  qwTickRef;
   KIRQL kIrql;
   
   //KeAcquireSpinLock( &pdx->register_page_lock, &kIrql);
   //Ezusb_KdPrint(("enter servicefire0\n"));
   ExAcquireFastMutex (&pdx->srv_mutex);
   //Ezusb_KdPrint(("enter servicefire1\n"));
   if(pdx->dmai) {
      //yuan add 03/09/10
      if(pdx->usec_timeout)
      {
   	qwTickRef.QuadPart = ((LONGLONG)((LONG)(-10)) * (LONGLONG)((ULONG)(pdx->usec_timeout)));
   	ntStatus = KeWaitForSingleObject((PRKEVENT) &pdx->evRequest, Executive, KernelMode, FALSE, &qwTickRef);
   	if(ntStatus != STATUS_SUCCESS) {
   		//KeReleaseSpinLock(&pdx->register_page_lock, kIrql);
   		ExReleaseFastMutex(&pdx->srv_mutex);
   		return ntStatus;
   	}
      } else 
      {
   	ntStatus = KeWaitForSingleObject((PRKEVENT) &pdx->evRequest, Executive, KernelMode, FALSE, NULL);
   	if(ntStatus != STATUS_SUCCESS) {
   		//KeReleaseSpinLock(&pdx->register_page_lock, kIrql);
   		ExReleaseFastMutex(&pdx->srv_mutex);
   		return ntStatus;
   	}      	
      }
   }
   pdx->dmai = 1;
   KeClearEvent(&pdx->evRequest);
  // Ezusb_KdPrint(("enter servicefire2\n"));
   //
   // allocate and fill in the Usb request (URB)
   //
   urbSize = sizeof(struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST);

   
   urb = ExAllocatePool(NonPagedPool,urbSize);

   if (!urb)
   {
      //KeReleaseSpinLock(&pdx->register_page_lock, kIrql);
      ExReleaseFastMutex(&pdx->srv_mutex);
      return STATUS_NO_MEMORY;
   }
   

   RtlZeroMemory(urb,urbSize);

   // transferFlags = USBD_SHORT_TRANSFER_OK;

   //
   // the type of request (class or vendor) and the recepient
   // (device, interface, endpoint, other) combine to determine the 
   // URB function.  The following ugly code transforms fields in
   // the input param into an URB function
   //
   
   transferFlags = USBD_SHORT_TRANSFER_OK;
   if (Direction == 1) {
        transferFlags |= USBD_TRANSFER_DIRECTION_IN;
   }   
   urbFunction = URB_FUNCTION_VENDOR_DEVICE;
   
UsbBuildVendorRequest(
    urb,
    urbFunction,
    sizeof(struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST),
    transferFlags,
    0,
    (UCHAR) Request,
    (USHORT) Value,
    (USHORT) Index,
    data,
    NULL,
    data_len,
    NULL
    );   
   
   /*urb->UrbHeader.Length = sizeof(struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST);
   urb->UrbHeader.Function = urbFunction;

   urb->UrbControlVendorClassRequest.TransferFlags = transferFlags;
   urb->UrbControlVendorClassRequest.TransferBufferLength = data_len;
   urb->UrbControlVendorClassRequest.TransferBuffer = data;
   urb->UrbControlVendorClassRequest.TransferBufferMDL = NULL;
   urb->UrbControlVendorClassRequest.Request = (UCHAR) Request;
   urb->UrbControlVendorClassRequest.Value = (USHORT) Value;
   urb->UrbControlVendorClassRequest.Index = (USHORT) Index;
   */
   //KeReleaseSpinLock(&pdx->register_page_lock, kIrql);
   //Ezusb_KdPrint(("enter servicefire3\n"));
   ExReleaseFastMutex(&pdx->srv_mutex);
   //Ezusb_KdPrint(("enter servicefire4\n"));
   //
   // Call the USB Stack.
   //
    ntStatus = Ezusb_CallUSBD(fdo, urb);

   //
   // If the transfer was successful, report the length of the transfer to the
   // caller by setting IoStatus.Information
   //
   if (NT_SUCCESS(ntStatus))
   {
      //Irp->IoStatus.Information = urb->UrbControlVendorClassRequest.TransferBufferLength;
      Ezusb_KdPrint(("Successfully transfered 0x%x bytes\n",urb->UrbControlVendorClassRequest.TransferBufferLength));
   }


    ExFreePool(urb);
    
    //...pdx->dmai = 0;
    //Ezusb_KdPrint(("enter servicefire5\n"));
    KeSetEvent(&pdx->evRequest, 0, FALSE);   
    //Ezusb_KdPrint(("enter servicefire6\n"));
       
   return ntStatus;
}

NTSTATUS
Ezusb_ConfigureDevice(
    IN  PDEVICE_OBJECT fdo
    )
/*++
Routine Description:
   Configures the USB device via USB-specific device requests and interaction
   with the USB software subsystem.

Arguments:
   fdo - pointer to the device object for this instance of the Ezusb Device
 
Return Value:
   NT status code
--*/
{
   PPCI_DEVEXT pdx;
   NTSTATUS ntStatus;
   PURB urb = NULL;
   ULONG siz;
   PUSB_CONFIGURATION_DESCRIPTOR configurationDescriptor = NULL;

   Ezusb_KdPrint (("enter Ezusb_ConfigureDevice\n"));

   pdx = fdo->DeviceExtension;

   //
   // Get memory for the USB Request Block (urb).
   //
   urb = ExAllocatePool(NonPagedPool,
                      sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));

   if (urb != NULL)
   {
      //
      // Set size of the data buffer.  Note we add padding to cover hardware faults
      // that may cause the device to go past the end of the data buffer
      //
        siz = sizeof(USB_CONFIGURATION_DESCRIPTOR) + 16;

        // Get the nonpaged pool memory for the data buffer
        configurationDescriptor = ExAllocatePool(NonPagedPool, siz);

        if (configurationDescriptor != NULL) {

            UsbBuildGetDescriptorRequest(urb,
                                         (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                         USB_CONFIGURATION_DESCRIPTOR_TYPE,
                                         0,
                                         0,
                                         configurationDescriptor,
                                         NULL,
                                         sizeof (USB_CONFIGURATION_DESCRIPTOR),/* Get only the configuration descriptor */
                                         NULL);

	    //add 12/29/11
	    ntStatus = Ezusb_CallUSBD2(fdo, urb);
            //ntStatus = Ezusb_CallUSBD(fdo, urb);

            if (NT_SUCCESS(ntStatus)) {
                Ezusb_KdPrint (("Configuration Descriptor is at %x, bytes txferred: %d\n\
                                  Configuration Descriptor Actual Length: %d\n",
                                  configurationDescriptor,
                                  urb->UrbControlDescriptorRequest.TransferBufferLength,
                                  configurationDescriptor->wTotalLength));
            }//if

        } else {
            ntStatus = STATUS_NO_MEMORY;
            goto Exit_EzusbConfigureDevice;
        }//if-else

        // Determine how much data is in the entire configuration descriptor
        // and add extra room to protect against accidental overrun
        siz = configurationDescriptor->wTotalLength + 16;

        //  Free up the data buffer memory just used
        ExFreePool(configurationDescriptor);
        configurationDescriptor = NULL;

        // Get nonpaged pool memory for the data buffer
        configurationDescriptor = ExAllocatePool(NonPagedPool, siz);

        // Now get the entire Configuration Descriptor
        if (configurationDescriptor != NULL) {
            UsbBuildGetDescriptorRequest(urb,
                                         (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                         USB_CONFIGURATION_DESCRIPTOR_TYPE,
                                         0,
                                         0,
                                         configurationDescriptor,
                                         NULL,
                                         siz,  // Get all the descriptor data
                                         NULL);

	    //modify 12/29/11
	    ntStatus = Ezusb_CallUSBD2(fdo, urb);
            //ntStatus = Ezusb_CallUSBD(fdo, urb);

            if (NT_SUCCESS(ntStatus)) {
                Ezusb_KdPrint (("Entire Configuration Descriptor is at %x, bytes txferred: %d\n",
                                  configurationDescriptor,
                                  urb->UrbControlDescriptorRequest.TransferBufferLength));
            } else {
                //Error in getting configuration descriptor
                goto Exit_EzusbConfigureDevice;
            }//else

        } else {
            // Failed getting data buffer (configurationDescriptor) memory
            ntStatus = STATUS_NO_MEMORY;
            goto Exit_EzusbConfigureDevice;
        }//if-else

    } else {
        // failed getting urb memory
        ntStatus = STATUS_NO_MEMORY;
        goto Exit_EzusbConfigureDevice;
    }//if-else

    /*
    // We have the configuration descriptor for the configuration
    // we want.
    //
    // Now we issue the SelectConfiguration command to get
    // the  pipes associated with this configuration.
    */
    if (configurationDescriptor) {
        // Get our pipes
        ntStatus = Ezusb_SelectInterfaces(fdo,
                                           configurationDescriptor,
                                           NULL // Device not yet configured
                                           );
    } //if

Exit_EzusbConfigureDevice:

    // Clean up and exit this routine
    if (urb != NULL) {
        ExFreePool(urb);                    // Free urb memory
    }//if

    if (configurationDescriptor != NULL) {
        ExFreePool(configurationDescriptor);// Free data buffer
    }//if

    Ezusb_KdPrint (("exit Ezusb_ConfigureDevice (%x)\n", ntStatus));
    return ntStatus;
}//Ezusb_ConfigureDevice


NTSTATUS
Ezusb_SelectInterfaces(
    IN PDEVICE_OBJECT fdo,
    IN PUSB_CONFIGURATION_DESCRIPTOR ConfigurationDescriptor,
    IN PUSBD_INTERFACE_INFORMATION Interface
    )
/*++

Routine Description:
    Initializes an Ezusb Device with multiple interfaces

Arguments:
    fdo            - pointer to the device object for this instance of the Ezusb Device
    ConfigurationDescriptor - pointer to the USB configuration descriptor containing the interface and endpoint
                              descriptors.
    Interface               - pointer to a USBD Interface Information Object
                            - If this is NULL, then this driver must choose its interface based on driver-specific
                              criteria, and the driver must also CONFIGURE the device.
                            - If it is NOT NULL, then the driver has already been given an interface and
                              the device has already been configured by the parent of this device driver.

Return Value:
    NT status code
--*/
{
   PPCI_DEVEXT pdx;
   NTSTATUS ntStatus;
   PURB urb;
   ULONG j;
   UCHAR alternateSetting, MyInterfaceNumber;
   PUSBD_INTERFACE_INFORMATION interfaceObject;
   USBD_INTERFACE_LIST_ENTRY interfaceList[2];

   Ezusb_KdPrint (("enter Ezusb_SelectInterfaces %d:\n", ConfigurationDescriptor->bNumInterfaces));

   pdx = fdo->DeviceExtension;
   MyInterfaceNumber = SAMPLE_INTERFACE_NBR;

   //
   // At this time there appears to be a bug in OSR2.1 related to setting interfaces
   // that have 0 endpoints.  The anchor generic device has a 0 endpoint alternate
   // setting at interface 0 alternate setting 0.  By default, I would like this code
   // to select that alternate setting.  Unfortunately, setting that interface will
   // cause the machine to hang (under OSR2.1) when the device is removed.  The cause
   // of this problem is still under investigation.  Until it it resolved, I have
   // introduced the following mess.  Check the VID and PID in the device descriptor.
   // If it is the EZUSB development board, then set to Interface 0 alternate setting
   // 1.  This AS has endpoints.  For all other PID's, we will set to Interface 0
   // Alternate Setting 0.
   //
   if (pdx->DeviceDescriptor->idVendor == 0x0547 &&
       pdx->DeviceDescriptor->idProduct == 0x0080)
   {
      alternateSetting = 1;
   }
   else
   {
      alternateSetting = 0;
   }

   /*
   // Call a USBD helper function that returns a ptr to a USB Interface Descriptor given
   // a USB Configuration Descriptor, an Interface Number, and an Alternate Setting for that Interface
   */
   interfaceList[0].InterfaceDescriptor =
   /****USBD_ParseConfigurationDescriptor(ConfigurationDescriptor,
                                    MyInterfaceNumber, //interface number (this is bInterfaceNumber from interface descr)
                                    alternateSetting);***/
	USBD_ParseConfigurationDescriptorEx(
                                            ConfigurationDescriptor, 
                                            ConfigurationDescriptor,
                                            0,
                                            0, -1, -1, -1);                                    

   ASSERT(interfaceList[0].InterfaceDescriptor != NULL);
//yuan tmp add
interfaceList[0].Interface = NULL;

   interfaceList[1].InterfaceDescriptor = NULL;
   interfaceList[1].Interface = NULL;

   urb = USBD_CreateConfigurationRequestEx(ConfigurationDescriptor,&interfaceList[0]);

   if (!urb)
   {
      Ezusb_KdPrint ((" USBD_CreateConfigurationRequestEx failed\n"));            
   }

   DumpBuffer(urb, urb->UrbHeader.Length);

   interfaceObject = (PUSBD_INTERFACE_INFORMATION) (&(urb->UrbSelectConfiguration.Interface));
   Ezusb_KdPrint (("pipes: %d \n", interfaceObject->NumberOfPipes)); 

   /*
   // We set up a default max transfer size for the endpoints.  Your driver will
   // need to change this to reflect the capabilities of your device's endpoints.
   */
   //for (j=0; j<interfaceList[0].InterfaceDescriptor->bNumEndpoints; j++)
      //interfaceObject->Pipes[j].MaximumTransferSize = (64 * 1024) - 1;
     // interfaceObject->Pipes[j].MaximumTransferSize = 1024 * 1024;

//yuan modify for usb30 10/25/11
   //ntStatus = Ezusb_CallUSBD(fdo, urb);
   ntStatus = Ezusb_CallUSBD2(fdo, urb);
   DumpBuffer(urb, urb->UrbHeader.Length);

   //if (NT_SUCCESS(ntStatus) && USBD_SUCCESS(urb->UrbHeader.Status))
   {

      // Save the configuration handle for this device
      pdx->ConfigurationHandle =
         urb->UrbSelectConfiguration.ConfigurationHandle;

      pdx->Interface = ExAllocatePool(NonPagedPool,
                                                interfaceObject->Length);

      // save a copy of the interfaceObject information returned
      RtlCopyMemory(pdx->Interface, interfaceObject, interfaceObject->Length);

      // Dump the interfaceObject to the debugger
      Ezusb_KdPrint (("---------\n"));
      Ezusb_KdPrint (("NumberOfPipes 0x%x\n", pdx->Interface->NumberOfPipes));
      Ezusb_KdPrint (("Length 0x%x\n", pdx->Interface->Length));
      Ezusb_KdPrint (("Alt Setting 0x%x\n", pdx->Interface->AlternateSetting));
      Ezusb_KdPrint (("Interface Number 0x%x\n", pdx->Interface->InterfaceNumber));

      // Dump the pipe info
      for (j=0; j<interfaceObject->NumberOfPipes; j++)
      {
         PUSBD_PIPE_INFORMATION pipeInformation;

         pipeInformation = &pdx->Interface->Pipes[j];

         Ezusb_KdPrint (("---------\n"));
         Ezusb_KdPrint (("PipeType 0x%x\n", pipeInformation->PipeType));
         Ezusb_KdPrint (("EndpointAddress 0x%x\n", pipeInformation->EndpointAddress));
         Ezusb_KdPrint (("MaxPacketSize 0x%x\n", pipeInformation->MaximumPacketSize));
         Ezusb_KdPrint (("Interval 0x%x\n", pipeInformation->Interval));
         Ezusb_KdPrint (("Handle 0x%x\n", pipeInformation->PipeHandle));
         Ezusb_KdPrint (("MaximumTransferSize 0x%x\n", pipeInformation->MaximumTransferSize));
      }

      Ezusb_KdPrint (("---------\n"));
   }


   Ezusb_KdPrint (("exit Ezusb_SelectInterfaces (%x)\n", ntStatus));

   return ntStatus;

}/* Ezusb_SelectInterfaces */


NTSTATUS
Ezusb_ReadWrite(
   IN  PDEVICE_OBJECT fdo,
   IN  PBULK_TRANSFER_CONTROL bulkControl,
   IN  PCHAR bulkbuffer,   
   IN  ULONG bufferLength,
   OUT PULONG actualLength,
   OUT BOOLEAN *end
   )
/*++
Routine Description:
    
Arguments:

Return Value:
    NT status code
        STATUS_SUCCESS:                 Read was done successfully
        STATUS_INVALID_PARAMETER_3:     The Endpoint Index does not specify an IN pipe 
        STATUS_NO_MEMORY:               Insufficient data memory was supplied to perform the READ

    This routine fills the status code into the Irp
    
--*/
{
   PPCI_DEVEXT          pdx = fdo->DeviceExtension;
   NTSTATUS                   ntStatus;
   //PIO_STACK_LOCATION         irpStack = IoGetCurrentIrpStackLocation (Irp);
   //PBULK_TRANSFER_CONTROL     bulkControl =
     //                         (PBULK_TRANSFER_CONTROL)Irp->AssociatedIrp.SystemBuffer;
   //ULONG                      bufferLength =
     //                         irpStack->Parameters.DeviceIoControl.OutputBufferLength;
   PURB                       urb = NULL;
   ULONG                      urbSize = 0, totalbuflen=bufferLength;
   ULONG                      transferFlags = 0;
   PUSBD_INTERFACE_INFORMATION interfaceInfo = NULL;
   PUSBD_PIPE_INFORMATION     pipeInfo = NULL;
   USBD_PIPE_HANDLE           pipeHandle = NULL;
   PCHAR tmpbuffer;

   Ezusb_KdPrint(("enter Ezusb_Read_Write()\n"));
   
   //
   // verify that the selected pipe is valid, and get a handle to it. If anything
   // is wrong, return an error
   //
   interfaceInfo = pdx->Interface;

   if (!interfaceInfo)
   {
      Ezusb_KdPrint(("Ezusb_Read_Write() no interface info - Exiting\n"));
      return STATUS_UNSUCCESSFUL;
   }
   
   if (bulkControl->pipeNum > interfaceInfo->NumberOfPipes)
   {
      Ezusb_KdPrint(("Ezusb_Read_Write() invalid pipe - Exiting\n"));
      return STATUS_INVALID_PARAMETER;
   }

   pipeInfo = &(interfaceInfo->Pipes[bulkControl->pipeNum]);

   if (!((pipeInfo->PipeType == UsbdPipeTypeBulk) ||
         (pipeInfo->PipeType == UsbdPipeTypeInterrupt)))
   {
      Ezusb_KdPrint(("Ezusb_Read_Write() invalid pipe - Exiting\n"));
      return STATUS_INVALID_PARAMETER;
   }

   pipeHandle = pipeInfo->PipeHandle;

   if (!pipeHandle)
   {
      Ezusb_KdPrint(("Ezusb_Read_Write() invalid pipe - Exiting\n"));
      return STATUS_UNSUCCESSFUL;
   }

   if (bufferLength > pipeInfo->MaximumTransferSize)
   {
      //...Ezusb_KdPrint(("Ezusb_Read_Write() invalid transfer size - Exiting\n"));
      //...return STATUS_INVALID_PARAMETER;
      bufferLength = pipeInfo->MaximumTransferSize;
   }

   //
   // allocate and fill in the Usb request (URB)
   //
   urbSize = sizeof(struct _URB_BULK_OR_INTERRUPT_TRANSFER);

   urb = ExAllocatePool(NonPagedPool,urbSize);

   if (!urb)
   {
      Ezusb_KdPrint(("Ezusb_Read_Write() unable to alloc URB - Exiting\n"));
      return STATUS_NO_MEMORY;
   }
   

   transferFlags = USBD_SHORT_TRANSFER_OK;

   //
   // get direction info from the endpoint address
   //
   if (USB_ENDPOINT_DIRECTION_IN(pipeInfo->EndpointAddress))
      transferFlags |= USBD_TRANSFER_DIRECTION_IN;
   tmpbuffer = bulkbuffer;
   *actualLength = 0;
   do {
      UsbBuildInterruptOrBulkTransferRequest(urb,        //ptr to urb
                        		(USHORT) urbSize,             //size of urb
					pipeHandle,                   //usbd pipe handle
					tmpbuffer, //bulkbuffer,//NULL,//TransferBuffer
					NULL,//Irp->MdlAddress,              //mdl
					bufferLength,                 //bufferlength
					transferFlags,                //flags
					NULL);                        //link

   //
   // Call the USB Stack.
   //
	ntStatus = Ezusb_CallUSBD(fdo, urb);

   //
   // If the transfer was successful, report the length of the transfer to the
   // caller by setting IoStatus.Information
   //
   if (NT_SUCCESS(ntStatus))
   {
   	*actualLength += urb->UrbBulkOrInterruptTransfer.TransferBufferLength;
   	if (USB_ENDPOINT_DIRECTION_IN(pipeInfo->EndpointAddress)) {//In only once to check if IN is done
   		if(urb->UrbBulkOrInterruptTransfer.TransferBufferLength < bufferLength)//totalbuflen)//bufferLength) 
   			*end = 1;
   		break;
   	}
   	totalbuflen -= bufferLength;
   	tmpbuffer += bufferLength;
   	bufferLength = (totalbuflen>=pipeInfo->MaximumTransferSize)? pipeInfo->MaximumTransferSize:totalbuflen;
   	
      //....Irp->IoStatus.Information = urb->UrbBulkOrInterruptTransfer.TransferBufferLength;
      //....Ezusb_KdPrint(("Successfully transfered 0x%x bytes\n",Irp->IoStatus.Information));
   } else {
      *actualLength = 0;
      break;
   }
   } while (totalbuflen);
   //
   // free the URB
   //
   ExFreePool(urb);

   return ntStatus;
}

ULONG
Ezusb_GetDeviceDescriptor(
    IN PDEVICE_OBJECT fdo,
    PVOID             pvOutputBuffer
    )
/*++
Routine Description:
    Gets a device descriptor from the given device object
    
Arguments:
    fdo - pointer to the sample device object

Return Value:
    Number of valid bytes in data buffer  

--*/
{
   NTSTATUS            ntStatus        = STATUS_SUCCESS;
   PURB                urb             = NULL;
   ULONG               length          = 0;
   PPCI_DEVEXT   pdx = NULL;

   Ezusb_KdPrint (("Enter Ezusb_GetDeviceDescriptor\n"));    

   pdx = fdo->DeviceExtension;

   urb = ExAllocatePool(NonPagedPool, 
                      sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));
                   
   if (urb)
   {
      if (pvOutputBuffer)
      {    
         UsbBuildGetDescriptorRequest(urb,
                                      (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                      USB_DEVICE_DESCRIPTOR_TYPE,    //descriptor type
                                      0,                             //index
                                      0,                             //language ID
                                      pvOutputBuffer,                //transfer buffer
                                      NULL,                          //MDL
                                      sizeof(USB_DEVICE_DESCRIPTOR), //buffer length
                                      NULL);                         //link
           
         //modify 12/29/11                                                         
         //ntStatus = Ezusb_CallUSBD(fdo, urb);
         ntStatus = Ezusb_CallUSBD2(fdo, urb);

      }
      else
      {
         ntStatus = STATUS_INVALID_PARAMETER;
      }    

      // If successful, get the length from the Urb, otherwise set length to 0
      if (NT_SUCCESS(ntStatus))
      {
         length = urb->UrbControlDescriptorRequest.TransferBufferLength;
      }
      else
         length = 0;

      Ezusb_KdPrint (("%d bytes of dev descriptor received\n",length));

      ExFreePool(urb);
        
   }
   else
   {
      ntStatus = STATUS_NO_MEMORY;        
   }        
    
   Ezusb_KdPrint (("Leaving Ezusb_GetDeviceDescriptor\n"));    

   return length;
}    


ULONG
Ezusb_GetConfigDescriptor(
    IN PDEVICE_OBJECT fdo,
    PVOID             pvOutputBuffer,
    ULONG             ulLength
    )
/*++

Routine Description:
    Gets a configuration descriptor from the given device object
    
Arguments:
    fdo    - pointer to the sample device object
    pvOutputBuffer  - pointer to the buffer where the data is to be placed
    ulLength        - length of the buffer

Return Value:
    Number of valid bytes in data buffer  

--*/
{
   NTSTATUS            ntStatus        = STATUS_SUCCESS;
   PURB                urb             = NULL;
   ULONG               length          = 0;

   Ezusb_KdPrint (("Enter Ezusb_GetConfigDescriptor\n"));    

   urb = ExAllocatePool(NonPagedPool, 
                      sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));
                         
   if (urb)
   {
      if (pvOutputBuffer)
      {    

         UsbBuildGetDescriptorRequest(urb,
                                      (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                      USB_CONFIGURATION_DESCRIPTOR_TYPE, //descriptor type
                                      0,                             //index
                                      0,                             //language ID
                                      pvOutputBuffer,                //transfer buffer
                                      NULL,                          //MDL
                                      ulLength,                      //buffer length
                                      NULL);                         //link

	 //modify 12/29/11
	 ntStatus = Ezusb_CallUSBD2(fdo, urb);                                                                  
         //ntStatus = Ezusb_CallUSBD(fdo, urb);

      }
      else
      {
         ntStatus = STATUS_INVALID_PARAMETER;
      }    

      // If successful, get the length from the Urb, otherwise set length to 0
      if (NT_SUCCESS(ntStatus))
      {
         length = urb->UrbControlDescriptorRequest.TransferBufferLength;
      }
      else
         length = 0;

      Ezusb_KdPrint (("%d bytes of cfg descriptor received\n",length));

      ExFreePool(urb);
        
   }
   else
   {
      ntStatus = STATUS_NO_MEMORY;        
   }        
    
   Ezusb_KdPrint (("Leaving Ezusb_GetConfigDescriptor\n"));    

   return length;
}    

ULONG
Ezusb_GetStringDescriptor(
    IN PDEVICE_OBJECT fdo,
    UCHAR             Index,
    USHORT            LanguageId,
    PVOID             pvOutputBuffer,
    ULONG             ulLength
    )
/*++

Routine Description:
    Gets the specified string descriptor from the given device object
    
Arguments:
    fdo    - pointer to the sample device object
    Index           - Index of the string descriptor
    LanguageId      - Language ID of the string descriptor
    pvOutputBuffer  - pointer to the buffer where the data is to be placed
    ulLength        - length of the buffer

Return Value:
    Number of valid bytes in data buffer  

--*/
{
   NTSTATUS            ntStatus        = STATUS_SUCCESS;
   PURB                urb             = NULL;
   ULONG               length          = 0;

   Ezusb_KdPrint (("Enter Ezusb_GetStringDescriptor\n"));    

   urb = ExAllocatePool(NonPagedPool, 
                      sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));
                         
   if (urb)
   {
      if (pvOutputBuffer)
      {    

         UsbBuildGetDescriptorRequest(urb,
                                      (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                      USB_STRING_DESCRIPTOR_TYPE,    //descriptor type
                                      Index,                         //index
                                      LanguageId,                    //language ID
                                      pvOutputBuffer,                //transfer buffer
                                      NULL,                          //MDL
                                      ulLength,                      //buffer length
                                      NULL);                         //link
                                                                  
         ntStatus = Ezusb_CallUSBD(fdo, urb);

      }
      else
      {
         ntStatus = STATUS_INVALID_PARAMETER;
      }    

      // If successful, get the length from the Urb, otherwise set length to 0
      if (NT_SUCCESS(ntStatus))
      {
         length = urb->UrbControlDescriptorRequest.TransferBufferLength;
      }
      else
         length = 0;

      Ezusb_KdPrint (("%d bytes of string descriptor received\n",length));

      ExFreePool(urb);
        
   }
   else
   {
      ntStatus = STATUS_NO_MEMORY;        
   }        
    
   Ezusb_KdPrint (("Leaving Ezusb_GetStringDescriptor\n"));    

   return length;
}    

PUSB_CONFIGURATION_DESCRIPTOR
GetConfigDescriptor(
    IN PDEVICE_OBJECT fdo
    )
/*++

Routine Description:

Arguments:



Return Value:

    NT status code

--*/
{
    PPCI_DEVEXT pdx;
    NTSTATUS ntStatus;
    PURB urb;
    ULONG siz;
    PUSB_CONFIGURATION_DESCRIPTOR configurationDescriptor = NULL;

    Ezusb_KdPrint (("Ezusb.SYS: enter Ezusb_GetConfigDescriptor\n"));

    pdx = fdo->DeviceExtension;

    urb = ExAllocatePool(NonPagedPool,
                         sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));

    if (urb) {


        siz = sizeof(USB_CONFIGURATION_DESCRIPTOR)+256;

get_config_descriptor_retry2:

        configurationDescriptor = ExAllocatePool(NonPagedPool,
                                                 siz);

        if (configurationDescriptor) {

            UsbBuildGetDescriptorRequest(urb,
                                         (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                         USB_CONFIGURATION_DESCRIPTOR_TYPE,
                                         0,
                                         0,
                                         configurationDescriptor,
                                         NULL,
                                         siz,
                                         NULL);

	    //modify 12/29/11
	    ntStatus = Ezusb_CallUSBD2(fdo, urb);
            //ntStatus = Ezusb_CallUSBD(fdo, urb);

            Ezusb_KdPrint (("Ezusb.SYS: Configuration Descriptor = %x, len %x\n",
                            configurationDescriptor,
                            urb->UrbControlDescriptorRequest.TransferBufferLength));
        } else {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
        }

        if (NT_SUCCESS(ntStatus) &&
            (urb->UrbControlDescriptorRequest.TransferBufferLength >= 
             sizeof(USB_CONFIGURATION_DESCRIPTOR)) &&
            (configurationDescriptor->wTotalLength >=
             sizeof(USB_CONFIGURATION_DESCRIPTOR)))
        {
            //
            // The Get Config Descriptor request did not return an error
            // AND at least enough data was transferred to fill a Config
            // Descriptor AND the Config Descriptor wLength is at least the
            // size of a Config Descriptor
            //
            if (configurationDescriptor->wTotalLength > siz)
            {
                //
                // The request buffer is not big enough to hold the
                // entire set of descriptors.  Free the current buffer
                // and retry with a buffer which should be big enough.
                //
                siz = configurationDescriptor->wTotalLength;
                ExFreePool(configurationDescriptor);
                configurationDescriptor = NULL;
                goto get_config_descriptor_retry2;
            }
            else if (configurationDescriptor->wTotalLength >
                     urb->UrbControlDescriptorRequest.TransferBufferLength)
            {
                //
                // The request buffer is greater than or equal to the
                // Config Descriptor wLength, but less data was transferred
                // than wLength.  Return NULL to indicate a device error.
                //
                ExFreePool(configurationDescriptor);
                configurationDescriptor = NULL;
            }
            //
            // else  everything is OK with the Config Descriptor, return it.
            //
        }
        else
        {
            //
            // The Get Config Descriptor request returned an error OR
            // not enough data was transferred to fill a Config Descriptor
            // OR the Config Descriptor wLength is less than the size of
            // a Config Descriptor.  Return NULL to indicate a device error.
            //
            ExFreePool(configurationDescriptor);
            configurationDescriptor = NULL;
        }

        ExFreePool(urb);

    } else {
        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
    }

    Ezusb_KdPrint (("Ezusb.SYS: exit Ezusb_GetConfigDescriptor\n"));

    return configurationDescriptor;
}

NTSTATUS
ConfigureDevice(
    IN  PDEVICE_OBJECT fdo
    )
/*++
Routine Description:
    Configures the USB device via USB-specific device requests and interaction
    with the USB software subsystem.

Arguments:
    fdo - pointer to the device object for this instance of the Ezusb Device

Return Value:
    NT status code
--*/
{
   PPCI_DEVEXT pdx = fdo->DeviceExtension;
   NTSTATUS ntStatus;
   PURB urb = NULL;
   ULONG numberOfPipes;
   PUSB_CONFIGURATION_DESCRIPTOR configurationDescriptor = NULL;
   PUSB_INTERFACE_DESCRIPTOR interfaceDescriptor = NULL;

   Ezusb_KdPrint (("enter ConfigureDevice\n"));

   //
   // Get the configuration Descriptor
   //
   configurationDescriptor = GetConfigDescriptor(fdo);

   if (!configurationDescriptor)
   {
      Ezusb_KdPrint (("Get Configuration Descriptor Failed\n"));
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupConfigureDevice;
   }

#define INTERFACENUMBER 0
#define ALTERNATESETTING 1
   //
   // Get the interface Descriptor for the interface we want
   //
   interfaceDescriptor = USBD_ParseConfigurationDescriptorEx(
                           configurationDescriptor,
                           configurationDescriptor,
                           INTERFACENUMBER,
                           ALTERNATESETTING,
                           -1,
                           -1,
                           -1);
   if (!interfaceDescriptor)
   {
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupConfigureDevice;
   }

   numberOfPipes = interfaceDescriptor->bNumEndpoints;
   
   //
   // Configure the Device, but don't select any interfaces
   //
   urb = USBD_CreateConfigurationRequestEx(configurationDescriptor, NULL);

   if (!urb)
   {
      Ezusb_KdPrint (("USBD_CreateConfigurationRequestEx Failed\n"));
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupConfigureDevice;
   }

//modify 12/29/11
   ntStatus = Ezusb_CallUSBD2(fdo, urb);
   //ntStatus = Ezusb_CallUSBD(fdo, urb);

   if (NT_SUCCESS(ntStatus))
   {
      pdx->ConfigurationHandle = urb->UrbSelectConfiguration.ConfigurationHandle;
   }
   else
   {
      Ezusb_KdPrint (("Configuration Request Failed\n"));
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupConfigureDevice;
   }

CleanupConfigureDevice:

   // Clean up and exit this routine
   if (urb != NULL)
   {
      ExFreePool(urb);
   }

   if (configurationDescriptor != NULL)
   {
      ExFreePool(configurationDescriptor);
   }

    Ezusb_KdPrint (("exit Ezusb_ConfigureDevice (%x)\n", ntStatus));
    return ntStatus;
}


NTSTATUS
SetInterface(
   IN PDEVICE_OBJECT fdo,
   IN UCHAR InterfaceNumber,
   IN UCHAR AlternateSetting
   )
{
   PUSBD_INTERFACE_INFORMATION interfaceInformation = NULL;
   PUSB_INTERFACE_DESCRIPTOR interfaceDescriptor = NULL;
   PUSB_CONFIGURATION_DESCRIPTOR configurationDescriptor = NULL;
   PPCI_DEVEXT pdx = fdo->DeviceExtension;
   PURB urb = NULL;
   USHORT urbSize;
   ULONG numberOfPipes;
   ULONG i,j;
   NTSTATUS ntStatus;

   //
   // Get the configuration Descriptor
   //
   configurationDescriptor = GetConfigDescriptor(fdo);
   if (!configurationDescriptor)
   {
      Ezusb_KdPrint (("Get Configuration Descriptor Failed\n"));
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupSetInterface;
   }

   //
   // Get the interface Descriptor for the interface we want
   //
   interfaceDescriptor = USBD_ParseConfigurationDescriptorEx(
                           configurationDescriptor,
                           configurationDescriptor,
                           InterfaceNumber,
                           AlternateSetting,
                           -1,
                           -1,
                           -1);

   if (!interfaceDescriptor)
   {
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupSetInterface;
   }

   numberOfPipes = interfaceDescriptor->bNumEndpoints;
   Ezusb_KdPrint (("numberOfPipes = %d\n", numberOfPipes));


   urbSize = (USHORT) GET_SELECT_INTERFACE_REQUEST_SIZE(numberOfPipes);
   Ezusb_KdPrint (("urbSize = %d\n", urbSize));
   urb = ExAllocatePool(NonPagedPool,urbSize);

   if (!urb)
   {
      ntStatus = STATUS_NO_MEMORY;
      goto CleanupSetInterface;
   }
   
   RtlZeroMemory(urb,urbSize);

   UsbBuildSelectInterfaceRequest(urb,
                                  urbSize,
                                  pdx->ConfigurationHandle,
                                  InterfaceNumber,
                                  AlternateSetting);


   interfaceInformation = &(urb->UrbSelectInterface.Interface);
   interfaceInformation->Length = (USHORT) GET_USBD_INTERFACE_SIZE(numberOfPipes);

   //for (i = 0 ;i < numberOfPipes ;i++ )
   //{
       //interfaceInformation->Pipes[i].MaximumTransferSize = (64*1024) -1;
	 //  interfaceInformation->Pipes[i].MaximumTransferSize = 1024*1024;
  // }

//modify 12/29/11
   ntStatus = Ezusb_CallUSBD2(fdo, urb);
   //ntStatus = Ezusb_CallUSBD(fdo, urb);

   //
   // If that succeeded, then update the Interface structure
   // in the device extension.
   //
   if (NT_SUCCESS(ntStatus))
   {
      for (j=0; j<interfaceInformation->NumberOfPipes; j++)
      {
         PUSBD_PIPE_INFORMATION pipeInformation;

         pipeInformation = &interfaceInformation->Pipes[j];

         Ezusb_KdPrint (("---------\n"));
         Ezusb_KdPrint (("PipeType 0x%x\n", pipeInformation->PipeType));
         Ezusb_KdPrint (("EndpointAddress 0x%x\n", pipeInformation->EndpointAddress));
         Ezusb_KdPrint (("MaxPacketSize 0x%x\n", pipeInformation->MaximumPacketSize));
         Ezusb_KdPrint (("Interval 0x%x\n", pipeInformation->Interval));
         Ezusb_KdPrint (("Handle 0x%x\n", pipeInformation->PipeHandle));
         Ezusb_KdPrint (("MaximumTransferSize 0x%x\n", pipeInformation->MaximumTransferSize));
      }

      if (pdx->Interface)
      {
         ExFreePool(pdx->Interface);
      }

      pdx->Interface = NULL;
      pdx->Interface = ExAllocatePool(NonPagedPool,interfaceInformation->Length);
      if (!pdx->Interface)
      {
         ntStatus = STATUS_NO_MEMORY;
         goto CleanupSetInterface;
      }
      RtlCopyMemory(pdx->Interface, interfaceInformation, interfaceInformation->Length);
   }

CleanupSetInterface:

   // Clean up and exit this routine
   if (urb != NULL)
   {
      ExFreePool(urb);
   }

   if (configurationDescriptor != NULL)
   {
      ExFreePool(configurationDescriptor);
   }

   return(ntStatus);
   
}   

#define BYTES_PER_LINE 0x10

void
DumpBuffer(PVOID pvBuffer, ULONG length)
{
    int                    nItems    = 0;
    char                   temp[64]  = "";
    char                   temp2[64]  = "";
	ULONG	i;
	ULONG   j;
	PUCHAR	ptr;


	ptr = (PUCHAR) pvBuffer;

	for (i = 0; i < ((length + BYTES_PER_LINE - 1) / BYTES_PER_LINE); i++)
	{
		sprintf(temp,"%04X ",(i*BYTES_PER_LINE));
		for (j = 0; j < BYTES_PER_LINE; j++)
		{
			if (((i * BYTES_PER_LINE) + j) < length)
			{
				sprintf(temp2,"%02X ",*ptr++);
				strcat(temp,temp2);
			}
		}
//	    SendMessage (hOutputBox, LB_ADDSTRING, 0, (LPARAM)temp);
         Ezusb_KdPrint (("%s\n",temp));
	}
}



///////////////////////////////////////////////////////////////////////////////
// @func Lock a SIMPLE device object
// @parm Address of our device extension
// @rdesc TRUE if it was possible to lock the device, FALSE otherwise.
// @comm A FALSE return value indicates that we're in the process of deleting
// the device object, so all new requests should be failed

BOOLEAN LockDevice(
   IN PDEVICE_OBJECT fdo
   )
{
   PPCI_DEVEXT pdx = (PPCI_DEVEXT) fdo->DeviceExtension;

   // Increment use count on our device object
   LONG usage = InterlockedIncrement(&pdx->usage);

   // AddDevice initialized the use count to 1, so it ought to be bigger than
   // one now. HandleRemoveDevice sets the "removing" flag and decrements the
   // use count, possibly to zero. So if we find a use count of "1" now, we
   // should also find the "removing" flag set.

   ASSERT(usage > 1 || pdx->removing);

   // If device is about to be removed, restore the use count and return FALSE.
   // If we're in a race with HandleRemoveDevice (maybe running on another CPU),
   // the sequence we've followed is guaranteed to avoid a mistaken deletion of
   // the device object. If we test "removing" after HandleRemoveDevice sets it,
   // we'll restore the use count and return FALSE. In the meantime, if
   // HandleRemoveDevice decremented the count to 0 before we did our increment,
   // its thread will have set the remove event. Otherwise, we'll decrement to 0
   // and set the event. Either way, HandleRemoveDevice will wake up to finish
   // removing the device, and we'll return FALSE to our caller.
   // 
   // If, on the other hand, we test "removing" before HandleRemoveDevice sets it,
   // we'll have already incremented the use count past 1 and will return TRUE.
   // Our caller will eventually call UnlockDevice, which will decrement the use
   // count and might set the event HandleRemoveDevice is waiting on at that point.

   if (pdx->removing)
	{
	   if (InterlockedDecrement(&pdx->usage) == 0)
		   KeSetEvent(&pdx->evRemove, 0, FALSE);
	   return FALSE;
	}

   return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// @func Unlock a SIMPLE device object
// @parm Address of our device extension
// @comm If the use count drops to zero, set the evRemove event because we're
// about to remove this device object.

void UnlockDevice(
   PDEVICE_OBJECT fdo
   )
{
   PPCI_DEVEXT pdx = (PPCI_DEVEXT) fdo->DeviceExtension;
   LONG usage = InterlockedDecrement(&pdx->usage);

   ASSERT(usage >= 0);

   if (usage == 0)
   {						// removing device
      ASSERT(pdx->removing);	// HandleRemoveDevice should already have set this
      KeSetEvent(&pdx->evRemove, 0, FALSE);
   }						// removing device
}

NTSTATUS
Ezusb_ResetPipe(
   IN PDEVICE_OBJECT fdo,
   ULONG PipeNum
   )
/*++

Routine Description:

   Reset a given USB pipe.
   NOTE: this will reset the data toggle on the host as well

Arguments:

Return Value:


--*/
{
   NTSTATUS ntStatus;
   //PDEVICE_EXTENSION             pdx = fdo->DeviceExtension;
   PPCI_DEVEXT pdx = (PPCI_DEVEXT) fdo->DeviceExtension;
   PUSBD_INTERFACE_INFORMATION   interfaceInfo = NULL;
   USBD_PIPE_HANDLE              pipeHandle = NULL;
   PURB urb;
   USBD_VERSION_INFORMATION VersionInformation;

   Ezusb_KdPrint (("EZUSB.SYS: Reset Pipe \n"));

   //
   // verify that the selected pipe is valid, and get a handle to it. If anything
   // is wrong, return an error
   //
   interfaceInfo = pdx->Interface;

   if (!interfaceInfo)
   {
      Ezusb_KdPrint(("Ezusb_ResetPipe() no interface info - Exiting\n"));
      return STATUS_UNSUCCESSFUL;
   }
   
   if (PipeNum > interfaceInfo->NumberOfPipes)
   {
      Ezusb_KdPrint(("Ezusb_ResetPipe() invalid pipe - Exiting\n"));
      return STATUS_INVALID_PARAMETER;
   }

   pipeHandle = interfaceInfo->Pipes[PipeNum].PipeHandle;

   urb = ExAllocatePool(NonPagedPool,
                      sizeof(struct _URB_PIPE_REQUEST));

   if (urb)
   {
      urb->UrbHeader.Length = (USHORT) sizeof (struct _URB_PIPE_REQUEST);
      urb->UrbHeader.Function = URB_FUNCTION_RESET_PIPE;
      urb->UrbPipeRequest.PipeHandle = pipeHandle;

      //
      // kludge.  Win98 changed the size of the URB_PIPE_REQUEST.
      // Check the USBDI version.  If it is prior to win98 (0x101)
      // make the structure smaller.
      // 
      USBD_GetUSBDIVersion(&VersionInformation);
      if (VersionInformation.USBDI_Version < 0x101) 
      {
         Ezusb_KdPrint(("Ezusb_ResetPipe() Detected OSR2.1\n"));
         urb->UrbHeader.Length -= sizeof(ULONG);
      }

      ntStatus = Ezusb_CallUSBD2(fdo, urb);

      ExFreePool(urb);
   }
   else
   {
      ntStatus = STATUS_INSUFFICIENT_RESOURCES;
   }

   return ntStatus;
}

NTSTATUS
Ezusb_AbortPipe(
    IN PDEVICE_OBJECT fdo,
    IN USBD_PIPE_HANDLE PipeHandle
    )
/*++

Routine Description:

   cancel pending transfers for a pipe

Arguments:

Return Value:


--*/
{
   NTSTATUS ntStatus;
   PURB urb;
   USBD_VERSION_INFORMATION VersionInformation;

   Ezusb_KdPrint (("'EZUSB.SYS: Reset Pipe \n"));

   urb = ExAllocatePool(NonPagedPool,
                      sizeof(struct _URB_PIPE_REQUEST));

   if (urb)
   {
      RtlZeroMemory(urb,sizeof(struct _URB_PIPE_REQUEST));
      urb->UrbHeader.Length = (USHORT) sizeof (struct _URB_PIPE_REQUEST);
      urb->UrbHeader.Function = URB_FUNCTION_ABORT_PIPE;
      urb->UrbPipeRequest.PipeHandle = PipeHandle;

      //
      // kludge.  Win98 changed the size of the URB_PIPE_REQUEST.
      // Check the USBDI version.  If it is prior to win98 (0x101)
      // make the structure smaller.
      // 
      USBD_GetUSBDIVersion(&VersionInformation);
      if (VersionInformation.USBDI_Version < 0x101) 
      {
         Ezusb_KdPrint(("Ezusb_ResetPipe() Detected OSR2.1\n"));
         urb->UrbHeader.Length -= sizeof(ULONG);
      }

      ntStatus = Ezusb_CallUSBD(fdo, urb);

      ExFreePool(urb);
   }
   else
   {
      ntStatus = STATUS_INSUFFICIENT_RESOURCES;
   }


   return ntStatus;
}
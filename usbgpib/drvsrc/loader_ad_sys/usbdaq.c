//////////////////////////////////////////////////////////////////////
//
// File:      ezloader.c
// $Archive: /USB/Drivers/ezloader/ezloader.c $
//
// Purpose:
//    driver for downloading firmware to pre-renumerated ezusb devices.
//
// Note:
//    derived from ezusbsys.c ver 15

// Environment:
//    kernel mode
//
// $Author: Mdn $
//
// $History: ezloader.c $           
//  
//  *****************  Version 2  *****************
//  User: Mdn          Date: 7/19/01    Time: 10:32a
//  Updated in $/USB/Drivers/ezloader
//  added support for FX2 - specifically, modified the 8051 reset code so
//  it will work with both EZ-USB and FX2
//  
//  *****************  Version 1  *****************
//  User: Tpm          Date: 6/09/00    Time: 6:30p
//  Created in $/USB/Drivers/ezloader
//  
//  *****************  Version 7  *****************
//  User: Markm        Date: 4/12/99    Time: 1:17p
//  Updated in $/EzUsb/Drivers/ezloader
//  
//  *****************  Version 6  *****************
//  User: Markm        Date: 4/12/99    Time: 1:16p
//  Updated in $/EzUsb/Drivers/ezloader
//  
//  *****************  Version 5  *****************
//  User: Markm        Date: 4/12/99    Time: 1:00p
//  Updated in $/EzUsb/Drivers/ezloader
//  minor changes to get rid of compiler warnings.
//  
//  *****************  Version 4  *****************
//  User: Markm        Date: 3/26/99    Time: 2:59p
//  Updated in $/EzUsb/Drivers/ezloader
//  Fixed a bug in the surprise removal code I just added.  I was returning
//  from the PnP dispatch function without unlocking the device object.
//  
//  *****************  Version 3  *****************
//  User: Markm        Date: 3/25/99    Time: 2:05p
//  Updated in $/EzUsb/Drivers/ezloader
//  Added code to allow unplugs (surprise removal) under NT5 without
//  notifying the user.
//  
//  *****************  Version 2  *****************
//  User: Markm        Date: 4/10/98    Time: 2:06p
//  Updated in $/EZUSB/ezloader
//  modified to download intel hex records and to download the loader
//  firmware for downloading to external RAM
//  
//  *****************  Version 1  *****************
//  User: Markm        Date: 2/24/98    Time: 5:26p
//  Created in $/EZUSB/ezloader
//  
//  
// Copyright (c) 1997 Anchor Chips, Inc.  May not be reproduced without
// permission.  See the license agreement for more details.
//
//////////////////////////////////////////////////////////////////////

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

NTSTATUS
DriverEntry(
    IN PDRIVER_OBJECT DriverObject,
    IN PUNICODE_STRING RegistryPath
    )
/*++

Routine Description:
   
Arguments:
   DriverObject - pointer to the driver object
   RegistryPath - pointer to a unicode string representing the path
                  to driver-specific key in the registry

Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise

--*/
{
   NTSTATUS ntStatus = STATUS_SUCCESS;
   PDEVICE_OBJECT deviceObject = NULL;

   Ezusb_KdPrint (("entering (Ezusb) DriverEntry (Build: %s/%s\n",__DATE__,__TIME__));

   DriverObject->DriverUnload = Ezusb_Unload;

   //
   // POWER and PNP IRPs go to the same dispatch function.  Under
   // Win95, there is just a single IRP for both, called
   // IRP_MJ_PNP_POWER.  This is assigned the same value as
   // IRP_MJ_PNP has under Win98 and NT5.  I'm only concerned
   // with basic PNP stuff, like START and REMOVE.  All other
   // PNP and POWER IRPs will simply be passed down the driver
   // stack.  This driver won't be around like enough to worry
   // about POWER IRPs.  That is, as soon as code is downloaded
   // to the device, the device will remove itself and the driver
   // will go away.
   //
   DriverObject->MajorFunction[IRP_MJ_PNP] =
   DriverObject->MajorFunction[IRP_MJ_POWER] = Ezusb_DispatchPnp;

   DriverObject->DriverExtension->AddDevice = Ezusb_PnPAddDevice;

   Ezusb_KdPrint (("exiting (Ezusb) DriverEntry (%x)\n", ntStatus));

   return ntStatus;
}

NTSTATUS
Ezusb_DefaultPnpHandler(
   IN PDEVICE_OBJECT fdo,
   IN PIRP Irp
   )
{
   PDEVICE_EXTENSION pdx = (PDEVICE_EXTENSION) fdo->DeviceExtension;

   IoSkipCurrentIrpStackLocation(Irp);
   return IoCallDriver(pdx->StackDeviceObject, Irp);
}

///////////////////////////////////////////////////////////////////////////////
// @func Handle completion of a request by a lower-level driver
// @parm Functional device object
// @parm I/O request which has completed
// @parm Context argument supplied to IoSetCompletionRoutine, namely address of
// KEVENT object on which ForwardAndWait is waiting
// @comm This is the completion routine used for requests forwarded by ForwardAndWait. It
// sets the event object and thereby awakens ForwardAndWait.
// @comm Note that it's *not* necessary for this particular completion routine to test
// the PendingReturned flag in the IRP and then call IoMarkIrpPending. You do that in many
// completion routines because the dispatch routine can't know soon enough that the
// lower layer has returned STATUS_PENDING. In our case, we're never going to pass a
// STATUS_PENDING back up the driver chain, so we don't need to worry about this.

NTSTATUS 
OnRequestComplete(
   IN PDEVICE_OBJECT fdo,
   IN PIRP Irp,
   IN PKEVENT pev
   )
/*++

Routine Description:
   Handle completion of a request by a lower-level driver

Arguments:
   DriverObject -  Functional device object
   Irp - I/O request which has completed
   pev - Context argument supplied to IoSetCompletionRoutine, namely address of
         KEVENT object on which ForwardAndWait is waiting

Return Value:
   STATUS_MORE_PROCESSING_REQUIRED
--*/
{
   KeSetEvent(pev, 0, FALSE);
   return STATUS_MORE_PROCESSING_REQUIRED;
}

NTSTATUS
ForwardAndWait(
   IN PDEVICE_OBJECT fdo,
   IN PIRP Irp
   )
/*++
Routine Description:
   Forward request to lower level and await completion

   The only purpose of this routine in this particular driver is to pass down
   IRP_MN_START_DEVICE requests and wait for the PDO to handle them.
   
   The processor must be at PASSIVE IRQL because this function initializes
   and waits for non-zero time on a kernel event object.

Arguments:
   fdo - pointer to a device object
   Irp          - pointer to an I/O Request Packet

Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise
--*/
{
	KEVENT event;
	PDEVICE_EXTENSION pdx = (PDEVICE_EXTENSION) fdo->DeviceExtension;
	NTSTATUS ntStatus;

   ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);
	
	//
   // Initialize a kernel event object to use in waiting for the lower-level
	// driver to finish processing the object. 
   //
	KeInitializeEvent(&event, NotificationEvent, FALSE);

	IoCopyCurrentIrpStackLocationToNext(Irp);
	IoSetCompletionRoutine(Irp, (PIO_COMPLETION_ROUTINE) OnRequestComplete,
		(PVOID) &event, TRUE, TRUE, TRUE);

	ntStatus = IoCallDriver(pdx->StackDeviceObject, Irp);

	if (ntStatus == STATUS_PENDING)
	{
      KeWaitForSingleObject(&event, Executive, KernelMode, FALSE, NULL);
      ntStatus = Irp->IoStatus.Status;
   }

	return ntStatus;
}

NTSTATUS
CompleteRequest(
   IN PIRP Irp,
   IN NTSTATUS status,   
#ifdef _WIN64   
   IN ULONG_PTR info
#else
   IN ULONG info	
#endif
   )
/*++
Routine Description:
   Mark I/O request complete

Arguments:
   Irp - I/O request in question
   status - Standard status code
   info Additional information related to status code

Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise
--*/
{
	Irp->IoStatus.Status = status;
	Irp->IoStatus.Information = info;
	IoCompleteRequest(Irp, IO_NO_INCREMENT);

   return status;
}

NTSTATUS
Ezusb_DispatchPnp(
   IN PDEVICE_OBJECT fdo,
   IN PIRP           Irp
   )
/*++
Routine Description:
   Process Plug and Play IRPs sent to this device.

Arguments:
   fdo - pointer to a device object
   Irp          - pointer to an I/O Request Packet

Return Value:
   NTSTATUS
--*/
{
   PIO_STACK_LOCATION irpStack;
   PDEVICE_EXTENSION pdx = fdo->DeviceExtension;
   ULONG fcn;
   NTSTATUS ntStatus;

   Ezusb_KdPrint (("Enter Ezusb_DispatchPnp\n"));

   if (!LockDevice(fdo))
		return CompleteRequest(Irp, STATUS_DELETE_PENDING, 0);

   //
   // Get a pointer to the current location in the Irp. This is where
   //     the function codes and parameters are located.
   //
   irpStack = IoGetCurrentIrpStackLocation (Irp);

   //ASSERT(irpStack->MajorFunction == IRP_MJ_PNP);

   fcn = irpStack->MinorFunction;

   switch (fcn)
   {
      case IRP_MN_START_DEVICE:

         Ezusb_KdPrint (("IRP_MN_START_DEVICE\n"));

         ntStatus = Ezusb_HandleStartDevice(fdo,Irp);

         break; //IRP_MN_START_DEVICE

      case IRP_MN_REMOVE_DEVICE:

         Ezusb_KdPrint (("IRP_MN_REMOVE_DEVICE\n"))

         ntStatus = Ezusb_HandleRemoveDevice(fdo,Irp);

         break; //IRP_MN_REMOVE_DEVICE

      case IRP_MN_QUERY_CAPABILITIES:
      {
         //
         // This code swiped from Walter Oney.  Please buy his book!!
         //

      	PDEVICE_CAPABILITIES pdc = irpStack->Parameters.DeviceCapabilities.Capabilities;

         Ezusb_KdPrint (("IRP_MN_QUERY_CAPABILITIES\n"))

         // Check to besure we know how to handle this version of the capabilities structure

	      if (pdc->Version < 1)
         {
		      ntStatus = Ezusb_DefaultPnpHandler(fdo, Irp);
            break;
         }

         ntStatus = ForwardAndWait(fdo, Irp);
	      if (NT_SUCCESS(ntStatus))
   		{						// IRP succeeded
      		pdc = irpStack->Parameters.DeviceCapabilities.Capabilities;
            // setting this field prevents NT5 from notifying the user when the
            // device is removed.
		      pdc->SurpriseRemovalOK = TRUE;
   		}						// IRP succeeded

	      ntStatus = CompleteRequest(Irp, ntStatus, Irp->IoStatus.Information);
      }
         break; //IRP_MN_QUERY_CAPABILITIES


      //
      // All other PNP IRP's are just passed down the stack by the default handler
      //
      default:
        Ezusb_KdPrint (("Passing down unhandled PnP IOCTL MJ=0x%x MN=0x%x\n",
           irpStack->MajorFunction, irpStack->MinorFunction));
        ntStatus = Ezusb_DefaultPnpHandler(fdo, Irp);

   } // switch MinorFunction

	if (fcn != IRP_MN_REMOVE_DEVICE)
      UnlockDevice(fdo);

   Ezusb_KdPrint (("Exit Ezusb_DispatchPnp %x\n", ntStatus));
   return ntStatus;

}//Ezusb_Dispatch


VOID
Ezusb_Unload(
    IN PDRIVER_OBJECT DriverObject
    )
/*++
Routine Description:
    Free all the allocated resources, etc.
    TODO: This is a placeholder for driver writer to add code on unload

Arguments:
    DriverObject - pointer to a driver object

Return Value:
    None
--*/
{
    Ezusb_KdPrint (("enter Ezusb_Unload\n"));
    /*
    // TODO: Free any global resources allocated in DriverEntry
    */
    Ezusb_KdPrint (("exit Ezusb_Unload\n"));
}

NTSTATUS
Ezusb_HandleRemoveDevice(
   IN PDEVICE_OBJECT fdo,
   IN PIRP Irp
   )
{
   NTSTATUS ntStatus;
   PDEVICE_EXTENSION pdx = (PDEVICE_EXTENSION) fdo->DeviceExtension;
	pdx->removing = TRUE;
	UnlockDevice(fdo);			// once for LockDevice at start of dispatch
	UnlockDevice(fdo);			// once for initialization during AddDevice
	KeWaitForSingleObject(&pdx->evRemove, Executive, KernelMode, FALSE, NULL);

	// Let lower-level drivers handle this request. Ignore whatever
	// result eventuates.

//   ntStatus = Ezusb_DefaultPnpHandler(fdo, Irp);

//   Ezusb_Cleanup(fdo);

   // Remove the device object

	Ezusb_RemoveDevice(fdo);

   ntStatus = Ezusb_DefaultPnpHandler(fdo, Irp);

   return ntStatus;				// lower-level completed IoStatus already

}


NTSTATUS
Ezusb_HandleStartDevice(
   IN PDEVICE_OBJECT fdo,
   IN PIRP Irp
   )
{
   NTSTATUS ntStatus;

   //
   // First let all lower-level drivers handle this request.
   //
   ntStatus = ForwardAndWait(fdo, Irp);
	if (!NT_SUCCESS(ntStatus))
		return CompleteRequest(Irp, ntStatus, Irp->IoStatus.Information);

   //
   // now do whatever we need to do to start the device
   //
   ntStatus = Ezusb_StartDevice(fdo);

	return CompleteRequest(Irp, ntStatus, 0);
}

NTSTATUS
Ezusb_RemoveDevice(
    IN  PDEVICE_OBJECT fdo
    )
/*++

Routine Description:
    Removes a given instance of a Ezusb Device device on the USB.

Arguments:
    fdo - pointer to the device object for this instance of a Ezusb Device

Return Value:
    NT status code

--*/
{
   PDEVICE_EXTENSION pdx;
   NTSTATUS ntStatus = STATUS_SUCCESS;

   Ezusb_KdPrint (("enter Ezusb_RemoveDevice\n"));

   pdx = fdo->DeviceExtension;

   IoDetachDevice(pdx->StackDeviceObject);

   IoDeleteDevice (fdo);

   Ezusb_KdPrint (("exit Ezusb_RemoveDevice (%x)\n", ntStatus));

   return ntStatus;
}

NTSTATUS
Ezusb_PnPAddDevice(
    IN PDRIVER_OBJECT DriverObject,
    IN PDEVICE_OBJECT PhysicalDeviceObject
    )
/*++
Routine Description:
    This routine is called to create a new instance of the device

Arguments:
    DriverObject - pointer to the driver object for this instance of Ezusb
    PhysicalDeviceObject - pointer to a device object created by the bus

Return Value:
    STATUS_SUCCESS if successful,
    STATUS_UNSUCCESSFUL otherwise

--*/
{
   NTSTATUS                ntStatus = STATUS_SUCCESS;
   PDEVICE_OBJECT          fdo = NULL;
   PDEVICE_EXTENSION       pdx;

   Ezusb_KdPrint(("enter Ezusb_PnPAddDevice\n"));

   ntStatus = IoCreateDevice (DriverObject,
                              sizeof (DEVICE_EXTENSION),
                              NULL,
                              FILE_DEVICE_UNKNOWN,
                              0,
                              FALSE,
                              &fdo);

   if (NT_SUCCESS(ntStatus))
   {
      pdx = fdo->DeviceExtension;

      //
      // Non plug and play drivers usually create the device object in
      // driver entry, and the I/O manager autimatically clears this flag.
      // Since we are creating the device object ourselves in response to 
      // a PnP START_DEVICE IRP, we need to clear this flag ourselves.
      //
      fdo->Flags &= ~DO_DEVICE_INITIALIZING;

      //
      // This driver uses direct I/O for read/write requests
      //
      fdo->Flags |= DO_DIRECT_IO;
      //
      //
      // store away the Physical device Object
      //
      pdx->PhysicalDeviceObject=PhysicalDeviceObject;

      //
      // Attach to the StackDeviceObject.  This is the device object that what we 
      // use to send Irps and Urbs down the USB software stack
      //
      pdx->StackDeviceObject =
         IoAttachDeviceToDeviceStack(fdo, PhysicalDeviceObject);

      ASSERT (pdx->StackDeviceObject != NULL);

	   pdx->usage = 1;				// locked until RemoveDevice
	   KeInitializeEvent(&pdx->evRemove,
                        NotificationEvent,
                        FALSE);              // set when use count drops to zero
   }

   Ezusb_KdPrint(("exit Ezusb_PnPAddDevice (%x)\n", ntStatus));

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
   PDEVICE_EXTENSION pdx;
   PIRP irp;
   KEVENT event;
   IO_STATUS_BLOCK ioStatus;
   PIO_STACK_LOCATION nextStack;

   Ezusb_KdPrint (("enter Ezusb_CallUSBD\n"));

   pdx = fdo->DeviceExtension;

   // issue a synchronous request (see notes above)
   KeInitializeEvent(&event, NotificationEvent, FALSE);

   irp = IoBuildDeviceIoControlRequest(
             IOCTL_INTERNAL_USB_SUBMIT_URB,
             pdx->StackDeviceObject,
             NULL,
             0,
             NULL,
             0,
             TRUE, /* INTERNAL */
             &event,
             &ioStatus);

   // Prepare for calling the USB driver stack
   nextStack = IoGetNextIrpStackLocation(irp);
   ASSERT(nextStack != NULL);

   // Set up the URB ptr to pass to the USB driver stack
   nextStack->Parameters.Others.Argument1 = Urb;

   Ezusb_KdPrint (("Calling USB Driver Stack\n"));

   //
   // Call the USB class driver to perform the operation.  If the returned status
   // is PENDING, wait for the request to complete.
   //
   ntStatus = IoCallDriver(pdx->StackDeviceObject,
                         irp);
                         

   Ezusb_KdPrint (("return from IoCallDriver USBD %x\n", ntStatus));

   if (ntStatus == STATUS_PENDING)
   {
      Ezusb_KdPrint (("Wait for single object\n"));

      status = KeWaitForSingleObject(
                    &event,
                    Suspended,
                    KernelMode,
                    FALSE,
                    NULL);

      Ezusb_KdPrint (("Wait for single object, returned %x\n", status));
   }
   else
   {
      ioStatus.Status = ntStatus;
   }

   Ezusb_KdPrint (("URB status = %x status = %x irp status %x\n",
     Urb->UrbHeader.Status, status, ioStatus.Status));

   ntStatus = ioStatus.Status;


   Ezusb_KdPrint(("exit Ezusb_CallUSBD (%x)\n", ntStatus));

   return ntStatus;
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
   PDEVICE_EXTENSION pdx = (PDEVICE_EXTENSION) fdo->DeviceExtension;

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
   PDEVICE_EXTENSION pdx = (PDEVICE_EXTENSION) fdo->DeviceExtension;
   LONG usage = InterlockedDecrement(&pdx->usage);

   ASSERT(usage >= 0);

   if (usage == 0)
   {						// removing device
      ASSERT(pdx->removing);	// HandleRemoveDevice should already have set this
      KeSetEvent(&pdx->evRemove, 0, FALSE);
   }						// removing device
}

NTSTATUS
UDServiceFire(
   IN PDEVICE_OBJECT fdo,
   ULONG Request, 
   ULONG Value, 
   ULONG Index, 
   ULONG data_len, 
   PUCHAR data,
   ULONG Direction
   )
{
   PDEVICE_EXTENSION          pdx = fdo->DeviceExtension;
   NTSTATUS                   ntStatus;
   UCHAR                      _urb[sizeof(struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST)];
   PURB                       urb = (PURB)_urb;
   //PURB                       urb = NULL;
   ULONG                      urbSize;

   ULONG                      transferFlags;
   USHORT                     urbFunction = 0;


   //
   // allocate and fill in the Usb request (URB)
   //
   urbSize = sizeof(struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST);

   /*
   urb = ExAllocatePool(NonPagedPool,urbSize);

   if (!urb)
   {
      return STATUS_NO_MEMORY;
   }
   */

   RtlZeroMemory(urb,urbSize);

   // transferFlags = USBD_SHORT_TRANSFER_OK;

   //
   // the type of request (class or vendor) and the recepient
   // (device, interface, endpoint, other) combine to determine the 
   // URB function.  The following ugly code transforms fields in
   // the input param into an URB function
   //
   if (Direction == 0) {
        transferFlags = 0;
   }
   else {
        transferFlags = USBD_TRANSFER_DIRECTION_IN | USBD_SHORT_TRANSFER_OK;
   }
   urbFunction = URB_FUNCTION_VENDOR_DEVICE;
   urb->UrbHeader.Length = sizeof(struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST);
   urb->UrbHeader.Function = urbFunction;

   urb->UrbControlVendorClassRequest.TransferFlags = transferFlags;
   urb->UrbControlVendorClassRequest.TransferBufferLength = data_len;
   urb->UrbControlVendorClassRequest.TransferBuffer = data;
   urb->UrbControlVendorClassRequest.TransferBufferMDL = NULL;
   urb->UrbControlVendorClassRequest.Request = (UCHAR) Request;
   urb->UrbControlVendorClassRequest.Value = (USHORT) Value;
   urb->UrbControlVendorClassRequest.Index = (USHORT) Index;

   //
   // Call the USB Stack.
   //
    Ezusb_KdPrint(("Start transfered 0x%x bytes\n",urb->UrbControlVendorClassRequest.TransferBufferLength));
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

   /*
    ExFreePool(urb);
    */
   
   return ntStatus;
}

BOOLEAN FPGA_Download(
	IN PDEVICE_OBJECT fdo,
	PUCHAR fImage, 
	ULONG FileLen 
	)
{
   ULONG        bufferLength;
   PUCHAR       ptr = fImage;
   BOOLEAN      end_of_file;
   ULONG        rcnt;
   CHAR         data;

    UDServiceFire(fdo, VR_FPGA, 0, 0, 0, NULL, 0);
   end_of_file = FALSE;
   rcnt = 0;
   while (!end_of_file) {
	if(FileLen - rcnt <= MAX_EP0_XFER_SIZE) {
            bufferLength = FileLen - rcnt;
            end_of_file = TRUE;
	}
	else 
	   bufferLength = MAX_EP0_XFER_SIZE;

        UDServiceFire(fdo, VR_FPGA, 1, end_of_file, bufferLength, ptr, 0);

        rcnt += bufferLength;
        if(!end_of_file)
            (PCHAR) ptr += bufferLength;
    }
  
    return TRUE;
}

BOOLEAN Firmware_Download_Int(
	IN PDEVICE_OBJECT fdo,
	PUCHAR fImage, 
	ULONG FileLen 
	)
{
   ULONG        bufferLength;
   PUCHAR       ptr = fImage;
   BOOLEAN      end_of_file;
   ULONG        rcnt;
   CHAR         data;

    Ezusb_8051Reset(fdo,1);
   end_of_file = FALSE;
   rcnt = 0;
   while (!end_of_file) {
	if(FileLen - rcnt <= MAX_EP0_XFER_SIZE) {
            bufferLength = FileLen - rcnt;
            end_of_file = TRUE;
	}
	else
	   bufferLength = MAX_EP0_XFER_SIZE;

        UDServiceFire(fdo, ANCHOR_LOAD_INTERNAL, rcnt, 0, bufferLength, ptr, 0);

        rcnt += bufferLength;
        if(!end_of_file)
            (PCHAR) ptr += bufferLength;
    }
    Ezusb_8051Reset(fdo,0);
  
    return TRUE;
}

BOOLEAN Firmware_Download_Ext(
	IN PDEVICE_OBJECT fdo,
	PUCHAR fImage, 
	ULONG FileLen 
	)
{
   ULONG        bufferLength;
   PUCHAR       ptr = fImage;
   BOOLEAN      end_of_file;
   ULONG        rcnt;

   end_of_file = FALSE;
   rcnt = 0;
   while (!end_of_file) {
	if(FileLen - rcnt <= MAX_EP0_XFER_SIZE) {
            bufferLength = FileLen - rcnt;
            end_of_file = TRUE;
	}
	else 
	   bufferLength = MAX_EP0_XFER_SIZE;

        UDServiceFire(fdo, VR_RAM, rcnt+0x4000, 0, bufferLength, ptr, 0);

        rcnt += bufferLength;
        if(!end_of_file)
            (PCHAR) ptr += bufferLength;
    }
  
    return TRUE;
}

BOOLEAN Firmware_Download_EEPROM(
	IN PDEVICE_OBJECT fdo,
	PUCHAR fImage, 
	ULONG FileLen 
	)
{
   ULONG        bufferLength;
   PUCHAR       ptr = fImage;
   BOOLEAN      end_of_file;
   ULONG        rcnt;

   end_of_file = FALSE;
   rcnt = 0;
   while (!end_of_file) {
	if(FileLen - rcnt <= MAX_EP0_XFER_SIZE) {
            bufferLength = FileLen - rcnt;
            end_of_file = TRUE;
	}
	else 
	   bufferLength = MAX_EP0_XFER_SIZE;

        UDServiceFire(fdo, VR_EEPROM, rcnt, 0, bufferLength, ptr, 0);

        rcnt += bufferLength;
        if(!end_of_file)
            (PCHAR) ptr += bufferLength;
    }
  
    return TRUE;
}

NTSTATUS
Download_BulkOut(
   IN PDEVICE_OBJECT fdo,
   ULONG data_len, 
   PUCHAR data,
   ULONG EndpointAddress
   )
{
   PDEVICE_EXTENSION          pdx = fdo->DeviceExtension;
   NTSTATUS                   ntStatus;
   UCHAR                      _urb[sizeof(struct _URB_BULK_OR_INTERRUPT_TRANSFER)];
   PURB                       urb = (PURB)_urb;
   //PURB                       urb = NULL;
   ULONG                      urbSize;

   ULONG                      transferFlags = 0;

   USBD_PIPE_HANDLE           BulkOut_Pipe_Handle;

   //
   // allocate and fill in the Usb request (URB)
   //
   urbSize = sizeof(struct _URB_BULK_OR_INTERRUPT_TRANSFER);

   /*
   urb = ExAllocatePool(NonPagedPool,urbSize);

   if (!urb)
   {
      return STATUS_NO_MEMORY;
   }
   */

   RtlZeroMemory(urb,urbSize);

   // transferFlags = USBD_SHORT_TRANSFER_OK;
   
   if (EndpointAddress == 0x2) {
        BulkOut_Pipe_Handle = pdx->BulkOut_Pipe_Handle;
   }
   else {
        BulkOut_Pipe_Handle = pdx->BulkOut_Pipe_Handle2;
   }

    UsbBuildInterruptOrBulkTransferRequest(
                            urb,
                            sizeof(struct _URB_BULK_OR_INTERRUPT_TRANSFER),
                            BulkOut_Pipe_Handle,
                            data,
                            NULL,
                            data_len,
                            transferFlags,
                            NULL);
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
      Ezusb_KdPrint(("Successfully transfered 0x%x bytes\n",urb->UrbBulkOrInterruptTransfer.TransferBufferLength));
   }

   /*
    ExFreePool(urb);
    */
   
   return ntStatus;
}

BOOLEAN Firmware_Download_Ext_BulkOut(
	IN PDEVICE_OBJECT fdo,
	PUCHAR fImage, 
	ULONG FileLen 
	)
{
   ULONG        bufferLength;
   PUCHAR       ptr = fImage;
   BOOLEAN      end_of_file;
   ULONG        rcnt;

   end_of_file = FALSE;
   rcnt = FileLen;
   while (!end_of_file) {
	if(rcnt <= MAX_BULK_XFER_SIZE) {
            bufferLength = rcnt;
            end_of_file = TRUE;
	}
	else 
	   bufferLength = MAX_BULK_XFER_SIZE;

        Download_BulkOut(fdo, bufferLength, ptr, 2);

        rcnt -= bufferLength;
        if(!end_of_file)
            (PCHAR) ptr += bufferLength;
    }
  
    return TRUE;
}

BOOLEAN FPGA_Download_BulkOut(
	IN PDEVICE_OBJECT fdo,
	PUCHAR fImage, 
	ULONG FileLen 
	)
{
   ULONG        bufferLength;
   PUCHAR       ptr = fImage;
   BOOLEAN      end_of_file;
   ULONG        rcnt;
   CHAR         data;

    UDServiceFire(fdo, VR_FPGA, 0, 0, 0, NULL, 0);
   end_of_file = FALSE;
   rcnt = 0;
   while (!end_of_file) {
	if(FileLen - rcnt <= MAX_BULK_XFER_SIZE) {
            bufferLength = FileLen - rcnt;
            end_of_file = TRUE;
	}
	else 
	   bufferLength = MAX_BULK_XFER_SIZE;

        Download_BulkOut(fdo, bufferLength, ptr, 6);

        rcnt += bufferLength;
        if(!end_of_file)
            (PCHAR) ptr += bufferLength;
    }
  
    return TRUE;
}

NTSTATUS Ezusb_8051Reset(
   PDEVICE_OBJECT fdo,
   UCHAR resetBit
   )
/*++

Routine Description:
   Uses the ANCHOR LOAD vendor specific command to either set or release the
   8051 reset bit in the EZ-USB chip.

Arguments:
   fdo - pointer to the device object for this instance of an Ezusb Device
   resetBit - 1 sets the 8051 reset bit (holds the 8051 in reset)
              0 clears the 8051 reset bit (8051 starts running)
              
Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise

--*/
{
    return UDServiceFire(fdo, ANCHOR_LOAD_INTERNAL, CPUCS_REG_FX2, 0, 1, &resetBit, 0);
}

NTSTATUS Ezusb_Reboot(
   PDEVICE_OBJECT fdo,
   USHORT address
   )
/*++

Routine Description:
   Uses the vendor specific command to jump to specific address for reboot.

Arguments:
   fdo - pointer to the device object for this instance of an Ezusb Device
   address - jump address
              
Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise

--*/
{
    return UDServiceFire(fdo, VR_LOADER_REBOOT, address, 0, 0, NULL, 0);
}

PCWSTR FileName;

NTSTATUS Ezusb_DownloadFirmware(
   PDEVICE_OBJECT fdo,
   ULONG op
   )
/*++

Routine Description:
    This routine opens a file for future mapping and reads its contents
    into allocated memory.
Arguments:
    fileName - The name of the file
Return Value:
    STATUS_INSUFFICIENT_RESOURCES if not enough memory for filename
         or fileimage buffer, or
    STATUS_NO_SUCH_FILE if the file cannot be opened,
    STATUS_UNSUCCESSFUL if the length of the read file is 1, or if the
         file cannot be read.                        
    STATUS_SUCCESS otherwise.

--*/
{
   PDEVICE_EXTENSION  pdx = fdo->DeviceExtension;
   NTSTATUS ntStatus;
   IO_STATUS_BLOCK IoStatus;
   HANDLE NtFileHandle;
   HANDLE hfile;
   OBJECT_ATTRIBUTES ObjectAttributes;
   ULONG LengthOfFile;
   //UNICODE_STRING usname;
   UNICODE_STRING FullFileName;
   ULONG FullFileNameLength;
   PVOID FileImage;
   ULONG numread;

   FILE_STANDARD_INFORMATION StandardInfo;
   //
   // Insert the correct path prefix.
   //
   RtlInitUnicodeString(&FullFileName, FileName);
   InitializeObjectAttributes ( &ObjectAttributes,
                                &FullFileName,
                                OBJ_CASE_INSENSITIVE/*|OBJ_KERNEL_HANDLE*/,
                                NULL,
                                NULL );
   ntStatus = ZwCreateFile( &NtFileHandle,
                            SYNCHRONIZE | FILE_READ_DATA,
                            &ObjectAttributes,
                            &IoStatus,
                            NULL,                          // alloc size = none
                            FILE_ATTRIBUTE_NORMAL,
                            FILE_SHARE_READ,
                            FILE_OPEN,
                            FILE_SYNCHRONOUS_IO_NONALERT,
                            NULL,  // eabuffer
                            0 );   // ealength
    if ( !NT_SUCCESS( ntStatus ) )
    {
        //ExFreePool(FullFileName.Buffer);
        ntStatus = STATUS_NO_SUCH_FILE;
        return ntStatus;
    }
    //ExFreePool(FullFileName.Buffer);
    //
    // Query the object to determine its length.
    //
    ntStatus = ZwQueryInformationFile( NtFileHandle,
                                      &IoStatus,
                                      &StandardInfo,
                                      sizeof(FILE_STANDARD_INFORMATION),
                                      FileStandardInformation );
    if (!NT_SUCCESS(ntStatus)) {
        ZwClose( NtFileHandle );
        return ntStatus;
    }
    LengthOfFile = StandardInfo.EndOfFile.LowPart;
    //
    // Might be corrupted.
    //
    if( LengthOfFile < 1 ) {
        ZwClose( NtFileHandle );
        ntStatus = STATUS_UNSUCCESSFUL;
        return ntStatus;
    }
    //
    // Allocate buffer for this file
    //
    FileImage = ExAllocatePool( NonPagedPool,
                             LengthOfFile );

    if( FileImage == NULL ) {
        ZwClose( NtFileHandle );
        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
        return ntStatus;
    }
    //
    // Read the file into our buffer.
    //
    ntStatus = ZwReadFile( NtFileHandle,
                          NULL,
                          NULL,
                          NULL,
                          &IoStatus,
                          FileImage,
                          LengthOfFile,
                          NULL,
                          NULL );

    if( (!NT_SUCCESS(ntStatus)) || (IoStatus.Information != LengthOfFile) )
    {
        ntStatus = STATUS_UNSUCCESSFUL;
        ExFreePool( FileImage );
        return ntStatus;
    }

    ZwClose( NtFileHandle );
    switch(op) {
        case DLFW_OP_INT:
            if(!(Firmware_Download_Int(fdo, (PUCHAR) FileImage, LengthOfFile)))
                ntStatus = STATUS_UNSUCCESSFUL;
            break;
        case DLFW_OP_EXT:
            if(!(Firmware_Download_Ext(fdo, (PUCHAR) FileImage, LengthOfFile)))
                ntStatus = STATUS_UNSUCCESSFUL;
            break;
        case DLFW_OP_FPGA:
            if(!(FPGA_Download(fdo, (PUCHAR) FileImage, LengthOfFile)))
                ntStatus = STATUS_UNSUCCESSFUL;
            break;
    }
    if (FileImage)
        ExFreePool( FileImage );

    return STATUS_SUCCESS;
}

NTSTATUS
GetDeviceDescriptor(
   IN PDEVICE_OBJECT fdo
   )
{
    PDEVICE_EXTENSION pdx;
    NTSTATUS ntStatus;
    PUSB_DEVICE_DESCRIPTOR deviceDescriptor = NULL;
    UCHAR _urb[sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST)];
    PURB urb = (PURB)_urb;
   //PURB urb = NULL;
    ULONG siz;

    Ezusb_KdPrint (("enter GetDeviceDescriptor\n"));

    pdx = fdo->DeviceExtension;

    /*
    // Get some memory from then non paged pool (fixed, locked system memory)
    // for use by the USB Request Block (urb) for the specific USB Request we
    // will be performing below (a USB device request).
    */
    /*
    urb = ExAllocatePool( NonPagedPool,
                          sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));

    if (urb) {
    */

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

            // Get the device descriptor
            ntStatus = Ezusb_CallUSBD(fdo, urb);

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
        } else if (deviceDescriptor) {
            /*
            // If the bus transaction failed, then free up the memory created to hold
            // the device descriptor, since the device is probably non-functional
            */
            ExFreePool(deviceDescriptor);
            pdx->DeviceDescriptor = NULL;
        }

    /*
        ExFreePool(urb);

    } else {
        // Failed getting memory for the Urb 
        ntStatus = STATUS_NO_MEMORY;
    }
    */

    Ezusb_KdPrint (("exit GetDeviceDescriptor (%x)\n", ntStatus));

    return ntStatus;
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
    PDEVICE_EXTENSION pdx;
    NTSTATUS ntStatus;
    UCHAR _urb[sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST)];
    PURB urb = (PURB)_urb;
   //PURB urb = NULL;
    ULONG siz;
    USB_CONFIGURATION_DESCRIPTOR UCD;
    PUSB_CONFIGURATION_DESCRIPTOR configurationDescriptor = &UCD;
    //PUSB_CONFIGURATION_DESCRIPTOR configurationDescriptor = NULL;

    Ezusb_KdPrint (("Ezusb.SYS: enter Ezusb_GetConfigDescriptor\n"));

    pdx = fdo->DeviceExtension;

    /*
    urb = ExAllocatePool(NonPagedPool,
                         sizeof(struct _URB_CONTROL_DESCRIPTOR_REQUEST));

    if (urb) {
    */


        siz = sizeof(USB_CONFIGURATION_DESCRIPTOR);

        /*
        configurationDescriptor = ExAllocatePool(NonPagedPool,
                                                 siz);
                                                 */
        /*
        if (configurationDescriptor) {
        */

            UsbBuildGetDescriptorRequest(urb,
                                         (USHORT) sizeof (struct _URB_CONTROL_DESCRIPTOR_REQUEST),
                                         USB_CONFIGURATION_DESCRIPTOR_TYPE,
                                         0,
                                         0,
                                         configurationDescriptor,
                                         NULL,
                                         siz,
                                         NULL);

            ntStatus = Ezusb_CallUSBD(fdo, urb);

            Ezusb_KdPrint (("Ezusb.SYS: Configuration Descriptor = %x, len %x\n",
                            configurationDescriptor,
                            urb->UrbControlDescriptorRequest.TransferBufferLength));
        /*
        } else {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
        }
        ExFreePool(configurationDescriptor);
        */
        siz = configurationDescriptor->wTotalLength;

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

            ntStatus = Ezusb_CallUSBD(fdo, urb);

            Ezusb_KdPrint (("Ezusb.SYS: Configuration Descriptor = %x, len %x\n",
                            configurationDescriptor,
                            urb->UrbControlDescriptorRequest.TransferBufferLength));
        } else {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
        }
    /*
        ExFreePool(urb);

    } else {
        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
    }
    */

    Ezusb_KdPrint (("Ezusb.SYS: exit Ezusb_GetConfigDescriptor\n"));

    return configurationDescriptor;
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
   PDEVICE_EXTENSION pdx = fdo->DeviceExtension;
   //PUSBD_INTERFACE_LIST_ENTRY  interfaceList = NULL;
   USBD_INTERFACE_LIST_ENTRY  interfaceList[2];
   PURB urb = NULL;
   USHORT urbSize;
   ULONG numberOfInterfaces;
   ULONG numberOfPipes;
   ULONG i,j;
   NTSTATUS ntStatus;
   USHORT *k;

   Ezusb_KdPrint (("enter SetInterface\n"));

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

    /*
    numberOfInterfaces = 1;
    interfaceList =
        ExAllocatePool(
               NonPagedPool, 
               sizeof(USBD_INTERFACE_LIST_ENTRY) * (numberOfInterfaces + 1));


   if (!interfaceList)
   {
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupSetInterface;
   }
   */
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
   interfaceList[0].InterfaceDescriptor = interfaceDescriptor;
   interfaceList[0].Interface = NULL;

   interfaceList[1].InterfaceDescriptor = NULL;
   interfaceList[1].Interface = NULL;
   urb = USBD_CreateConfigurationRequestEx(configurationDescriptor, &interfaceList[0]);

   if (!urb)
   {
      Ezusb_KdPrint (("USBD_CreateConfigurationRequestEx Failed\n"));
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupSetInterface;
   }

   ntStatus = Ezusb_CallUSBD(fdo, urb);

   if (NT_SUCCESS(ntStatus))
   {
      pdx->ConfigurationHandle = urb->UrbSelectConfiguration.ConfigurationHandle;
   }
   else
   {
      Ezusb_KdPrint (("Configuration Request Failed\n"));
      ntStatus = STATUS_UNSUCCESSFUL;
      goto CleanupSetInterface;
   }

    ExFreePool(urb);
    urb = NULL;

   numberOfPipes = interfaceDescriptor->bNumEndpoints;
   Ezusb_KdPrint (("numberOfPipes = %d\n", numberOfPipes));

   i = GET_SELECT_INTERFACE_REQUEST_SIZE(numberOfPipes);
   k = (USHORT*)&i;
   urbSize = *k;
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
   i = GET_USBD_INTERFACE_SIZE(numberOfPipes);
   k = (USHORT*)&i;
   interfaceInformation->Length = *k;

   ntStatus = Ezusb_CallUSBD(fdo, urb);

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
         if (pipeInformation->EndpointAddress == 0x02) {
             pdx->BulkOut_Pipe_Handle = pipeInformation->PipeHandle;
         }
         else if (pipeInformation->EndpointAddress == 0x06) {
             pdx->BulkOut_Pipe_Handle2 = pipeInformation->PipeHandle;
         }
      }
   }

CleanupSetInterface:

   // Clean up and exit this routine
   /*
   if (interfaceList != NULL)
   {
      ExFreePool(interfaceList);
   }
   */

   if (urb != NULL)
   {
      ExFreePool(urb);
   }

   if (configurationDescriptor != NULL)
   {
      ExFreePool(configurationDescriptor);
   }

    Ezusb_KdPrint (("exit SetInterface (%x)\n", ntStatus));
   return(ntStatus);
   
}   

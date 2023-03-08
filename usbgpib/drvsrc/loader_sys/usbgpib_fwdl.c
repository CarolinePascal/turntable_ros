/*
 * USB Skeleton driver - 2.0
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c 
 * but has been rewritten to be easy to read and use, as no locks are now
 * needed anymore.
 *
 */

#include <linux/init.h>
#include <linux/module.h>   /*Specifically, a module*/
#include <linux/kernel.h>   /*We're doing kernel work*/
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pnp.h>
#include <linux/sysctl.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/kref.h>
//#include <linux/smp_lock.h>
#include <linux/usb.h>
#include <asm/uaccess.h>
#include "usbgpib.h"

#include <linux/version.h>

#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	#include <linux/smp_lock.h>    /* For (un)lock_kernel */
#endif

/* Define these values to match your devices */
#define USB_GPIBFL_VENDOR_ID	0x144A
#define USB_GPIBFL_PRODUCT_ID	0x8051

/* table of devices that work with this driver */
static struct usb_device_id fl_table [] = {
	{ USB_DEVICE(USB_GPIBFL_VENDOR_ID, USB_GPIBFL_PRODUCT_ID) },
	{ }/* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, fl_table);


/* Get a minor range for your devices from the usb maintainer */
#define USB_FL_MINOR_BASE	192

/* Structure to hold all of our device specific stuff */
struct usb_fl {
	struct usb_device *	udev;			/* the usb device for this device */
	struct usb_interface *	interface;		/* the interface for this device */
	struct kref		kref;
};
#define to_fl_dev(d) container_of(d, struct usb_fl, kref)

static struct usb_driver fl_driver;

static void fl_delete(struct kref *kref)
{	
	struct usb_fl *dev = to_fl_dev(kref);

	usb_put_dev(dev->udev);
	kfree (dev);
}

static int fl_open(struct inode *inode, struct file *file)
{
	struct usb_fl *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&fl_driver, subminor);
	if (!interface) {
	printk ("%s - error, can't find device for minor %d\n",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}
	
	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	return retval;
}

static int fl_release(struct inode *inode, struct file *file)
{
	struct usb_fl *dev;

	dev = (struct usb_fl *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* decrement the count on our device */
	kref_put(&dev->kref, fl_delete);
	return 0;
}

static struct file_operations fl_fops = {
	.owner =	THIS_MODULE,
	.open =		fl_open,
	.release =	fl_release,
};

/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver fl_class = {
	.name = "usb/usbgpibfl%d",
	.fops = &fl_fops,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
	.mode = S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
#endif
	.minor_base = USB_FL_MINOR_BASE,
};

static int fl_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_fl *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* allocate memory for our device state and initialize it */
	dev = kmalloc(sizeof(struct usb_fl), GFP_KERNEL);
	if (dev == NULL) {
		printk("Out of memory\n");
		goto error;
	}
	memset(dev, 0x00, sizeof (*dev));
	kref_init(&dev->kref);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = interface->cur_altsetting;
printk("desc->bcdDevice: %x\n", dev->udev->descriptor.bcdDevice );
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

#if 0
		if (!dev->bulk_in_endpointAddr &&
		    (endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk in endpoint */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				err("Could not allocate bulk_in_buffer");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk out endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
#endif		
	}
#if 0	
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		err("Could not find both bulk-in and bulk-out endpoints");
		goto error;
	}
#endif
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

    retval = fx2_fpga_download( dev->udev, "/etc/adgpib/fw/usb-gpib.rbf");
    if (retval != 1) {  
        goto error;  
    }  


    Firmware_Download_Ext(dev->udev, usbgpib_ext, usbgpib_ext_size);
    Firmware_Download_Int(dev->udev, usbgpib_int, usbgpib_int_size);

#if 0
	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &fl_class);
	if (retval) {
		/* something prevented us from registering this driver */
		printk("Not able to get a minor for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	printk("USB Skeleton device now attached to usbgpibfl-%d\n", interface->minor);
#endif
	return 0;

error:
	if (dev)
		kref_put(&dev->kref, fl_delete);
	return retval;
}

static void fl_disconnect(struct usb_interface *interface)
{
	struct usb_fl *dev;
	int minor = interface->minor;

	/* prevent skel_open() from racing skel_disconnect() */
	//....lock_kernel();

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
#if 0
	/* give back our minor */
	usb_deregister_dev(interface, &fl_class);

	//....unlock_kernel();

	/* decrement our usage count */
	kref_put(&dev->kref, fl_delete);
#endif
	printk("USB Skeleton #%d now disconnected\n", minor);
}

static struct usb_driver fl_driver = {

	//.owner = THIS_MODULE,
	.name = "usbgpibfl",
	.id_table = fl_table,
	.probe = fl_probe,
	.disconnect = fl_disconnect,
};

static int __init usb_fl_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&fl_driver);
	if (result)
		printk("usb_register failed. Error number %d\n", result);

	return result;
}

static void __exit usb_fl_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&fl_driver);
}

module_init (usb_fl_init);
module_exit (usb_fl_exit);

MODULE_LICENSE("GPL");

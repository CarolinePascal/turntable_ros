#!/usr/bin/perl

%DRV_NAME;
%MAJOR_NUM;

$DRIVER_PATH;
$DRIVER_DIR;
$INSMOD_COMMAND;

$KERNEL_VER;

############### check the kernel version 

if( open(ker_info, "</proc/sys/kernel/osrelease" ) ){
    my $kernel_version = <ker_info>;

    ($rest) = $kernel_version =~ /(.*)/;
    close ker_info;

    $DRIVER_DIR = "/drivers/";
    $KERNEL_VER = $rest . "/";     
    $test = `pwd`;
    ($rest) = $test =~ /(.*)/;
    $test = $rest . "/" . $KERNEL_VER;
    if(! -d $test) {
        $KERNEL_VER = "";
    }
}

############### continue the old operation 
#======================================================================
# copy the *.rbf 

if( ! -d "/etc/adgpib" ){
  `mkdir -p /etc/adgpib/fw`;
}

if( ! -d "/etc/adgpib/fw" ){
  `mkdir /etc/adgpib/fw`;
}

$INSMOD_COMMAND ="cp -fp *.rbf /etc/adgpib/fw";

`$INSMOD_COMMAND`;

#======================================================================
# copy the *.cfg 

if( ! -d "/etc/adlgpib" ){
  `mkdir -p /etc/adgpib`;
}

$INSMOD_COMMAND ="cp -fp adgpib.cfg /etc/adgpib";

`$INSMOD_COMMAND`;

#======================================================================
# copy the libpci_dask.so to /usr/lib

 $INSMOD_COMMAND ="cp -fp ../lib/libadgpib32.so /usr/lib";

`$INSMOD_COMMAND`;

# copy udev rule to /etc/udev-rules/
$INSMOD_COMMAND = "cp -p 60-u348a.rules /etc/udev/rules.d";
print("copy udev-rules 60-u348a.rules to /etc/udev/rules.d/\n");
`$INSMOD_COMMAND`;

#======================================================================

#======================================================================
#insert adlswitch module
    $DRIVER_PATH = $KERNEL_VER;
#insert "u348a.ko" "usbgpibfl.ko" "usbgpibflad.ko" "kusbgpibfl.ko"
    $INSMOD_COMMAND = "insmod " . $DRIVER_PATH . "u348a.ko";
    print("$INSMOD_COMMAND \n");
    `$INSMOD_COMMAND`;
    $INSMOD_COMMAND = "insmod " . $DRIVER_PATH . "usbgpibfl.ko";
    print("$INSMOD_COMMAND \n");
    `$INSMOD_COMMAND`;
    $INSMOD_COMMAND = "insmod " . $DRIVER_PATH . "usbgpibflad.ko";
    print("$INSMOD_COMMAND \n");
    `$INSMOD_COMMAND`;

exit 0;











































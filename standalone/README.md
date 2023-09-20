# Standalone ROS-free turntable tool

## Requirements 

1) Build and install the USB-348A Linux Driver - See the corresponding [README](../usbgpib/readme)
2) Load the drivers before using the turntable

## Build & install

```shell
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=../ ..
make
make install
```

## Run 

```shell
cd bin 
./turntable <turntable_PAD_(if known)>
```

## Supported commands

```shell
getPosition #Returns current position
getAcceleration #Returns current acceleration as a 1 to 10 range number
isMax360    #Tells wether the turntable is limited to 360°

setAcceleration <acceleration_int> #Sets acceleration

setAbsPosition  <absolute_position_int>   #Sets absolute position
setRelPosition  <relative_position_int>   #Sets relative positon offset

setMax360   <0/1>   #Desactivate/activate the 360° limitation
set0reference   #Sets the 0° reference to the current position

startRotation   <speed_float>   #Starts continuous rotation motion at a given speed (seconds/revolution)
stopRotation    #Stops rotation motion
```

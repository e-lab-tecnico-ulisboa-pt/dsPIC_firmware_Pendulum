# dsPIC_firmware_Pendulum

With the help of a programmer software, the dspic can be programmed using the Raspberry Pi.

The dsPIC can be programed using the picpgm software which is hosted here. Once downloaded and installed, edit the "pgmifcfg.xml" file to associate the Raspberry Pi GPIO pins with their function while taking into account the connection with the dsPIC pins.
As an example, for the World Pendulum equipped with a dsPIC33F the part of the "pgmifcfg.xml" file corresponding to the GPIO pins is shown below:

```
...
    </PgmIf>
    <PgmIf name="GPIO Programmer (Raspberry Pi)" typ="LVP" connection="GPIO">
        <PinCfg name="MCLR" pin="27" invert="0" />
        <PinCfg name="PGM" pin="22" invert="0" />
        <PinCfg name="CLK" pin="3" invert="0" />
        <PinCfg name="DOUT" pin="2" invert="0" />
        <PinCfg name="DIN" pin="4" invert="0" />
    </PgmIf>
</Config>
```
In the general elab case, using a dsPIC 3.0 development board based on the dsPIC30F equipped with the optoisolated programmer, this file as a different pins assignation:

```
...    
    </PgmIf>
    <PgmIf name="GPIO Programmer (Raspberry Pi)" typ="LVP" connection="GPIO">
        <PinCfg name="MCLR" pin="5" invert="1" />
        <PinCfg name="PGM" pin="2" invert="0" />
        <PinCfg name="CLK" pin="13" invert="0" />
        <PinCfg name="DOUT" pin="19" invert="0" />
        <PinCfg name="DIN" pin="26" invert="0" />
    </PgmIf>
</Config>
```

The pin number (pin="xx") corresponds to the GPIO pin number on the Raspberry Pi (see figure in dsPIC 33F section). The configuration example shown above is the one used to program the dsPIC of the World Pendulum Alliance.

Once the configuration is set-up and the dsPIC is powered and connected to the Raspberry Pi, one can start loading the program (hexadecimal file) into the dsPIC memory. To correctly program the dsPIC, both the program memory and the config memory (special registers) must be programmed. Some devices also have an EEPROM memory. The config memory is programmed using the following command (in Linux):

``` 
sudo picpgm -p_cfg hexadecimalFile.hex
``` 

This will program the bit of the code corresponding to the "#pragma config" instructions. To load the program into the program memory, use the following command (in Linux):
```
sudo picpgm -p_code hexadecimalFile.hex
```
This command will write and check (read) the dsPIC memory to make sure that it has be written properly. If writing errors occurred, rerun the command and make sure that no external electrical interference affects the dsPIC during the programming process. For further option and usage of picpgm, simply run:
```
picpgm
```
and a help menu will appear.

If the Raspberry Pi is to remain connected to the dsPIC, one has to make sure that the MCLR pin is set not active (3.3 V) to allow the dsPIC to run (no longer in reset state). To ensure such pin state on the RPi side, one can add the following command in a boot script, as shown below:

Bash file @ /etc/rc.local

```
#The following lines should be added to file rc.local to allow the pic to run after RPI boot
#The wiringpi package needs to be installed
gpio -g mode 5 down
gpio -g mode 2 down
#For the picpgm configuration (pgmifcfg.xml) shown above it would correspond to pin 27:
gpio -g mode 27 up
```

## Source files

On the folder `Source files` contains the source files corresponding to the WPA firmware version `2021_01_06_22_08_46`. A `instructions.txt` file containing a set of steps to guide the user on how to create a MPLAB X IDE project is also in the file. The date and time of the compilation is stored in the `.hex` file. This means that any firmware (`.hex`) files complied in different dates/times will differ from each other. This feature was introduced to allow a clear distinction of each newly created `.hex` file.

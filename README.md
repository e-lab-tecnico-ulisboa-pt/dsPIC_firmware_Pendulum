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

# LinuxCNCConfigs
LinuxCNC Configurations for various systems here in the shop

1. SkinnyCNC is a homemade stepper based system that's a long skinny system for cutting custom panels
2. SabreCNC is a retrofit of a Gerber Saber 408 System

##  These configs are symlinked to linuxcnc/configs depending on the system  
## for example
```
cd ~
git clone git@github.com:SkyMoCo/LinuxCNCConfigs.git
mv ~linuxcnc/configs  ~linuxcnc/configs.orig
ln -s ~LinuxCNCConfigs/SabreCNC/configs ~linuxcnc/configs
```

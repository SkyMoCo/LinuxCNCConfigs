# Notes on tuning the servos

* First followed this guide to get close

https://forum.linuxcnc.org/10-advanced-configuration/32367-servo-wiring-and-tuning-detailed-how-to-example-mesa-7i77?start=0

For whatever reason, I was thinking I needed to keep the P value as low as possible, so I didn't want to go above 30 or so and had problems with way overshooting where I wanted it to stop.

I ran accross an example somewhere with the P value of 400 and it occured to me I could continue to raise the P value, I increased it a few steps at a time until I got to the current setting of 250

The way I think of it is the P value is the amount force used to move the Axis

The D value is the speed at which we respond.


	

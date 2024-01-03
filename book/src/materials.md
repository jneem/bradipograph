# List of materials

## Motors and servos

The two spools are turned by 28BYJ-48 stepper motors, powered
by ULN2003 drivers. These motors are often sold together with the
drivers; as of this writing, a pack of 5 motors and 5 drivers is available
for $15 on Amazon or $10 on AliExpress.

The pen is lifted with an SG90 servo. A 5-pack costs about $10 on Amazon,
or $5 on AliExpress.

## The batteries

I'm using two rechargeable 9v Li-ion batteries. At about $20 for a 4-pack
on Amazon, this is definitely the most expensive part. You'll also need two 9v
battery connectors with wire leads. A 10-pack is about $5 on Amazon.

## Power supply

We'll power the device with 9v batteries, but that 9v needs to be converted
down to 5v. Adjustable DC step-down converters are available in packs
of 5 for about $10 on Amazon or $5 on AliExpress.

## The brains

The device is controlled by an ESP32-C3 chip. Development boards with ESP32-C3
chips are available on AliExpress for as little as $2. There doesn't seem to
be much difference between the different boards, so probably any of them
will work. But if you want to use the exact same board as me, I'm using
WeActStudio's ESP32-C3FH4 Core Board.

## Misc

You'll need something to hold the pen or marker with. I'm using an extra-large binder
clip. Its base measures 2cm by 5cm, and it's just the right size to hold my
markers (which are about 1.5cm in diameter). You'll also need a marker.

You'll need wires to hook everything up. The stepper motors and servo usually
come with their own wires, so you'll mostly just need wires to hook power up
to everything.

You don't *need* a power switch, but I think it's nice to have one.

You'll need some string for hanging the device (sewing thread or thin fishing line both work),
and some way to attach it to the drawing surface. For drawing on whiteboards or windows,
suction cups work well.

Finally, you'll need some screws to hold it together: 15 M3 screws (two of them need to be at least 8mm long; the
rest can be shorter) and two M2 screws.

## Tools and equipment

- a soldering iron
- a 3d printer (or the use of one for a few hours)
- a screwdriver
- a tool for cutting and stripping wire
- a USB-C data cable
- a computer

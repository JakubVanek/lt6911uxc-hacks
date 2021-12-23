# WIP attempts to understand the LT6911UXC HDMI2MIPI bridge

This repository contains some hacks that I tried to make LT6911UXC
work on RPi4 Compute Module.

LT6911UXC is the chip that powers the Auvidea B112 HDMI-to-CSI2 board.
This one might eventually replace the Auvidea B102 board that uses the TC358743XBG chip.

## Preexisting drivers

I've found at least five different drivers:
 * Driver written by researchers at ZHAW Zurich University of Applied Sciences: https://github.com/InES-HPMM/Lontium_lt6911uxc
   - This driver is Tegra-specific, but this could be resolved - it is built on the V4L2 API, it just also uses some Tegra-only functions.
 * Two similar drivers found in obscure Intel repositories:
   - https://github.com/projectceladon/linux-intel-lts2020-yocto/blob/f6b201c626fc25c5c69407aad63e1ad33613f250/drivers/media/i2c/lt6911uxc.c
   - https://github.com/projectacrn/acrn-kernel/blob/f5a69c541df42a6cd3c50a26df044bba516baf9c/drivers/media/i2c/lt6911uxc.c
   - Both of these are likely based on the first driver - the overall structure is very similar.
   - This driver is Intel-specific to a degree - it lacks OF/device tree bindings (ACPI is used instead?).
     However, these should be relatively easy to add. These drivers would likely be the ideal starting point for adaptation for RPis
 * https://github.com/LineageOS/android_kernel_nvidia_nvidia/blob/b10f76ea6b11022e19457693d37a6d8903f2b0a6/drivers/media/i2c/lt6911uxc.c
   - Tegra-only driver written by Nvidia
   - Very minimal, little to no register accesses -> unsuitable
 * https://github.com/zhangwenxu-sxs/test/tree/master/starnet/lt6911uxc
   - Unknown driver, perhaps leaked from somewhere
   - Contains some interesting register accesses that other drivers don't

## Datasheet

I haven't found the full datasheet ("functional specification") anywhere
on the internet. Reaching out to Lontium and asking for it under a NDA
seems to be the only way to get it.

I've tried to extract useful information from the drivers linked above.
These information are scattered around in the [`regmap`](regmap) directory.

## Experiments

Before going to kernel-space and creating a driver there, I wanted to make
sure that the device itself works. For this, I've created a Python
script for controlling the bridge: [python_hacks/lt6911uxc.py](python_hacks/lt6911uxc.py).

Unfortunately, the chip doesn't work as-is. I2C communication works -
I'm able to read the chip ID (0x1704) without issues. However, the chip doesn't
react to the HDMI cable being plugged in and doesn't announce itself to the
transmitting device. Therefore, the device doesn't send any signal. The only
observable reaction is that values in two or three internals registers change.

To check that the problem isn't with being in userspace, I've tried to replicate
the same for the Auvidea B102. I've ended up using 6by9's [https://github.com/6by9/raspi_tc358743](raspi_tc358743).
This script still works with B102 on RPi4CM with modified `config.txt`. I've
then half-blindly tried to port the script the LT6911UXC.
This didn't work - the chip still doesn't send any video.

However, I've been searching around the internet and the issue might be:

## Firmware

Several sources mention that the chip has a firmware:
 - LT6911UXC "propagation material": http://www.lontiumsemi.com/UploadFiles/2021-07/LT6911UXC_Brief_R1.0.pdf

    > The device is capable of automatic operation which is
    > enabled by an integrated microprocessor that uses an
    > embedded SPI flash for firmware storage. System
    > control is also available through the configuration I2C
    > slave interface.

 - ZHAW researcher's blogpost: https://blog.zhaw.ch/high-performance/tag/lt6911uxc/

    > If RGB or YUV streams are accepted depends on the corresponding LT6911UXC
    > firmware provided by Lontium. Contact Lontium for the firmware.

 - Some article on CSDN: https://blog.csdn.net/TSZ0000/article/details/109182122

   /google translate'd/:

    > The conversion chip has no firmware by default, the read data is all 0xFF,
    > and the firmware is programmed: LT6911UXC_V2P9_YUV422_20201012_GSW_X2.hex

 - README of a project based on the chip: https://github.com/peng-zhihui/HDMI-PI

   /google translate'd/:

    > There is also a Longxun solution LT6911 made in China. Compared with the above solution,
    > Longxun has a slightly weaker performance, but the chip has a built-in 51-core MCU, so it
    > can be programmed directly on the chip (Toshiba needs an additional single-chip microcomputer
    > with I2C Configure the chip).
    >
    > The advantage of this scheme is that the cost is relatively low, and the peripheral circuit
    > of the chip is more concise. The disadvantage is that the data is less than that of Toshiba...
    >
    > Manufacturers do not open software and hard data, and do not even have a datasheet, so it is
    > almost impossible to personally develop it. However , the Almighty Wild Iron Man obtained some
    > information from the agent through some special methods, including some source code (the core
    > lib is encapsulated and I can't get it, only the upper-level API). But because I signed the NDA
    > confidentiality agreement, it is difficult for me to share the source code, except for
    > the source code, I have open sourced the other parts, and you don’t need the source code for DIY.
    > I can provide pre-compiled firmware for everyone to download, so this solution is suitable for
    > Direct copy of the project's classmates' reference.

 - Nvidia's driver source code: https://github.com/LineageOS/android_kernel_nvidia_nvidia/blob/b10f76ea6b11022e19457693d37a6d8903f2b0a6/drivers/media/i2c/lt6911uxc.c#L87

    > /* As soon as the reset pin is released, the bridge starts streaming

   This leads me to think that the firmware is the key to making the chip work.

I'd guess that my chip doesn't have any firmware yet. This would explain
why it didn't do anything by itself. The hardware could still be controlled by a
sophisticated driver via I2C. However, the data needed for creating that (full register maps
and more) are not available. Asking Lontium for a firmware blob and a flasher
might be the best bet here.

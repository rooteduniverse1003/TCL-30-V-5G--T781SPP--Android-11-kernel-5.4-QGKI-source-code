# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT)		+= input-core.o
input-core-y := input.o input-compat.o input-mt.o input-poller.o ff-core.o

obj-$(CONFIG_INPUT_FF_MEMLESS)	+= ff-memless.o
obj-$(CONFIG_INPUT_POLLDEV)	+= input-polldev.o
obj-$(CONFIG_INPUT_SPARSEKMAP)	+= sparse-keymap.o
obj-$(CONFIG_INPUT_MATRIXKMAP)	+= matrix-keymap.o

obj-$(CONFIG_INPUT_LEDS)	+= input-leds.o
obj-$(CONFIG_INPUT_MOUSEDEV)	+= mousedev.o
obj-$(CONFIG_INPUT_JOYDEV)	+= joydev.o
obj-$(CONFIG_INPUT_EVDEV)	+= evdev.o
obj-$(CONFIG_INPUT_EVBUG)	+= evbug.o

obj-$(CONFIG_INPUT_KEYBOARD)	+= keyboard/
obj-$(CONFIG_INPUT_MOUSE)	+= mouse/
obj-$(CONFIG_INPUT_JOYSTICK)	+= joystick/
obj-$(CONFIG_INPUT_TABLET)	+= tablet/
obj-$(CONFIG_INPUT_TOUCHSCREEN)	+= touchscreen/
obj-$(CONFIG_INPUT_MISC)	+= misc/

obj-$(CONFIG_INPUT_APMPOWER)	+= apm-power.o

obj-$(CONFIG_RMI4_CORE)		+= rmi4/
#add by chen-liang for sar sensor
obj-$(CONFIG_AW96103_SAR) += aw96103_sar/
obj-$(CONFIG_AW96105_SAR) += aw96105_sar/

#qiaozhen.li add for focal fingerprint
obj-$(CONFIG_FOCALTECH_FINGERPRINT) += fingerprint_focaltech/

obj-$(CONFIG_FOCALFP_FINGERPRINT) += focaltech_transformervzw/

obj-$(CONFIG_SUNWAVE_FP_IRVINEVZW) += sunwave_irvinevzw/

obj-$(CONFIG_FOCALTECH_FP_IRVINEVZW) += focaltech_irvinevzw/

#modifed by zhikui.li,add fingerprint
obj-$(CONFIG_GOODIX_FINGERPRINT) += fingerprint_goodix/

#Added by baiwei.peng for hall switch driver on 2021/05/22
obj-$(CONFIG_TCT_HALL_SWITCH) += hall/

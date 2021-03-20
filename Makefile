obj-m += madgwick.o
madgwick-objs := Fusion/FusionAhrs.o Fusion/FusionBias.o Fusion/FusionCompass.o madgwick_main.o
include /home/cps/machinekit/src/Makefile.modinc

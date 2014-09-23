test:  shakeyourpeace_external_pedalometer
	./$+

debug:  shakeyourpeace_external_pedalometer.cc
	gcc -o shakeyourpeace_external_pedalometer --debug $+
	gdb shakeyourpeace_external_pedalometer

shakeyourpeace_external_pedalometer:  shakeyourpeace_external_pedalometer.cc
	gcc -o $@ $+

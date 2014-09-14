test:  shakeyourpeace_external_pedalometer
	./$+

shakeyourpeace_external_pedalometer:  shakeyourpeace_external_pedalometer.cc
	gcc -o $@ $+

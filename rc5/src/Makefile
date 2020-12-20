all: spe ppe

ppe:
	cd ppu;make

spe: 
	cd spu;make

clean:
	cd ppu;rm *.o *.d *~ -f
	cd spu;make clean;rm *~ -f

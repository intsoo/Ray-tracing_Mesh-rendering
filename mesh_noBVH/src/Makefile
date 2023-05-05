CC=nvcc 

a.out: main.cu mesh.h utility.h vec3.h
	$(CC) -G -g --expt-relaxed-constexpr -o a.out main.cu
	$ ./a.out

clean:
	rm -rf *.out


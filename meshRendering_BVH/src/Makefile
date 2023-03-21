CC=nvcc 

a.out: main.cu mesh.h utility.h vec3.h
	$(CC) -G -g --expt-relaxed-constexpr -o a.out main.cu
	$ ./a.out > debug.txt; mv debug.txt ../txt/txt_bvh
	$ mv *.ppm ../img/img_bvh

clean:
	rm -rf *.out


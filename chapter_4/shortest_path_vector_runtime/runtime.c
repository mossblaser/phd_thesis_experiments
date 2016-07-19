/**
 * Compute the shortest paths between all pairs of points in a hexagonal torus
 * topology of the specified size. The runtime of this script can be used to
 * infer the average runtime of each shortest path function. The tool outputs
 * the sum of all route lengths which may be used as a sanity-check for
 * the output consistency of each algorithm.
 *
 * Usage:
 *    $ gcc -O2 runtime.c algos.c -o runtime
 *    $ time ./runtime ALGO WIDTH HEIGHT
 * Where:
 * - ALGO is 0, 1 or 2 for INSEE method, IQ method or XYZ protocol, resp.
 * - WIDTH and HEIGHT are the dimensions of the network
 */

#include<stdio.h>
#include<stdlib.h>

#include "algos.h"

int main(int argc, char *argv[]) {
	int exp = atoi(argv[1]);
	int w = atoi(argv[2]);
	int h = atoi(argv[3]);
	int out_o[3];
	
	int total = 0;
	
	for (int sx = 0; sx < w; sx++) {
		for (int sy = 0; sy < h; sy++) {
			for (int dx = 0; dx < w; dx++) {
				for (int dy = 0; dy < h; dy++) {
					switch (exp) {
						case 0: insee_method(sx, sy, dx, dy, w, h, out_o); break;
						case 1: iq_method(sx, sy, dx, dy, w, h, out_o); break;
						case 2: xyz_protocol(sx, sy, dx, dy, w, h, out_o); break;
					}
					total += (abs(out_o[0]) + abs(out_o[1]) + abs(out_o[2]));
				}
			}
		}
	}
	printf("Total length: %ld\n", total);
	return 0;
}

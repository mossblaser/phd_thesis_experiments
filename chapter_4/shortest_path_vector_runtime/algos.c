/**
 * Algorithms for computing shortest path vectors in hexagonal torus
 * topologies.
 *
 * All functions take the following arguments:
 *  - src_x, src_y: Start coordinates (cast to 2D)
 *  - dest_x, dest_y: End coordinates (cast to 2D)
 *  - w, h: System dimensions
 *  - out: Return value. Pointer to three ints (x, y and z) the vector will be
 *         written to.
 */

#include<stdlib.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) < (b)) ? (b) : (a))

#define ABS(x) ((x)<0 ? -(x) : (x))
#define SIGN(x) ((x)==0 ? 0 : (x)<0 ? -1 : 1)

// Given a point along an axis and the length of that axis, get the distance
// from the far-end of the axis to that point
#define FLIP(d, len) ((d) - (SIGN((d)) * ((len))))


/**
 * The INSEE method of computing shortest paths vectors in hexagonal toruses.
 *
 * This function was taken from INSEE and written by Javier Navaridas, licensed
 * under GPLv2.
 */
void insee_method(int sx, int sy, int dx, int dy, int width, int height, int *out) {
	#define D_X 0
	#define D_Y 1
	#define D_Z 2
	
	static int Ax1, Ax2, Ay1, Ay2;
	static int rx, ry, rz, dist, res_size;
	
	// distance in each axis
	Ax1=dx-sx;
	Ax2=-1*SIGN(Ax1)*(width-ABS(Ax1));

	Ay1=dy-sy;
	Ay2=-1*SIGN(Ay1)*(height-ABS(Ay1));

	// all routing possibilities are calculated here. the best one is selected.
	
	out[D_Z]=Ay1;
	out[D_X]=Ax1-Ay1;
	out[D_Y]=0;
	res_size = ABS(out[D_X]) + ABS(out[D_Y]) + ABS(out[D_Z]);
	
	rz=Ax1;
	ry=Ay1-Ax1;
	rx=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rx=Ax1;
	ry=Ay1;
	rz=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rz=Ay2;
	rx=Ax1-Ay2;
	ry=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rz=Ax1;
	ry=Ay2-Ax1;
	rx=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rx=Ax1;
	ry=Ay2;
	rz=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rz=Ay1;
	rx=Ax2-Ay1;
	ry=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rz=Ax2;
	ry=Ay1-Ax2;
	rx=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rx=Ax2;
	ry=Ay1;
	rz=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rz=Ay2;
	rx=Ax2-Ay2;
	ry=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rz=Ax2;
	ry=Ay2-Ax2;
	rx=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}

	rx=Ax2;
	ry=Ay2;
	rz=0;
	dist = ABS(rx)+ABS(ry)+ABS(rz);
	if (dist<res_size){
		out[D_X]=rx;
		out[D_Y]=ry;
		out[D_Z]=rz;
		res_size=dist;
	}
	
	// XXX: In INSEE, Z goes the 'wrong' way, correct this
	out[D_Z] = -out[D_Z];
	
	#undef D_X
	#undef D_Y
	#undef D_Z
}


/**
 * The IQ method of computing shortest paths vectors in hexagonal toruses.
 *
 * Written by Jonathan Heathcote, released into the public domain.
 */
void iq_method(int sx, int sy, int dx, int dy, int w, int h, int *out) {
    // Translate destination by the vector used to translate the source to
    // (0,0,0)
    dx = (dx - sx);
    dy = (dy - sy);
    
    dx = (dx < 0) ? dx + w :
         (dx >= w) ? dx - w : dx;
    dy = (dy < 0) ? dy + h :
         (dy >= h) ? dy - h : dy;

    //                   Distance         Vector
    int approaches[] = { MAX(dx, dy),     //(dx, dy)
                         w-dx+dy,         //(-(w-dx), dy)
                         dx+h-dy,         //(dx, -(h-dy))
                         MAX(w-dx, h-dy)  //(-(w-dx), -(h-dy))
                       };

    // Select the best approach.
    int best = MIN(approaches[0], MIN(approaches[1], MIN(approaches[2], approaches[3])));
    int x, y;
    
    if (approaches[0] == best) {
        x = dx;
        y = dy;
    } else if (approaches[1] == best) {
        x = -(w-dx);
        y = dy;
    } else if (approaches[2] == best) {
        x = dx;
        y = -(h-dy);
    } else { // if (approaches[3] == best) {
        x = -(w-dx);
        y = -(h-dy);
    }
    
    int z = 0;
    
    int median = MAX(MIN(x,y), MIN(MAX(x,y),z));
    
    out[0] = x-median;
    out[1] = y-median;
    out[2] = z-median;
}


/**
 * The XYZ-protocol for computing shortest paths vectors in hexagonal toruses.
 *
 * Algorithm proposed by Hoffmann and Deserable in 'Routing by Cellular
 * Automata Agents in the Triangular Lattice'.
 *
 * Implementation by Jonathan Heathcote, released into the public domain.
 */
void xyz_protocol(int src_x, int src_y, int dest_x, int dest_y, int w, int h, int *out) {
	// Step 0
	int dx = dest_x - src_x;
	int dy = dest_y - src_y;
	
	// Step 1
	if (ABS(dx) > w/2)
		dx = FLIP(dx, w);
	if (ABS(dy) > h/2)
		dy = FLIP(dy, h);
	
	// Step 2
	if ((dx * dy) < 0) {
		int new_dx = dx;
		int new_dy = dy;
		if ((ABS(dx) > ABS(dy)) && (ABS(FLIP(dx,w)) < (ABS(dx) + ABS(dy))))
			new_dx = FLIP(dx, w);
		if ((ABS(dy) >= ABS(dx)) && (ABS(FLIP(dy,h)) < (ABS(dx) + ABS(dy))))
			new_dy = FLIP(dy, h);
		dx = new_dx;
		dy = new_dy;
	}
	
	// Step 3: Postprocessing to make directions consistent in middle
	if ((dx == -w/2) && (dy == -h/2)) {
		dx = ABS(dx);
		dy = ABS(dy);
	}
	
	{
		int new_dx = dx;
		int new_dy = dy;
		if ((dx == -w/2) && (dy == 0))
			new_dx = ABS(dx);
		if ((dy == -h/2) && (dx == 0))
			new_dy = ABS(dy);
		dx = new_dx;
		dy = new_dy;
	}
	
	// Minimal route computation
	if (dx * dy < 0) {
		out[0] = dx;
		out[1] = dy;
		out[2] = 0;
	} else { //if (dx * dy > 0) {
		out[2] = SIGN(dx) * MIN(ABS(dx), ABS(dy));
		out[1] = dy - out[2];
		out[0] = dx - out[2];
	}
	
	// Also uses 'wrong' Z direction...
	out[2] = -out[2];
}

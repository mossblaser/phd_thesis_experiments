#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <assert.h>

#include "pqueue.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

//Original values for 256x256
//#define CENT_MIN_DIST 48
//#define CENT_RANGE 64
//#define CENT_SIZE 15
//#define SOMA_SIZE 30

//draft values for 48x48
#define CENT_MIN_DIST 12
#define CENT_RANGE 8
#define CENT_SIZE 5
#define SOMA_SIZE 8

#ifndef NUM_REPEATS_SCALE
#define NUM_REPEATS_SCALE 16
#endif

// Most of the infrastructure for performing Dijkstra's algorithm is
// shared with the A* algorithm.
#ifdef DIJKSTRA
	#ifndef A_STAR
		#define A_STAR
	#endif
#endif

#ifndef MAX_DST
#define MAX_DST 256 // should be enough!!!!
#endif

#ifndef CONNECTION
#define CONNECTION 0 // 0: means connect to any route point. 7: means connect to nodes only
#endif

#ifndef REP
#define REP 10000
#endif

#ifndef RANGE
#define RANGE 20
#endif

#ifndef FIRST_ALG
#define FIRST_ALG 0
#endif

#ifndef LAST_ALG
#ifdef A_STAR
#define LAST_ALG 4
#else
#define LAST_ALG 3
#endif
#endif

#ifndef BKT_SIZE
#define BKT_SIZE 512
#endif

#ifndef BKT_NUM
#define BKT_NUM 132
#endif

#ifndef BKT_LEN
#define BKT_LEN 1
#endif

#define abs(a)            (((a) >= 0) ? (a) : (-a))
#define sign(a)            (((a) >= 0) ? (1) : (-1)) // sign of 0 should be 0, not used for 0 here
#define min(a,b)            (((a) > (b)) ? (b) : (a))
#define max(a,b)            (((a) < (b)) ? (b) : (a))

#ifndef MAX_PROXY
#define MAX_PROXY 0
#endif
//#define ZIGZAG
//#define REVERSEDIM
//#define A_STAR
//#define DIJKSTRA
//#define SPINN_LINK_FAIL

long a_star_required = 0; // +1'd when a dead link is routed through and the
                              // path must be fixed in a later post-processing
                              // step.

#define EAST       0
#define NORTH_EAST 1
#define NORTH      2
#define WEST       3
#define SOUTH_WEST 4
#define SOUTH      5

#define SINK       6
// Get the opposite direction to the one given
#define OPP(direction) (((direction) + 3) % 6)

// Get the direction from a dimension and direction pair
#define DIR(dim, inc) (DIRECTIONS[dim][inc!=-1])
static const long DIRECTIONS[3][2] = { {EAST, WEST }, { NORTH, SOUTH }, { NORTH_EAST, SOUTH_WEST } };

// Get the dimension from a direction
#define DIM(dir) (DIMENSIONS[(dir)])
static const long DIMENSIONS[6] = {
	0, // EAST
	2, // NORTH_EAST
	1, // NORTH
	0, // WEST
	2, // SOUTH_WEST
	1, // SOUTH
};


// Get the X and Y deltas produced by moving one unit along the given
// dimension.
#define DIRDX(dir) (DIRECTION_DELTAS[(dir)][0])
#define DIRDY(dir) (DIRECTION_DELTAS[(dir)][1])
static const long DIRECTION_DELTAS[6][2] = {
	{+1, +0}, // EAST
	{+1, +1}, // NORTH_EAST
	{+0, +1}, // NORTH
	{-1, +0}, // WEST
	{-1, -1}, // SOUTH_WEST
	{+0, -1}, // SOUTH
};

// Get the direction for a given difference in x and y.
#define XYTODIR(x, y) (DELTA_DIRECTION[(y+1)][(x+1)])
static const long DELTA_DIRECTION[3][3] = {
	{SOUTH_WEST, SOUTH, -1},
	{WEST,       -1,    EAST},
	{-1,         NORTH, NORTH_EAST},
};

long x, xmax, xmin;
long y, ymax, ymin;
long arg=0; // extra space when printing the map.

long hop;
long nodeX[MAX_DST];
long nodeY[MAX_DST];
long direc[MAX_DST];
long proxy;

long    p[BKT_NUM][BKT_SIZE][3];
long    bucket[BKT_NUM];
long    map[341][341];
long  accum_map[341][341][6];  // Summed resources consumed at each link
long  entry_map[341][341];  // Number of routing entries at each router
long  fault[341][341][6];// status of the links 0=working, 1=fault // 0:E, 1: NE, 2:N, 3:W, 4:SW, 5:S
long  connected[BKT_NUM][BKT_SIZE];	// does this destination got connected: 1=yes, 0=no
long disconnected2;

// The next 'colour' to label a fragment of the routing tree with.
long next_a_star_colour;

///* A datatype which stores the data used by the A* algorithm.
struct a_star_data {
	// Each disconnected tree in the route will be given its own unique colour.
	// This way, it is possible to tell if the search has collided with another
	// part of the same tree or different tree. The next
	long colour;

	// Has the algorithm visited this node yet? (A counter which gives the
	// current visit number so that we don't need to go and reset this flag
	// every time).
	long visited;

	// When performing the graph traversal, the first visitor to this node
	// places its coordinates in these fields. This allows back-tracking of the
	// shortest discovered path.
	long nx, ny;

	// The route directions through this vertex (ORed together EAST, NORTH_EAST
	// etc.)
	long route;

	#ifdef A_STAR
	#ifndef DIJKSTRA
		// Fields used by the priority queue implementation
		pqueue_pri_t pri;
		size_t pos;
	#endif
	#endif
};

// A-Star Data (ASD). Used by the A-star post-processing step to hold
// information about parts of a rotue in the system.
struct a_star_data asd[341][341];
long a_star_cur_visit_num = 0;

// a_star_disconnections is a linked-list of parent/node pairs which have been
// left disconnected by a fault.
struct a_star_disconnection_list;
struct a_star_disconnection_list {
	long px, py;
	long x, y;
	struct a_star_disconnection_list *next;
};
struct a_star_disconnection_list *a_star_disconnections;

long double resrc[12][32][5] ;//resources
long double  ent[12][32][5] ;//entries
long double useX[12][32][5] ;//dim X
long double useY[12][32][5] ;//dim Y
long double useD[12][32][5] ;//dim Z
long double exec[12][32][5] ;//exec. time
long double post_exec[12][32][5] ;//postprocessing exec. time
long double disconnected[12][32][5] ; //disconnected nodes

long double        resrc_min[12][32][5] ;//resources
long double          ent_min[12][32][5] ;//entries
long double         useX_min[12][32][5] ;//dim X
long double         useY_min[12][32][5] ;//dim Y
long double         useD_min[12][32][5] ;//dim Z
long double         exec_min[12][32][5] ;//exec. time
long double    post_exec_min[12][32][5] ;//post-processing exec. time
long double disconnected_min[12][32][5] ; //disconnected nodes

long double        resrc_max[12][32][5] ;//resources
long double          ent_max[12][32][5] ;//entries
long double         useX_max[12][32][5] ;//dim X
long double         useY_max[12][32][5] ;//dim Y
long double         useD_max[12][32][5] ;//dim Z
long double         exec_max[12][32][5] ;//exec. time
long double    post_exec_max[12][32][5] ;//post-processing exec. time
long double disconnected_max[12][32][5] ; //disconnected nodes


long double        resrc_sq[12][32][5] ;//resources
long double          ent_sq[12][32][5] ;//entries
long double         useX_sq[12][32][5] ;//dim X
long double         useY_sq[12][32][5] ;//dim Y
long double         useD_sq[12][32][5] ;//dim Z
long double         exec_sq[12][32][5] ;//exec. time
long double    post_exec_sq[12][32][5] ;//post-processing exec. time
long double disconnected_sq[12][32][5] ; //disconnected nodes

long double        resrc_stdev[12][32][5] ;//resources
long double          ent_stdev[12][32][5] ;//entries
long double         useX_stdev[12][32][5] ;//dim X
long double         useY_stdev[12][32][5] ;//dim Y
long double         useD_stdev[12][32][5] ;//dim Z
long double         exec_stdev[12][32][5] ;//exec. time
long double    post_exec_stdev[12][32][5] ;//post-processing exec. time
long double disconnected_stdev[12][32][5] ; //disconnected nodes

FILE *fp;

static long POW[]={1,2,4};

long kk;

///* Generates a normal(0,1) distributed random number
long double rnd_norm(){
	long double a, b, c;
	a=((long double)rand())/RAND_MAX;
	b=((long double)rand())/RAND_MAX;
	c=sqrt(-2*log(a))*cos(2*M_PI*b);
	return c;
}

///* Generates a normal(mu, sigma) distributed random number
long double rand_norm(long double mu, long double sigma){
	return mu+(rnd_norm()*sigma);
}

/**
 * Generates random number following an exponential distribution [ln(1-rnd)/(- \lambda )],
 * being rnd a random number in [0,1) and \lambda the frequency.
 */
long double rnd_exp(long double lambda){
	return (log(1.0-((long double)rand()/(long double)RAND_MAX))/(-lambda));
}


/**
 * Reset the A-star algorithm's state, ready to start the algorithm.
 */
void a_star_reset(void) {
	// Sets initial values in all datastructures
	long x, y;
	for (x = 0; x < 341; x++) {
		for (y = 0; y < 341; y++) {
			asd[x][y].colour = -1;
			asd[x][y].visited = false;
			asd[x][y].route = 0;
		}
	}

	next_a_star_colour = 0;
}


/**
 * Internally used by a_star.
 *
 * Traverses the routing tree and initialise the asd structure.
 *
 * Sets up a graph in the asd matrix which represents the same connectivity as
 * the map matrix. The key difference is that the connectivity is given as
 * directions and not just a set of axes. Further, each disconnected subtree is
 * labelled with a distinct colour.
 *
 * @param direction The direction of the hop that took us from the parent to this node.
 * @param x The x coordinate of the node to traverse.
 * @param y The y coordinate of the node to traverse.
 * @param colour The colour of the subtree this node is in.
 */
void _a_star_init(long direction, long x, long y, long colour) {
	long c;
	struct a_star_disconnection_list *disconnection;

	asd[x][y].visited = a_star_cur_visit_num;

	asd[x][y].colour = colour;
	if (map[x][y] > 7)
		asd[x][y].route |= 1 << SINK;

	// A macro which recurses into children in a given direction as long as
	// they are not the parent and as long as the child actually has something
	// on it (i.e. we're actually connected to it). The colour is also changed
	// if the link we're heading down is marked as faulty (we presume that the
	// link at the current node is working).
	#define RECURSE_IF_CHILD(dir, xx, yy) \
		do { \
			if ((map[(xx)][(yy)] & (POW[DIM(dir)] | 0x8)) \
			    && asd[(xx)][(yy)].visited != a_star_cur_visit_num) { \
				if (fault[x][y][(dir)]) { \
					c = next_a_star_colour++; \
					disconnection = malloc(sizeof(struct a_star_disconnection_list)); \
					assert(disconnection); \
					disconnection->px = x; \
					disconnection->py = y; \
					disconnection->x = (xx); \
					disconnection->y = (yy); \
					disconnection->next = a_star_disconnections; \
					a_star_disconnections = disconnection; \
				} else { \
					c = colour; \
					asd[x][y].route |= 1 << (dir); \
				} \
				_a_star_init((dir), (xx), (yy), c); \
			} \
		} while (0)

	if (map[x][y] & 0x1) {
		RECURSE_IF_CHILD(WEST, x-1, y);
		RECURSE_IF_CHILD(EAST, x+1, y);
	}
	if (map[x][y] & 0x2) {
		RECURSE_IF_CHILD(SOUTH, x, y-1);
		RECURSE_IF_CHILD(NORTH, x, y+1);
	}
	if (map[x][y] & 0x4) {
		RECURSE_IF_CHILD(SOUTH_WEST, x-1, y-1);
		RECURSE_IF_CHILD(NORTH_EAST, x+1, y+1);
	}

	#undef RECURSE_IF_CHILD
}

/**
 * Internal function used by _a_star_reconnect which colours all nodes down a
 * particular routing tree with a specificed colour.
 *
 * Warning: this function assumes it is used on a tree and does not perform
 * cycle detection!
 */
void _a_star_colour_subtree(long x, long y, long colour) {
	long dir;

	asd[x][y].colour = colour;

	for (dir = 0; dir < 6; dir++) {
		if (asd[x][y].route & (1 << dir)) {
			_a_star_colour_subtree(x + DIRDX(dir), y + DIRDY(dir), colour);
		}
	}
}

/**
 * Attempt to reconnect the disconnected subtree rooted at (x, y) with any
 * other subtree, heuristically trying to move towards (px, py).
 *
 * Returns success.
 */
bool _a_star_reconnect(long px, long py, long x, long y) {
	// The colour of the subtree we're trying to re-attach. We'll attempt to
	// reattach this subtree to any available subtree with any other colour.
	long st_colour = asd[x][y].colour;

	// The coordinates of the node we're currently visiting
	long xx, yy;

	// The coordinates of the next node to visit
	long xxx, yyy;

	// The coordinates of the tree where the disconnected subtree has been
	// attached
	long ax, ay;

	long dir;

	bool path_found = false;

	#ifdef DIJKSTRA
		// A simple circular buffer queue. The head points to the next free
		// space, the tail points to the next item to remove. When the head
		// pointer and tail pointer are the same, the queue is empty.
		struct qentry { long x, y; };
		size_t qsize = 64; // Max size of the queue
		struct qentry *q = malloc(qsize * sizeof(struct qentry));
		assert(q);
		size_t qhead = 0; // Index of the head of the queue
		size_t qtail = 0; // Index of the tail of the queue

		#define QPUSH(xx, yy) do { \
			/* Enlarge the queue if full. */ \
			if (qhead + 1 == qtail || qhead + 1 - qsize == qtail) { \
				q = realloc(q, qsize * 2 * sizeof(struct qentry)); \
				assert(q); \
				/* When the tail is after the head in memory the tail must \
				 * be shifted towards the new end of the buffer. */ \
				if (qhead < qtail) { \
					/* Note: memcpy is safe since doubling the length of the buffer \
					 * guaruntees a non-overlapping copy. */ \
					memcpy(q + qsize + qtail, \
					       q + qtail, \
					       (qsize - qtail) * sizeof(struct qentry)); \
					qtail += qsize; \
				} \
				qsize *= 2; \
			} \
			/* Insert the new value. */ \
			q[qhead].x = (xx);\
			q[qhead].y = (yy);\
			/* Advance the head. */ \
			qhead++; \
			if (qhead >= qsize) \
				qhead -= qsize; \
		} while(0)

		#define QPOP(xx, yy) do { \
			assert(qhead != qtail); \
			/* Extract the value at the tail. */ \
			(xx) = q[qtail].x; \
			(yy) = q[qtail].y; \
			/* Advance the tail. */ \
			qtail++; \
			if (qtail >= qsize) \
				qtail -= qsize; \
		} while(0)

		#define QEMPTY() (qtail == qhead)

	#elif defined(A_STAR)
		int cmp_pri(pqueue_pri_t next, pqueue_pri_t curr) {
			return (next < curr);
		}
		pqueue_pri_t get_pri(void *a) {
			return ((struct a_star_data *) a)->pri;
		}
		void set_pri(void *a, pqueue_pri_t pri) {
			((struct a_star_data *) a)->pri = pri;
		}
		size_t get_pos(void *a) {
			return ((struct a_star_data *) a)->pos;
		}
		void set_pos(void *a, size_t pos) {
			((struct a_star_data *) a)->pos = pos;
		}
		long distance(long x, long y) {
			long z = 0;
			if (sign(x) == sign(y)) {
				if (x < y && sign(x) < 0) {
					x -= y;
					z -= y;
					y -= y;
				} else {
					y -= x;
					z -= x;
					x -= x;
				}
			}
			return abs(x) + abs(y) + abs(z);
		}

		pqueue_t *pq;
		size_t q_initial_size = 64;
		pq = pqueue_init(64, cmp_pri, get_pri, set_pri, get_pos, set_pos);
		assert(pq);

		#define QPUSH(x_, y_) do { \
			asd[(x_)][(y_)].pri = distance((x_), (y_)); \
			pqueue_insert(pq, &(asd[(x_)][(y_)])); \
		} while (0)

		#define QPOP(x_, y_) do { \
			struct a_star_data *entry = pqueue_pop(pq); \
			(x_) = (entry - &(asd[0][0])) / 341; \
			(y_) = (entry - &(asd[0][0])) % 341; \
		} while (0)

		#define QEMPTY() (pqueue_size(pq) == 0)
	#else
		#define QPUSH(x, y)
		#define QPOP(x, y)
		#define QEMPTY() 0
	#endif

	// Start the search at the root of the disconnected subtree
	QPUSH(x, y);
	asd[x][y].visited = a_star_cur_visit_num;
	while (!QEMPTY()) {
		QPOP(xx, yy);

		// If we've reached another subtree, terminate now
		if (asd[xx][yy].colour != -1 && asd[xx][yy].colour != st_colour) {
			path_found = true;
			break;
		}

		for (dir = 0; dir < 6; dir++) {
			xxx = xx + DIRDX(dir);
			yyy = yy + DIRDY(dir);
			// Search for routes via neighbours which are connected back to
			// this node (and which haven't already been visited). Note we also
			// don't allow searching the nodes at the extreme edges of the
			// system as a bodge to ensure that we can always access any
			// visited node's neighbours without having to perform a bounds
			// check.
			if (xxx > 0 && xxx < 340 && yyy > 0 && yyy < 340
			    && !fault[xxx][yyy][OPP(dir)]
			    && asd[xxx][yyy].visited != a_star_cur_visit_num) {
				// Mark the neighbour as visited
				asd[xxx][yyy].visited = a_star_cur_visit_num;

				// Indicate that the neighbour was first visited from this node
				asd[xxx][yyy].nx = xx;
				asd[xxx][yyy].ny = yy;

				// Continue searching from that neighbour
				QPUSH(xxx, yyy);
			}
		}
	}

	// Fail if no path was found
	if (!path_found)
		return false;

	// Work backwards along the discovered path to re-connect the disconnected
	// subtree.
	ax = xx;
	ay = yy;
	while (xx != x || yy != y) {
		xxx = asd[xx][yy].nx;
		yyy = asd[xx][yy].ny;

		// If we're stepping on the disconnected tree, make the previous step
		// in the newly inserted path the new parent of this node we're
		// stepping on. This will prevent a cycle being formed.
		if (asd[xxx][yyy].colour == st_colour) {
			for (dir = 0; dir < 6; dir++) {
				long px = xxx + DIRDX(dir);
				long py = yyy + DIRDY(dir);
				// Remove all routes pointing at this node (a new route into
				// that node will be added next at node xx yy).
				asd[px][py].route &= ~(1 << OPP(dir));
			}
		}

		// Add a route along this subtree
		asd[xx][yy].route |= 1 << XYTODIR(xxx - xx, yyy - yy);

		if (xx>xmax)
			xmax=xx;
		else if (xx<xmin)
			xmin=xx;
		if (yy>ymax)
			ymax=yy;
		else if (yy<ymin)
			ymin=yy;

		// Move to the next hop
		xx = xxx;
		yy = yyy;
	}

	// Colour-in reconnected subtree to give it the same colour
	// as the tree it has been connected to. This means if the search to
	// reconnect parent tree passes through this newly attached tree segment,
	// it knows it hasn't reached a new destination.
	_a_star_colour_subtree(ax, ay, asd[ax][ay].colour);

	#ifdef DIJKSTRA
		free(q);
	#elif defined(A_STAR)
		pqueue_free(pq);
	#endif

	#undef QPUSH
	#undef QPOP
	#undef QEMPTY

	return true;
}


/**
 * Take the fixed-up path in the asd datastructure and translate it for the
 * 'map' datastructure. The initial caller is responsible for initialising the
 * map value for the x, y given.
 */
void _a_star_asd_to_map(long x, long y) {
	long dir;
	long xx, yy;

	// Mark the node if it was a sink
	if (asd[x][y].route & (1 << SINK))
		map[x][y] |= 8;

	for (dir = 0; dir < 6; dir++) {
		if (asd[x][y].route & (1 << dir)) {
			xx = x + DIRDX(dir);
			yy = y + DIRDY(dir);

			// Add the route to this node
			map[x][y] |= POW[DIM(dir)];

			// Initially reset the map table to just include the pointer back
			// into this node.
			map[xx][yy] = POW[DIM(dir)];

			_a_star_asd_to_map(xx, yy);
		}
	}
}


/**
 * Post-process a set of routes such that all faults are avoided.
 *
 * Returns success.
 */
bool a_star(void) {
	struct a_star_disconnection_list *disconnection;

	// Find the full set of disconnected subtrees and initialise the asd data
	// structure along with the a_star_disconnections list.
	a_star_cur_visit_num++;
	_a_star_init(0, 170, 170, next_a_star_colour++);

	// Re-connect each disconnected tree segment
	while (a_star_disconnections) {
		disconnection = a_star_disconnections;

		a_star_cur_visit_num++;
		if (!_a_star_reconnect(disconnection->px, disconnection->py,
		                       disconnection->x, disconnection->y)) {
			return false;
		}

		// Move onto the next disconnection
		a_star_disconnections = disconnection->next;
		free(disconnection);
	}

	// Put the newly fixed up route into the map datastructure, ready for later
	// analysis.
	map[170][170] = 0;
	_a_star_asd_to_map(170, 170);

	return true;
}


/**
* Updates the map with the last created branch of the route
*/
void mark(){
	long i;

	for (i=0; i<hop; i++){
		if ((map[nodeX[i]][nodeY[i]]&POW[direc[i]])==0)
			map[nodeX[i]][nodeY[i]]+=POW[direc[i]]; // next hop
		if (i>0 && (map[nodeX[i]][nodeY[i]]&POW[direc[i-1]])==0)
			map[nodeX[i]][nodeY[i]]+=POW[direc[i-1]]; // previous hop (to calculate routing table entries correctly)
	}
	hop=0; // just in case
}



/**
* checks whether a greedy oblivious route can be created and if so, creates it.
* sx, sy: source coordinates.
* hx, hy, hw: hops per dimension.
* d0: dimension to start moving through.
* last: true if this check is verifying the last route which will be tried
*       before giving up. If false, behaviour is normal. If true and A_STAR is
*       defined, the route will be inserted anyway, ignoring any faulty links
*       along the way, to be fixed up later by the A_STAR post-processing step.
* returns -1 if no route was found. other value (1?) if a route was created.
*/
long check(long sx, long sy, long hx, long hy, long hw, long d0, bool last){
	long h[3], inc[3];
	long i, x, y, d;
	long px,py;	// coordinates of a proxy.
	long hop_copy;
	bool last_proxy;

	h[0]=hx;
	h[1]=hy;
	h[2]=hw;

	for(i=0; i<3; i++)
		if(h[i]>0)
			inc[i]=1;
		else
			inc[i]=-1;

	x=sx;
	y=sy;
	d=d0;

	// let's check whether we can reach destination
	while(h[0]!=0 || h[1]!=0 || h[2]!=0) {  // not at the destination yet
		// if (kk)
			// printf(".");
		// if (sx==166 && sy==169)
			// printf("%d(%d)\n",h[1],inc[1]);

#ifdef ZIGZAG
		if (fault[x][y][DIR(d,inc[d])]!=0 || h[d]==0)
            // cannot continue in this direction, check the other dimensions
			if(h[(d+1)%3]!=0 && fault[x][y][DIR((d+1)%3,inc[(d+1)%3])]==0) // can continue through next dimension?
				d=(d+1)%3;
			else if(h[(d+2)%3]!=0 && fault[x][y][DIR((d+2)%3,inc[(d+2)%3])]==0) // can continue through next dimension?
				d=(d+2)%3;
			else{	// no suitable step forward. route not created. should check for a proxy here!!!!
#else
        if (h[d]==0){
            // cannot continue in this direction, check the other dimensions
			if(h[(d+1)%3]!=0) // can continue through next dimension?
				d=(d+1)%3;
			else if(h[(d+2)%3]!=0) // can continue through next dimension?
				d=(d+2)%3;
        }

        if (fault[x][y][DIR(d,inc[d])]!=0){	// no suitable step forward. route not created. should check for a proxy here!!!!
#endif
 // ZIGZAG
            if (proxy<MAX_PROXY){
//                printf("Using PROXY #%ld from <%ld,%ld> to <%ld, %ld>\n", proxy,x,y,x+h[0]+h[2],y+h[1]+h[2]);
                hop_copy=hop;
                proxy ++;
                last_proxy = (proxy == MAX_PROXY);
                if (h[0]==0){
                    px=x+((h[2])/2)+inc[2];
                    py=y+((h[1])/2)+inc[1]+((h[2])/2)+inc[2];
//                    printf("Try node <%ld, %ld>\n", px,py);
                    // printf("0. <%d,%d,%d> Proxying: [%d,%d]<%d, %d, %d>, [%d,%d]<%d, %d, %d>, proxy=%d, hop=%d|%d\n",h[0],h[1],h[2],
                        // x,y,h[0],(h[1]/2)+inc[1],(h[2]/2)+inc[2],px,py,h[0], h[1]-((h[1]/2)+inc[1]), h[2]-((h[2]/2)+inc[2]),proxy,hop,hop_copy);
                    if(check(x,y,h[0],(h[1]/2)+inc[1],(h[2]/2)+inc[2],(6-d)%3,last&&last_proxy)!=-1 &&
                       check(px,py,h[0], h[1]-((h[1]/2)+inc[1]), h[2]-((h[2]/2)+inc[2]),d,last&&last_proxy)!=-1) // Found a valid proxy!!!!
                        return 1;
//                    printf("    ... but didn't work\n");
                    hop=hop_copy;
                }

                if (h[1]==0){
                    px=x+((h[0]/2)+inc[0])+((h[2]/2)+inc[2]);
                    py=y+((h[2]/2)+inc[2]);
//                    printf("Try node <%ld, %ld>\n", px,py);
                    // printf("1. <%d,%d,%d> Proxying: [%d,%d]<%d, %d, %d>, [%d,%d]<%d, %d, %d>, proxy=%d, hop=%d|%d\n",h[0],h[1],h[2],
                        // x,y,(h[0]/2)+inc[0],h[1],(h[2]/2)+inc[2],px,py, h[0]-((h[0]/2)+inc[0]), h[1], h[2]-((h[2]/2)+inc[2]),proxy,hop,hop_copy);
                    if(check(x,y,(h[0]/2)+inc[0],h[1],(h[2]/2)+inc[2],(5-d)%3,last&&last_proxy)!=-1 &&
                       check(px,py,h[0]-((h[0]/2)+inc[0]), h[1], h[2]-((h[2]/2)+inc[2]),d,last&&last_proxy)!=-1) // Found a valid proxy!!!!
                        return 1;
                    hop=hop_copy;
                }

                if (h[2]==0){
                    px=x+((h[0]/2)+inc[0]);
                    py=y+((h[1]/2)+inc[1]);
//                    printf("Try node <%ld, %ld>\n", px,py);
                    // printf("2. <%d,%d,%d> Proxying: [%d,%d]<%d, %d, %d>, [%d,%d]<%d, %d, %d>, proxy=%d, hop=%d|%d\n",h[0],h[1],h[2],
                        // x,y,(h[0]/2)+inc[0],(h[1]/2)+inc[1],h[2],px,py,h[0], h[0]-((h[0]/2)+inc[0]), h[1]-((h[1]/2)+inc[1]), h[2], proxy,hop,hop_copy);
                    if(check(x,y,(h[0]/2)+inc[0],(h[1]/2)+inc[1],h[2],(4-d)%3,last&&last_proxy)!=-1 &&
                       check(px,py,h[0]-((h[0]/2)+inc[0]), h[1]-((h[1]/2)+inc[1]), h[2],d,last&&last_proxy)!=-1) // Found a valid proxy!!!!
                        return 1;
                    hop=hop_copy; // comment removed for consistency, we may not always enter this if body
                }
                proxy--;
            }
            // If that A* algorithm is to be used and this is the last route to
            // be attempted, we will simply continue and ignore the fault (to
            // let it get patched up by A* later.
            #ifdef A_STAR
            if (!last) {
            #endif
                hop=0; // just in case
                return -1;
            #ifdef A_STAR
            } else {
                // Record that the A_STAR algorithm will be required to patch
                // up a broken route.
                a_star_required++;

                // Make sure we're about to advance down a valid path since the zig-zag heuristic may not leave 'd' on a valid dimension if faults are present.
                if (h[d]==0){
                    // cannot continue in this direction, check the other dimensions
                    if(h[(d+1)%3]!=0) // can continue through next dimension?
                        d=(d+1)%3;
                    else if(h[(d+2)%3]!=0) // can continue through next dimension?
                        d=(d+2)%3;
                }
            }
            #endif
		}
		// if (kk)
			// printf("[%d, %d]<%d>\n",x,y,d);

		nodeX[hop]=x;
		nodeY[hop]=y;
		direc[hop]=d;

		hop++;
		if (hop>=MAX_DST){
			printf("Exceeded maximum distance %ld, %ld!!!\n    %ld,%ld,%ld hops to go!!!\n", sx,sy,h[0],h[1],h[2]);
			 for (i=0; i<MAX_DST; i++)
				 printf("%ld, %ld, %ld, %ld\n",i,nodeX[i], nodeY[i],direc[i]);
			exit(-1);
		}

		//if we are here we are going to advance through d
		if (d==0 || d==2)
			x+=inc[d];
		if (d==1 || d==2)
			y+=inc[d];

		h[d]-=inc[d];
    }
	// if(kk)
		// printf("succesful (%d)!!!\n",hop);

	return 1;
}

/**
 * Filling the routes using DOR
 */
void dor(long f){
	long i,d,bkt;
	a_star_required = 0;

    for (bkt=0;bkt<BKT_NUM;bkt++){
        for (i=0;i<bucket[bkt];i++){
            connected[bkt][i]=1;
            proxy=0;
            hop=0;
            // kk=0;
            // if (p[0][i]==0 && p[1][i]==0){
                // printf("%d -- ",p[2][i]);
                // kk=1;
            // }

#ifdef REVERSEDIM
            if ((p[bkt][i][0]==0) || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], 0, false)==-1) { // XYW not possible
                // printf("Trying YWX!\n");
                // if (kk)
                    // printf("d=0 failed, ");
                // Try doing the dimensions in a different order
                hop=0;static int A[3][2] = { { 1, 2 }, { 3, 4 }, { 5, 6 } };
                proxy=0;
                if ((p[bkt][i][1]==0) || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], 1, false)==-1) { // YWX not possible
                    // printf("Trying WXY!\n");
                    // if (kk)
                        // printf("d=1 failed, ");
                    hop=0;
                    proxy=0;
                    // If A_STAR is being used to patch things up, on this last
                    // possible dimension ordering, always attempt to check.
                    if (
                        #ifndef A_STAR
                        (p[bkt][i][2]==0) ||
                        #endif
                        check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], 2, true)==-1) { // WXY not possible
                        // if (kk)
                            // printf("d=2 failed, ");
                        connected[bkt][i]=0;
                        disconnected2++;
                        continue;
                    }
                }
            }
#else
            if (p[bkt][i][0]!=0)
                d=0;
            else if (p[bkt][i][1]!=0)
                d=1;
            else
                d=2;

            // Given that this is the only attempt, set 'last' to allow A_STAR
            // to fix it up if enabled.
            if(check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, true)==-1) { // XYW not possible
                connected[bkt][i]=0;
                disconnected2++;
                continue;
            }
#endif
            mark();
        }
    }
}

/**
 * Filling the routes using longest first
 */
void lf(long f){
	long i,bkt;
	long d;
	a_star_required = 0;

    for (bkt=0;bkt<BKT_NUM;bkt++){
        for (i=0;i<bucket[bkt];i++){
            connected[bkt][i]=1;
            proxy=0;
            hop=0;

            if (abs(p[bkt][i][0])>abs(p[bkt][i][1]) && abs(p[bkt][i][0])>abs(p[bkt][i][2]))
                d=0;
            else if (abs(p[bkt][i][1])>abs(p[bkt][i][0]) && abs(p[bkt][i][1])>abs(p[bkt][i][2]))
                d=1;
            else
                d=2;

#ifdef REVERSEDIM
            if (p[bkt][i][d]==0 || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, false)==-1) { // XYW not possible
                // printf("Trying YWX!\n");
 // Try doing the dimensions in a different order
                proxy=0;
                hop=0;
                if (p[bkt][i][(d+1)%3]==0 || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+1)%3, false)==-1) { // YWX not possible
                    // printf("Trying WXY!\n");
                    proxy=0;
                    hop=0;

                    // If A_STAR is being used to patch things up, on this last
                    // possible dimension ordering, always attempt to check.
                    if (
                        #ifndef A_STAR
                        p[bkt][i][(d+2)%3]==0 ||
                        #endif
                        check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+2)%3, true)==-1) { // WXY not possible
                        connected[bkt][i]=0;
                        disconnected2++;
                        continue;
                    }
                }
            }
#else
            // Given that this is the only attempt, set 'last' to allow A_STAR
            // to fix it up if enabled.
            if (check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, true)==-1) {
                connected[bkt][i]=0;
                disconnected2++;
                continue;
            }
#endif

            mark();
        }
	}
}

/**
* Insertion sort:
*/
void insertionsort(long f) {
	long i, j,bkt;
	long x,y,w;

    for (bkt=0;bkt<BKT_NUM;bkt++){
        for (i=0;i<bucket[bkt];i++){
            j=i;
            while (j>0 && (abs(p[bkt][j-1][0])+abs(p[bkt][j-1][1])+abs(p[bkt][j-1][2])) > abs(p[bkt][j][0])+abs(p[bkt][j][1])+abs(p[bkt][j][2])){
                x=p[bkt][j-1][0];
                y=p[bkt][j-1][1];
                w=p[bkt][j-1][2];

                p[bkt][j-1][0]=p[bkt][j][0];
                p[bkt][j-1][1]=p[bkt][j][1];
                p[bkt][j-1][2]=p[bkt][j][2];

                p[bkt][j][0]=x;
                p[bkt][j][1]=y;
                p[bkt][j][2]=w;
                j--;
            }
        }
    }
}

/**
 * Sorts the destinations from the closest to the farest
 */
void bubblesort(long f) {
	long i, stop=1,bkt;
	long x,y,w;

    for (bkt=0;bkt<BKT_NUM;bkt++){
        while(stop!=0){
            stop=0;
            for (i=0;i<bucket[bkt];i++){
                if(abs(p[bkt][i][0])+abs(p[bkt][i][1])+abs(p[bkt][i][2]) > abs(p[bkt][i+1][0])+abs(p[bkt][i+1][1])+abs(p[bkt][i+1][2])){
                    x=p[bkt][i][0];
                    y=p[bkt][i][1];
                    w=p[bkt][i][2];

                    p[bkt][i][0]=p[bkt][i+1][0];
                    p[bkt][i][1]=p[bkt][i+1][1];
                    p[bkt][i][2]=p[bkt][i+1][2];

                    p[bkt][i+1][0]=x;
                    p[bkt][i+1][1]=y;
                    p[bkt][i+1][2]=w;
                    stop=1;
                }
            }
        }
    }
}

/**
 * Optimized using shortest paths.
 */
void opt(long f){
	long i,j,k,bkt;
	long inc, res;
	long a,b,d;
	long x2,y2,ox,oy;
	long h[3];
	long c1,c2;		// index of the coordinates we are searching through
	long d1,d2,e1,e2,f1,f2;	// coefficients for p and a for the if and else cases for x and y
	a_star_required = 0;
#ifdef BSORTING
	bubblesort(f);
#endif
#ifdef ISORTING
	insertionsort(f);
#endif

	for (bkt=0;bkt<BKT_NUM;bkt++){
        for (i=0;i<bucket[bkt];i++){
            x=170+p[bkt][i][0]+p[bkt][i][2];
            y=170+p[bkt][i][1]+p[bkt][i][2];
            map[x][y]=0;
        }
	}

	for (bkt=0;bkt<BKT_NUM;bkt++){
        for (i=0;i<bucket[bkt];i++){
			connected[bkt][i]=1;
			proxy=0;
			hop=0;

			ox=170;
			oy=170;
			x=170+p[bkt][i][0]+p[bkt][i][2];
			y=170+p[bkt][i][1]+p[bkt][i][2];

			// printf("%d, %d: ",x,y);
					// printf("%d,%d,%d\n",p[0][i],p[1][i],p[2][i]);
			// this hierarchy of ifs can be reduced even more. Not done so to have each separated.
			if (x>170 && y<170){	// S----*
				c1=0;	c2=1;		// |    |
				d1=-1;	d2=0;		// *----d
				e1=-1;	e2=1;
				f1=1;	f2=1;
			}
			else if (x<170 && y>170){	// d----*
				c1=0;	c2=1;			// |    |
				d1=-1;	d2=0;			// *----S
				e1=-1;	e2=-1;
				f1=-1;	f2=-1;
			}
			else if (x>170 && y>170 && x>y){	//   *----d
				c1=0;	c2=2;					//  /    /
				d1=0;	d2=-1;					// S----*
				e1=1;	e2=-1;
				f1=0;	f2=-1;
			}
			else if (x<170 && y<170 && x<y){	//   *----S
				c1=0;	c2=2;					//  /    /
				d1=0;	d2=1;					// d----*
				e1=1;	e2=1;
				f1=0;	f2=1;
			}
			else if (x>170 && y>170 && x<y){	//    d
				c1=1;	c2=2;					//  / |
				d1=1;	d2=-1;					// *  *
				e1=0;	e2=-1;					// | /
				f1=-1;	f2=0;					// S
			}
			else if (x<170 && y<170 && x>y){	//    S
				c1=1;	c2=2;					//  / |
				d1=1;	d2=1;					// *  *
				e1=0;	e2=1;					// | /
				f1=1;	f2=0;					// d
			}
			else { // (x==170 || y == 170 || x=y) i.e. the destination is in a straight line (in either X, Y or D),

				if (abs(p[bkt][i][0])>abs(p[bkt][i][1]) && abs(p[bkt][i][0])>abs(p[bkt][i][2]))
					d=0;
				else if (abs(p[bkt][i][1])>abs(p[bkt][i][0]) && abs(p[bkt][i][1])>abs(p[bkt][i][2]))
					d=1;
				else
					d=2;

#ifdef REVERSEDIM
                if (p[bkt][i][d]==0 || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, false)==-1) { // XYW not possible
                    // printf("Trying YWX!\n");
     // Try doing the dimensions in a different order
                    proxy=0;
                    hop=0;
                    if (p[bkt][i][(d+1)%3]==0 || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+1)%3, false)==-1) { // YWX not possible
                        // printf("Trying WXY!\n");
                        proxy=0;
                        hop=0;
                        // If A_STAR is being used to patch things up, on this last
                        // possible dimension ordering, always attempt to check.
                        if (p[bkt][i][(d+2)%3]==0 || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+2)%3, false)==-1) { // WXY not possible
                            connected[bkt][i]=0;
                            disconnected2++;
                            proxy=0;
                            hop=0;
//                            printf("opt1: %d, %d, %d -- d:%d\n",p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d);
                            continue;
                        }
                    }
                }
#else
                if (check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, false)==-1) {
                    connected[bkt][i]=0;
                    disconnected2++;
                    proxy=0;
                    hop=0;
                    continue;
                }
#endif
				goto there;
			}

			for(a=1; a<abs(p[bkt][i][c1])+abs(p[bkt][i][c2]); a++){
				if (a>abs(p[bkt][i][c1])){
					x2=x+(d1*p[bkt][i][c1])+(d2*a);
					y2=y+(e1*p[bkt][i][c1])+(e2*a);
				}
				else {
					x2=x+(e1*e2*a);
					y2=y+(d1*d2*a);
				}
				// printf("%d, %d,%d\n",a,x2,y2);
				for (b=0;b<=min(a,min(abs(p[bkt][i][c1]),min(abs(p[bkt][i][c2]), abs(p[bkt][i][c1])+abs(p[bkt][i][c2])-a)));b++){
					// map[x2][y2]+=64;
	#ifdef ENTRY
					if (map[x2][y2]==3 || (map[x2][y2]>4 && map[x2][y2]<16))		// we found a part of the path
	#else
					if (map[x2][y2]>CONNECTION && map[x2][y2]<16)
	#endif
					{	// Now, we have the closest route point (or the origin). Proceed using longest first.
						ox=x-x2;
						oy=y-y2; //distance from the closest point to the destination.
						if (ox*oy<=0){
							h[0]=ox;
							h[1]=oy;
							h[2]=0;
						}
						else if (abs(ox)>abs(oy)){
							h[0]=ox-oy;
							h[1]=0;
							h[2]=oy;
						}
						else {
							h[0]=0;
							h[1]=oy-ox;
							h[2]=ox;
						}
						// Now, we have the hops from the closest to the destination

						if (abs(h[0])>abs(h[1]) && abs(h[0])>abs(h[2])){
							d=0;
						}
						else if (abs(h[1])>abs(h[0]) && abs(h[1])>abs(h[2])){
							d=1;
						}
						else {
							d=2;
						}

#ifdef REVERSEDIM
						if ( (h[d]==0) || check(x2, y2, h[0], h[1], h[2], d, false)==-1){
                            // Try doing the dimensions in a different order
							proxy=0;
							hop=0;
							if ( (h[(d+1)%3]==0) || check(x2, y2, h[0], h[1], h[2], (d+1)%3, false)==-1){
								proxy=0;
								hop=0;
								if ( (h[(d+2)%3]==0) || check(x2, y2, h[0], h[1], h[2], (d+2)%3, false)==-1){
									proxy=0;
									hop=0;
									continue;
								}
							}
						}
#else
                        if ( check(x2, y2, h[0], h[1], h[2], d, false)==-1){
                            proxy=0;
                            hop=0;
                            continue;
                        }
#endif
						// if we are here, we have found a possible route.
						goto there;
					}
	//					map[x2][y2]=55;
					x2+=f1;
					y2+=f2;
				}
			}

			// if we reached here is that we haven't found any connection point

			// try direct route just in case. not really needed. route to
			// source should have been tried before. If A* is used, we will
			// keep this final attempt at finding a route regardless and let A*
			// patch up the gaps.
			if (abs(p[bkt][i][0])>abs(p[bkt][i][1]) && abs(p[bkt][i][0])>abs(p[bkt][i][2]))
					d=0;
			else if (abs(p[bkt][i][1])>abs(p[bkt][i][0]) && abs(p[bkt][i][1])>abs(p[bkt][i][2]))
					d=1;
			else
					d=2;

#ifdef REVERSEDIM
			if ((p[bkt][i][d]==0) || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, false)==-1) { // XYW not possible
				// printf("Trying YWX!\n");

				proxy=0;
				hop=0;
				if ((p[bkt][i][(d+1)%3]==0) || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+1)%3, false)==-1) { // YWX not possible
					// printf("Trying WXY!\n");
					proxy=0;
					hop=0;
					// If A_STAR is being used to patch things up, on this last
					// possible dimension ordering, always attempt to check.
					if (
					    #ifndef A_STAR
					    (p[bkt][i][(d+2)%3]==0) ||
					    #endif
					    check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+2)%3, true)==-1) {// WXY not possible
						connected[bkt][i]=0;
						disconnected2++;
						proxy=0;
                        hop=0;
//						printf("opt2: %d, %d, %d -- d:%d\n",p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d);
                    }
				}
			}
#else
            // This is a last ditch effort, A_STAR can patch up if required.
            if (check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, true)==-1) { // XYW not possible
			    connected[bkt][i]=0;
                disconnected2++;
                proxy=0;
                hop=0;
            }
#endif


	there: // damned goto!!! long double break doesn't work :(
			// printf("!!!\n");
			map[x][y]+=8;
			mark();
		}
	}
}

/**
 * Performs a search for routes in each destination's surroundings.
 */
void smart(long f, long top){
	long i,j,k,bkt;
	long inc, res;
	long a,b,c,d,sc;
	long x2,y2,ox,oy;
	long h[3];
	long sax[6]={1, 1, 0,-1,-1, 0};	// coefficient for a and x in each of the hexagonal directions of the search    	2 1 1
	long sbx[6]={0,-1,-1, 0, 1, 1}; // coefficient for b and x in each of the hexagonal directions of the search	  2		0
	long say[6]={0, 1, 1, 0,-1,-1}; // coefficient for a and y in each of the hexagonal directions of the search	3	d	0
	long sby[6]={1, 0,-1,-1, 0, 1}; // coefficient for b and y in each of the hexagonal directions of the search	3	  5
									//																				4 4 5

//	long kk;
	a_star_required = 0;

#ifdef BSORTING
	bubblesort(f);
#endif
#ifdef ISORTING
	insertionsort(f);
#endif

	for (bkt=0;bkt<BKT_NUM;bkt++){
        for (i=0;i<bucket[bkt];i++){
            x=170+p[bkt][i][0]+p[bkt][i][2];
            y=170+p[bkt][i][1]+p[bkt][i][2];
            map[x][y]=0;
        }
	}

	for (bkt=0;bkt<BKT_NUM;bkt++){
        for (i=0;i<bucket[bkt];i++){
            ox=170;
            oy=170;
            x=170+p[bkt][i][0]+p[bkt][i][2];
            y=170+p[bkt][i][1]+p[bkt][i][2];

    //		if (p[i][0]==0 && p[i][1]==-7 && p[i][2]==-12)
    //			kk=1;
    //		else
    //			kk=0;

            // Which dimension to start looking for hooks!
            if (abs(p[bkt][i][0])>abs(p[bkt][i][1]) && abs(p[bkt][i][0])>abs(p[bkt][i][2]))
                sc=0;
            else if (abs(p[bkt][i][1])>abs(p[bkt][i][0]) && abs(p[bkt][i][1])>abs(p[bkt][i][2]))
                sc=1;
            else
                sc=2;
            if (p[bkt][i][sc]>0)
                sc+=3;

            connected[bkt][i]=1;
            proxy=0;
            hop=0;

            for(a=1; a<=top; a++){
                for (b=0;b<a;b++){
                    for (c=0; c<6; c++){
                        x2=x+(sax[(sc+c)%6]*a)+(sbx[(sc+c)%6]*b);
                        y2=y+(say[(sc+c)%6]*a)+(sby[(sc+c)%6]*b);

                        // if (kk==1){
                            // if(a==1)
                                // printf("%d,%d ::: %d,%d!\n",x,y,x2,y2);
                            // map[x2][y2]+=64;
                        // }

                        if ( (x2<=340 && y2<=340 && x2>=0 && y2>=0) && // valid values for x2 and y2
                             ((x2==170 && y2==170) ||	// we found the source
#ifdef ENTRY
                              (map[x2][y2]==3 || (map[x2][y2]>4 && map[x2][y2]<16))		// we found a part of the path
#else
                              (map[x2][y2]>CONNECTION && map[x2][y2]<16)
#endif
                        )){
                            ox=x-x2;
                            oy=y-y2; //distance from the closest point to the destination.
                            if (ox*oy<=0){
                                h[0]=ox;
                                h[1]=oy;
                                h[2]=0;
                            }
                            else if (abs(ox)>abs(oy)){
                                h[0]=ox-oy;
                                h[1]=0;
                                h[2]=oy;
                            }
                            else {
                                h[0]=0;
                                h[1]=oy-ox;
                                h[2]=ox;
                            }
                            // Now, we have the hops from the closest to the destination

                            if (abs(h[0])>abs(h[1]) && abs(h[0])>abs(h[2])){
                                d=0;
                            }
                            else if (abs(h[1])>abs(h[0]) && abs(h[1])>abs(h[2])){
                                d=1;
                            }
                            else {
                                d=2;
                            }

#ifdef REVERSEDIM
                            if ( (h[d]==0) || check(x2, y2, h[0], h[1], h[2], d, false)==-1){
                                proxy=0;
                                hop=0;
                                if ( (h[(d+1)%3]==0) || check(x2, y2, h[0], h[1], h[2], (d+1)%3, false)==-1){
                                    proxy=0;
                                    hop=0;
                                    if ( (h[(d+2)%3]==0) || check(x2, y2, h[0], h[1], h[2], (d+2)%3, false)==-1){
                                        proxy=0;
                                        hop=0;
                                        continue;
                                    }
                                }
                            }
#else
                            if ( check(x2, y2, h[0], h[1], h[2], d, false)==-1){
                                proxy=0;
                                hop=0;
                                continue;
                            }
#endif
                            // if we are here, we have found a possible route.
                            goto here;
                        }
                    }
                }
            }
            proxy=0;
            hop=0;
            // If we are here this means we haven't find any possible hooking point. Try direct route
            if (abs(p[bkt][i][0])>abs(p[bkt][i][1]) && abs(p[bkt][i][0])>abs(p[bkt][i][2]))
                d=0;
            else if (abs(p[bkt][i][1])>abs(p[bkt][i][0]) && abs(p[bkt][i][1])>abs(p[bkt][i][2]))
                d=1;
            else
                d=2;

#ifdef REVERSEDIM
            if (p[bkt][i][d]==0 || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, false)==-1) { // XYW not possible
                // printf("Trying YWX!\n");
                proxy=0;
                hop=0;
                if (p[bkt][i][(d+1)%3]==0 || check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+1)%3, false)==-1) { // YWX not possible
                    // printf("Trying WXY!\n");
                    proxy=0;
                    hop=0;
                    // If A_STAR is being used to patch things up, on this last
                    // possible dimension ordering, always attempt to check.
                    if (
                        #ifndef A_STAR
                        p[bkt][i][(d+2)%3]==0 ||
                        #endif
                        check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], (d+2)%3, true)==-1) { // WXY not possible
                        connected[bkt][i]=0;
                        disconnected2++;
//                        printf("smart1: %d, %d, %d -- d:%d\n",p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d);
                    }
                }
            }
#else
            // This is the last attempt that will be made, fall back on A*
            if (check(170, 170, p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d, true)==-1) { // XYW not possible
                connected[bkt][i]=0;
                disconnected2++;
//                printf("smart2: %d, %d, %d -- d:%d\n",p[bkt][i][0], p[bkt][i][1], p[bkt][i][2], d);
            }
#endif

    here: // damned goto!!! long double break doesn't work :(
            map[x][y]+=8;
            mark();
        }
	}
}


/**
 * A routing algorithm which uses the a_star-related functions to perform the
 * complete routing process from scratch.
 */
void just_a_star(long f) {
	// Add all destinations to the asd datastructure and a_star_disconnections
	// list. This does the equivilent of the _a_star_init function for this
	// router.
	long i, bkt, x, y;
	struct a_star_disconnection_list *disconnection;
	a_star_required = 0; // We won't need to patch up these routes later!
	asd[170][170].colour = next_a_star_colour++;
	for (bkt=0;bkt<BKT_NUM;bkt++){
		for (i=0;i<bucket[bkt];i++){
			x = 170 + (p[bkt][i][0] + p[bkt][i][2]);
			y = 170 + (p[bkt][i][1] + p[bkt][i][2]);

			// Mark the node in asd.
			asd[x][y].colour = next_a_star_colour++;
			asd[x][y].route = 1 << SINK;
		}
	}

	// Use the graph search to find routes for each destination.
	for (bkt=0;bkt<BKT_NUM;bkt++){
		for (i=0;i<bucket[bkt];i++){
			x = 170 + (p[bkt][i][0] + p[bkt][i][2]);
			y = 170 + (p[bkt][i][1] + p[bkt][i][2]);
			a_star_cur_visit_num++;
			_a_star_reconnect(170, 170, x, y);
		}
	}

	// Translate the asd results into the map datastructure for analysis.
	map[170][170] = 0;
	_a_star_asd_to_map(170, 170);
}


/**
 * Flood-fill a non-zero-filled the map datastructure with 0s, traversing only
 * working links. Used by create() to check for disconnected regions in a fault
 * set.
 */
void flood_fill_map(long x, long y) {
	long dir;
	long xx, yy;

	if (map[x][y] == 0)
		return;
	map[x][y] = 0;

	for (dir = 0; dir < 6; dir++) {
		xx = x + DIRDX(dir);
		yy = y + DIRDY(dir);
		if (xx >= 0 && xx < 341 && yy >= 0 && yy < 341 && !fault[x][y][dir]) {
			flood_fill_map(xx, yy);
		}
	}
}


/**
 * Add a number of faults to the map. Used by create().
 */
void create_faults(long faults, long width, long height) {
	long x, y, k, hop;
	long ax, ay, pt;

	bool fully_connected_map = false;
	while (!fully_connected_map) {
		for (x=0; x<341; x++)
			for (y=0; y<341; y++){
				map[x][y]=-1; // Later set to 0 by flood-fill
				for (k=0; k<6; k++)
    	            fault[x][y][k]=0;
			}

		#ifndef SPINN_LINK_FAIL
			for (k=0; k<faults;k++){
				do {
					// close quarters to print map
					// ax=(rand()%25) + 158;
					// ay=(rand()%25) + 158;

					ax=(170 - (width/2)) + (rand()%width);
					ay=(170 - (height/2)) + (rand()%height);
					pt=rand()%6;
				} while (fault[ax][ay][pt]!=0);
				fault[ax][ay][pt]=1;
				fault[ax + DIRDX(pt)][ay + DIRDY(pt)][OPP(pt)]=1; \
			}
		#else
			// We will decrement k until the requisite number of faults has been
			// created
			k = faults;
			while (k > 0) {
				// Pick a 12x12 region in which a fault will be created
				ax=(170 - (width/2)) + ((rand()%(width / 12)) * 12);
				ay=(170 - (height/2)) + ((rand()%(height / 12)) * 12);

				// Each pair of states alternates as a simple state machine to kill
				// sequences of links in a machine.
				enum states {
					SW00_W01, W01_SW00, // Kill SW, Kill W & Move north
					S00_E11,  E11_S00,  // Kill S, Kill E & Move north east
					S10_SW00, SW00_S10, // Kill S & Move east, Kill SW
				} state;

				// Within that region, pick one of 9 possible spinnlinks to kill
				//
				//     1|   /6
				//      |__/
				//    8/ 4 |
				//    /   2|__
				//    |   /  5
				//   0|__/7
				//     3
				switch (rand() % 9) {
					case 0: x = ax + 0; y = ay + 0; state = SW00_W01; break;
					case 1: x = ax + 4; y = ay + 8; state = SW00_W01; break;
					case 2: x = ax + 8; y = ay + 4; state = SW00_W01; break;
					case 3: x = ax + 0; y = ay + 0; state = S10_SW00; break;
					case 4: x = ax + 4; y = ay + 8; state = S10_SW00; break;
					case 5: x = ax + 8; y = ay + 4; state = S10_SW00; break;
					case 6: x = ax + 8; y = ay + 8; state = S00_E11;  break;
					case 7: x = ax + 4; y = ay + 0; state = S00_E11;  break;
					case 8: x = ax + 0; y = ay + 4; state = S00_E11;  break;
					default: assert(false); break;
				}

				// Try again if we've picked a spinnlink which has already been killed
				switch (state) {
					case SW00_W01: pt=SOUTH_WEST; break;
					case S10_SW00: pt=SOUTH; break;
					case S00_E11:  pt=SOUTH; break;
					default: assert(false); break;
				}
				if (fault[x][y][pt])
					continue;

				// Kill all links in the spinnlink
				for (hop = 0; hop < 8 && k > 0; hop++, k--) {
					#define KILL(link) do { \
						fault[x][y][link]=1; \
						fault[x + DIRDX(link)][y + DIRDY(link)][OPP(link)]=1; \
					} while (0)

					switch (state) {
						case SW00_W01: KILL(SOUTH_WEST); x+=0; y+=0; state=W01_SW00; break;
						case W01_SW00: KILL(WEST);       x+=0; y+=1; state=SW00_W01; break;

						case S00_E11:  KILL(SOUTH);      x+=0; y+=0; state=E11_S00;  break;
						case E11_S00:  KILL(EAST);       x+=1; y+=1; state=S00_E11;  break;

						case S10_SW00: KILL(SOUTH);      x+=1; y+=0; state=SW00_S10; break;
						case SW00_S10: KILL(SOUTH_WEST); x+=0; y+=0; state=S10_SW00; break;

						default: assert(false); break;
					}

					#undef KILL
				}
			}

		#endif

		flood_fill_map(170, 170);

		fully_connected_map = true;
		for (x=0; x<341; x++) {
			for (y=0; y<341; y++) {
				if (map[x][y] != 0) {
					fully_connected_map = false;
					goto retry_fault_map;
				}
			}
		}
		retry_fault_map: continue;
	}
}


// Given an array, reverse its order in place in the given range.
// e.g.
// int my_array[10] = {...};
// REVERSE_ARRAY(my_array[, ], 0, 10)
#define REVERSE_ARRAY(arr_pre, arr_post, start, length) \
	do { \
		for (size_t l = (start), r = (start) + (length) - 1; l < r; l++, r--) { \
			arr_pre l arr_post ^= arr_pre r arr_post; \
			arr_pre r arr_post ^= arr_pre l arr_post; \
			arr_pre l arr_post ^= arr_pre r arr_post; \
		} \
	} while(0)


// Given an array, rotate it by the supplied number of steps.
// e.g.
// int my_array[10] = {...};
// ROTATE_ARRAY(my_array[, ], 3, 0, 10);
#define ROTATE_ARRAY(arr_pre, arr_post, steps, start, length) \
	do { \
		if (steps > 0) { \
			REVERSE_ARRAY(arr_pre, arr_post, (start), (steps)); \
			REVERSE_ARRAY(arr_pre, arr_post, (start) + (steps), (length) - (steps)); \
			REVERSE_ARRAY(arr_pre, arr_post, (start), (length)); \
		} \
	} while(0)


// Rotate the fault set around the network by the supplied distance. The width
// and height indicates the width and height of the rectangle centered on
// 170,170 in which faults should be rotated.
void rotate_faults(long dx, long dy, long width, long height) {
	for (int link = 0; link < 6; link++) {
		for (int x = 0; x < width; x++) {
			ROTATE_ARRAY(fault[x + 170 - (width/2)][,][link],
			             dy, 170 - (height/2), height);
		}
		for (int y = 0; y < height; y++) {
			ROTATE_ARRAY(fault[,][y + 170 - (height/2)][link],
			             dx, 170 - (width/2), width);
		}
	}
}

// Rotate the resource accumulations around the network by the supplied
// distance. The width and height indicates the width and height of the
// rectangle centered on 170,170 in which faults should be rotated.
void rotate_accum_map(long dx, long dy, long width, long height) {
	// accum_map
	for (int link = 0; link < 6; link++) {
		for (int x = 0; x < width; x++) {
			ROTATE_ARRAY(accum_map[x + 170 - (width/2)][,][link],
			             dy, 170 - (height/2), height);
		}
		for (int y = 0; y < height; y++) {
			ROTATE_ARRAY(accum_map[,][y + 170 - (height/2)][link],
			             dx, 170 - (width/2), width);
		}
	}
	
	// entry_map
	for (int x = 0; x < width; x++) {
		ROTATE_ARRAY(entry_map[x + 170 - (width/2)][,],
		             dy, 170 - (height/2), height);
	}
	for (int y = 0; y < height; y++) {
		ROTATE_ARRAY(entry_map[,][y + 170 - (height/2)],
		             dx, 170 - (width/2), width);
	}
}


#ifndef CENTROIDS
///* Generates a collection of f destinations distributed uniformly
long double create(long f, long width, long height){
	long dst, total=0;
	long i,j,k,bkt;

	long soma_dest=0;
	long cX[10], cY[10], cent_dest[10];
	long pt,ax,ay,r;

	for (long x = 0; x < 341; x++)
		for (long y = 0; y < 341; y++)
			map[x][y] = 0;

    for(i=0;i<BKT_NUM;i++){
        bucket[i]=0;
    }

	xmax=ymax=ymin=xmin=170;

	for (i=0;i<f;i++) {
#ifdef PUREUNIFORM
		long ox, oy;
		long h[3];

		do{
			x=(340 - (width/2))+(rand()%width);
			y=(340 - (height/2))+(rand()%height);
		}while ((x==170 && y==170) || map[x][y]!=0);

        ox=x-170;
        oy=y-170; //distance from the closest point to the destination.
        if (ox*oy<=0){
            h[0]=ox;
            h[1]=oy;
            h[2]=0;
        }
        else if (abs(ox)>abs(oy)){
            h[0]=ox-oy;
            h[1]=0;
            h[2]=oy;
        }
        else {
            h[0]=0;
            h[1]=oy-ox;
            h[2]=ox;
        }
        bkt=(abs(h[0])+abs(h[1])+abs(h[2]))/BKT_LEN;
        while(bucket[bkt]>=BKT_SIZE){
            bkt=(bkt+BKT_NUM-1)%BKT_NUM;    // == (bkt-1)%13, but returns a positive number
        }
        p[bkt][bucket[bkt]][0]=h[0];
        p[bkt][bucket[bkt]][1]=h[1];
        p[bkt][bucket[bkt]][2]=h[2];
        bucket[bkt]++;
#else
		do {
			// dst=1+(rand()%19);
			dst=1+(rand()%(width/2)); // XXX: assumes width == height...
			bkt=dst/BKT_LEN;
            while(bucket[bkt]>=BKT_SIZE){
                bkt=(bkt+BKT_NUM-1)%BKT_NUM;    // == (bkt-1)%13, but returns a positive number
            }

			switch (rand()%6) {
			case 0:
				p[bkt][bucket[bkt]][0]=rand()%(dst+1);	//X
				p[bkt][bucket[bkt]][1]=p[bkt][bucket[bkt]][0]-dst;	//Y
				p[bkt][bucket[bkt]][2]=0;		//W
				break;
			case 1:
				p[bkt][bucket[bkt]][0]=-(rand()%(dst+1));//X
				p[bkt][bucket[bkt]][1]=p[bkt][bucket[bkt]][0]+dst;	//Y
				p[bkt][bucket[bkt]][2]=0;		//W
				break;
			case 2:
				p[bkt][bucket[bkt]][0]=rand()%(dst+1);	//X
				p[bkt][bucket[bkt]][1]=0;		//Y
				p[bkt][bucket[bkt]][2]=dst-p[bkt][bucket[bkt]][0];	//W
				break;
			case 3:
				p[bkt][bucket[bkt]][0]=-(rand()%(dst+1));//X
				p[bkt][bucket[bkt]][1]=0;		//Y
				p[bkt][bucket[bkt]][2]=-(dst+p[bkt][bucket[bkt]][0]);	//W
				break;
			case 4:
				p[bkt][bucket[bkt]][0]=0;		//X
				p[bkt][bucket[bkt]][1]=rand()%(dst+1);	//Y
				p[bkt][bucket[bkt]][2]=dst-p[bkt][bucket[bkt]][1];	//W
				break;
			case 5:
				p[bkt][bucket[bkt]][0]=0;		//X
				p[bkt][bucket[bkt]][1]=-(rand()%(dst+1));//Y
				p[bkt][bucket[bkt]][2]=-(dst+p[bkt][bucket[bkt]][1]);	//W
				break;
			default:
				perror("Should not be here in create!!!\n");
				exit(-1);
			}
			x=170+p[bkt][bucket[bkt]][0]+p[bkt][bucket[bkt]][2];
			y=170+p[bkt][bucket[bkt]][1]+p[bkt][bucket[bkt]][2];

		} while ((x < 170 - (width/2)) || (x >= 170 + (width/2)) ||
		         (y < 170 - (height/2)) || (y >= 170 + (height/2)) ||
		         map[x][y]!=0);
#endif
		bucket[bkt]++;

		if (x>xmax)
			xmax=x;
		else if (x<xmin)
			xmin=x;
		if (y>ymax)
			ymax=y;
		else if (y<ymin)
			ymin=y;

		map[x][y]=8;
		total+=dst;
//		printf("%d: %d, %d, %d\n",dst,p[0][i],p[1][i],p[2][i]);
	}
	xmin=xmin-MAX_PROXY;
	xmax=xmax+MAX_PROXY;
	ymin=ymin-MAX_PROXY;
	ymax=ymax+MAX_PROXY;

	return ((long double)total)/f;
}
#else
///* Generates a collection of f destinations based on . Centroids
long double create(long f, long width, long height){
    static const long cent=CENTROIDS;
	long dst, total=0;
	long i,j,k,bkt;

	long soma_dest=0;
	long cX[20], cY[20], cent_dest[20];
	long pt,ax,ay,r;

	for (long x = 0; x < 341; x++)
		for (long y = 0; y < 341; y++)
			map[x][y] = 0;

	for(i=0;i<BKT_NUM;i++){
        bucket[i]=0;
    }

	xmax=ymax=ymin=xmin=170;

	//printf("1.place centroids\n");fflush(NULL);

	for (i=0; i<cent; i++){
		cent_dest[i]=0;
		dst=CENT_MIN_DIST+(rand()%CENT_RANGE);
		switch (rand()%6) {
			case 0:
				cX[i]=dst - (rand()%(dst+1));
				cY[i]=dst;
				break;
			case 1:
				cX[i]=dst;
				cY[i]=dst - (rand()%(dst+1));
				break;
			case 2:
				cX[i]=-(dst - (rand()%(dst+1)));
				cY[i]=-dst;
				break;
			case 3:
				cX[i]=-dst;
				cY[i]=-(dst - (rand()%(dst+1)));
				break;
			case 4:
				cX[i]=rand()%(dst+1);
				cY[i]=-(dst - cX[i]);
				break;
			case 5:
				cX[i]=-(rand()%(dst+1));
				cY[i]=dst + cX[i];
				break;
		}
	}

//printf("3.put nodes\n");fflush(NULL);

	for (i=0;i<f;i++) {
		if ((r=rand()%100) < 5*cent){
			//printf("centroids.\n");fflush(NULL);
			// destination around a centroid
			r=r/5;
			cent_dest[r]++;
			//printf("select dest location.\n");fflush(NULL);
			do {
				dst=ceil(rnd_exp(1.0/(1+(sqrt(cent_dest[r])/6.0))));
			//	printf("1.\n");fflush(NULL);
				if (dst>CENT_SIZE || dst<=0)
					dst=CENT_SIZE;  // for aprox. 2K (10%)// should be 16+ for 8K
			//	printf("2.\n");fflush(NULL);
				switch (rand()%6) {
				case 0: //+X -Y
					pt=rand()%(dst+1);
					ax=cX[r]+pt;	//X
					ay=cY[r]-dst+pt;	//Y
					break;
				case 1: //-X +Y
					pt=rand()%(dst+1);
					ax=cX[r]-pt;	//X
					ay=cY[r]+dst-pt;	//Y
					break;
				case 2: // +D +Y
					pt=rand()%(dst+1);
					ax=cX[r]+pt;	//X
					ay=cY[r]+dst;	//Y
					break;
				case 3: // -D -Y
					pt=rand()%(dst+1);
					ax=cX[r]-pt;	//X
					ay=cY[r]-dst;	//Y
					break;
				case 4: // +D +X
					pt=rand()%(dst+1);
					ax=cX[r]+dst;	//X
					ay=cY[r]+pt;	//Y
					break;
				case 5: // -D -X
					pt=rand()%(dst+1);
					ax=cX[r]-dst;	//X
					ay=cY[r]-pt;	//Y
					break;
				default:
					perror("Should not be here in create!!!\n");
					exit(-1);
				}
			//	printf("3.\n");fflush(NULL);
				x=170+ax;
				y=170+ay;
			} while (map[x][y]!=0);
			//printf("unknown.\n");fflush(NULL);
			if (sign(ax)==sign(ay)) { // X or Y + D
				dst=max(abs(ax),abs(ay));
				bkt=dst/BKT_LEN;
				while(bucket[bkt]>=BKT_SIZE){
					bkt=(bkt+BKT_NUM-1)%BKT_NUM;    // == (bkt-1)%13, but returns a positive number
				}
				if (abs(ax)>abs(ay)){
					p[bkt][bucket[bkt]][0]=ax-ay;	//X
					p[bkt][bucket[bkt]][1]=0;	//Y
					p[bkt][bucket[bkt]][2]=ay;		//W
				}
				else
				{
					p[bkt][bucket[bkt]][0]=0;	//X
					p[bkt][bucket[bkt]][1]=ay-ax;	//Y
					p[bkt][bucket[bkt]][2]=ax;		//W
				}
			}
			else {	// X + Y
				dst=abs(ax)+abs(ay);
                bkt=dst/BKT_LEN;
				while(bucket[bkt]>=BKT_SIZE){
					bkt=(bkt+BKT_NUM-1)%BKT_NUM;    // == (bkt-1)%13, but returns a positive number
				}
				p[bkt][bucket[bkt]][0]=ax;	//X
				p[bkt][bucket[bkt]][1]=ay;	//Y
				p[bkt][bucket[bkt]][2]=0;		//W
			}
		} else {
			// destination around the origin
			//printf("soma.\n");fflush(NULL);
			soma_dest++;
			do {
				dst=ceil(rnd_exp(1.0/(1+(sqrt(soma_dest)/6.0))));
				if (dst>SOMA_SIZE || dst<=0)
					dst=SOMA_SIZE;  // for aprox. 2K // should be 50+ for 8K
			//	printf("s: %d\n",dst);
                bkt=dst/BKT_LEN;
				while(bucket[bkt]>=BKT_SIZE){
					bkt=(bkt+BKT_NUM-1)%BKT_NUM;    // == (bkt-1)%13, but returns a positive number
				}
				switch (rand()%6) {
				case 0:
					p[bkt][bucket[bkt]][0]=rand()%(dst+1);	//X
					p[bkt][bucket[bkt]][1]=p[bkt][bucket[bkt]][0]-dst;	//Y
					p[bkt][bucket[bkt]][2]=0;		//W
					break;
				case 1:
					p[bkt][bucket[bkt]][0]=-(rand()%(dst+1));//X
					p[bkt][bucket[bkt]][1]=p[bkt][bucket[bkt]][0]+dst;	//Y
					p[bkt][bucket[bkt]][2]=0;		//W
					break;
				case 2:
					p[bkt][bucket[bkt]][0]=rand()%(dst+1);	//X
					p[bkt][bucket[bkt]][1]=0;		//Y
					p[bkt][bucket[bkt]][2]=dst-p[bkt][bucket[bkt]][0];	//W
					break;
				case 3:
					p[bkt][bucket[bkt]][0]=-(rand()%(dst+1));//X
					p[bkt][bucket[bkt]][1]=0;		//Y
					p[bkt][bucket[bkt]][2]=-(dst+p[bkt][bucket[bkt]][0]);	//W
					break;
				case 4:
					p[bkt][bucket[bkt]][0]=0;		//X
					p[bkt][bucket[bkt]][1]=rand()%(dst+1);	//Y
					p[bkt][bucket[bkt]][2]=dst-p[bkt][bucket[bkt]][1];	//W
					break;
				case 5:
					p[bkt][bucket[bkt]][0]=0;		//X
					p[bkt][bucket[bkt]][1]=-(rand()%(dst+1));//Y
					p[bkt][bucket[bkt]][2]=-(dst+p[bkt][bucket[bkt]][1]);	//W
					break;
				default:
					perror("Should not be here in create!!!\n");
					exit(-1);
				}
				x=170+p[bkt][bucket[bkt]][0]+p[bkt][bucket[bkt]][2];
				y=170+p[bkt][bucket[bkt]][1]+p[bkt][bucket[bkt]][2];
			} while (map[x][y]!=0);
		}
		bucket[bkt]++;
		// printf("bucket[%d]=%d\n",bkt,bucket[bkt]);
		if (x>xmax)
			xmax=x;
		else if (x<xmin)
			xmin=x;
		if (y>ymax)
			ymax=y;
		else if (y<ymin)
			ymin=y;

		map[x][y]=8;
		total+=dst;
//		printf("%d: %d, %d, %d\n",dst,p[0][i],p[1][i],p[2][i]);
	}
	xmin=xmin-MAX_PROXY; // to ensure we are printing everything
	xmax=xmax+MAX_PROXY;
	ymin=ymin-MAX_PROXY;
	ymax=ymax+MAX_PROXY;

	return ((long double)total)/f;
}
#endif // CENTROIDS

/// This doesn't work too well now that we have 2 direction for failures
long print_map(){
	long i,j,k,n,bkt;
	long inc, res=0, tab=0, use[3]={0,0,0};

#ifdef PRINT_DISCONNECTED
    for (bkt=0;bkt<BKT_NUM;bkt++)
        for (i=0;i<bucket[bkt];i++)
            map[170+p[bkt][i][0]+p[bkt][i][2]][170+p[bkt][i][1]+p[bkt][i][2]]=-1;
#endif

	for (j=ymin-arg; j<=ymax+arg; j++)
	{
		for (i=xmin-arg; i<=xmax+arg; i++)
		{
			n = (fault[i][j][0] || fault[i][j][3])
                + (2*(fault[i][j][1] || fault[i][j][4]))
                + (4*(fault[i][j][2] || fault[i][j][5]));
			switch(n){
				case 0:	printf("\x1b[0m"); // black/white
						break;
				case 1:	printf("\x1b[31m"); // red
						break;
				case 2:	printf("\x1b[32m"); // green
						break;
				case 3:	printf("\x1b[33m"); // red+green (yellow)
						break;
				case 4:	printf("\x1b[34m"); // blue
						break;
				case 5:	printf("\x1b[35m"); // blue + red (magenta)
						break;
				case 6:	printf("\x1b[36m"); // blue + green (cyan)
						break;
				case 7:	printf("\x1b[37m"); // blue + green + red (grey-ish)
						break;
			}

			if (i==170 && j==170)
				printf("X");
			else if (map[i][j]>=64) // disconnected node?? needs to be done!
				printf("#");
			else if (map[i][j]>7){
				printf("o");
				res++;
				tab++;
			}
			else if (map[i][j]==1){
				if ((fault[i][j][0] || fault[i][j][3])==0)
					printf("-");
				else
					printf("$");
				res++;
			}
			else if (map[i][j]==2){
				if ((fault[i][j][2] || fault[i][j][5])==0)
					printf("|");
				else
					printf("£");
				res++;
			}
			else if (map[i][j]==4){
				if ((fault[i][j][1] || fault[i][j][4])==0)
					printf("\\");
				else
					printf("&");
				res++;
			}
			else if (map[i][j]==0)
				printf(".");
			else if (map[i][j]==-1) // disconnected node?? needs to be done!
				printf("@");

			else {
				printf("*");
				res++;
				tab++;
			}
			if ((map[i][j]&1)!=0)
				use[0]++;
			if ((map[i][j]&2)!=0)
				use[1]++;
			if ((map[i][j]&4)!=0)
				use[2]++;
		}
		printf("\n");
	}
	printf("\nres, %ld, %ld, %ld, %ld, %ld\n",res,tab,use[0],use[1],use[2]);
	return res;
}

///* Covers the communication map counting the resources used.
long resources(){
	long i,j;
	long res=0;

	for (i=xmin; i<=xmax; i++)
		for (j=ymax; j>=ymin; j--)
			if (map[i][j])
				res++;
	return res-1; // source should not be counted
}

long entries(){
	long i,j;
	long res=0;

	for (i=xmin; i<=xmax; i++)
		for (j=ymax; j>=ymin; j--)
			if (map[i][j]!=0 && map[i][j]!=1 && map[i][j]!=2 && map[i][j]!=4)
				res++;
	return res+1; // source is not counted
}

long dir(long r){
	long i,j;
	long res=0;

	for (i=xmin; i<=xmax; i++)
		for (j=ymax; j>=ymin; j--)
			if ((map[i][j] & r)!=0)
				res++;
	if (res>1)
		res--;

	return res;
}

long accum_map_reset(void) {
	long x, y, link;
	for (x = 0; x < 341; x++)
		for (y = 0; y < 341; y++) {
			for (link = 0; link < 6; link++)
				accum_map[x][y][link] = 0;
			entry_map[x][y] = 0;
		}
}

/**
 * Count the number of times each link is used and the number of routing
 * entries at each router.
 *
 * @param direction The direction of the hop that took us from the parent to this node.
 * @param x The x coordinate of the node to traverse.
 * @param y The y coordinate of the node to traverse.
 * @param colour The colour of the subtree this node is in.
 */
void accum_map_update(long direction, long x, long y) {
	long c;

	asd[x][y].visited = a_star_cur_visit_num;

	// A macro which recurses into children in a given direction as long as
	// they are not the parent and as long as the child actually has something
	// on it (i.e. we're actually connected to it).
	#define RECURSE_IF_CHILD(dir, xx, yy) \
		do { \
			if ((map[(xx)][(yy)] & (POW[DIM(dir)] | 0x8)) \
			    && asd[(xx)][(yy)].visited != a_star_cur_visit_num) { \
				accum_map[x][y][(dir)]++; \
				accum_map_update((dir), (xx), (yy)); \
			} \
		} while (0)

	if (map[x][y] & 0x1) {
		RECURSE_IF_CHILD(WEST, x-1, y);
		RECURSE_IF_CHILD(EAST, x+1, y);
	}
	if (map[x][y] & 0x2) {
		RECURSE_IF_CHILD(SOUTH, x, y-1);
		RECURSE_IF_CHILD(NORTH, x, y+1);
	}
	if (map[x][y] & 0x4) {
		RECURSE_IF_CHILD(SOUTH_WEST, x-1, y-1);
		RECURSE_IF_CHILD(NORTH_EAST, x+1, y+1);
	}
	
	// Count non-straight-line segments as routing entries
	if (map[x][y] != 0x1 && map[x][y] != 0x2 && map[x][y] != 0x4) {
		entry_map[x][y]++;
	}

	#undef RECURSE_IF_CHILD
}

int main(int argc, char *argv[]){
	long width = 48;
	long height = 48;

	long num_repeats = width * height * NUM_REPEATS_SCALE;

	if (argc != 2) {
		fprintf(stderr, "ERROR: expect the random seed as an argument\n");
		return 1;
	}
	int seed = atoi(argv[1]);
	
	long dests[] = {16};
	#define NUM_DEST_VALUES (sizeof(dests)/sizeof(dests[0]))
	
	#define FROM_PERCENTAGE(perc) ((long)round((width*height*3.0) * ((perc) / 100.0)))
	const long faults[] = {FROM_PERCENTAGE(0.00),
	                       FROM_PERCENTAGE(0.02),
	                       FROM_PERCENTAGE(0.04),
	                       FROM_PERCENTAGE(0.08),
	                       FROM_PERCENTAGE(0.16),
	                       FROM_PERCENTAGE(0.31),
	                       FROM_PERCENTAGE(0.62),
	                       FROM_PERCENTAGE(1.25),
	                       FROM_PERCENTAGE(2.50),
	                       FROM_PERCENTAGE(5.00),
	                      };
	#undef FROM_PERCENTAGE
	#define NUM_FAULT_VALUES (sizeof(faults)/sizeof(faults[0]))
	
	printf("dest,faults,router,max_resource,mean_resource,max_entries,mean_entries,total_disconnections\n");
	for (long dest_index = 0; dest_index < NUM_DEST_VALUES; dest_index++) {
		long num_destinations = dests[dest_index];
		for (long fault_index = 0; fault_index < NUM_FAULT_VALUES; fault_index++) {
			long num_faults = faults[fault_index];
			for (long algorithm = FIRST_ALG; algorithm <= LAST_ALG; algorithm++) {
				// Create a fault map
				srand(seed);
				create_faults(num_faults, width, height);
				accum_map_reset();

				long total_dx = 0;
				long total_dy = 0;
				long total_disconnections = 0;
				for (long rep = 0; rep < num_repeats; rep++) {
					// Set up the current run
					long dx = rand() % width;
					long dy = rand() % height;
					total_dx += dx;
					total_dy += dy;
					rotate_faults(dx, dy, width, height);
					rotate_accum_map(dx, dy, width, height);
					create(num_destinations, width, height);
					a_star_reset();

					// Route
					switch(algorithm){
						default:
						case 0: dor(num_destinations); break;
						case 1: lf(num_destinations); break;
						case 2: opt(num_destinations); break;
						case 3: smart(num_destinations,RANGE); break;
						#ifdef A_STAR
						case 4: just_a_star(num_destinations); break;
						#endif
					}

					// Patch up anything remaining
					#ifdef A_STAR
						total_disconnections += a_star_required;
						if (a_star_required)
							if (!a_star()) {
								// Just try again if we got a situation where
								// rotating the faults made them unroutable.
								rep--;
								continue;
							}
					#endif

					// Accumulate resource consumption
					a_star_cur_visit_num++;
					accum_map_update(0, 170, 170);
				}

				long max_resource = 0;
				long total_resource = 0;
				long max_entries = 0;
				long total_entries = 0;
				for (long x = 170 - (width/2); x < 170 + (width/2); x++) {
					for (long y = 170 - (height/2); y < 170 + (height/2); y++) {
						for (long link = 0; link < 6; link++) {
							if (accum_map[x][y][link] > max_resource)
								max_resource = accum_map[x][y][link];
							total_resource += accum_map[x][y][link];
						}
						if (entry_map[x][y] > max_entries)
							max_entries = entry_map[x][y];
						total_entries += entry_map[x][y];
					}
				}
				printf("%d,%.2f,%s,%d,%f,%d,%f,%d\n",
				       num_destinations,
				       (num_faults / (width*height*3.0)) * 100.0,
				       (algorithm == 0) ? "dor" :
				       (algorithm == 1) ? "lf" :
				       (algorithm == 2) ? "opt" :
				       (algorithm == 3) ? "smart" :
				       (algorithm == 4) ? "graph_search" : "unknown",
				       max_resource,
				       total_resource/((double)width * height * 6),
				       max_entries,
				       total_entries/((double)width * height),
				       total_disconnections);
				
				// Plot a a heatmap of resource/table consuption
				#ifdef PRINT_HEATMAP
				printf("x,y,entries,resources,faults\n");
				for (long x = 170 - (width/2); x < 170 + (width/2); x++) {
					for (long y = 170 - (height/2); y < 170 + (height/2); y++) {
						printf("%d,%d,%d,%d,%d\n",
						       x, y,
						       entry_map[x][y],
						       accum_map[x][y][0] + accum_map[x][y][1] +
						       accum_map[x][y][2] + accum_map[x][y][3] +
						       accum_map[x][y][4] + accum_map[x][y][5],
						       fault[x][y][0] + fault[x][y][1] +
						       fault[x][y][2] + fault[x][y][3] +
						       fault[x][y][4] + fault[x][y][5]);
					}
				}
				#endif
			}
		}
	}

	return 0;
}


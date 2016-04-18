/*
 *  C Implementation of Kuhn's Hungarian Method
 *  Copyright (C) 2003  Brian Gerkey <gerkey@robotics.usc.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * a test/example program for the C implementation of the Hungarian method
 */

#define USAGE "Usage: ./test [-m <m>] [-n <n>] [-f <fname>]"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
/* one of the following will define PATH_MAX */
#include <limits.h> 
#include <sys/param.h>

#include "hungarian.h"

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

// problem dimensions
#define n 3
#define m 3

// input data file
char input_fname[PATH_MAX];

void parse_args(int argc, char** argv);


struct point {
    int x;
    int y;
};

int point_distance(struct point *a, struct point *b) {
    int dx = a->x - b->x;
    int dy = a->y - b->y;

    return (int) (1000 * sqrt(dx*dx + dy*dy));
}

int
main(int argc, char** argv)
{
    hungarian_t prob;

    int r[m][n];

    struct point ppos[m] = { {1,2}, {3,8}, {5,2} };
    struct point fpos[n] = { {5,6}, {4,4}, {9,0} };

    int i;
    for (i = 0; i < m; i++) {
        int j;
        for (j = 0; j < n; j++) {
            r[i][j] = point_distance(&ppos[i], &fpos[j]);
        }
    }

    hungarian_init(&prob,(int*)r,m,n,HUNGARIAN_MIN);
    hungarian_print_rating(&prob);
    hungarian_solve(&prob);
    hungarian_print_assignment(&prob);

    printf("\nfeasible? %s\n", 
            (hungarian_check_feasibility(&prob)) ? "yes" : "no");
    printf("benefit: %d\n\n", hungarian_benefit(&prob));

    hungarian_fini(&prob);
    return(0);
}

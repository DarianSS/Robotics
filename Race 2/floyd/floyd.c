#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define INF 1<<16

int maze[17][17], paths[17][17], aux;

typedef struct path {
	int square;
	struct path *next;
} path;

path *p_start, *p_current;

void maze_initialization() {
	for (int i=0; i<=16; ++i) 
		for (int j=0; j<=16; ++j)
			maze[i][j] = INF;
}

void floyd() {
	for (int i=0; i<=16; i++)
		for (int j=0; j<=16; j++)
			paths[i][j] = -1;
	
	for (int k=0; k<=16; k++)
		 for (int i=0; i<=16; i++)
		 	for (int j=0; j<=16; j++)
		 		if (maze[i][k] + maze[k][j] < maze[i][j]) {
		 			maze[i][j] = maze[i][k] + maze[k][j];
		 			paths[i][j] = k;
		 		}
}

void floyd_reconstruction(int i, int j) {
	int k = paths[i][j];
	if (k != -1) {
		floyd_reconstruction(i,k);
	}	
	path *new;
	new = (path *)malloc(sizeof(path));
	new->square = j;
	p_current->next = new;
	p_current = p_current->next;
	printf("%d ", j);
} 

int main() {
	maze_initialization();
	freopen ("in.txt", "r", stdin);
	for (int i=0; i<=16; ++i)
		for (int j=0; j<=16; ++j) {
			scanf ("%d", &aux);
			if (aux == 1)
				maze[i][j] = 1;
		}
	fclose (stdin);

	p_current = (path *)malloc(sizeof(path));
	p_current->square = 0;
	p_start = p_current;

	printf("floyd: "); //
	floyd ();
	floyd_reconstruction(0,16);
	printf("\n");  //

	/*for (i=0; i<=16; ++i) {
		for (j=0; j<=16; ++j)
			printf ("%d ", paths[i][j]);
		printf ("\n");
	}*/

	p_current = p_start;
	while (p_current) {
		//printf("%d ",p_current->square);
		p_current = p_current->next;
	} 
	printf("\n"); 
	
	/*p_current = p_start;
	free(p_start);
	while (p_current) {
		free(p_current);
	}*/ 	

	return 0;
}
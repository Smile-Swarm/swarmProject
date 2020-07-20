/*

Description:
Basic point to point communication between processes

From: https://mpitutorial.com/

*/

#include <mpi.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {

// Initialize the MPI environment
MPI_Init(NULL, NULL);

// Rank
int world_rank;
MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

// Size
int world_size;
MPI_Comm_size(MPI_COMM_WORLD, &world_size);

// Using two processes (ranks) for this task
if(world_size < 2) {
fprintf(stderr, "World Size must be greater than 1 for %s\n", argv[0]);
MPI_Abort(MPI_COMM_WORLD, 1);
}

int number;
if(world_rank == 0) {
number = -1;
MPU_Send(%number, 1, MPI_INT, 1, 0, MPI_COMM_WORLD);
}
else if(world_rank == 1) {
MPI_Recv(&number, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
printf("Process 1 received number %d from process 0\n", number);
}

MPI_Finalize();

} //End of main

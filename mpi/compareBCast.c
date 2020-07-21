/*

Description:
Comparing the timinig results between a hand written broad casting vs mpi bcast syntax.

From: https://mpitutorial.com/

*/

#include <stdio.h>
#include <stdlib.h>
#include <mpi.h>
#include <assert.h>

void my_bcast(void* data, int count, MPI_Datatype datatype, int root, MPI_Comm communicator) {
int world_rank;
MPI_Comm_rank(communicator, &world_rank);
int world_size;
MPI_Comm_size(communicator, &world_size);

if(world_rank == root) {
// If we are at the root prcoessor, send our data to everyone
int i;
for(i = 0; i < world_size; i++)  {
if(i != world_rank) {
MPI_Send(data, count, datatype, i, 0, communicator);
} // End of if
} // End of for
} // End of if
else {
MPI_Recv(data, count, datatype, root, 0, communicator, MPI_STATUS<IGNORE);
} // End of else
} // End of my_bcast

int main(int argc, char** argv) {

if (argc != 3) {
fprintf(stderr, "Usage: compare_bcast num_elements num_trials\n");
exit(1);
} // End of if

int num_elements = atoi(argv[1]);
int num_trials = atoi(argv[2]);

MPI_Init(NULL, NULL);

int world_rank;
MPI_comm_rank(MPI_COMM_WORLD, &world_rank);

double total_my_bcast_time = 0.0;
double total_mpi_bcast_time = 0.0;
int i;
int* data = (int*)malloc(sizeof(int) * num_elements);
assert(data != NULL);

for(i = 0; i < num_trails; i++) {
// Time my_bcast
// Synchronize before starting timing
MPI_Barrier(MPI_COMM_WORLD);
total_my_bcast_time -= MPI_Wtime();
my_bcast(data, num_elements, MPI_INT, 0, MPI_COMM_WORLD);
// Synchronize again before obtaining final time
MPI_Barrier(MPI_COMM_WORLD);
total_my_bcast_time += MPI_Wtime();

// Time MPI_Bcast
MPI_Barrier(MPI_COMM_WORLD);
total_mpi_bcast_time -= MPI_Wtime();
MPI_Bcast(data, num_elements, MPI_INT, 0, MPI_COMM_WORLD);
MPI_Barrier(MPI_COMM_WORLD);
total_mpi_bcast_time += MPI_Wtime();
} // End of for

// Print off timing information
if(world_rank == 0) {
printf("Data size = %d, Trials = %d\n", num_elements * (int)sizeof(int), num_trials);
printf("Avg my_bcast time = %lf\n", total_my_bcast_time / num_trials);
printf("Avg MPI_Bcast time = %lf\n", total_mpi_bcast_time / num_trials);
} // End of if

free(data);
MPI_Finalize();

} // End of main

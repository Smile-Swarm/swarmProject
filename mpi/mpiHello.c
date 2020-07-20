/*

Getting familiar with the environment and syntaxes

*/

#include <mpi.h>
#include <stdio.h>

int main(int argc, char** argv) {

// Initialize the MPI environment
MPI_Init(NULL, NULL);

// Get number of processes
int world_size;
MPI_Comm_size(MPI_COMM_WORLD, &world_size);

// Get that rank of process
int world_rank;
MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

// Get the name of processor
char processor_name[MPI_MAX_PROCESSOR_NAME];
int name_len;
MPI_Get_processor_name(processor_name, &name_len);

// Print the hello message
printf("Hello world from processor %s, rank %d out of %d processors\n", processor_name, world_rank, world_size;

MPI_Finalize();

}

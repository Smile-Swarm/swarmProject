/*

Description:
Point to point communiation with n-numbers of processes in a ring topology

From: https://mpitutorial.com/

*/

#include <mpi.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {

// Initialize MPI environment
MPI_Init(NULL, NULL);

// Rank
int rank;
MPI_Comm_rank(MPI_COMM_WORLD, &rank);

// Size
int size;
MPI_Comm_size(MPI_COMM_WORLD, &size);

int data;
if(rank != 0)
{
MPI_Recv(&data, 1, MPI_INT, rank-1, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
printf("Process %d received data %d from process %d\n", rank, data, rank-1);
}
else
{
// In process 0
data = -1;
}
MPI_Send(&data, 1, MPI_INT, (rank+1) % size, 0, MPI_COMM_WORLD);

if(rank == 0)
{
MPI_Recv(&data, 1, MPI_INT, size-1, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
printf("Process %d received data  %d from process %d\n", rank, data, size-1);
}

MPI_Finalize();

} // End of main

/*

Description:
Point to point communication between two ranks. Same as send and receive but with the added complication.

From: https://mpitutorial.com/

*/

#include <mpi.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {

const int PING_PONG_LIMIT = 5;

// Initialize the MPI environment
MPI_Init(NULL, NULL);

// Rank
int world_rank;
MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

// Size
int world_size;
MPI_Comm_size(MPI_COMM_WORLD, &world_size);

// Going to use 2 processes for this task
if (world_size != 2) {
fprintf(stderr, "World size must be two for %s\n", argv[0]);
MPI_Abort(MPI_COMM_WORLD, 1);
}

int count = 0;
int partner_rank = (world_rank + 1) % 2;

while (count < PING_PONG_LIMIT) {
if (world_rank == count % 2) 
{
// Increment the ping pong count before you send it
count++;
MPI_Send(&count, 1, MPI_INT, partner_rank, 0, MPI_COMM_WORLD);
printf("%d sent and incremented ping_pong_count %d to %d\n", world_rank, count, partner_rank);
} 
else 
{
MPI_Recv(&count, 1, MPI_INT, partner_rank, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
printf("%d received ping_pong_count %d from %d\n", world_rank, count, partner_rank);
}
} //End of while

MPI_Finalize();

} // End Main

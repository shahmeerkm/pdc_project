#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <pthread.h>
#include <string.h>

#define V 1000         // Number of vertices in the graph
#define MAX_DIST 10    // Maximum distance between vertices
#define NUM_THREADS 10 // Number of threads for multithreading

// Struct to hold thread arguments
struct ThreadArgs {
    int graph[V][V];
    int src;
    int thread_id;
    double avgDist; // Average distance calculated by the thread
};

// A utility function to find the vertex with minimum distance value
int minDistance(double dist[], bool sptSet[]) {
    double min = DBL_MAX;
    int min_index;

    for (int v = 0; v < V; v++) {
        if (!sptSet[v] && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }

    return min_index;
}

// Function to generate a random distance matrix
void generateRandomGraph(int graph[V][V]) {
    srand(time(NULL)); // Seed for random number generation

    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            if (i == j) {
                graph[i][j] = 0; // Distance from a vertex to itself is 0
            } else {
                // Generate random distances between vertices
                graph[i][j] = rand() % MAX_DIST + 1; // Random distance from 1 to MAX_DIST
                graph[j][i] = graph[i][j]; // Graph is undirected, so update symmetrically
            }
        }
    }
}

// Function that implements Dijkstra's algorithm
void dijkstra(int graph[V][V], int src, double *avgDist) {
    double dist[V]; // The output array. dist[i] will hold the shortest distance from src to i
    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest path tree

    // Initialize all distances as INFINITE and sptSet[] as false
    for (int i = 0; i < V; i++) {
        dist[i] = DBL_MAX;
        sptSet[i] = false;
    }

    dist[src] = 0; // Distance of source vertex from itself is 0

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != DBL_MAX &&
                dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }

    // Calculate average distance for this thread's portion
    double sumDist = 0.0;
    int numVertices = V - 1; // Exclude source vertex from average calculation

    for (int i = 0; i < V; i++) {
        if (i != src) {
            sumDist += dist[i];
        }
    }
    *avgDist = sumDist / numVertices;
}

// Dijkstra's algorithm function for each thread
void *dijkstraThread(void *arguments) {
    struct ThreadArgs *args = (struct ThreadArgs *)arguments;
    dijkstra(args->graph, args->src, &(args->avgDist));
    pthread_exit(NULL);
}

int main() {
    int graph[V][V]; // Adjacency matrix representation of the graph
    generateRandomGraph(graph); // Generate random distance matrix

    int src = 5;
    printf("Source vertex: %d\n", src);

    pthread_t threads[NUM_THREADS]; // Array to hold thread IDs
    struct ThreadArgs args[NUM_THREADS]; // Array to hold thread arguments

    clock_t start, end;
    double cpu_time_used;

    start = clock();

    for (int i = 0; i < NUM_THREADS; i++) {
        args[i].src = src;
        args[i].thread_id = i;
        // Assign a portion of the graph to each thread
        // Modify this based on the number of threads and size of the graph
        // For simplicity, dividing equally here
        memcpy(args[i].graph, graph, sizeof(graph));

        if (pthread_create(&threads[i], NULL, dijkstraThread, (void *)&args[i])) {
            fprintf(stderr, "Error creating thread %d\n", i);
            return 1;
        }
    }

    // Wait for all threads to finish
    for (int i = 0; i < NUM_THREADS; i++) {
        if (pthread_join(threads[i], NULL)) {
            fprintf(stderr, "Error joining thread %d\n", i);
            return 2;
        }
    }

    end = clock();
    cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

    // Now that all threads have finished, calculate and print average distance
    double totalDist = 0.0;

    for (int i = 0; i < NUM_THREADS; i++) {
        totalDist += args[i].avgDist;
    }

    double avgDist = totalDist / NUM_THREADS;
    printf("Average distance from source vertex %d: %.2f\n", src, avgDist);
    printf("CPU time used: %.6f seconds\n", cpu_time_used);

    return 0;
}

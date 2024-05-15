#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <omp.h>
#include <float.h>

#define V 1000 // Number of vertices in the graph
#define MAX_DIST 10 // Maximum distance between vertices

// A utility function to find the vertex with minimum distance value
int minDistance(double dist[], bool sptSet[]) {
    double min = DBL_MAX;
    int min_index = -1; // Initialize min_index

#pragma omp parallel for shared(min, min_index)
    for (int v = 0; v < V; v++) {
        if (!sptSet[v] && dist[v] <= min) {
            #pragma omp critical // Ensure only one thread modifies min and min_index at a time
            {
                if (dist[v] <= min) {
                    min = dist[v];
                    min_index = v;
                }
            }
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
void dijkstra(int graph[V][V], int src) {
    double dist[V]; // The output array. dist[i] will hold the shortest distance from src to i
    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest path tree

    // Initialize all distances as INFINITE and sptSet[] as false
    #pragma omp parallel for
    for (int i = 0; i < V; i++) {
        dist[i] = DBL_MAX;
        sptSet[i] = false;
    }

    dist[src] = 0; // Distance of source vertex from itself is 0

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        #pragma omp parallel for
        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != DBL_MAX &&
                dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }

    // Calculate and print average distance
    double sumDist = 0.0;
    int numVertices = V - 1; // Exclude source vertex from average calculation

    #pragma omp parallel for reduction(+:sumDist)
    for (int i = 0; i < V; i++) {
        if (i != src && dist[i] != DBL_MAX) {
            sumDist += dist[i];
        }
    }
    double avgDist = sumDist / numVertices;
    printf("Average distance from source vertex %d: %.2f\n", src, avgDist);
}

int main() {
    int graph[V][V]; // Adjacency matrix representation of the graph

    generateRandomGraph(graph); // Generate random distance matrix

    // Source vertex
    int src = 5;

    printf("Source vertex: %d\n", src);

    // For cpu time usage
    clock_t start, end;
    double cpu_time_used;

    start = clock();
    // Run Dijkstra's algorithm
    dijkstra(graph, src);
    end = clock();

    cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
    printf("CPU time used: %.6f seconds\n", cpu_time_used);

    return 0;
}

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <xmmintrin.h> // Include SSE3 header

#define V 10 // Number of vertices in the graph
#define MAX_DIST 10.0f // Maximum distance between vertices (float)

// Function prototype for generateRandomGraph
void generateRandomGraph(float graph[V][V]);

// A utility function to find the vertex with minimum distance value
int minDistance(float dist[], bool sptSet[]) {
    float min = FLT_MAX;
    int min_index = -1; // Initialize min_index to an invalid value

    for (int v = 0; v < V; v++) {
        if (!sptSet[v] && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }

    return min_index;
}

// Function that implements Dijkstra's algorithm with SIMD vectorization
void dijkstra(float graph[V][V], int src) {
    float dist[V]; // The output array. dist[i] will hold the shortest distance from src to i
    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest path tree

    // Initialize all distances as INFINITE and sptSet[] as false
    for (int i = 0; i < V; i++) {
        dist[i] = FLT_MAX;
        sptSet[i] = false;
    }

    dist[src] = 0.0f; // Distance of source vertex from itself is 0

    // For CPU time usage
    clock_t start, end;
    double cpu_time_used;

    start = clock();

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        // Vectorized distance update using SIMD instructions
        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != FLT_MAX &&
                dist[u] + graph[u][v] < dist[v]) {
                // SIMD instructions for distance update
                __m128 distVec = _mm_loadu_ps(&dist[v]);
                __m128 graphVec = _mm_loadu_ps(&graph[u][v]);
                __m128 updatedDistVec = _mm_add_ps(_mm_loadu_ps(&dist[u]), graphVec);
                updatedDistVec = _mm_min_ps(updatedDistVec, _mm_loadu_ps(&dist[v]));
                _mm_storeu_ps(&dist[v], updatedDistVec);
            }
        }
    }

    end = clock();
    cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

    // Calculate and print average distance
    float sumDist = 0.0f;
    int numVertices = V - 1; // Exclude source vertex from average calculation

    for (int i = 0; i < V; i++) {
        if (i != src && dist[i] != FLT_MAX) { // Check for valid distances
            sumDist += dist[i];
        }
    }
    float avgDist = sumDist / numVertices;
    printf("Average distance from source vertex %d: %.2f\n", src, avgDist);
    printf("CPU time used: %.6f seconds\n", cpu_time_used);
}

// Function to generate a random distance matrix
void generateRandomGraph(float graph[V][V]) {
    srand(time(NULL)); // Seed for random number generation

    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            if (i == j) {
                graph[i][j] = 0.0f; // Distance from a vertex to itself is 0
            } else {
                // Generate random distances between vertices
                graph[i][j] = (float)(rand() % (int)MAX_DIST) + 1.0f; // Random distance from 1 to MAX_DIST
            }
        }
    }
}

int main() {
    float graph[V][V]; // Adjacency matrix representation of the graph

    generateRandomGraph(graph); // Generate random distance matrix

    // Source vertex
    int src = 5;

    printf("Source vertex: %d\n", src);

    // Run Dijkstra's algorithm
    dijkstra(graph, src);

    return 0;
}

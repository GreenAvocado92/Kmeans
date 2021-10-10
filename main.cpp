#include "kmeans.hpp"

using namespace std;

int main(int argc, char **argv) {
    Kmeans kmean;
    kmean.setInput(argv[1]);
    kmean.setNumberofClusters(10);
    kmean.setMaxCorrespondenceDistance(1);
    kmean.compute();

    kmean.save_clusters("./output");
    return 0;
}

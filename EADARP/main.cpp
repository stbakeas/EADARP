#include <iostream>
#include "Instance.h"
#include "Algorithms.h"
#include "Solution.h"
#include "Route.h"
#include <filesystem>
#include <chrono>
#include <thread>
#include "RandLib.h"
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <fstream>
#include <cmath>
#include <numeric> 
#include "FixedDouble.h"


using namespace std;
namespace fs = std::filesystem;


typedef std::array<double, 3> Configuration;

/*
* 1. Run your algorithm on every a-instance for x runs.
* 2. Write the output(Best Cost, Average Cost, Average Iterations Until Best Was Found, Average Running Time)
*    for each instance in a file.
*/
void CordeauExperiment(int numberOfRuns) {
	const fs::path pathToShow("Cordeau-EADARP");
	ofstream cordeauFile("Experiment.txt");
	for (const auto& entry : fs::directory_iterator(pathToShow)) {
		const auto filenameStr = entry.path().filename().string();
		if (entry.is_regular_file()) {
			std::cout << "Instance: " << filenameStr << '\n';
			inst.loadCordeau(pathToShow.string() + "/" + filenameStr);
			Solution best;
			best.total_travel_distance = DBL_MAX;
			double avgRunTime = 0.0, avgCost = 0.0;
			int avg_best_iter(0), successfulRuns(0);
			for (int i = 0; i < numberOfRuns; i++) {
				printf("%s%d\n", "Run ID: ", i);
				Run run = algorithms::ALNS(algorithms::details::Init1(), 15000, INT_MAX, 6, 0.1, 100, 0.5);
				if (run.best.rejected.empty()) {
					successfulRuns++;
					avgCost += run.best.objectiveValue();
					if (run.best.objectiveValue() < best.objectiveValue()) {
						best = run.best;
						avg_best_iter += run.best_iter;
					}
				}
				avgRunTime += run.elapsed_seconds;
			}
			cordeauFile << filenameStr << " " << successfulRuns << "/" << numberOfRuns << " " << best.objectiveValue() << " " << avgCost / successfulRuns << " " << avgRunTime / successfulRuns << " " << avg_best_iter / successfulRuns << "\n";
			inst.~Instance();
		}
	}
	cordeauFile.close();
}

int main(){
	inst.loadCordeau("Cordeau-EADARP/a2-16-0.7.txt");
	Solution best;
	best.total_travel_distance = DBL_MAX;
	for (int i = 0; i < 1; i++) {
		printf("%s%d\n", "Run ID: ", i);
		Run run = algorithms::ALNS(algorithms::details::Init1(), 25000, INT_MAX, 6, 0.1, 100, 0.5);
		if (run.best.rejected.empty()) {
			if (run.best.objectiveValue() < best.objectiveValue()) {
				best = run.best;
			}
		}
	}
	best.Display(0);
	best.Display(1);
	return EXIT_SUCCESS;
}

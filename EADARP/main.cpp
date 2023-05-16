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


using namespace std;
namespace fs = std::filesystem;


typedef std::array<double, 3> Configuration;

/*
* 1. Run your algorithm on every a-instance for x runs.
* 2. Write the output(Best Cost, Average Cost, Average Iterations Until Best Was Found, Average Running Time)
*    for each instance in a file.
*/
void PerformanceEvaluation(int numberOfRunsPerInstance) {
	std::array<double, 3> gamma_values = { 0.1,0.4,0.7 };
	for (double gamma : gamma_values) {
		const fs::path pathToShow("Cordeau-EADARP");
		ofstream outputFile("Type-a-" + to_string(dbl_round(gamma,1)) + ".txt");
		std::cout << "Returned Battery Percentage = " << gamma << "\n\n";
		for (const auto& entry : fs::directory_iterator(pathToShow)) {
			const auto filenameStr = entry.path().filename().string();
			if (entry.is_regular_file()) {
				std::cout << "Instance: " << filenameStr << '\n';
				inst.loadInstance(pathToShow.string() + "/" + filenameStr,gamma);
				Solution best;
				best.total_travel_distance = DBL_MAX;
				double avgRunTime = 0.0, avgCost = 0.0;
				int avg_best_iter(0), successfulRuns(0);
				for (int i = 0; i < numberOfRunsPerInstance; i++) {
					printf("%s%d\n", "Run ID: ", i);
					Run run = algorithms::ALNS(algorithms::details::Init1(), 10000, INT_MAX, 0.05, 7, 0.1, 100, 0.5);
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
				if (successfulRuns > 0) {
					avgRunTime /= successfulRuns;
					avg_best_iter /= successfulRuns;
					avgCost /= successfulRuns;
				}
				outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" << avgCost << "		" << avgRunTime << "	" << avg_best_iter << "\n";
				inst.~Instance();
			}
		}
		outputFile.close();
	}
	
}

void ParameterTuning(int numberOfRunsPerInstance) {
	std::array<double, 3> gamma_values = {0.1,0.4,0.7};
	for (double gamma : gamma_values) {
		const fs::path pathToShow("TrainingSet");
		ofstream outputFile("Malheiros-" + to_string(dbl_round(gamma, 1)) + ".txt");
		printf("Battery Return Percentage = %f\n",gamma);
		double omega = 0.1;
		while (omega < 0.35) {
			printf("Removal Percentage = %f\n", omega);
			outputFile << "Removal Percentage = " <<omega<< "\n\n";
			for (const auto& entry : fs::directory_iterator(pathToShow)) {
				const auto filenameStr = entry.path().filename().string();
				std::cout << pathToShow.filename().string() + "/" + filenameStr << "\n";
				if (entry.is_regular_file()) {
					inst.loadMalheiros(pathToShow.filename().string() + "/" + filenameStr, gamma);
					Solution best;
					Solution initial = algorithms::details::Init1();
					best.total_travel_distance = DBL_MAX;
					double avgRunTime = 0.0, avgCost = 0.0;
					int avg_best_iter(0), successfulRuns(0);
					for (int i = 0; i < numberOfRunsPerInstance; i++) {
						printf("%s%d\n", "Run ID: ", i);
						Run run = algorithms::ALNS(initial, 10000, INT_MAX, 0.05, 7, omega, 100, 0.5);
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
					if (successfulRuns > 0) {
						avgRunTime /= successfulRuns;
						avg_best_iter /= successfulRuns;
						avgCost /= successfulRuns;
					}
					outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" << avgCost << "		" << avgRunTime << "	" << avg_best_iter << "\n";
					inst.~Instance();
				}
			}
			omega += 0.05;
		}
		outputFile.close();
	}
}

int main(){
	inst.loadInstance("Cordeau-EADARP/a2-16.txt",0.7);
	Run run = algorithms::ALNS(algorithms::details::Init1(), 10000, INT_MAX, 0.05, 7, 0.1, 100, 0.5);
	return EXIT_SUCCESS;
}

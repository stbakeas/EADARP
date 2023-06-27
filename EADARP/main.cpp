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

/*
* 1. Run your algorithm on every a-instance for x runs.
* 2. Write the output(Best Cost, Average Cost, Average Iterations Until Best Was Found, Average Running Time)
*    for each instance in a file.
*/
void RejectedImpact(int numberOfRunsPerInstance) {
	std::array<double, 3> gamma_values = { 0.1,0.4,0.7 };
	std::array<std::vector<algorithms::Statistics>, 2> stats;
	stats[0].emplace_back(algorithms::details::RandomRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::ZeroSplitRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::WorstRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::SimilarityRemoval, 0.25, 0, 0);
	stats[1].emplace_back(algorithms::details::RandomInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedGreedyInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedRegretInsertion, 0.33, 0, 0);
	int gammaIndex = 0;
	const fs::path pathToShow("TrainingSet");
	ofstream outputFile("Rejected.txt");
	for (double gamma : gamma_values) {
		outputFile << "Battery Return Percentage =" << gamma << "\n";
		for (const auto& entry : fs::directory_iterator(pathToShow)) {
			const auto filenameStr = entry.path().filename().string();
			std::cout << pathToShow.filename().string() + "/" + filenameStr << "\n";
			if (entry.is_regular_file()) {
				inst.loadInstance(pathToShow.filename().string() + "/" + filenameStr, gamma);
				Solution best;
				Solution initial = algorithms::details::Init1();
				best.total_travel_distance = DBL_MAX;
				double totalRunTime = 0.0, avgCost = 0.0;
				int avg_best_iter(0), successfulRuns(0);
				for (int i = 0; i < numberOfRunsPerInstance; i++) {
					printf("%s%d\n", "Run ID: ", i);
					Run run = algorithms::ALNS(initial, stats, 10000, 0.05, 7,0.3, 100, 0.5);
					if (run.best.rejected.empty()) successfulRuns++;
					avgCost += run.best.objectiveValue();
					avg_best_iter += run.best_iter;
					totalRunTime += run.elapsed_seconds;
					if (run.best.objectiveValue() < best.objectiveValue())
						best = run.best;

				}
				avg_best_iter /= numberOfRunsPerInstance;
				avgCost /= numberOfRunsPerInstance;
				double averageValue = (inst.bestKnownSolutionCost[gammaIndex] + avgCost) / 2;
				double percentageGap = (std::abs(inst.bestKnownSolutionCost[gammaIndex] - avgCost) / averageValue) * 100;
				outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" << avgCost << "		" << totalRunTime << "	" << avg_best_iter << "   " << dbl_round(percentageGap, 2) << "\n";
				inst.~Instance();
			}
		}
		outputFile << "\n";
		gammaIndex++;
	}
	outputFile.close();
}

void ParameterTuning(int numberOfRunsPerInstance) {
	std::array<double, 3> gamma_values = {0.1,0.4,0.7};
	std::array<std::vector<algorithms::Statistics>, 2> stats;
	stats[0].emplace_back(algorithms::details::RandomRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::ZeroSplitRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::WorstRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::SimilarityRemoval, 0.25, 0, 0);
	stats[1].emplace_back(algorithms::details::RandomInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedGreedyInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedRegretInsertion, 0.33, 0, 0);
	int gammaIndex = 0;
	for (double gamma : gamma_values) {
		const fs::path pathToShow("TrainingSet");
		ofstream outputFile("Tuning-" + to_string(gamma) + ".txt");
		printf("Battery Return Percentage = %f\n",gamma);
		double omega = 0.1;
		while (omega < 0.35) {
			printf("Removal Percentage = %f\n", omega);
			outputFile << "Removal Percentage = " <<omega<< "\n\n";
			for (const auto& entry : fs::directory_iterator(pathToShow)) {
				const auto filenameStr = entry.path().filename().string();
				std::cout << pathToShow.filename().string() + "/" + filenameStr << "\n";
				if (entry.is_regular_file()) {
					inst.loadInstance(pathToShow.filename().string() + "/" + filenameStr, gamma);
					Solution best;
					Solution initial = algorithms::details::Init1();
					best.total_travel_distance = DBL_MAX;
					double avgRunTime = 0.0, avgCost = 0.0;
					int avg_best_iter(0), successfulRuns(0);
					for (int i = 0; i < numberOfRunsPerInstance; i++) {
						printf("%s%d\n", "Run ID: ", i);
						Run run = algorithms::ALNS(initial,stats,10000, 0.05, 7, omega, 100, 0.5);
						if (run.best.rejected.empty()) successfulRuns++;
						avgCost += run.best.objectiveValue();
						avg_best_iter += run.best_iter;
						avgRunTime += run.elapsed_seconds;
						if (run.best.objectiveValue() < best.objectiveValue())
							best = run.best;

					}
					avgRunTime /= numberOfRunsPerInstance;
					avg_best_iter /= numberOfRunsPerInstance;
					avgCost /= numberOfRunsPerInstance;
					double averageValue = (inst.bestKnownSolutionCost[gammaIndex] + avgCost) / 2;
					double percentageGap = (std::abs(inst.bestKnownSolutionCost[gammaIndex] - avgCost) / averageValue) * 100;
					outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" << avgCost << "		" << avgRunTime << "	" << avg_best_iter << "   " << dbl_round(percentageGap,2) << "\n";
					inst.~Instance();
				}
			}
			omega += 0.05;
		}
		outputFile.close();
		gammaIndex++;
	}
}

void OperatorImpact(int numberOfRunsPerInstance) {
	std::array<double, 1> gamma_values = {0.7};
	std::array<std::vector<algorithms::Statistics>, 2> stats;
	stats[0].emplace_back(algorithms::details::RandomRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::ZeroSplitRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::WorstRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::SimilarityRemoval, 0.25, 0, 0);
	stats[1].emplace_back(algorithms::details::RandomInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedGreedyInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedRegretInsertion, 0.33, 0, 0);
	int gammaIndex = 2;
	for (double gamma : gamma_values) {
		const fs::path pathToShow("TrainingSet");
		ofstream outputFile("OperatorImpact2-"+to_string(gamma)+".txt");
		printf("Battery Return Percentage = %f\n", gamma);

		printf("Excluding None: \n");
		outputFile << "Excluding none. \n\n";
		for (const auto& entry : fs::directory_iterator(pathToShow)) {
			const auto filenameStr = entry.path().filename().string();
			std::cout << pathToShow.filename().string() + "/" + filenameStr << "\n";
			if (entry.is_regular_file()) {
				inst.loadInstance(pathToShow.filename().string() + "/" + filenameStr, gamma);
				Solution best;
				Solution initial = algorithms::details::Init1();
				best.total_travel_distance = DBL_MAX;
				double totalRunTime = 0.0, avgCost = 0.0;
				int avg_best_iter(0), successfulRuns(0);
				for (int i = 0; i < numberOfRunsPerInstance; i++) {
					printf("%s%d\n", "Run ID: ", i);
					Run run = algorithms::ALNS(initial, stats, 10000, 0.05, 7, 0.3, 100, 0.5);
					if (run.best.rejected.empty()) successfulRuns++;
					avgCost += run.best.objectiveValue();
					avg_best_iter += run.best_iter;
					totalRunTime += run.elapsed_seconds;
					if (run.best.objectiveValue() < best.objectiveValue())
						best = std::move(run.best);

				}
				avg_best_iter /= numberOfRunsPerInstance;
				avgCost /= numberOfRunsPerInstance;
				double averageValue = (inst.bestKnownSolutionCost[gammaIndex] + avgCost) / 2;
				double percentageGap = (std::abs(inst.bestKnownSolutionCost[gammaIndex] - avgCost) / averageValue) * 100;
				outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" << avgCost << "		" << totalRunTime << "	" << avg_best_iter << "   " << dbl_round(percentageGap, 2) << "\n";
				inst.~Instance();

			}
		}

		for (int i = 0; i < 4; i++) {
			algorithms::Statistics removalOperator = stats[0][i];
			stats[0].erase(stats[0].begin() + i);
			printf("Removing removal operator: %u\n",i);
			outputFile << "Removing removal operator: " << i << "\n\n";
			for (const auto& entry : fs::directory_iterator(pathToShow)) {
				const auto filenameStr = entry.path().filename().string();
				std::cout << pathToShow.filename().string() + "/" + filenameStr << "\n";
				if (entry.is_regular_file()) {
					inst.loadInstance(pathToShow.filename().string() + "/" + filenameStr, gamma);
					Solution best;
					Solution initial = algorithms::details::Init1();
					best.total_travel_distance = DBL_MAX;
					double totalRunTime = 0.0, avgCost = 0.0;
					int avg_best_iter(0), successfulRuns(0);
					for (int i = 0; i < numberOfRunsPerInstance; i++) {
						printf("%s%d\n", "Run ID: ", i);
						Run run = algorithms::ALNS(initial, stats, 10000, 0.05, 7, 0.3, 100, 0.5);
						if (run.best.rejected.empty()) successfulRuns++;
						avgCost += run.best.objectiveValue();
						avg_best_iter += run.best_iter;
						totalRunTime += run.elapsed_seconds;
						if (run.best.objectiveValue() < best.objectiveValue())
							best = std::move(run.best);

					}
					avg_best_iter /= numberOfRunsPerInstance;
					avgCost /= numberOfRunsPerInstance;
					double averageValue = (inst.bestKnownSolutionCost[gammaIndex] + avgCost) / 2;
					double percentageGap = (std::abs(inst.bestKnownSolutionCost[gammaIndex] - avgCost) / averageValue) * 100;
					outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" << avgCost << "		" << totalRunTime << "	" << avg_best_iter << "   " << dbl_round(percentageGap, 2) << "\n";
					inst.~Instance();

				}
			}
			stats[0].insert(stats[0].begin() + i, removalOperator);
		}
		for (int i = 0; i < 3; i++) {
			algorithms::Statistics insertionOperator = stats[1][i];
			stats[1].erase(stats[1].begin() + i);
			printf("Removing insertion operator: %u\n", i);
			outputFile << "Removing insertion operator: " << i << "\n\n";
			for (const auto& entry : fs::directory_iterator(pathToShow)) {
				const auto filenameStr = entry.path().filename().string();
				std::cout << pathToShow.filename().string() + "/" + filenameStr << "\n";
				if (entry.is_regular_file()) {
					inst.loadInstance(pathToShow.filename().string() + "/" + filenameStr, gamma);
					Solution best;
					Solution initial = algorithms::details::Init1();
					best.total_travel_distance = DBL_MAX;
					double totalRunTime = 0.0, avgCost = 0.0;
					int avg_best_iter(0), successfulRuns(0);
					for (int i = 0; i < numberOfRunsPerInstance; i++) {
						printf("%s%d\n", "Run ID: ", i);
						Run run = algorithms::ALNS(initial, stats, 10000, 0.05, 7, 0.3, 100, 0.5);
						if (run.best.rejected.empty()) successfulRuns++;
						avgCost += run.best.objectiveValue();
						avg_best_iter += run.best_iter;
						totalRunTime += run.elapsed_seconds;
						if (run.best.objectiveValue() < best.objectiveValue())
							best = std::move(run.best);

					}
					avg_best_iter /= numberOfRunsPerInstance;
					avgCost /= numberOfRunsPerInstance;
					double averageValue = (inst.bestKnownSolutionCost[gammaIndex] + avgCost) / 2;
					double percentageGap = (std::abs(inst.bestKnownSolutionCost[gammaIndex] - avgCost) / averageValue) * 100;
					outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" << avgCost << "		" << totalRunTime << "	" << avg_best_iter << "   " << dbl_round(percentageGap, 2) << "\n";
					inst.~Instance();

				}
			}
			stats[1].insert(stats[1].begin() + i, insertionOperator);
		}
		outputFile.close();
		gammaIndex++;
		
	}
}

void PerformanceEvaluation(int numberOfRunsPerInstance,const std::string& instanceType) {
	std::array<double, 3> gamma_values = { 0.1,0.4,0.7 };
	std::array<std::vector<algorithms::Statistics>, 2> stats;
	stats[0].emplace_back(algorithms::details::RandomRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::ZeroSplitRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::WorstRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::SimilarityRemoval, 0.25, 0, 0);
	stats[1].emplace_back(algorithms::details::RandomInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedGreedyInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedRegretInsertion, 0.33, 0, 0);
	int gammaIndex = 0;
	for (double gamma : gamma_values) {
		const fs::path pathToShow("Type-"+instanceType);
		ofstream outputFile(instanceType + "Performance-" + to_string(gamma) + ".txt");
		printf("Battery Return Percentage = %f\n", gamma);
		for (const auto& entry : fs::directory_iterator(pathToShow)) {
			const auto filenameStr = entry.path().filename().string();
			std::cout << pathToShow.filename().string() + "/" + filenameStr << "\n";
			if (entry.is_regular_file()) {
				inst.loadInstance(pathToShow.filename().string() + "/" + filenameStr, gamma);
				Solution best;
				Solution initial = algorithms::details::Init1();
				best.total_travel_distance = DBL_MAX;
				double runTime = 0.0, avgCost = 0.0;
				int avg_best_iter(0), successfulRuns(0);
				for (int i = 0; i < numberOfRunsPerInstance; i++) {
					printf("%s%d\n", "Run ID: ", i);
					Run run = algorithms::ALNS(initial, stats, 10000, 0.05, 7, 0.3, 100, 0.5);
					if (run.best.rejected.empty()) successfulRuns++;
					avgCost += run.best.objectiveValue();
					avg_best_iter += run.best_iter;
					runTime += run.elapsed_seconds;
					if (run.best.objectiveValue() < best.objectiveValue())
						best = std::move(run.best);

				}
				avg_best_iter /= numberOfRunsPerInstance;
				avgCost /= numberOfRunsPerInstance;
				double AC_percentage = 100*(avgCost-inst.bestKnownSolutionCost[gammaIndex]) / inst.bestKnownSolutionCost[gammaIndex];
				double BC_percentage = 100*(best.objectiveValue()- inst.bestKnownSolutionCost[gammaIndex]) / inst.bestKnownSolutionCost[gammaIndex];
				outputFile << filenameStr << "		" << successfulRuns << "/" << numberOfRunsPerInstance << "		" << best.objectiveValue() << "		" <<dbl_round(BC_percentage,2)<< "		" << avgCost << "		" << dbl_round(AC_percentage,2) << "		" << runTime << "	" << avg_best_iter  << "\n";
				inst.~Instance();
			}
		}
		outputFile.close();
		gammaIndex++;
	}
}

int main(){
	std::array<std::vector<algorithms::Statistics>, 2> stats;
	stats[0].emplace_back(algorithms::details::RandomRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::ZeroSplitRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::WorstRemoval, 0.25, 0, 0);
	stats[0].emplace_back(algorithms::details::SimilarityRemoval, 0.25, 0, 0);
	stats[1].emplace_back(algorithms::details::RandomInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedGreedyInsertion, 0.33, 0, 0);
	stats[1].emplace_back(algorithms::details::CachedRegretInsertion, 0.33, 0, 0);

	inst.loadUber("Dataset/Type-u/u2-16.txt", 0.1);
	Run run = algorithms::ALNS(algorithms::details::Init1(), stats, 10000, 0.05, 7, 0.3, 100, 0.5);
	run.best.Display(0);
	return EXIT_SUCCESS;
}

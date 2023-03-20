#include <iostream>
#include "Instance.h"
#include "Algorithms.h"
#include "Solution.h"
#include "Route.h"
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


typedef std::array<double, 3> Configuration;

std::vector<int> rankTransform(std::vector<double>& arr) {
	std::vector<double> copyArr = arr;
	std::vector<int> rankings(arr.size());
	std::sort(copyArr.begin(), copyArr.end());
	std::unordered_map<double, int> hashMap;
	for (double num : copyArr) {
		if (!hashMap.contains(num)) hashMap[num]=hashMap.size() + 1;
	}
	for (int i = 0; i < arr.size(); i++) {
		rankings[i] = hashMap[arr[i]];
	}
	return rankings;
}

double chi_square_critical_value(double degrees_of_freedom, double confidence_level) {
	double alpha = 1.0 - confidence_level;
	double x = degrees_of_freedom / 2.0;
	double a = x - 1.0 / 3.0;
	double b = 1.0 / (9.0 * a);
	double c = pow(a, 3) * (1.0 - b + sqrt(b * (1.0 - b)));
	c = c * (1.0 / (a * a * a));
	double y = 1.0 - alpha / 2.0;
	double z = log(y);
	z = z / log(1.0 - c);
	z = x * (1.0 - c) * pow(z, 3.0);
	double critical_value = x + z;
	return critical_value;
}

int main() {

	
	inst.loadCordeau("Cordeau-EADARP/a2-24-0.7.txt");
	Solution init = algorithms::details::Init1();
	printf("%f,%d\n", init.objectiveValue(), init.rejected.size());
	Solution best;
	best.total_travel_distance = DBL_MAX;
	double avgRunTime = 0.0;
	int numberOfRuns = 1;
	int counter = 0;
	for (int i = 0; i < numberOfRuns; i++) {
		printf("%s%d\n", "Run ID: ", i);
		Run run = algorithms::ALNS(init, 10000, INT_MAX, 5, 0.1,100,0.5);
		if (run.best.objectiveValue() < best.objectiveValue()) best = run.best;
	}

	double actual(0.0);
	for (EAV* v : inst.vehicles) {
		for (int i = 0; i < best.routes[v].path.size() - 1; i++)
			actual += inst.getTravelTime(best.routes[v].path[i], best.routes[v].path[i + 1]);
	}
	std::cout << "Computed: " << FixedDouble(best.objectiveValue(), 2) << "\n";
	std::cout << "Actual: " << actual << "\n";
	best.Display(0);
	
	return EXIT_SUCCESS;
}

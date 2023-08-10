#pragma once
#include "Run.h"
#include "RandLib.h" 
#include <iostream>
#include "Solution.h"
#include "Instance.h"
#include <stack>

namespace algorithms
{
	typedef void(*Heuristic)(Solution&, const std::array<double, 2>&);

	struct Statistics {
		Heuristic heuristic;
		double weight;
		int score;
		int attempts;

		Statistics(Heuristic heuristic, double weight, int score, int attempts)
			:heuristic(heuristic), weight(weight), score(score), attempts(attempts){}

	};

	int RouletteWheelSelection(const std::vector<Statistics>& stats, double weightSum,RandLib& randlib);

	Run ALNS(const Solution& initial,std::array<std::vector<Statistics>,2> stats,unsigned int max_iterations, double temperature_control,
		double removalRandomness,double removalPercentage, int segment_size,double reaction_factor);

	inline bool SimulatedAnnealingAcceptanceCriterion(const Solution& candidate,const Solution& incumbent, double current_temperature,RandLib& randlib);

    namespace details
    {

		std::vector<Position> InsertionNeighborhood(const Request* r, Solution& s,const std::vector<EAV*>& available_vehicles, bool includeCS = false);

		bool Backtracking(Solution& s, std::unordered_set<int>& availableDepots, std::stack<EAV*>& vehiclesInRandomOrder,double threshold);
       
		/**
         * Randomly selects a request and inserts it at its best position.
         */
		Solution Init1();

		/*
		* Removal Ratio = 0 
		* Randomness = 1
		*/
		void WorstRemoval(Solution& s, const std::array<double, 2>& arguments);

		//Removal Ratio = 0;
		void RandomRemoval(Solution& s, const std::array<double, 2>& arguments);

		//Removal Ratio = 0;
		void SimilarityRemoval(Solution& s, const std::array<double, 2>& arguments);

		/*
		* Removal Ratio = 0
		* Randomness = 1
		*/
		void ZeroSplitRemoval(Solution& s, const std::array<double, 2>& arguments);

		
		//Empty
		void GreedyInsertion(Solution& s, const std::array<double, 2>& arguments);

		void CachedGreedyInsertion(Solution& s, const std::array<double, 2>& arguments);

		/*
		* Regret Degree = 0
		*/
		void RegretInsertion(Solution& s, const std::array<double, 2>& arguments);

		void CachedPositionRegretInsertion(Solution& s, const std::array<double, 2>& arguments);

		void CachedRegretInsertion(Solution& s, const std::array<double, 2>& arguments);

		void RandomInsertion(Solution& s, const std::array<double, 2>& arguments);

		/*
		Proposed by Ying Luo and Paul Schonfeld in 2007
		*/
		void rejectedReinsertion(Solution& s);
    } 
}



#pragma once
#include "Run.h"
#include "RandLib.h" 
#include <iostream>
#include "Solution.h"
#include "Instance.h"

namespace algorithms
{
	enum class NeighborChoice { BEST, FIRST, RANDOM, SMART};

	typedef std::pair<Request*, EAV*> Assignment;

	typedef Solution(*Heuristic)(Solution, std::vector<double>);

	struct Statistics {
		Heuristic heuristic;
		double weight;
		int score;
		int attempts;

		Statistics(Heuristic heuristic,double weight, int score, int attempts) {
			this->heuristic = heuristic;
			this->weight = weight;
			this->score = score;
			this->attempts = attempts;
		}

	};

	int RouletteWheelSelection(std::vector<Statistics> stats, double weightSum);

	Run ALNS(Solution initial,unsigned int max_iterations, int max_seconds,
		double removalRandomness,double removalPercentage, int segment_size,double reaction_factor);

	bool SimulatedAnnealingAcceptanceCriterion(const Solution& candidate,const Solution& incumbent, double current_temperature);

    namespace details
    {

		std::vector<Position> InsertionNeighborhood(Request* r, Solution s, std::vector<EAV*> available_vehicles, bool includeCS = false);

		double DistanceBetweenUsers(Request* r1,Request* r2);

		/*
		* Inspired by list heuristic proposed by Atahran et. al(2014)
		*/
		Solution H1(int iter);

        /**
         * Randomly selects a request and inserts it at its best position.
         */
		Solution Init1();

		/*
		* Sequential Insertion Heuristic based on acquisition cost
		*/
		Solution Init4();

		/*
		 Removes a zero-load arcs from two routes and then connects 
		 the first part of the first route with the second part of the second route and vice versa.
		*/
		Solution two_opt(Solution s,NeighborChoice strategy);

		/*
		* Intra-route 
		
		
		. Removes a charging station and reinserts it in every feasible position.
		*/
		void moveStation(Solution& s,NeighborChoice strategy);

		/*
		 Selects two requests from two different routes and swaps their positions
		*/
		Solution swapRequest(Solution s,NeighborChoice strategy);
		/*
		  Select a rejected request and swap it with an already assigned one
		*/
		Solution swapRejected(Solution s,NeighborChoice strategy);

		/*Inspired By Yves Molenbruch et. al.(2016)
		*/
		Solution swapNaturalSequence(Solution s, NeighborChoice strategy);

		/*
		  Select two vehicles and swap their routes
		 (i.e. the first will visit all the vertices of the second and vice versa)
		*/
		Solution swapRoute(Solution s,NeighborChoice strategy);

		void exchangeOrigin(Solution& s, NeighborChoice strategy);

		void exchangeDestination(Solution& s, NeighborChoice strategy);

		void exchangeConsecutive(Solution& s, NeighborChoice strategy);

		/*
		* Removal Ratio = 0 
		* Randomness = 1
		*/
		Solution WorstRemoval(Solution s, std::vector<double> arguments);

		//Removal Ratio = 0;
		Solution RandomRemoval(Solution s, std::vector<double> arguments);

		//Removal Ratio = 0;
		Solution SimilarityRemoval(Solution s, std::vector<double> arguments);

		/*
		* Removal Ratio = 0
		* Randomness = 1
		*/
		Solution ZeroSplitRemoval(Solution s, std::vector<double> arguments);

		// Removal Ratio = 0
		Solution EntireRouteRemoval(Solution s, std::vector<double> arguments);

		//Empty
		Solution GreedyInsertion(Solution s, std::vector<double> arguments);

		//Empty
		Solution LeastOptionsInsertion(Solution s, std::vector<double> arguments);

		/*
		* Regret Degree = 0
		*/
		Solution RegretInsertion(Solution s, std::vector<double> arguments);


		/*
		Proposed by Ying Luo and Paul Schonfeld in 2007
		*/
		void rejectedReinsertion(Solution& s);
    } 
}



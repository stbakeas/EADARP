#pragma once
#include "Run.h"
#include "RandLib.h" 
#include <iostream>
#include "Solution.h"
#include "Instance.h"

namespace algorithms
{
	enum class NeighborChoice { BEST, FIRST, RANDOM, SMART};
	
	/*
	* Multi-directional Iterated Greedy. Different destruction and construction steps
	* are designed for each objective.
	*/
	Run MDIG(std::vector<Solution> initPop, int max_iterations, int max_seconds);

	Run IteratedGreedy(Solution initial,int max_iterations, int max_seconds);

	Run DeterministicAnnealing(Solution initial, int Nb_iter, int T_max, int T_red, int n_imp);

	bool SimulatedAnnealingAcceptanceCriterion(Solution candidate, Solution incumbent, int current_temperature);

    namespace details
    {

		/**
	    * Define as "Move" a method that receives a Solution and returns another Solution.
	    */
		typedef Solution(*Move)(Solution s, NeighborChoice strategy);

		Route PairInsertion(Request* r, Solution s, std::vector<EAV*> available_vehicles,Instance::Objective objective,bool includeCS=false);

		double DistanceBetweenUsers(Request* a,Request* b);

		/*
		* Inspired by list heuristic proposed by Atahran et. al(2014)
		*/
		Solution H1(int iter);

        /**
         * Randomly selects a request and inserts it at its best position.
         */
		Solution Init1(int iter);

        /**
        * Parallel Insertion Heuristic proposed by Yue Su, Nicolas Dupin, Jakob Puchinger
        */
        Solution Init2();

		/**
		* First,selects seed request according to battery and time feasibility criteria.
		* Proceeds with simple greedy.
		*/
		Solution Init3(int iter,bool sortBasedOnShiftDuration);

		/*
		* Sequential Insertion Heuristic based on ascending order of shift duration.
		*/
		Solution Init4();

		/*
		 Removes a zero-load arcs from two routes and then connects 
		 the first part of the first route with the second part of the second route and vice versa.
		*/
		Solution two_opt(Solution s,NeighborChoice strategy);

		/*
		* Intra-route operator. Removes a charging station and reinserts it in every feasible position.
		*/
		Solution moveStation(Solution s,NeighborChoice strategy);

		/*
		 Removes a request from its route and reinserts it into its best position of another route.
		*/
		Solution relocate(Solution s,NeighborChoice strategy);

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

		Solution exchangeOrigin(Solution s, NeighborChoice strategy);

		Solution exchangeDestination(Solution s, NeighborChoice strategy);

		Solution exchangeConsecutive(Solution s, NeighborChoice strategy);

		Solution WorstRemoval(Solution s, double removal_ratio, float randomness);

		Solution GreedyInsertion(Solution s);

		Solution Destroy(Solution s, double removal_ratio, float randomness, Instance::Objective objective);
		
		Solution Repair(Solution s,Instance::Objective objective);


    } 
}



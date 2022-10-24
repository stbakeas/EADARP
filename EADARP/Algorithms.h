#pragma once
#include "Run.h"
#include <iostream>

namespace algorithms
{


    namespace details
    {
		struct Position {
			EAV* vehicle;
			int origin_pos;
			int dest_pos;
			double cost;
			CStation* charging_station;
			int cs_pos;

			Position() {};
			~Position() {};

			Position(EAV* vehicle, int origin_pos, int dest_pos,CStation* charging_station,int cs_pos) {
				this->vehicle = vehicle;
				this->origin_pos = origin_pos;
				this->dest_pos = dest_pos;
				this->charging_station = charging_station;
				this->cs_pos = cs_pos;
				cost = 0;
			}

			bool operator<(const Position& a) {
				if (this->vehicle->id < a.vehicle->id) return true;
				else if (this->vehicle->id > a.vehicle->id) return false;
				else {
					if (this->origin_pos < a.origin_pos) return true;
					else if (this->origin_pos > a.origin_pos) return false;
					else {
						if (this->dest_pos < a.dest_pos) return true;
						else return false;
					}
				}
			}

			bool operator!=(const Position& a) {
				if (this->vehicle->id != a.vehicle->id) return true;
				else {
					if (this->origin_pos != a.origin_pos) return true;
					else {
						if (this->dest_pos != a.dest_pos) return true;
						else {
							if (this->charging_station->id != a.charging_station->id) return true;
							else {
								if (this->cs_pos != a.cs_pos) return true;
								return false;
							}
						}
						
					}
				}
			}

			std::ostream& operator<<(std::ostream& os) {
				os << "(" << this->vehicle->id << " , " << this->origin_pos << " , " << this->dest_pos << ")";
				return os;
			}
		};

		double DistanceBetweenUsers(Request* a,Request* b);

        /**
         * Define as "Move" a method that receives a Solution and returns another Solution.
         */
        typedef Solution(*Move)(Solution s);

        /**
         * Construct an initial solution
         */
        Solution constructInitialSolution(int numberOfIterations);

        /**
        * Parallel Insertion Heuristic proposed by Yue Su, Nicolas Dupin, Jakob Puchinger
        */
        Solution ParallelInsertionHeuristic();

    } 
}



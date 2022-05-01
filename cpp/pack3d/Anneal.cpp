#include "Anneal.h"

#include <cmath>
#include <random>

ModelPtr Anneal(ModelPtr state, double maxTemp, double minTemp, int steps) {

	double factor = -std::log(maxTemp / minTemp);
	
	// ! not sure how to do this copy
	//state = state.Copy();
	ModelPtr bestState = std::make_shared<Model>();
	*bestState = *state;

	auto bestEnergy = state->Energy();
	auto previousEnergy = bestEnergy;
	auto rate = double(steps) / 200.0;
	
	for (auto step = 0; step < steps; ++step) {
		
		auto pct = double(step) / double(steps - 1);
		auto temp = maxTemp * std::exp(factor * pct);
		auto undo = state->DoMove();
		auto energy = state->Energy();
		auto change = energy - previousEnergy;

		if (change > 0 && std::exp(-change / temp) < dis(gen)/*rand.Float64()*/) {
			
			state->UndoMove(undo);
		}
		else {
			
			previousEnergy = energy;
			
			if (energy < bestEnergy) {

				bestEnergy = energy;
				*bestState = *state;
			}
		}
	}
	
	return bestState;
}
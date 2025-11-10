#include <iostream>
#include <string>
#include <thread>

#include "SaiGraphics.h"
#include "SaiSimulation.h"
#include "unistd.h"

using namespace std;

const string world_fname =
	string(EXAMPLES_FOLDER) + "/01-load_world_and_simulate/world.urdf";
const string robot_name = "RRBot";

bool fSimulationRunning = false;

// sim
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char **argv)
{
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_fname);

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_fname);
	graphics->setBackgroundColor(0.2, 0.2, 0.2);

	cout << endl
		 << "This example loads a double pendulum and simulates gravity. The "
			"graphics display is updated by the simulated robot joint angles. "
			"After a certain time, the simulated robot starts compensating for "
			"gravity. After some more time, the gravity compensation "
			"stops."
		 << endl
		 << endl;

	// start the simulation
	thread sim_thread(simulation, sim);

	unsigned long long counter = 0;

	// while window is open:
	while (graphics->isWindowOpen())
	{
		// update graphics from simulation.
		graphics->updateRobotGraphics(robot_name,
									  sim->getJointPositions(robot_name));
		graphics->renderGraphicsWorld();

		if (counter == 300)
		{
			cout << "\nenabling gravity compensation\n"
				 << endl;
			sim->enableGravityCompensation(true);
		}
		if (counter == 500)
		{
			cout << "\ndisabling gravity compensation\n"
				 << endl;
			sim->enableGravityCompensation(false);
		}
		counter++;
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim)
{
	fSimulationRunning = true;
	double timestep = sim->timestep();

	while (fSimulationRunning)
	{
		// wait
		usleep(timestep * 1e6);
		// integrate forward
		sim->integrate();
	}
}
// This example application loads a URDF world file and simulates two robots
// with physics and contact in a SaiSimulation virtual world. A graphics model
// of it is also shown using Chai3D.

#include <SaiGraphics.h>
#include <SaiSimulation.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/07-contact_info/world.urdf";
const string robot_name = "PPPBot";

unsigned long long simulation_counter = 0;
Eigen::VectorXd q_robot;

int main()
{
	SaiModel::URDF_FOLDERS["EXAMPLE_07_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/07-contact_info";
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new SaiSimulation::SaiSimulation(world_file);
	sim->setTimestep(0.01);

	// load graphics scene
	auto graphics = new SaiGraphics::SaiGraphics(world_file);

	// offset a joint initial condition
	sim->setJointPosition(robot_name, 0,
						  sim->getJointPositions(robot_name)[0] + 0.5);
	sim->setCollisionRestitution(0);

	// while window is open:
	while (graphics->isWindowOpen())
	{
		// update simulation
		sim->integrate();

		// update graphics.
		graphics->updateRobotGraphics(robot_name,
									  sim->getJointPositions(robot_name));
		for (const auto sensor_data : sim->getAllForceSensorData())
		{
			graphics->updateDisplayedForceSensor(sensor_data);
		}
		graphics->renderGraphicsWorld();

		if (simulation_counter % 100 == 0)
		{
			auto contact_list = sim->getContactList(robot_name, "sensor_link");
			if (!contact_list.empty())
			{
				std::cout << "contact at " << robot_name << " sensor_link"
						  << std::endl;
				int n = contact_list.size();
				for (int i = 0; i < n; i++)
				{
					std::cout << "contact point " << i << " : "
							  << contact_list[i].first.transpose() << "\n";
					std::cout << "contact force " << i << " : "
							  << contact_list[i].second.transpose() << "\n";
				}
				std::cout << endl;
			}
		}

		simulation_counter++;
	}

	return 0;
}
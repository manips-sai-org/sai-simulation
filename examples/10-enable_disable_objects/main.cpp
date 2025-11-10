#include <SaiGraphics.h>
#include <SaiSimulation.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/10-enable_disable_objects/world.urdf";

int main()
{
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);
	sim->setTimestep(0.01);
	sim->setCollisionRestitution(0);
	sim->enableGravityCompensation(true);
	VectorXd init_joint_positions = VectorXd::Zero(sim->dof("PANDA"));
	init_joint_positions << 0, 25, 15, -110, 90, 90, 0;
	sim->setJointPositions("PANDA", init_joint_positions * M_PI / 180.0);
	Affine3d init_object_pose = sim->getObjectPose("Box1");

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);

	// graphics->showLinkFrame(true, "PANDA", "", 0.3);
	sim->setDynamicsEnabled(false, "PANDA");
	graphics->setRenderingEnabled(false, "PANDA");

	unsigned long long counter = 0;
	// while window is open:
	while (graphics->isWindowOpen())
	{
		// update simulation
		sim->integrate();

		// update graphics.
		for (const auto &object_name : sim->getObjectNames())
		{
			graphics->updateObjectGraphics(object_name,
										   sim->getObjectPose(object_name),
										   sim->getObjectVelocity(object_name));
		}
		for (const auto &robot_name : sim->getRobotNames())
		{
			graphics->updateRobotGraphics(robot_name,
										  sim->getJointPositions(robot_name),
										  sim->getJointVelocities(robot_name));
		}

		graphics->renderGraphicsWorld();

		if (counter % 500 == 0)
		{
			cout << "Resetting object pose" << endl;
			sim->setObjectPose("Box1", init_object_pose);
			sim->setObjectVelocity("Box1", Vector3d::Zero());
		}

		if (counter == 1025)
		{
			cout << "Disabling dynamics for object" << endl;
			sim->setDynamicsEnabled(false, "Box1");
		}
		if (counter == 1400)
		{
			cout << "disabling rendering for object" << endl;
			graphics->setRenderingEnabled(false, "Box1");
		}
		if (counter == 1500)
		{
			cout << "Enabling dynamics and rendering for robot" << endl;
			sim->setDynamicsEnabled(true, "PANDA");
			graphics->setRenderingEnabled(true, "PANDA");
		}
		if (counter == 2000)
		{
			cout << "Enabling dynamics and rendering for object" << endl;
			sim->setDynamicsEnabled(true, "Box1");
			graphics->setRenderingEnabled(true, "Box1");
		}
		if (counter == 2499)
		{
			cout << "Exiting" << endl;
			break;
		}

		counter++;
	}

	return 0;
}
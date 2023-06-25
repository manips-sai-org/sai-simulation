/*
 * Sai2Simulation.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 *  Update: ported to sai2-simulation project on Nov 17, 2017
 *    By: Shameek Ganguly
 */

#ifndef SAI2_SIMULATION_H
#define SAI2_SIMULATION_H

#include <Sai2Model.h>

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <vector>

#include "force_sensor_sim/ForceSensorSim.h"

// forward define from sai2-simulation
class cDynamicWorld;

namespace Sai2Simulation {

class Sai2Simulation {
public:
	/**
	 * @brief Creates a simulation interface to a Sai2-Simulation engine.
	 * Supports time integration, constraint-based contact and collision
	 * resolution.
	 * @param path_to_world_file A path to the file containing the model of the
	 * virtual world (urdf and yml files supported).
	 * @param verbose To display information about the robot model creation in
	 * the terminal or not.
	 */
	Sai2Simulation(const std::string& path_to_world_file, bool verbose = false);

	// \brief Destructor to clean up internal Sai2-Simulation model
	~Sai2Simulation() = default;

	/**
	 * @brief Get degrees of freedom of a particular robot.
	 *        NOTE: Assumes serial or tree chain robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 */
	unsigned int dof(const std::string& robot_name) const;
	unsigned int q_size(const std::string& robot_name) const;

	void resetWorld(const std::string& path_to_world_file,
					bool verbose = false);

	double timestep() const { return _timestep; }
	void setTimestep(const double dt);

	double time() const { return _time; }

	double isPaused() const { return _is_paused; }
	void pause() { _is_paused = true; }
	void unpause() { _is_paused = false; }

	void enableGravityCompensation(const bool enable) {
		_gravity_compensation_enabled = enable;
	}
	bool isGravityCompensationEnabled() const {
		return _gravity_compensation_enabled;
	}

	/**
	 * @brief Set joint positions as an array. Assumes serial or tree chain
	 * robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param q Desired joint position values.
	 */
	void setJointPositions(const std::string& robot_name,
						   const Eigen::VectorXd& q);

	/**
	 * @brief Read back joint positions as an array.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param q_ret Array to write back joint positions into.
	 */
	void getJointPositions(const std::string& robot_name,
						   Eigen::VectorXd& q_ret) const;

	/**
	 * @brief Read back position and orientation of an object.
	 * @param object_name Name of the object for which transaction is required.
	 * @param pos Array to write back position into.
	 * @param ori Quaternion to write back orientation into.
	 */
	void getObjectPosition(const std::string& object_name, Eigen::Vector3d& pos,
						   Eigen::Quaterniond& ori) const;

	// set object pose
	void setObjectPosition(const std::string& object_name,
						   const Eigen::Vector3d& pos,
						   const Eigen::Quaterniond& ori) const;

	/**
	 * @brief Set joint position for a single joint
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param joint_id Joint number on which to set value.
	 * @param position Value to set.
	 */
	void setJointPosition(const std::string& robot_name, unsigned int joint_id,
						  double position);

	/**
	 * @brief Set joint velocities as an array. Assumes serial or tree chain
	 * robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param dq Desired joint velocity values.
	 */
	void setJointVelocities(const std::string& robot_name,
							const Eigen::VectorXd& dq);

	/**
	 * @brief Set joint velocity for a single joint
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param joint_id Joint number on which to set value.
	 * @param velocity Value to set.
	 */
	void setJointVelocity(const std::string& robot_name, unsigned int joint_id,
						  double velocity);

	/**
	 * @brief Read back joint velocities as an array.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param dq_ret Array to write back joint velocities into.
	 */
	void getJointVelocities(const std::string& robot_name,
							Eigen::VectorXd& dq_ret) const;

	/**
	 * @brief Read back linear and angular velocities of an object.
	 * @param object_name Name of the object for which transaction is required.
	 * @param lin_vel Array to write back linear velocity into.
	 * @param ang_vel Array to write back angular velocity into.
	 */
	void getObjectVelocity(const std::string& object_name,
						   Eigen::Vector3d& lin_vel,
						   Eigen::Vector3d& ang_vel) const;

	/**
	 * @brief Set joint torques as an array. Assumes serial or tree chain robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param tau Desired joint torque values.
	 */
	void setJointTorques(const std::string& robot_name,
						 const Eigen::VectorXd& tau);

	/**
	 * @brief Set joint torque for a single joint
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param joint_id Joint number on which to set value.
	 * @param tau Value to set.
	 */
	void setJointTorque(const std::string& robot_name, unsigned int joint_id,
						double tau);

	/**
	 * @brief Read back joint accelerations as an array.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param ddq_ret Array to write back joint accelerations into.
	 */
	void getJointAccelerations(const std::string& robot_name,
							   Eigen::VectorXd& ddq_ret) const;

	/**
	 * @brief Integrate the virtual world over the time step (1ms by default or
	 * defined by calling the setTimestep function).
	 */
	void integrate();

	/**
	 * @brief      Shows the contact information, whenever a contact occurs
	 */
	void showContactInfo();

	/**
	 * @brief Shows which links of the given robot are in contact
	 *
	 * @param robot_name the robot name
	 */
	void showLinksInContact(const std::string robot_name);

	/**
	 * @brief      Gets a vector of contact points and the corresponding contact
	 * forces at a gicen link of a given object. Gives everything in base frame.
	 *
	 * @param      contact_points  The contact points vector to be returned
	 * @param      contact_forces  The contact forces vector to be returned
	 * @param[in]  robot_name      The robot name
	 * @param[in]  link_name       The link name
	 */
	void getContactList(std::vector<Eigen::Vector3d>& contact_points,
						std::vector<Eigen::Vector3d>& contact_forces,
						const ::std::string& robot_name,
						const std::string& link_name);

	void addSimulatedForceSensor(
		const std::string& robot_name, const std::string& link_name,
		const Eigen::Affine3d transform_in_link = Eigen::Affine3d::Identity());

	Eigen::Vector3d getSensedForce(const std::string& robot_name,
								   const std::string& link_name,
								   const bool in_sensor_frame = true) const;
	Eigen::Vector3d getSensedMoment(const std::string& robot_name,
									const std::string& link_name,
									const bool in_sensor_frame = true) const;

	std::vector<Sai2Model::ForceSensorData> getAllForceSensorData() const;

	/* Sai2-Simulation specific interface */

	/**
	 * @brief      Set the co-efficient of kinematic restitution: for all
	 * objects
	 * @param      restitution  Value to set
	 */
	void setCollisionRestitution(double restitution);

	/**
	 * @brief      Set the co-efficient of kinematic restitution: for a named
	 * object
	 * @param      object_name  Object on which to set the value
	 * @param      restitution  Value to set
	 */
	void setCollisionRestitution(const std::string& object_name,
								 double restitution);

	/**
	 * @brief      Set the co-efficient of kinematic restitution: for a named
	 * link on a named robot
	 * @param      robot_name  Robot on which to set the value
	 * @param      link_name  Robot on which to set the value
	 * @param      restitution  Value to set
	 */
	void setCollisionRestitution(const std::string& robot_name,
								 const std::string& link_name,
								 double restitution);

	/**
	 * @brief      Get the co-efficient of kinematic restitution: for a named
	 * object
	 * @param      object_name  Object from which to get the value
	 * @return     Current value
	 */
	double getCollisionRestitution(const std::string& object_name) const;

	/**
	 * @brief      Get the co-efficient of kinematic restitution: for a named
	 * link on a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @param      link_name  Robot from which to get the value
	 * @return     Current value
	 */
	double getCollisionRestitution(const std::string& robot_name,
								   const std::string& link_name) const;

	/**
	 * @brief      Set the co-efficient of static friction: for all objects
	 * @param      restitution  Value to set
	 */
	void setCoeffFrictionStatic(double static_friction);  // for all objects

	/**
	 * @brief      Set the co-efficient of kinematic restitution: for a named
	 * object
	 * @param      object_name  Object on which to set the value
	 * @param      restitution  Value to set
	 */
	void setCoeffFrictionStatic(const std::string& object_name,
								double static_friction);

	/**
	 * @brief      Set the co-efficient of kinematic restitution: for a named
	 * link on a named robot
	 * @param      robot_name  Robot on which to set the value
	 * @param      link_name  Robot on which to set the value
	 * @param      restitution  Value to set
	 */
	void setCoeffFrictionStatic(const std::string& robot_name,
								const std::string& link_name,
								double static_friction);

	/**
	 * @brief      Get the co-efficient of static friction: for a named object
	 * @param      object_name  Robot from which to get the value
	 * @return     Current value
	 */
	double getCoeffFrictionStatic(const std::string& object_name) const;

	/**
	 * @brief      Get the co-efficient of static friction: for a named link on
	 * a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @param      link_name  Robot from which to get the value
	 * @return     Current value
	 */
	double getCoeffFrictionStatic(const std::string& robot_name,
								  const std::string& link_name) const;

	/**
	 * @brief      Set the co-efficient of dynamic friction: for all objects
	 * @param      restitution  Value to set
	 */
	void setCoeffFrictionDynamic(double dynamic_friction);	// for all objects

	/**
	 * @brief      Set the co-efficient of kinematic restitution: for a named
	 * object
	 *
	 * @param      object_name  Robot on which to set the value
	 * @param      restitution  Value to set
	 */
	void setCoeffFrictionDynamic(const std::string& object_name,
								 double dynamic_friction);

	/**
	 * @brief      Set the co-efficient of kinematic restitution: for a named
	 * link on a named robot
	 *
	 * @param      robot_name  Robot on which to set the value
	 * @param      link_name  Robot on which to set the value
	 * @param      restitution  Value to set
	 */
	void setCoeffFrictionDynamic(const std::string& robot_name,
								 const std::string& link_name,
								 double dynamic_friction);

	/**
	 * @brief      Get the co-efficient of dynamic friction: for a named object
	 * @param      object_name  Robot from which to get the value
	 * @return     Current value
	 */
	double getCoeffFrictionDynamic(const std::string& object_name) const;

	/**
	 * @brief      Get the co-efficient of dynamic friction: for a named link on
	 * a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @param      link_name  Robot from which to get the value
	 * @return     Current value
	 */
	double getCoeffFrictionDynamic(const std::string& robot_name,
								   const std::string& link_name) const;

	/**
	 * @brief      Get affine transform from the global frame to the base frame
	 * of a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @return     Transform
	 */
	Eigen::Affine3d getRobotBaseTransform(const std::string& robot_name) const;

	const std::shared_ptr<cDynamicWorld> getDynamicWorld() const {
		return _world;
	}

	std::vector<std::string> getRobotNames() const;

private:
	bool existsInSimulatedWorld(const std::string& robot_or_object_name,
								const std::string link_name = "") const;

	int findSimulatedForceSensor(const std::string& robot_name,
								 const std::string& link_name) const;

	void setAllJointTorquesInternal();

	/**
	 * @brief Internal dynamics world object.
	 */
	std::shared_ptr<cDynamicWorld> _world;

	std::map<std::string, std::string> _robot_filenames;
	std::map<std::string, std::shared_ptr<Sai2Model::Sai2Model>> _robot_models;
	std::map<std::string, Eigen::VectorXd> _applied_robot_torques;

	bool _is_paused;
	double _time;
	double _timestep = 0.001;

	bool _gravity_compensation_enabled;

	std::vector<std::shared_ptr<ForceSensorSim>> _force_sensors;

	std::map<std::string, Eigen::Vector3d> _dyn_object_base_pos;
	std::map<std::string, Eigen::Quaterniond> _dyn_object_base_rot;
};

}  // namespace Sai2Simulation

#endif	// SAI2_SIMULATION_H

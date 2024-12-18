/*
 * SaiSimulation.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 *  Update: ported to sai-simulation project on Nov 17, 2017
 *    By: Shameek Ganguly
 */

#ifndef SAI_SIMULATION_H
#define SAI_SIMULATION_H

#include <SaiModel.h>

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <vector>

#include "force_sensor_sim/ForceSensorSim.h"

// forward define from sai-simulation
class cDynamicWorld;

namespace SaiSimulation {

class SaiSimulation {
public:
	/**
	 * @brief Creates a simulation interface to a Sai-Simulation engine.
	 * Supports time integration, constraint-based contact and collision
	 * resolution.
	 * @param path_to_world_file A path to the file containing the model of the
	 * virtual world (urdf and yml files supported).
	 * @param verbose To display information about the robot model creation in
	 * the terminal or not.
	 */
	SaiSimulation(const std::string& path_to_world_file, bool verbose = false);

	// \brief Destructor to clean up internal Sai-Simulation model
	~SaiSimulation() = default;

	/**
	 * @brief Get degrees of freedom of a particular robot.
	 *        NOTE: Assumes serial or tree chain robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 */
	const unsigned int dof(const std::string& robot_name) const;
	const unsigned int qSize(const std::string& robot_name) const;

	void resetWorld(const std::string& path_to_world_file,
					bool verbose = false);

	const double& timestep() const { return _timestep; }
	void setTimestep(const double dt);

	const double& time() const { return _time; }

	const bool& isPaused() const { return _is_paused; }
	void pause() { _is_paused = true; }
	void unpause() { _is_paused = false; }

	void enableJointLimits(const std::string& robot_name);
	void disableJointLimits(const std::string& robot_name);

	void enableGravityCompensation(const bool enable) {
		_gravity_compensation_enabled = enable;
	}
	const bool& isGravityCompensationEnabled() const {
		return _gravity_compensation_enabled;
	}

	const Vector3d getWorldGravity() const {
		return _world->getGravity().eigen();
	}

	const MatrixXd computeAndGetMassMatrix(const std::string& robot_name);

	/**
	 * @brief Set joint positions as an array. Assumes serial or tree chain
	 * robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param q Desired joint position values.
	 */
	void setJointPositions(const std::string& robot_name,
						   const Eigen::VectorXd& q);

	/**
	 * @brief Set joint position for a single joint
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param joint_id Joint number on which to set value.
	 * @param position Value to set.
	 */
	void setJointPosition(const std::string& robot_name, unsigned int joint_id,
						  double position);

	/**
	 * @brief Read back joint positions as an array.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @return joint positions for that robot.
	 */
	const Eigen::VectorXd getJointPositions(
		const std::string& robot_name) const;

	/**
	 * @brief Read back position and orientation of an object (dynamic or static).
	 * @param object_name Name of the object for which transaction is required.
	 * @return pose of that object.
	 */
	const Eigen::Affine3d getObjectPose(const std::string& object_name) const;

	/**
	 * @brief Set the Object Pose of a dynamic object
	 * 
	 * @param object_name 
	 * @param pose 
	 */
	void setObjectPose(const std::string& object_name,
					   const Eigen::Affine3d& pose) const;

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
	 * @brief Get the joint velocities for a robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @return joint velocities for that robot.
	 */
	const Eigen::VectorXd getJointVelocities(
		const std::string& robot_name) const;

	/**
	 * @brief Read back linear and angular velocities of an object as a 6d
	 * vector (linear first, angular second). Dynamic objects only.
	 * @param object_name Name of the object for which transaction is required.
	 * @return a 6d vector containing [lin_vel, ang_vel].
	 */
	const Eigen::VectorXd getObjectVelocity(
		const std::string& object_name) const;

	/**
	 * @brief Set the Object Velocity (dynamic objects only)
	 *
	 * @param object_name
	 * @param linear_velocity
	 * @param angular_velocity
	 */
	void setObjectVelocity(
		const std::string& object_name, const Eigen::Vector3d& linear_velocity,
		const Eigen::Vector3d& angular_velocity = Eigen::Vector3d::Zero());

	/**
	 * @brief Set joint torques as an array. Assumes serial or tree chain
	 * robot.
	 * @param robot_name Name of the robot for which transaction is
	 * required.
	 * @param tau Desired joint torque values.
	 */
	void setJointTorques(const std::string& robot_name,
						 const Eigen::VectorXd& tau);

	/**
	 * @brief Set the force and torque for the given object (dynamic object only), expressed in world
	 * frame (xyz force first, xyz torques second)
	 *
	 * @param object_name
	 * @param tau
	 */
	void setObjectForceTorque(const std::string& object_name,
							  const Eigen::Vector6d& force_torque);

	/**
	 * @brief Set joint torque for a single joint
	 * @param robot_name Name of the robot for which transaction is required.
	 * @param joint_id Joint number on which to set value.
	 * @param tau Value to set.
	 */
	void setJointTorque(const std::string& robot_name, unsigned int joint_id,
						double tau);

	/**
	 * @brief Get the joint accelerations for a given robot.
	 * @param robot_name Name of the robot for which transaction is required.
	 * @return joint accelerations for that robot.
	 */
	const Eigen::VectorXd getJointAccelerations(
		const std::string& robot_name) const;

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
	 * @brief Shows which links of the given robot or object are in contact
	 *
	 * @param robot_or_object_name the robot or object name
	 */
	void showLinksInContact(const std::string robot_or_object_name);

	/**
	 * @brief      Gets the list of contacts on a given robot at a given link
	 *
	 * @param[in]  robot_name      The robot name
	 * @param[in]  link_name       The link name
	 * @return  a vector of pairs, the first element of the pair contains the
	 * location of the contact point in world coordinates, and the second
	 * contains the contact force in world coordinates
	 */
	const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
	getContactList(const ::std::string& robot_name,
				   const std::string& link_name) const;

	/**
	 * @brief      Gets the list of contacts on a given object
	 *
	 * @param[in]  object_name      The object name
	 * @return  a vector of pairs, the first element of the pair contains the
	 * location of the contact point in world coordinates, and the second
	 * contains the contact force in world coordinates
	 */
	const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
	getContactList(const ::std::string& object_name) const;

	/**
	 * @brief Adds a simulated sorce sensor to a given robot at a given link,
	 * with possibility to filter the force data with a second order butterworth
	 * filter. Only one force sensor can be added to a given link
	 *
	 * @param robot_name name of the robot to which to add the force sensor
	 * @param link_name link to which we want to add the sensor
	 * @param transform_in_link transform from the link frame to the sensor
	 * frame
	 * @param filter_cutoff_frequency cutoff frequency of the filter (in Hz). If
	 * 0, the filter is not enabled
	 */
	void addSimulatedForceSensor(
		const std::string& robot_name, const std::string& link_name,
		const Eigen::Affine3d transform_in_link = Eigen::Affine3d::Identity(),
		const double filter_cutoff_frequency = 0.0);
	void addSimulatedForceSensor(
		const std::string& object_name,
		const Eigen::Affine3d transform_in_link = Eigen::Affine3d::Identity(),
		const double filter_cutoff_frequency = 0.0);

	/**
	 * @brief Get the Sensed Force for a given robot at a given link (only if a
	 * sensor was added to that link). The force can be expressed in the world
	 * frame or in the sensor frame (default is sensor frame). This represents
	 * the force that the robot applies to the environment
	 *
	 * @param robot_name the name of the robot that has the sensor
	 * @param link_name the name of the link that has the sensor
	 * @param in_sensor_frame if true, the force is expressed in the sensor
	 * frame, if false, it is in world frame
	 * @return const Eigen::Vector3d& The force sensed by the simulated force
	 * sensor
	 */
	Eigen::Vector3d getSensedForce(const std::string& robot_name,
								   const std::string& link_name,
								   const bool in_sensor_frame = true) const;
	Eigen::Vector3d getSensedForce(const std::string& object_name,
								   const bool in_sensor_frame = true) const;

	/**
	 * @brief Get the Sensed Moment for a given robot at a given link (only if a
	 * sensor was added to that link). The moment can be expressed in the world
	 * frame or in the sensor frame (default is sensor frame). This represents
	 * the moment that the robot applies to the environment
	 *
	 * @param robot_name the name of the robot that has the sensor
	 * @param link_name the name of the link that has the sensor
	 * @param in_sensor_frame if true, the moment is expressed in the sensor
	 * frame, if false, it is in world frame
	 * @return const Eigen::Vector3d& The moment sensed by the simulated force
	 * sensor
	 */
	Eigen::Vector3d getSensedMoment(const std::string& robot_name,
									const std::string& link_name,
									const bool in_sensor_frame = true) const;
	Eigen::Vector3d getSensedMoment(const std::string& object_name,
									const bool in_sensor_frame = true) const;

	/**
	 * @brief Get the All Force Sensor Data object for all the sensors that have
	 * been added to the simulation
	 *
	 * @return const std::vector<SaiModel::ForceSensorData> a vector of all the
	 * most recent force sensor data
	 */
	const std::vector<SaiModel::ForceSensorData> getAllForceSensorData() const;

	/**
	 * @brief Enable or disable the dynamics for a given object or robot
	 * If dynamics are disabled, the object/robot won't be considered in the
	 * integration step (so it won't move and won't be considered for
	 * collisions)
	 *
	 * @param enabled bolean to enable or disable the dynamics
	 * @param robot_or_object_name
	 */
	void setDynamicsEnabled(const bool enabled,
							const string robot_or_object_name);

	void setJointDamping(const double damping, const string robot_or_object_name = "",
					const string link_name = "");

	/**
	 * @brief Set the Collision Restitution for all objects, or a specific
	 * object, and a specific link
	 *
	 * @param restitution the coefficient to set
	 * @param robot_or_object_name if empty, will apply to all robots and
	 * objects
	 * @param link_name if empty, will apply to all the links of the given
	 * robot/object
	 */
	void setCollisionRestitution(const double restitution,
								 const string robot_or_object_name = "",
								 const string link_name = "");

	/**
	 * @brief      Get the co-efficient of kinematic restitution: for a named
	 * object
	 * @param      object_name  Object from which to get the value
	 * @return     Current value
	 */
	const double getCollisionRestitution(const std::string& object_name) const;

	/**
	 * @brief      Get the co-efficient of kinematic restitution: for a named
	 * link on a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @param      link_name  Robot from which to get the value
	 * @return     Current value
	 */
	const double getCollisionRestitution(const std::string& robot_name,
										 const std::string& link_name) const;

	/**
	 * @brief Set the static friction coefficient for all objects, or a specific
	 * object, and a specific link
	 *
	 * @param static_friction the coefficient to set
	 * @param robot_or_object_name if empty, will apply to all robots and
	 * objects
	 * @param link_name if empty, will apply to all the links of the given
	 * robot/object
	 */
	void setCoeffFrictionStatic(const double static_friction,
								const string robot_or_object_name = "",
								const string link_name = "");

	/**
	 * @brief      Get the co-efficient of static friction: for a named object
	 * @param      object_name  Robot from which to get the value
	 * @return     Current value
	 */
	const double getCoeffFrictionStatic(const std::string& object_name) const;

	/**
	 * @brief      Get the co-efficient of static friction: for a named link on
	 * a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @param      link_name  Robot from which to get the value
	 * @return     Current value
	 */
	const double getCoeffFrictionStatic(const std::string& robot_name,
										const std::string& link_name) const;

	/**
	 * @brief Set the dynamic friction coefficient for all objects, or a
	 * specific object, and a specific link
	 *
	 * @param dynamic_friction the coefficient to set
	 * @param robot_or_object_name if empty, will apply to all robots and
	 * objects
	 * @param link_name if empty, will apply to all the links of the given
	 * robot/object
	 */
	void setCoeffFrictionDynamic(const double dynamic_friction,
								 const string robot_or_object_name = "",
								 const string link_name = "");

	/**
	 * @brief      Get the co-efficient of dynamic friction: for a named object
	 * @param      object_name  Robot from which to get the value
	 * @return     Current value
	 */
	const double getCoeffFrictionDynamic(const std::string& object_name) const;

	/**
	 * @brief      Get the co-efficient of dynamic friction: for a named link on
	 * a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @param      link_name  Robot from which to get the value
	 * @return     Current value
	 */
	const double getCoeffFrictionDynamic(const std::string& robot_name,
										 const std::string& link_name) const;

	/**
	 * @brief      Get affine transform from the global frame to the base frame
	 * of a named robot
	 * @param      robot_name  Robot from which to get the value
	 * @return     Transform
	 */
	const Eigen::Affine3d getRobotBaseTransform(
		const std::string& robot_name) const;

	const std::shared_ptr<cDynamicWorld>& getDynamicWorld() const {
		return _world;
	}

	const std::vector<std::string> getRobotNames() const;
	const std::vector<std::string> getObjectNames() const;

	const bool modelExistsInWorld(const std::string& model_name) const {
		return robotExistsInWorld(model_name) ||
			   dynamicObjectExistsInWorld(model_name) ||
			   staticObjectExistsInWorld(model_name);
	}

	const bool robotExistsInWorld(const std::string& robot_name,
								  const std::string link_name = "") const;

	const bool dynamicObjectExistsInWorld(const std::string& object_name) const;
	const bool staticObjectExistsInWorld(const std::string& object_name) const;

private:
	const int findSimulatedForceSensor(const std::string& robot_or_object_name,
									   const std::string& link_name) const;

	void setAllJointTorquesInternal();

	/**
	 * @brief Internal dynamics world object.
	 */
	std::shared_ptr<cDynamicWorld> _world;

	std::map<std::string, std::string> _robot_filenames;
	std::map<std::string, std::shared_ptr<SaiModel::SaiModel>> _robot_models;
	std::map<std::string, Eigen::VectorXd> _applied_robot_torques;
	std::map<std::string, Eigen::Matrix<double, 6, 1>> _applied_object_torques;

	bool _is_paused;
	double _time;
	double _timestep;

	bool _gravity_compensation_enabled;

	std::vector<std::shared_ptr<ForceSensorSim>> _force_sensors;

	std::map<std::string, Eigen::Affine3d> _dyn_objects_init_pose;
	std::map<std::string, Eigen::Affine3d> _static_objects_pose;

	std::map<std::string, std::shared_ptr<Eigen::Affine3d>> _dyn_objects_pose;
	std::map<std::string, Eigen::VectorXd> _dyn_objects_velocity;
};

}  // namespace SaiSimulation

#endif	// SAI_SIMULATION_H

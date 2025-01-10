// ForceSensorSim.h
// Force sensor for SAI-Simulation
#ifndef FORCE_SENSOR_SIM_H
#define FORCE_SENSOR_SIM_H

#include <SaiModel.h>
#include <filters/ButterworthFilter.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "dynamics3d.h"

namespace SaiSimulation {

/**
 * @brief Class to represent a simulated force sensor.
 * note that this implementation ignores the mass and inertia of the object
 * attached beyond the force sensor. note also that this assumes the force
 * sensor to be located just between the link and the end-effector. That is, it
 * cannot account for any joint reaction forces in case the sensor is to be
 * attached on the link between two joints.
 *
 */
class ForceSensorSim {
public:
	/**
	 * @brief Construct a new simulated force sensor attached to a robot
	 *
	 * @param robot_name name of the robot to which the sensor is attached
	 * @param link_name name of the link to which the sensor is attached
	 * @param transform_in_link pose of the sensor in the link frame
	 * @param model robot model of the robot to which the sensor is attached
	 * (used for reading purposes)
	 * @param filter_normalized_cutoff_freq normalized cutoff frequency for the
	 * force and moment filter (0.0 for no filtering, must be between 0 and 0.5)
	 */
	ForceSensorSim(const std::string& robot_name, const std::string& link_name,
				   const Eigen::Affine3d& transform_in_link,
				   std::shared_ptr<SaiModel::SaiModel> model,
				   const double filter_normalized_cutoff_freq = 0.0);

	/**
	 * @brief Construct a new simulated force sensor attached to an object
	 *
	 * @param object_name name of the object to which the sensor is attached
	 * @param link_name name of the link to which the sensor is attached
	 * @param transform_in_link pose of the sensor in the link frame
	 * @param object_pose pointer to the object pose (used for reading purposes)
	 * @param filter_normalized_cutoff_freq normalized cutoff frequency for the
	 * force and moment filter (0.0 for no filtering, must be between 0 and 0.5)
	 */
	ForceSensorSim(const std::string& object_name, const std::string& link_name,
				   const Eigen::Affine3d& transform_in_link,
				   std::shared_ptr<Eigen::Affine3d> object_pose,
				   const double filter_normalized_cutoff_freq = 0.0);

	/// @brief destructor
	~ForceSensorSim();

	/**
	 * @brief Update the force sensor with the latest data from the simulation
	 * 
	 * @param dyn_world 
	 */
	void update(const std::shared_ptr<cDynamicWorld> dyn_world);

	/// @brief get force applied by sensor body to the environment in world coordinates
	const Eigen::Vector3d& getForceWorldFrame() const {
		return _data.force_world_frame;
	};

	/// @brief get force applied by sensor body to the environment in local sensor frame
	const Eigen::Vector3d& getForceLocalFrame() const {
		return _data.force_local_frame;
	};

	/// @brief get moment applied by sensor body to the environment in world coordinates
	const Eigen::Vector3d& getMomentWorldFrame() const {
		return _data.moment_world_frame;
	};

	/// @brief get moment applied by sensor body to the environment in local sensor
	// frame
	const Eigen::Vector3d& getMomentLocalFrame() const {
		return _data.moment_local_frame;
	};

	/// @brief get full data
	SaiModel::ForceSensorData getData() const { return _data; }

	/// @brief Enable filtering of the force and moment data
	void enableFilter(const double normalized_cutoff_freq);

	/// @brief get the normalized cutoff frequency of the filter
	/// @return the normalized cutoff frequency
	double getNormalizedCutoffFreq() const {
		return _filter_force->getNormalizedCutoffFreq();
	}

private:
	/// @brief handle to model interface if the sesnor is attached to a robot
	std::shared_ptr<SaiModel::SaiModel> _robot;

	/// @brief handle to object pose if the sensor is attached to an object
	std::shared_ptr<Eigen::Affine3d> _object_pose;

	/// @brief last updated data
	SaiModel::ForceSensorData _data;

	/// @brief flag to know if the filter is enabled
	bool _use_filter;
	/// @brief filter for force
	std::unique_ptr<SaiCommon::ButterworthFilter> _filter_force;
	/// @brief filter for moment
	std::unique_ptr<SaiCommon::ButterworthFilter> _filter_moment;
};

}  // namespace SaiSimulation

#endif	// FORCE_SENSOR_SIM_H

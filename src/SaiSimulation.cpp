/*
 * SaiSimulation.cpp
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 *  Update: ported to sai-simulation project on Nov 17, 2017
 *		By: Shameek Ganguly
 */

#include "SaiSimulation.h"

#include <math/CMatrix3d.h> // chai math library

#include <iostream>
#include <map>
#include <set>
#include <stdexcept>

#include "dynamics3d.h"
#include "parser/URDFToDynamics3d.h"

namespace SaiSimulation
{

	// ctor
	SaiSimulation::SaiSimulation(const std::string &path_to_world_file,
								 bool verbose)
	{
		resetWorld(path_to_world_file, verbose);
		_timestep = 0.001;
	}

	void SaiSimulation::resetWorld(const std::string &path_to_world_file,
								   bool verbose)
	{
		_is_paused = false;
		_time = 0;
		_gravity_compensation_enabled.clear();

		// clean up robot names, models, torques and force sensors
		_robot_filenames.clear();
		_robot_models.clear();
		_force_sensors.clear();
		_applied_robot_torques.clear();
		_applied_object_torques.clear();
		_dyn_objects_init_pose.clear();
		_dyn_objects_pose.clear();
		_static_objects_pose.clear();
		_dyn_objects_velocity.clear();

		// create a dynamics world
		_world = std::make_shared<cDynamicWorld>(nullptr);

		URDFToDynamics3dWorld(path_to_world_file, _world, _dyn_objects_init_pose,
							  _static_objects_pose, _robot_filenames, verbose);

		// enable dynamics for all robots and objects in this world
		for (auto dyn_element : _world->m_dynamicObjects)
		{
			dyn_element->enableDynamics(true);
		}

		setCollisionRestitution(0);

		// create robot models
		for (const auto &pair : _robot_filenames)
		{
			const auto &robot_name = pair.first;
			const auto &robot_file = pair.second;
			_robot_models[robot_name] =
				std::make_shared<SaiModel::SaiModel>(robot_file, false);
			_robot_models.at(robot_name)
				->setTRobotBase(getRobotBaseTransform(robot_name));
			_robot_models.at(robot_name)
				->setWorldGravity(_world->getGravity().eigen());
			_applied_robot_torques[robot_name] =
				Eigen::VectorXd::Zero(dof(robot_name));
			_gravity_compensation_enabled[robot_name] = false;
			setJointPositions(robot_name, _robot_models.at(robot_name)->q());
			enableJointLimits(robot_name);
		}
		for (const auto &object_name : getObjectNames())
		{
			_applied_object_torques[object_name] = Eigen::VectorXd::Zero(6);
			_dyn_objects_pose[object_name] = std::make_shared<Eigen::Affine3d>(
				_dyn_objects_init_pose.at(object_name));
			_dyn_objects_velocity[object_name] = Eigen::VectorXd::Zero(6);
		}
	}

	const std::vector<std::string> SaiSimulation::getRobotNames() const
	{
		std::vector<std::string> robot_names;
		for (const auto &it : _robot_filenames)
		{
			robot_names.push_back(it.first);
		}
		return robot_names;
	}

	const std::vector<std::string> SaiSimulation::getObjectNames() const
	{
		std::vector<std::string> object_names;
		for (const auto &it : _dyn_objects_init_pose)
		{
			object_names.push_back(it.first);
		}
		return object_names;
	}

	// get dof
	const unsigned int SaiSimulation::dof(const std::string &robot_name) const
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot get dof for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		return _robot_models.at(robot_name)->dof();
	}

	const unsigned int SaiSimulation::qSize(const std::string &robot_name) const
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot get dof for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		return _robot_models.at(robot_name)->qSize();
	}

	void SaiSimulation::setTimestep(const double dt)
	{
		if (dt <= 0)
		{
			throw std::invalid_argument(
				"simulation timestep cannot be 0 or negative");
		}
		double prev_timestep = _timestep;
		_world->setCollisionRate(dt);
		for (auto sensor : _force_sensors)
		{
			double cutoff = sensor->getNormalizedCutoffFreq() / prev_timestep;
			sensor->enableFilter(cutoff * dt);
		}
		_timestep = dt;
	}

	void SaiSimulation::enableJointLimits(const std::string &robot_name)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot enable joint limits robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		VectorXd q = getJointPositions(robot_name);
		auto dyn_robot = _world->getBaseNode(robot_name);
		for (const SaiModel::JointLimit joint_limit :
			 _robot_models.at(robot_name)->jointLimits())
		{
			if (joint_limit.position_upper == std::numeric_limits<double>::max() ||
				joint_limit.position_lower == -std::numeric_limits<double>::max())
			{
				continue;
			}
			auto dyn_joint = dyn_robot->getJoint(joint_limit.joint_name);
			double current_joint_pos =
				q(_robot_models.at(robot_name)->jointIndex(joint_limit.joint_name));
			if (current_joint_pos > joint_limit.position_upper ||
				current_joint_pos < joint_limit.position_lower)
			{
				throw std::runtime_error(
					"Cannot enable joint limits for joint [" +
					joint_limit.joint_name + "] in robot [" + robot_name +
					"] that is currently outside of its limits");
			}
			dyn_joint->setJointLimits(joint_limit.position_lower,
									  joint_limit.position_upper);
		}
	}

	void SaiSimulation::disableJointLimits(const std::string &robot_name)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot disable joint limits robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		auto dyn_robot = _world->getBaseNode(robot_name);
		for (const SaiModel::JointLimit joint_limit :
			 _robot_models.at(robot_name)->jointLimits())
		{
			auto dyn_joint = dyn_robot->getJoint(joint_limit.joint_name);
			dyn_joint->removeJointLimits();
		}
	}

	void SaiSimulation::enableGravityCompensation(const bool enable, const std::string &robot_name)
	{
		if (robot_name.empty())
		{
			// Apply to all robots if no name is specified
			for (auto &pair : _gravity_compensation_enabled)
			{
				pair.second = enable;
			}
		}
		else
		{
			// Apply only to the specific robot
			if (!robotExistsInWorld(robot_name))
			{
				throw std::invalid_argument(
					"cannot enable gravity compensation for robot [" + robot_name +
					"] that does not exist in the simulated world");
			}
			_gravity_compensation_enabled.at(robot_name) = enable;
		}
	}

	const bool SaiSimulation::isGravityCompensationEnabled(const std::string &robot_name) const
	{
		if (robot_name.empty())
		{
			// Return true if ANY robot has gravity compensation enabled
			for (const auto &pair : _gravity_compensation_enabled)
			{
				if (pair.second)
					return true;
			}
			return false;
		}

		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot check gravity compensation for robot [" + robot_name +
				"] that does not exist in the simulated world");
		}
		return _gravity_compensation_enabled.at(robot_name);
	}

	// set joint positions
	void SaiSimulation::setJointPositions(const std::string &robot_name,
										  const Eigen::VectorXd &q)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot set positions for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		if (q.size() != qSize(robot_name))
		{
			throw std::invalid_argument(
				"q_size mismatch, cannot set positions for robot [" + robot_name +
				"]");
		}
		auto robot = _world->getBaseNode(robot_name);
		uint q_ind_counter = 0;
		uint sph_joint_counter = 0;
		uint dofs = dof(robot_name);
		chai3d::cQuaternion sph_quat;
		for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i)
		{
			if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL)
			{
				sph_quat.w = q[dofs + sph_joint_counter];
				sph_quat.x = q[q_ind_counter];
				sph_quat.y = q[q_ind_counter + 1];
				sph_quat.z = q[q_ind_counter + 2];
				robot->m_dynamicJoints[i]->setPosSpherical(sph_quat);
				sph_joint_counter++;
				q_ind_counter += 3;
			}
			else
			{
				robot->m_dynamicJoints[i]->setPos(q[q_ind_counter++]);
			}
		}
	}

	// read joint positions
	const Eigen::VectorXd SaiSimulation::getJointPositions(
		const std::string &robot_name) const
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot get positions for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		auto robot = _world->getBaseNode(robot_name);
		Eigen::VectorXd q_ret(qSize(robot_name));
		uint q_ind_counter = 0;
		uint sph_joint_counter = 0;
		uint dofs = dof(robot_name);
		chai3d::cQuaternion sph_quat;
		for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i)
		{
			if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL)
			{
				sph_quat = robot->m_dynamicJoints[i]->getPosSpherical();
				q_ret[dofs + sph_joint_counter] = sph_quat.w;
				q_ret[q_ind_counter] = sph_quat.x;
				q_ret[q_ind_counter + 1] = sph_quat.y;
				q_ret[q_ind_counter + 2] = sph_quat.z;
				sph_joint_counter++;
				q_ind_counter += 3;
			}
			else
			{
				q_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getPos();
			}
		}
		return q_ret;
	}

	// read object pose
	const Eigen::Affine3d SaiSimulation::getObjectPose(
		const std::string &object_name) const
	{
		if (dynamicObjectExistsInWorld(object_name))
		{
			return *_dyn_objects_pose.at(object_name);
		}
		if (staticObjectExistsInWorld(object_name))
		{
			return _static_objects_pose.at(object_name);
		}
		throw std::invalid_argument("cannot get pose for object [" + object_name +
									"] that does not exists in simulated world");
	}

	// set object pose
	void SaiSimulation::setObjectPose(const std::string &object_name,
									  const Eigen::Affine3d &pose) const
	{
		if (!dynamicObjectExistsInWorld(object_name))
		{
			throw std::invalid_argument(
				"cannot set pose for dynamic object [" + object_name +
				"] that does not exists in simulated world");
		}
		auto object = _world->getBaseNode(object_name);
		Eigen::Affine3d pose_local =
			_dyn_objects_init_pose.at(object_name).inverse() * pose;
		for (int i = 0; i < 3; i++)
		{
			object->m_dynamicJoints[i]->setPos(pose_local.translation()(i));
		}
		chai3d::cQuaternion quat;
		quat.fromRotMat(pose_local.rotation());
		object->m_dynamicJoints[3]->setPosSpherical(quat);

		*_dyn_objects_pose.at(object_name) = pose;
	}

	// set joint position for a single joint
	void SaiSimulation::setJointPosition(const std::string &robot_name,
										 unsigned int joint_id, double position)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot set positions for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		auto robot = _world->getBaseNode(robot_name);
		if (joint_id >= robot->m_dynamicJoints.size())
		{
			throw std::invalid_argument(
				"cannot set positions for a joint out of bounds");
		}
		if (robot->m_dynamicJoints[joint_id]->getJointType() ==
			DYN_JOINT_SPHERICAL)
		{
			throw std::invalid_argument(
				"cannot set individual position for a spherical joint");
		}
		robot->m_dynamicJoints[joint_id]->setPos(position);
	}

	// set joint velocities
	void SaiSimulation::setJointVelocities(const std::string &robot_name,
										   const Eigen::VectorXd &dq)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot set velocities for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		if (dq.size() != dof(robot_name))
		{
			throw std::invalid_argument(
				"dq size mismatch, cannot set velocities for robot [" + robot_name +
				"]");
		}
		auto robot = _world->getBaseNode(robot_name);
		uint q_ind_counter = 0;
		chai3d::cVector3d sph_vel;
		for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i)
		{
			if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL)
			{
				sph_vel = Eigen::Vector3d(dq[q_ind_counter], dq[q_ind_counter + 1],
										  dq[q_ind_counter + 2]);
				robot->m_dynamicJoints[i]->setVelSpherical(sph_vel);
				q_ind_counter += 3;
			}
			else
			{
				robot->m_dynamicJoints[i]->setVel(dq[q_ind_counter++]);
			}
		}
	}

	// set joint velocity for a single joint
	void SaiSimulation::setJointVelocity(const std::string &robot_name,
										 unsigned int joint_id, double velocity)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot set velocities for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		auto robot = _world->getBaseNode(robot_name);
		if (joint_id >= robot->m_dynamicJoints.size())
		{
			throw std::invalid_argument(
				"cannot set velocities for a joint out of bounds");
		}
		if (robot->m_dynamicJoints[joint_id]->getJointType() ==
			DYN_JOINT_SPHERICAL)
		{
			throw std::invalid_argument(
				"cannot set individual velocity for a spherical joint");
		}
		robot->m_dynamicJoints[joint_id]->setVel(velocity);
	}

	// read joint velocities
	const Eigen::VectorXd SaiSimulation::getJointVelocities(
		const std::string &robot_name) const
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot get velocities for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		auto robot = _world->getBaseNode(robot_name);
		Eigen::VectorXd dq_ret(dof(robot_name));
		uint q_ind_counter = 0;
		uint dofs = dof(robot_name);
		chai3d::cVector3d sph_vel;
		for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i)
		{
			if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL)
			{
				sph_vel = robot->m_dynamicJoints[i]->getVelSpherical();
				dq_ret[q_ind_counter] = sph_vel.x();
				dq_ret[q_ind_counter + 1] = sph_vel.y();
				dq_ret[q_ind_counter + 2] = sph_vel.z();
				q_ind_counter += 3;
			}
			else
			{
				dq_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getVel();
			}
		}
		return dq_ret;
	}

	// read object velocities
	const Eigen::VectorXd SaiSimulation::getObjectVelocity(
		const std::string &object_name) const
	{
		if (!dynamicObjectExistsInWorld(object_name))
		{
			throw std::invalid_argument(
				"cannot get velocities for dynamic object [" + object_name +
				"] that does not exists in simulated world");
		}
		return _dyn_objects_velocity.at(object_name);
	}

	void SaiSimulation::setObjectVelocity(
		const std::string &object_name, const Eigen::Vector3d &linear_velocity,
		const Eigen::Vector3d &angular_velocity)
	{
		if (!dynamicObjectExistsInWorld(object_name))
		{
			throw std::invalid_argument(
				"cannot set velocities for dynamic object [" + object_name +
				"] that does not exists in simulated world");
		}
		Vector3d linear_vel_object_frame =
			_dyn_objects_init_pose.at(object_name).rotation().transpose() *
			linear_velocity;
		Vector3d angular_vel_object_frame =
			_dyn_objects_init_pose.at(object_name).rotation().transpose() *
			angular_velocity;
		auto object = _world->getBaseNode(object_name);
		object->m_dynamicJoints[0]->setVel(linear_vel_object_frame(0));
		object->m_dynamicJoints[1]->setVel(linear_vel_object_frame(1));
		object->m_dynamicJoints[2]->setVel(linear_vel_object_frame(2));
		object->m_dynamicJoints[3]->setVelSpherical(chai3d::cVector3d(
			angular_vel_object_frame(0), angular_vel_object_frame(1),
			angular_vel_object_frame(2)));

		_dyn_objects_velocity.at(object_name).head(3) = linear_velocity;
		_dyn_objects_velocity.at(object_name).tail(3) = angular_velocity;
	}

	// set joint torques
	void SaiSimulation::setJointTorques(const std::string &robot_name,
										const Eigen::VectorXd &tau)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot set torques for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		if (tau.size() != dof(robot_name))
		{
			throw std::invalid_argument(
				"size of torque vector inconsistent in setJointTorques");
		}
		_applied_robot_torques.at(robot_name) = tau;
		for (const auto spherical_joint_description :
			 _robot_models.at(robot_name)->sphericalJoints())
		{
			int index = spherical_joint_description.index;
			string child_link = spherical_joint_description.child_link_name;
			Matrix3d R = _robot_models.at(robot_name)->rotationInWorld(child_link);
			_applied_robot_torques.at(robot_name).segment<3>(index) =
				R.transpose() * tau.segment<3>(index);
		}
	}

	// set joint torque for a single joint
	void SaiSimulation::setJointTorque(const std::string &robot_name,
									   unsigned int joint_id, double tau)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot set torque for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		_applied_robot_torques.at(robot_name)(joint_id) = tau;
		for (const auto spherical_joint_description :
			 _robot_models.at(robot_name)->sphericalJoints())
		{
			int index = spherical_joint_description.index;
			if (index == joint_id || index == joint_id - 1 ||
				index == joint_id - 2)
			{
				Vector3d tau_sph = Vector3d::Zero();
				tau_sph(joint_id - index) = tau;
				string child_link = spherical_joint_description.child_link_name;
				Matrix3d R =
					_robot_models.at(robot_name)->rotationInWorld(child_link);
				_applied_robot_torques.at(robot_name).segment<3>(index) =
					R.transpose() * tau_sph;
			}
		}
	}

	void SaiSimulation::setObjectForceTorque(const std::string &object_name,
											 const Eigen::Vector6d &tau)
	{
		if (!dynamicObjectExistsInWorld(object_name))
		{
			throw std::invalid_argument(
				"cannot set torques for dynamic object [" + object_name +
				"] that does not exists in simulated world");
		}
		// linear forces are applied in object base frame (before the rotation
		// from the spherical joint)
		_applied_object_torques.at(object_name).head<3>() =
			_dyn_objects_init_pose.at(object_name).rotation().transpose() *
			tau.head<3>();
		// spherical torques are applied in the object local frame (after
		// rotation from the spherical joint)
		Eigen::Affine3d object_pose = getObjectPose(object_name);
		_applied_object_torques.at(object_name).tail<3>() =
			object_pose.rotation().transpose() * tau.tail<3>();
	}

	void SaiSimulation::setAllJointTorquesInternal()
	{
		for (const auto &pair : _applied_robot_torques)
		{
			auto robot_name = pair.first;
			auto tau = pair.second;
			auto robot_model = _robot_models.at(robot_name);
			Eigen::VectorXd gravity_torques =
				Eigen::VectorXd::Zero(robot_model->dof());
			if (_gravity_compensation_enabled.at(robot_name))
			{
				gravity_torques = robot_model->jointGravityVector();
			}
			auto robot = _world->getBaseNode(robot_name);
			uint q_ind_counter = 0;
			chai3d::cVector3d sph_tau;
			for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i)
			{
				if (robot->m_dynamicJoints[i]->getJointType() ==
					DYN_JOINT_SPHERICAL)
				{
					sph_tau = Eigen::Vector3d(
						tau[q_ind_counter] + gravity_torques[q_ind_counter],
						tau[q_ind_counter + 1] + gravity_torques[q_ind_counter + 1],
						tau[q_ind_counter + 2] +
							gravity_torques[q_ind_counter + 2]);
					robot->m_dynamicJoints[i]->setTorque(sph_tau);
					q_ind_counter += 3;
				}
				else
				{
					robot->m_dynamicJoints[i]->setForce(
						gravity_torques[q_ind_counter] + tau[q_ind_counter]);
					q_ind_counter++;
				}
			}
		}
		for (const auto &pair : _applied_object_torques)
		{
			auto object_name = pair.first;
			auto tau = pair.second;
			auto object = _world->getBaseNode(object_name);
			object->m_dynamicJoints[0]->setForce(tau(0));
			object->m_dynamicJoints[1]->setForce(tau(1));
			object->m_dynamicJoints[2]->setForce(tau(2));
			chai3d::cVector3d sph_tau = chai3d::cVector3d(tau(3), tau(4), tau(5));
			object->m_dynamicJoints[3]->setTorque(sph_tau);
		}
	}

	// read joint accelerations
	const Eigen::VectorXd SaiSimulation::getJointAccelerations(
		const std::string &robot_name) const
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot get accelerations for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		auto robot = _world->getBaseNode(robot_name);
		Eigen::VectorXd ddq_ret(dof(robot_name));
		uint q_ind_counter = 0;
		chai3d::cVector3d sph_acc;
		for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i)
		{
			if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL)
			{
				sph_acc = robot->m_dynamicJoints[i]->getAccelSpherical();
				ddq_ret[q_ind_counter] = sph_acc.x();
				ddq_ret[q_ind_counter + 1] = sph_acc.y();
				ddq_ret[q_ind_counter + 2] = sph_acc.z();
				q_ind_counter += 3;
			}
			else
			{
				ddq_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getAccel();
			}
		}
		return ddq_ret;
	}

	const MatrixXd SaiSimulation::computeAndGetMassMatrix(
		const std::string &robot_name)
	{
		if (!robotExistsInWorld(robot_name))
		{
			throw std::invalid_argument(
				"cannot get mass matrix for robot [" + robot_name +
				"] that does not exists in simulated world");
		}
		_robot_models.at(robot_name)->updateModel();
		return _robot_models.at(robot_name)->M();
	}

	// integrate ahead
	void SaiSimulation::integrate()
	{
		if (_is_paused)
		{
			return;
		}

		setAllJointTorquesInternal();

		// update dynamic world
		_world->updateDynamics(_timestep);
		_time += _timestep;
		// update robot models
		for (auto pair : _robot_models)
		{
			auto robot_name = pair.first;
			auto robot_model = pair.second;
			robot_model->setQ(getJointPositions(robot_name));
			robot_model->updateKinematics();
		}
		// update dynamic objects poses and velocities
		for (auto pair : _dyn_objects_init_pose)
		{
			auto object_name = pair.first;
			auto object_init_pose = pair.second;
			auto object = _world->getBaseNode(object_name);
			chai3d::cQuaternion quat =
				object->m_dynamicJoints[3]->getPosSpherical();
			Eigen::Affine3d obj_pose_from_base =
				Eigen::Affine3d(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
			obj_pose_from_base.translation()
				<< object->m_dynamicJoints[0]->getPos(),
				object->m_dynamicJoints[1]->getPos(),
				object->m_dynamicJoints[2]->getPos();
			*_dyn_objects_pose.at(object_name) =
				object_init_pose * obj_pose_from_base;

			Eigen::VectorXd velocity_in_object_frame(6);
			velocity_in_object_frame.head(3)
				<< object->m_dynamicJoints[0]->getVel(),
				object->m_dynamicJoints[1]->getVel(),
				object->m_dynamicJoints[2]->getVel();
			velocity_in_object_frame.tail(3) =
				object->m_dynamicJoints[3]->getVelSpherical().eigen();
			_dyn_objects_velocity.at(object_name).head(3) =
				_dyn_objects_init_pose.at(object_name).rotation() *
				velocity_in_object_frame.head(3);
			_dyn_objects_velocity.at(object_name).tail(3) =
				_dyn_objects_init_pose.at(object_name).rotation() *
				velocity_in_object_frame.tail(3);
		}
		// update force sensors if any
		for (auto sensor : _force_sensors)
		{
			sensor->update(getDynamicWorld());
		}
	}

	void SaiSimulation::showContactInfo()
	{
		std::list<cDynamicBase *>::iterator i;
		for (i = _world->m_dynamicObjects.begin();
			 i != _world->m_dynamicObjects.end(); ++i)
		{
			cDynamicBase *object = *i;
			int num_contacts = object->m_dynamicContacts->getNumContacts();
			// consider only contacting objects
			if (num_contacts > 0)
			{
				std::cout << "model name : " << object->m_name << std::endl;
				std::cout << "num contacts : " << num_contacts << std::endl;
				for (int k = 0; k < num_contacts; k++)
				{
					cDynamicContact *contact =
						object->m_dynamicContacts->getContact(k);
					std::cout << "contact " << k
							  << " at link : " << contact->m_dynamicLink->m_name
							  << std::endl;
					std::cout << "contact position : " << contact->m_globalPos
							  << std::endl;
					std::cout << "contact normal : " << contact->m_globalNormal
							  << std::endl;
					std::cout << "contact normal force : "
							  << contact->m_globalNormalForce << std::endl;
					std::cout << "contact friction force : "
							  << contact->m_globalFrictionForce << std::endl;
					std::cout << "contact force magnitude : "
							  << contact->m_normalForceMagnitude << std::endl;
					std::cout << "time : " << contact->m_time << std::endl;
				}
				std::cout << std::endl;
			}
		}
	}

	std::vector<Contact3d> SaiSimulation::getContactInfo(const std::string &robot_name, const std::string &link_name)
	{
		std::vector<Contact3d> all_contacts;
		std::list<cDynamicBase *>::iterator i;

		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> contacts_list = getContactList(robot_name, link_name);
		for (i = _world->m_dynamicObjects.begin();
			 i != _world->m_dynamicObjects.end(); ++i)
		{
			cDynamicBase *object = *i;
			int num_contacts = object->m_dynamicContacts->getNumContacts();
			// consider only contacting objects
			if (num_contacts > 0)
			{
				for (int k = 0; k < num_contacts; k++)
				{
					cDynamicContact *contact =
						object->m_dynamicContacts->getContact(k);
					if (!contact)
						continue; // skip if pointer is null
					if (!contact->m_dynamicLink)
						continue; // skip if link pointer is null

					for (const auto &contact_pair : contacts_list)
					{
						Eigen::Vector3d contact_pos(contact->m_globalPos.x(),
													contact->m_globalPos.y(),
													contact->m_globalPos.z());
						Eigen::Vector3d contact_normal(contact->m_globalNormal.x(),
													   contact->m_globalNormal.y(),
													   contact->m_globalNormal.z());
						if (((contact_pos - contact_pair.first).norm() > 1e-6) || (contact_normal.dot(contact_pair.second) < 0))
						{
							continue;
						}

						Contact3d contact_final;
						contact_final.position = Eigen::Vector3d(contact->m_globalPos.x(),
																 contact->m_globalPos.y(),
																 contact->m_globalPos.z());
						contact_final.normal = Eigen::Vector3d(contact->m_globalNormal.x(),
															   contact->m_globalNormal.y(),
															   contact->m_globalNormal.z());
						contact_final.force = contact_pair.second;
						all_contacts.push_back(contact_final);
					}
				}
			}
		}
		return all_contacts;
	}

	void SaiSimulation::showLinksInContact(
		const std::string robot_or_object_name)
	{
		std::list<cDynamicBase *>::iterator i;
		for (i = _world->m_dynamicObjects.begin();
			 i != _world->m_dynamicObjects.end(); ++i)
		{
			cDynamicBase *object = *i;
			if (object->m_name == robot_or_object_name)
			{
				int num_contacts = object->m_dynamicContacts->getNumContacts();
				if (num_contacts > 0)
				{
					std::set<std::string> contact_links;
					std::cout << "contacts on model : " << robot_or_object_name
							  << std::endl;
					for (int k = 0; k < num_contacts; k++)
					{
						contact_links.insert(
							object->m_dynamicContacts->getContact(k)
								->m_dynamicLink->m_name);
					}
					for (std::set<std::string>::iterator it = contact_links.begin();
						 it != contact_links.end(); ++it)
					{
						std::cout << (*it) << std::endl;
					}
					std::cout << std::endl;
				}
			}
		}
	}

	const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
	SaiSimulation::getContactList(const ::std::string &robot_name,
								  const std::string &link_name) const
	{
		return _world->getContactList(robot_name, link_name);
	}

	const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
	SaiSimulation::getContactList(const ::std::string &object_name) const
	{
		return _world->getContactList(object_name, object_link_name);
	}

	void SaiSimulation::addSimulatedForceSensor(
		const std::string &robot_name, const std::string &link_name,
		const Eigen::Affine3d transform_in_link,
		const double filter_cutoff_frequency)
	{
		if (!robotExistsInWorld(robot_name, link_name))
		{
			std::cout << "\n\nWARNING: trying to add a force sensor to an "
						 "unexisting robot or link in "
						 "SaiSimulation::addSimulatedForceSensor\n"
					  << std::endl;
			return;
		}
		if (link_name == object_link_name)
		{
			std::cout << "\n\nWARNING: a force sensor cannot be attached to a "
						 "robot link with name ["
					  << object_link_name
					  << "] please rename this link or attach the sensor to "
						 "another link\n"
					  << std::endl;
			return;
		}
		if (findSimulatedForceSensor(robot_name, link_name) != -1)
		{
			std::cout << "\n\nWARNING: only one force sensor is supported per "
						 "link in SaiSimulation::addSimulatedForceSensor. Not "
						 "adding the second one\n"
					  << std::endl;
			return;
		}
		_force_sensors.push_back(std::make_shared<ForceSensorSim>(
			robot_name, link_name, transform_in_link, _robot_models[robot_name],
			filter_cutoff_frequency * _timestep));
	}

	void SaiSimulation::addSimulatedForceSensor(
		const std::string &object_name, const Eigen::Affine3d transform_in_link,
		const double filter_cutoff_frequency)
	{
		if (findSimulatedForceSensor(object_name, object_link_name) != -1)
		{
			std::cout << "\n\nWARNING: only one force sensor is supported per "
						 "link in SaiSimulation::addSimulatedForceSensor. Not "
						 "adding the second one\n"
					  << std::endl;
			return;
		}
		if (staticObjectExistsInWorld(object_name))
		{
			_force_sensors.push_back(std::make_shared<ForceSensorSim>(
				object_name, object_link_name, transform_in_link,
				std::make_shared<Affine3d>(_static_objects_pose.at(object_name)),
				filter_cutoff_frequency * _timestep));
		}
		else if (dynamicObjectExistsInWorld(object_name))
		{
			_force_sensors.push_back(std::make_shared<ForceSensorSim>(
				object_name, object_link_name, transform_in_link,
				_dyn_objects_pose.at(object_name),
				filter_cutoff_frequency * _timestep));
		}
		else
		{
			std::cout << "\n\nWARNING: trying to add a force sensor to an "
						 "unexisting object in "
						 "SaiSimulation::addSimulatedForceSensor\n"
					  << std::endl;
		}
	}

	Eigen::Vector3d SaiSimulation::getSensedForce(
		const std::string &robot_name, const std::string &link_name,
		const bool in_sensor_frame) const
	{
		int sensor_index = findSimulatedForceSensor(robot_name, link_name);
		if (sensor_index == -1)
		{
			std::cout << "WARNING: no force sensor registered on robot ["
					  << robot_name << "] and link [" << link_name
					  << "]. Returning Zero forces" << std::endl;
			return Eigen::Vector3d::Zero();
		}
		if (in_sensor_frame)
		{
			return _force_sensors.at(sensor_index)->getForceLocalFrame();
		}
		return _force_sensors.at(sensor_index)->getForceWorldFrame();
	}

	Eigen::Vector3d SaiSimulation::getSensedForce(
		const std::string &object_name, const bool in_sensor_frame) const
	{
		int sensor_index = findSimulatedForceSensor(object_name, object_link_name);
		if (sensor_index == -1)
		{
			std::cout << "WARNING: no force sensor registered on object ["
					  << object_name << "]. Returning Zero forces" << std::endl;
			return Eigen::Vector3d::Zero();
		}
		if (in_sensor_frame)
		{
			return _force_sensors.at(sensor_index)->getForceLocalFrame();
		}
		return _force_sensors.at(sensor_index)->getForceWorldFrame();
	}

	Eigen::Vector3d SaiSimulation::getSensedMoment(
		const std::string &robot_name, const std::string &link_name,
		const bool in_sensor_frame) const
	{
		int sensor_index = findSimulatedForceSensor(robot_name, link_name);
		if (sensor_index == -1)
		{
			std::cout << "WARNING: no force sensor registered on robot ["
					  << robot_name << "] and link [" << link_name
					  << "]. Returning Zero moments" << std::endl;
			return Eigen::Vector3d::Zero();
		}
		if (in_sensor_frame)
		{
			return _force_sensors.at(sensor_index)->getMomentLocalFrame();
		}
		return _force_sensors.at(sensor_index)->getMomentWorldFrame();
	}

	Eigen::Vector3d SaiSimulation::getSensedMoment(
		const std::string &object_name, const bool in_sensor_frame) const
	{
		int sensor_index = findSimulatedForceSensor(object_name, object_link_name);
		if (sensor_index == -1)
		{
			std::cout << "WARNING: no force sensor registered on object ["
					  << object_name << "]. Returning Zero moments" << std::endl;
			return Eigen::Vector3d::Zero();
		}
		if (in_sensor_frame)
		{
			return _force_sensors.at(sensor_index)->getMomentLocalFrame();
		}
		return _force_sensors.at(sensor_index)->getMomentWorldFrame();
	}

	const std::vector<SaiModel::ForceSensorData>
	SaiSimulation::getAllForceSensorData() const
	{
		std::vector<SaiModel::ForceSensorData> sensor_data;
		for (const auto &sensor : _force_sensors)
		{
			sensor_data.push_back(sensor->getData());
		}
		return sensor_data;
	}

	const int SaiSimulation::findSimulatedForceSensor(
		const std::string &robot_or_object_name,
		const std::string &link_name) const
	{
		for (int i = 0; i < _force_sensors.size(); ++i)
		{
			if ((_force_sensors.at(i)->getData().robot_or_object_name ==
				 robot_or_object_name) &&
				(_force_sensors.at(i)->getData().link_name == link_name))
			{
				return i;
			}
		}
		return -1;
	}

	const bool SaiSimulation::robotExistsInWorld(
		const std::string &robot_name, const std::string link_name) const
	{
		auto it = _robot_models.find(robot_name);
		if (it == _robot_models.end())
		{
			return false;
		}
		if (link_name != "")
		{
			return _robot_models.at(robot_name)->isLinkInRobot(link_name);
		}
		return true;
	}

	const bool SaiSimulation::dynamicObjectExistsInWorld(
		const std::string &object_name) const
	{
		auto it = _dyn_objects_init_pose.find(object_name);
		if (it == _dyn_objects_init_pose.end())
		{
			return false;
		}
		return true;
	}

	const bool SaiSimulation::staticObjectExistsInWorld(
		const std::string &object_name) const
	{
		auto it = _static_objects_pose.find(object_name);
		if (it == _static_objects_pose.end())
		{
			return false;
		}
		return true;
	}

	void SaiSimulation::setDynamicsEnabled(const bool enabled,
										   const string robot_or_object_name)
	{
		for (cDynamicBase *base : _world->m_dynamicObjects)
		{
			if (base->m_name == robot_or_object_name)
			{
				base->enableDynamics(enabled);
			}
		}
	}

	void SaiSimulation::setJointDamping(const double damping,
										const string robot_or_object_name,
										const string joint_name)
	{
		for (cDynamicBase *base : _world->m_dynamicObjects)
		{
			if (robot_or_object_name.empty() ||
				base->m_name == robot_or_object_name)
			{
				for (cDynamicJoint *joint : base->m_dynamicJoints)
				{
					if (joint_name.empty() || joint->m_name == joint_name)
					{
						joint->setDamping(damping);
					}
				}
			}
		}
	}

	void SaiSimulation::setCollisionRestitution(const double restitution,
												const string robot_or_object_name,
												const string link_name)
	{
		for (cDynamicBase *base : _world->m_dynamicObjects)
		{
			if (robot_or_object_name.empty() ||
				base->m_name == robot_or_object_name)
			{
				for (cDynamicLink *link : base->m_dynamicLinks)
				{
					if (link_name.empty() || link->m_name == link_name)
					{
						cDynamicMaterial *mat = link->getDynamicMaterial();
						mat->setEpsilon(restitution);
					}
				}
			}
		}
	}

	const double SaiSimulation::getCollisionRestitution(
		const std::string &object_name) const
	{
		return getCollisionRestitution(object_name, object_link_name);
	}

	// get co-efficient of restitution: for a named robot and link
	const double SaiSimulation::getCollisionRestitution(
		const std::string &robot_name, const std::string &link_name) const
	{
		if (!robotExistsInWorld(robot_name, link_name) &&
			(!dynamicObjectExistsInWorld(robot_name) ||
			 link_name != object_link_name))
		{
			throw std::invalid_argument(
				"cannot get collision restitution for link [" + link_name +
				"] of [" + robot_name + "] that doesn't exists in simulation");
		}
		auto robot = _world->getBaseNode(robot_name);
		auto link = robot->getLink(link_name);
		cDynamicMaterial *mat = link->getDynamicMaterial();
		return mat->getEpsilon();
	}

	void SaiSimulation::setCoeffFrictionStatic(const double static_friction,
											   const string robot_or_object_name,
											   const string link_name)
	{
		for (cDynamicBase *base : _world->m_dynamicObjects)
		{
			if (robot_or_object_name.empty() ||
				base->m_name == robot_or_object_name)
			{
				for (cDynamicLink *link : base->m_dynamicLinks)
				{
					if (link_name.empty() || link->m_name == link_name)
					{
						cDynamicMaterial *mat = link->getDynamicMaterial();
						mat->setStaticFriction(static_friction);
					}
				}
			}
		}
	}

	const double SaiSimulation::getCoeffFrictionStatic(
		const std::string &object_name) const
	{
		return getCoeffFrictionStatic(object_name, object_link_name);
	}

	// get co-efficient of static friction: for a named robot and link
	const double SaiSimulation::getCoeffFrictionStatic(
		const std::string &robot_name, const std::string &link_name) const
	{
		if (!robotExistsInWorld(robot_name, link_name) &&
			(!dynamicObjectExistsInWorld(robot_name) ||
			 link_name != object_link_name))
		{
			throw std::invalid_argument(
				"cannot get static coefficient of friction for link [" + link_name +
				"] of [" + robot_name + "] that doesn't exists in simulation");
		}
		auto robot = _world->getBaseNode(robot_name);
		auto link = robot->getLink(link_name);
		cDynamicMaterial *mat = link->getDynamicMaterial();
		return mat->getStaticFriction();
	}

	void SaiSimulation::setCoeffFrictionDynamic(const double dynamic_friction,
												const string robot_or_object_name,
												const string link_name)
	{
		for (cDynamicBase *base : _world->m_dynamicObjects)
		{
			if (robot_or_object_name.empty() ||
				base->m_name == robot_or_object_name)
			{
				for (cDynamicLink *link : base->m_dynamicLinks)
				{
					if (link_name.empty() || link->m_name == link_name)
					{
						cDynamicMaterial *mat = link->getDynamicMaterial();
						mat->setDynamicFriction(dynamic_friction);
					}
				}
			}
		}
	}

	const double SaiSimulation::getCoeffFrictionDynamic(
		const std::string &object_name) const
	{
		return getCoeffFrictionDynamic(object_name, object_link_name);
	}

	// get co-efficient of dynamic friction: for a named robot and link
	const double SaiSimulation::getCoeffFrictionDynamic(
		const std::string &robot_name, const std::string &link_name) const
	{
		if (!robotExistsInWorld(robot_name, link_name) &&
			(!dynamicObjectExistsInWorld(robot_name) ||
			 link_name != object_link_name))
		{
			throw std::invalid_argument(
				"cannot get dynamic coefficient of friction to link [" + link_name +
				"] of [" + robot_name + "] that doesn't exists in simulation");
		}
		auto robot = _world->getBaseNode(robot_name);
		auto link = robot->getLink(link_name);
		cDynamicMaterial *mat = link->getDynamicMaterial();
		return mat->getDynamicFriction();
	}

	// get pose of robot base in the world frame
	const Eigen::Affine3d SaiSimulation::getRobotBaseTransform(
		const std::string &robot_name) const
	{
		Eigen::Affine3d gToRobotBase;
		const auto base = _world->getBaseNode(robot_name);
		gToRobotBase.translation() = base->getLocalPos().eigen();
		gToRobotBase.linear() = base->getLocalRot().eigen();
		return gToRobotBase;
	}

} // namespace SaiSimulation
/*
 * Sai2Simulation.cpp
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 *  Update: ported to sai2-simulation project on Nov 17, 2017
 *		By: Shameek Ganguly
 */

#include "Sai2Simulation.h"

#include <math/CMatrix3d.h>	 // chai math library

#include <iostream>
#include <map>
#include <set>
#include <stdexcept>

#include "dynamics3d.h"
#include "parser/URDFToDynamics3d.h"

namespace Sai2Simulation {

// ctor
Sai2Simulation::Sai2Simulation(const std::string& path_to_world_file,
							   bool verbose) {
	resetWorld(path_to_world_file, verbose);
	_timestep = 0.001;
}

void Sai2Simulation::resetWorld(const std::string& path_to_world_file,
								bool verbose) {
	_is_paused = false;
	_time = 0;
	_gravity_compensation_enabled = false;

	// clean up robot names, models, torques and force sensors
	_robot_filenames.clear();
	_robot_models.clear();
	_force_sensors.clear();
	_applied_robot_torques.clear();
	_applied_object_torques.clear();
	_dyn_object_base_pose.clear();

	// create a dynamics world
	_world = std::make_shared<cDynamicWorld>(nullptr);

	URDFToDynamics3dWorld(path_to_world_file, _world, _dyn_object_base_pose,
						  _robot_filenames, verbose);

	// enable dynamics for all robots in this world
	// TODO: consider pushing up to the API?
	for (auto robot : _world->m_dynamicObjects) {
		robot->enableDynamics(true);
	}

	setCollisionRestitution(0);

	// create robot models
	for (const auto& pair : _robot_filenames) {
		const auto& robot_name = pair.first;
		const auto& robot_file = pair.second;
		_robot_models[robot_name] =
			std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
		_robot_models.at(robot_name)->setTRobotBase(
			getRobotBaseTransform(robot_name));
		_robot_models.at(robot_name)->setWorldGravity(
			_world->getGravity().eigen());
		_applied_robot_torques[robot_name] =
			Eigen::VectorXd::Zero(dof(robot_name));
		enableJointLimits(robot_name);
		_robot_models.at(robot_name)->setQ(getJointPositions(robot_name));
		_robot_models.at(robot_name)->updateModel();
	}
	for (const auto& object_name : getObjectNames()) {
		_applied_object_torques[object_name] = Eigen::VectorXd::Zero(6);
	}
}

const std::vector<std::string> Sai2Simulation::getRobotNames() const {
	std::vector<std::string> robot_names;
	for (const auto& it : _robot_filenames) {
		robot_names.push_back(it.first);
	}
	return robot_names;
}

const std::vector<std::string> Sai2Simulation::getObjectNames() const {
	std::vector<std::string> object_names;
	for (const auto& it : _dyn_object_base_pose) {
		object_names.push_back(it.first);
	}
	return object_names;
}

// get dof
const unsigned int Sai2Simulation::dof(const std::string& robot_name) const {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot get dof for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	return _robot_models.at(robot_name)->dof();
}

const unsigned int Sai2Simulation::qSize(const std::string& robot_name) const {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot get dof for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	return _robot_models.at(robot_name)->qSize();
}

void Sai2Simulation::setTimestep(const double dt) {
	if (dt <= 0) {
		throw std::invalid_argument(
			"simulation timestep cannot be 0 or negative");
	}
	double prev_timestep = _timestep;
	for (auto sensor : _force_sensors) {
		double cutoff = sensor->getNormalizedCutoffFreq() / prev_timestep;
		sensor->enableFilter(cutoff * dt);
	}
	_timestep = dt;
}

void Sai2Simulation::enableJointLimits(const std::string& robot_name) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot enable joint limits robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	VectorXd q = getJointPositions(robot_name);
	auto dyn_robot = _world->getBaseNode(robot_name);
	for (const Sai2Model::JointLimit joint_limit :
		 _robot_models.at(robot_name)->jointLimits()) {
		auto dyn_joint = dyn_robot->getJoint(joint_limit.joint_name);
		double current_joint_pos =
			q(_robot_models.at(robot_name)->jointIndex(joint_limit.joint_name));
		if (current_joint_pos > joint_limit.position_upper ||
			current_joint_pos < joint_limit.position_lower) {
			throw std::runtime_error(
				"Cannot enable joint limits for joint [" +
				joint_limit.joint_name + "] in robot [" + robot_name +
				"] that is currently outside of its limits");
		}
		dyn_joint->setJointLimits(joint_limit.position_lower,
								  joint_limit.position_upper);
	}
}

void Sai2Simulation::disableJointLimits(const std::string& robot_name) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot disable joint limits robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	auto dyn_robot = _world->getBaseNode(robot_name);
	for (const Sai2Model::JointLimit joint_limit :
		 _robot_models.at(robot_name)->jointLimits()) {
		auto dyn_joint = dyn_robot->getJoint(joint_limit.joint_name);
		dyn_joint->removeJointLimits();
	}
}

// set joint positions
void Sai2Simulation::setJointPositions(const std::string& robot_name,
									   const Eigen::VectorXd& q) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot set positions for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	if (q.size() != qSize(robot_name)) {
		throw std::invalid_argument(
			"q_size mismatch, cannot set positions for robot [" + robot_name +
			"]");
	}
	auto robot = _world->getBaseNode(robot_name);
	uint q_ind_counter = 0;
	uint sph_joint_counter = 0;
	uint dofs = dof(robot_name);
	chai3d::cQuaternion sph_quat;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_quat.w = q[dofs + sph_joint_counter];
			sph_quat.x = q[q_ind_counter];
			sph_quat.y = q[q_ind_counter + 1];
			sph_quat.z = q[q_ind_counter + 2];
			robot->m_dynamicJoints[i]->setPosSpherical(sph_quat);
			sph_joint_counter++;
			q_ind_counter += 3;
		} else {
			robot->m_dynamicJoints[i]->setPos(q[q_ind_counter++]);
		}
	}
}

// read joint positions
const Eigen::VectorXd Sai2Simulation::getJointPositions(
	const std::string& robot_name) const {
	if (!robotExistsInWorld(robot_name)) {
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
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_quat = robot->m_dynamicJoints[i]->getPosSpherical();
			q_ret[dofs + sph_joint_counter] = sph_quat.w;
			q_ret[q_ind_counter] = sph_quat.x;
			q_ret[q_ind_counter + 1] = sph_quat.y;
			q_ret[q_ind_counter + 2] = sph_quat.z;
			sph_joint_counter++;
			q_ind_counter += 3;
		} else {
			q_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getPos();
		}
	}
	return q_ret;
}

// read object pose
const Eigen::Affine3d Sai2Simulation::getObjectPose(
	const std::string& object_name) const {
	if (!objectExistsInWorld(object_name)) {
		throw std::invalid_argument(
			"cannot get pose for object [" + object_name +
			"] that does not exists in simulated world");
	}
	auto object = _world->getBaseNode(object_name);
	chai3d::cQuaternion quat = object->m_dynamicJoints[3]->getPosSpherical();

	Eigen::Affine3d obj_pose_from_base =
		Eigen::Affine3d(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
	obj_pose_from_base.translation() << object->m_dynamicJoints[0]->getPos(),
		object->m_dynamicJoints[1]->getPos(),
		object->m_dynamicJoints[2]->getPos();

	return _dyn_object_base_pose.at(object_name) * obj_pose_from_base;
}

// set object pose
void Sai2Simulation::setObjectPose(const std::string& object_name,
								   const Eigen::Affine3d& pose) const {
	if (!objectExistsInWorld(object_name)) {
		throw std::invalid_argument(
			"cannot set pose for object [" + object_name +
			"] that does not exists in simulated world");
	}
	auto object = _world->getBaseNode(object_name);

	Eigen::Affine3d pose_local =
		_dyn_object_base_pose.at(object_name).inverse() * pose;

	for (int i = 0; i < 3; i++) {
		object->m_dynamicJoints[i]->setPos(pose_local.translation()(i));
	}
	chai3d::cQuaternion quat;
	quat.fromRotMat(pose_local.rotation());

	object->m_dynamicJoints[3]->setPosSpherical(quat);
}

// set joint position for a single joint
void Sai2Simulation::setJointPosition(const std::string& robot_name,
									  unsigned int joint_id, double position) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot set positions for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	auto robot = _world->getBaseNode(robot_name);
	if (joint_id >= robot->m_dynamicJoints.size()) {
		throw std::invalid_argument(
			"cannot set positions for a joint out of bounds");
	}
	if (robot->m_dynamicJoints[joint_id]->getJointType() ==
		DYN_JOINT_SPHERICAL) {
		throw std::invalid_argument(
			"cannot set individual position for a spherical joint");
	}
	robot->m_dynamicJoints[joint_id]->setPos(position);
}

// set joint velocities
void Sai2Simulation::setJointVelocities(const std::string& robot_name,
										const Eigen::VectorXd& dq) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot set velocities for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	if (dq.size() != dof(robot_name)) {
		throw std::invalid_argument(
			"dq size mismatch, cannot set velocities for robot [" + robot_name +
			"]");
	}
	auto robot = _world->getBaseNode(robot_name);
	uint q_ind_counter = 0;
	chai3d::cVector3d sph_vel;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_vel = Eigen::Vector3d(dq[q_ind_counter], dq[q_ind_counter + 1],
									  dq[q_ind_counter + 2]);
			robot->m_dynamicJoints[i]->setVelSpherical(sph_vel);
			q_ind_counter += 3;
		} else {
			robot->m_dynamicJoints[i]->setVel(dq[q_ind_counter++]);
		}
	}
}

// set joint velocity for a single joint
void Sai2Simulation::setJointVelocity(const std::string& robot_name,
									  unsigned int joint_id, double velocity) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot set velocities for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	auto robot = _world->getBaseNode(robot_name);
	if (joint_id >= robot->m_dynamicJoints.size()) {
		throw std::invalid_argument(
			"cannot set velocities for a joint out of bounds");
	}
	if (robot->m_dynamicJoints[joint_id]->getJointType() ==
		DYN_JOINT_SPHERICAL) {
		throw std::invalid_argument(
			"cannot set individual velocity for a spherical joint");
	}
	robot->m_dynamicJoints[joint_id]->setVel(velocity);
}

// read joint velocities
const Eigen::VectorXd Sai2Simulation::getJointVelocities(
	const std::string& robot_name) const {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot get velocities for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	auto robot = _world->getBaseNode(robot_name);
	Eigen::VectorXd dq_ret(dof(robot_name));
	uint q_ind_counter = 0;
	uint dofs = dof(robot_name);
	chai3d::cVector3d sph_vel;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_vel = robot->m_dynamicJoints[i]->getVelSpherical();
			dq_ret[q_ind_counter] = sph_vel.x();
			dq_ret[q_ind_counter + 1] = sph_vel.y();
			dq_ret[q_ind_counter + 2] = sph_vel.z();
			q_ind_counter += 3;
		} else {
			dq_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getVel();
		}
	}
	return dq_ret;
}

// read object velocities
const Eigen::VectorXd Sai2Simulation::getObjectVelocity(
	const std::string& object_name) const {
	if (!objectExistsInWorld(object_name)) {
		throw std::invalid_argument(
			"cannot get velocities for object [" + object_name +
			"] that does not exists in simulated world");
	}
	auto object = _world->getBaseNode(object_name);
	Eigen::VectorXd vel_ret(6);
	vel_ret.head(3) << object->m_dynamicJoints[0]->getVel(),
		object->m_dynamicJoints[1]->getVel(),
		object->m_dynamicJoints[2]->getVel();
	vel_ret.tail(3) = object->m_dynamicJoints[3]->getVelSpherical().eigen();
	vel_ret.head(3) =
		_dyn_object_base_pose.at(object_name).rotation() * vel_ret.head(3);
	vel_ret.tail(3) =
		_dyn_object_base_pose.at(object_name).rotation() * vel_ret.tail(3);
	return vel_ret;
}

// set joint torques
void Sai2Simulation::setJointTorques(const std::string& robot_name,
									 const Eigen::VectorXd& tau) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot set torques for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	if (tau.size() != dof(robot_name)) {
		throw std::invalid_argument(
			"size of torque vector inconsistent in setJointTorques");
	}
	_applied_robot_torques.at(robot_name) = tau;
}

// set joint torque for a single joint
void Sai2Simulation::setJointTorque(const std::string& robot_name,
									unsigned int joint_id, double tau) {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot set torque for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	_applied_robot_torques.at(robot_name)(joint_id) = tau;
}

void Sai2Simulation::setObjectForceTorque(const std::string& object_name,
										  const Eigen::Vector6d& tau) {
	if (!objectExistsInWorld(object_name)) {
		throw std::invalid_argument(
			"cannot set torques for object [" + object_name +
			"] that does not exists in simulated world");
	}
	// linear forces are applied in object base frame (before the rotation from
	// the spherical joint)
	_applied_object_torques.at(object_name).head<3>() =
		_dyn_object_base_pose.at(object_name).rotation().transpose() *
		tau.head<3>();
	// spherical torques are applied in the object local frame (after rotation
	// from the spherical joint)
	Eigen::Affine3d object_pose = getObjectPose(object_name);
	_applied_object_torques.at(object_name).tail<3>() =
		object_pose.rotation().transpose() * tau.tail<3>();
}

void Sai2Simulation::setAllJointTorquesInternal() {
	for (const auto& pair : _applied_robot_torques) {
		auto robot_name = pair.first;
		auto tau = pair.second;
		auto robot_model = _robot_models.at(robot_name);
		Eigen::VectorXd gravity_torques =
			Eigen::VectorXd::Zero(robot_model->dof());
		if (_gravity_compensation_enabled) {
			gravity_torques = robot_model->jointGravityVector();
		}

		auto robot = _world->getBaseNode(robot_name);
		uint q_ind_counter = 0;
		chai3d::cVector3d sph_tau;
		for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
			if (robot->m_dynamicJoints[i]->getJointType() ==
				DYN_JOINT_SPHERICAL) {
				sph_tau = Eigen::Vector3d(
					tau[q_ind_counter] + gravity_torques[q_ind_counter],
					tau[q_ind_counter + 1] + gravity_torques[q_ind_counter + 1],
					tau[q_ind_counter + 2] +
						gravity_torques[q_ind_counter + 2]);
				robot->m_dynamicJoints[i]->setTorque(sph_tau);
				q_ind_counter += 3;
			} else {
				robot->m_dynamicJoints[i]->setForce(
					gravity_torques[q_ind_counter] + tau[q_ind_counter]);
				q_ind_counter++;
			}
		}
	}
	for (const auto& pair : _applied_object_torques) {
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
const Eigen::VectorXd Sai2Simulation::getJointAccelerations(
	const std::string& robot_name) const {
	if (!robotExistsInWorld(robot_name)) {
		throw std::invalid_argument(
			"cannot get accelerations for robot [" + robot_name +
			"] that does not exists in simulated world");
	}
	auto robot = _world->getBaseNode(robot_name);
	Eigen::VectorXd ddq_ret(dof(robot_name));
	uint q_ind_counter = 0;
	chai3d::cVector3d sph_acc;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_acc = robot->m_dynamicJoints[i]->getAccelSpherical();
			ddq_ret[q_ind_counter] = sph_acc.x();
			ddq_ret[q_ind_counter + 1] = sph_acc.y();
			ddq_ret[q_ind_counter + 2] = sph_acc.z();
			q_ind_counter += 3;
		} else {
			ddq_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getAccel();
		}
	}
	return ddq_ret;
}

// integrate ahead
void Sai2Simulation::integrate() {
	if (_is_paused) {
		return;
	}

	setAllJointTorquesInternal();

	// update dynamic world
	_world->updateDynamics(_timestep);
	_time += _timestep;
	// update robot models
	for (auto pair : _robot_models) {
		auto robot_name = pair.first;
		auto robot_model = pair.second;
		robot_model->setQ(getJointPositions(robot_name));
		robot_model->updateKinematics();
	}
	// update force sensors if any
	for (auto sensor : _force_sensors) {
		sensor->update(getDynamicWorld());
	}
}

void Sai2Simulation::showContactInfo() {
	std::list<cDynamicBase*>::iterator i;
	for (i = _world->m_dynamicObjects.begin();
		 i != _world->m_dynamicObjects.end(); ++i) {
		cDynamicBase* object = *i;
		int num_contacts = object->m_dynamicContacts->getNumContacts();
		// consider only contacting objects
		if (num_contacts > 0) {
			std::cout << "object name : " << object->m_name << std::endl;
			std::cout << "num contacts : " << num_contacts << std::endl;
			for (int k = 0; k < num_contacts; k++) {
				cDynamicContact* contact =
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

void Sai2Simulation::showLinksInContact(const std::string robot_name) {
	std::list<cDynamicBase*>::iterator i;
	for (i = _world->m_dynamicObjects.begin();
		 i != _world->m_dynamicObjects.end(); ++i) {
		cDynamicBase* object = *i;
		if (object->m_name == robot_name) {
			int num_contacts = object->m_dynamicContacts->getNumContacts();
			if (num_contacts > 0) {
				std::set<std::string> contact_links;
				std::cout << "contacts on robot : " << robot_name << std::endl;
				for (int k = 0; k < num_contacts; k++) {
					contact_links.insert(
						object->m_dynamicContacts->getContact(k)
							->m_dynamicLink->m_name);
				}
				for (std::set<std::string>::iterator it = contact_links.begin();
					 it != contact_links.end(); ++it) {
					std::cout << (*it) << std::endl;
				}
				std::cout << std::endl;
			}
		}
	}
}

const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
Sai2Simulation::getContactList(const ::std::string& robot_name,
							   const std::string& link_name) const {
	return _world->getContactList(robot_name, link_name);
}

void Sai2Simulation::addSimulatedForceSensor(
	const std::string& robot_name, const std::string& link_name,
	const Eigen::Affine3d transform_in_link,
	const double filter_cutoff_frequency) {
	if (!robotExistsInWorld(robot_name, link_name)) {
		std::cout << "\n\nWARNING: trying to add a force sensor to an "
					 "unexisting robot or link in "
					 "Sai2Simulation::addSimulatedForceSensor\n"
				  << std::endl;
		return;
	}
	if (findSimulatedForceSensor(robot_name, link_name) != -1) {
		std::cout << "\n\nWARNING: only one force sensor is supported per "
					 "link in Sai2Simulation::addSimulatedForceSensor. Not "
					 "adding the second one\n"
				  << std::endl;
		return;
	}
	_force_sensors.push_back(std::make_shared<ForceSensorSim>(
		robot_name, link_name, transform_in_link, _robot_models[robot_name],
		filter_cutoff_frequency * _timestep));
}

Eigen::Vector3d Sai2Simulation::getSensedForce(
	const std::string& robot_name, const std::string& link_name,
	const bool in_sensor_frame) const {
	int sensor_index = findSimulatedForceSensor(robot_name, link_name);
	if (sensor_index == -1) {
		std::cout << "WARNING: no force sensor registered on robot ["
				  << robot_name << "] and link [" << link_name
				  << "]. Returning Zero forces" << std::endl;
		return Eigen::Vector3d::Zero();
	}
	if (in_sensor_frame) {
		return _force_sensors.at(sensor_index)->getForceLocalFrame();
	}
	return _force_sensors.at(sensor_index)->getForceWorldFrame();
}

Eigen::Vector3d Sai2Simulation::getSensedMoment(
	const std::string& robot_name, const std::string& link_name,
	const bool in_sensor_frame) const {
	int sensor_index = findSimulatedForceSensor(robot_name, link_name);
	if (sensor_index == -1) {
		std::cout << "WARNING: no force sensor registered on robot ["
				  << robot_name << "] and link [" << link_name
				  << "]. Returning Zero moments" << std::endl;
		return Eigen::Vector3d::Zero();
	}
	if (in_sensor_frame) {
		return _force_sensors.at(sensor_index)->getMomentLocalFrame();
	}
	return _force_sensors.at(sensor_index)->getMomentWorldFrame();
}

const std::vector<Sai2Model::ForceSensorData>
Sai2Simulation::getAllForceSensorData() const {
	std::vector<Sai2Model::ForceSensorData> sensor_data;
	for (const auto& sensor : _force_sensors) {
		sensor_data.push_back(sensor->getData());
	}
	return sensor_data;
}

const int Sai2Simulation::findSimulatedForceSensor(
	const std::string& robot_name, const std::string& link_name) const {
	for (int i = 0; i < _force_sensors.size(); ++i) {
		if ((_force_sensors.at(i)->getData().robot_name == robot_name) &&
			(_force_sensors.at(i)->getData().link_name == link_name)) {
			return i;
		}
	}
	return -1;
}

const bool Sai2Simulation::robotExistsInWorld(
	const std::string& robot_name, const std::string link_name) const {
	auto it = _robot_models.find(robot_name);
	if (it == _robot_models.end()) {
		return false;
	}
	if (link_name != "") {
		return _robot_models.at(robot_name)->isLinkInRobot(link_name);
	}
	return true;
}

const bool Sai2Simulation::objectExistsInWorld(
	const std::string& object_name) const {
	auto it = _dyn_object_base_pose.find(object_name);
	if (it == _dyn_object_base_pose.end()) {
		return false;
	}
	return true;
}

// set restitution co-efficients: for all objects
void Sai2Simulation::setCollisionRestitution(double restitution) {
	// for all dynamic base
	for (cDynamicBase* base : _world->m_dynamicObjects) {
		// for all links in this base
		for (cDynamicLink* link : base->m_dynamicLinks) {
			cDynamicMaterial* mat = link->getDynamicMaterial();
			mat->setEpsilon(restitution);
		}
	}
}

// set restitution co-efficients: for a named object
void Sai2Simulation::setCollisionRestitution(const std::string& object_name,
											 double restitution) {
	setCollisionRestitution(object_name, "object_link", restitution);
}

// set restitution co-efficients: for a named link
void Sai2Simulation::setCollisionRestitution(const std::string& robot_name,
											 const std::string& link_name,
											 double restitution) {
	if (!robotExistsInWorld(robot_name, link_name)) {
		throw std::invalid_argument(
			"cannot set collision restitution to link [" + link_name +
			"] of [" + robot_name + "] that doesn't exists in simulation");
	}
	auto robot = _world->getBaseNode(robot_name);
	auto link = robot->getLink(link_name);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	mat->setEpsilon(restitution);
}

const double Sai2Simulation::getCollisionRestitution(
	const std::string& object_name) const {
	return getCollisionRestitution(object_name, "object_link");
}

// get co-efficient of restitution: for a named robot and link
const double Sai2Simulation::getCollisionRestitution(
	const std::string& robot_name, const std::string& link_name) const {
	if (!robotExistsInWorld(robot_name, link_name)) {
		throw std::invalid_argument(
			"cannot get collision restitution to link [" + link_name +
			"] of [" + robot_name + "] that doesn't exists in simulation");
	}
	auto robot = _world->getBaseNode(robot_name);
	auto link = robot->getLink(link_name);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	return mat->getEpsilon();
}

// set co-efficient of static friction: for all objects
void Sai2Simulation::setCoeffFrictionStatic(double static_friction) {
	// for all dynamic base
	for (cDynamicBase* base : _world->m_dynamicObjects) {
		// for all links in this base
		for (cDynamicLink* link : base->m_dynamicLinks) {
			cDynamicMaterial* mat = link->getDynamicMaterial();
			mat->setStaticFriction(static_friction);
		}
	}
}

void Sai2Simulation::setCoeffFrictionStatic(const std::string& object_name,
											double static_friction) {
	setCoeffFrictionStatic(object_name, "object_link", static_friction);
}

// set co-efficient of static friction: for a named robot and link
void Sai2Simulation::setCoeffFrictionStatic(const std::string& robot_name,
											const std::string& link_name,
											double static_friction) {
	if (!robotExistsInWorld(robot_name, link_name)) {
		throw std::invalid_argument(
			"cannot set static coefficient of friction to link [" + link_name +
			"] of [" + robot_name + "] that doesn't exists in simulation");
	}
	auto robot = _world->getBaseNode(robot_name);
	auto link = robot->getLink(link_name);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	mat->setStaticFriction(static_friction);
}

const double Sai2Simulation::getCoeffFrictionStatic(
	const std::string& object_name) const {
	return getCoeffFrictionStatic(object_name, "object_link");
}

// get co-efficient of static friction: for a named robot and link
const double Sai2Simulation::getCoeffFrictionStatic(
	const std::string& robot_name, const std::string& link_name) const {
	if (!robotExistsInWorld(robot_name, link_name)) {
		throw std::invalid_argument(
			"cannot get static coefficient of friction to link [" + link_name +
			"] of [" + robot_name + "] that doesn't exists in simulation");
	}
	auto robot = _world->getBaseNode(robot_name);
	auto link = robot->getLink(link_name);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	return mat->getStaticFriction();
}

// set co-efficient of dynamic friction: for all object
void Sai2Simulation::setCoeffFrictionDynamic(double dynamic_friction) {
	// for all dynamic base
	for (cDynamicBase* base : _world->m_dynamicObjects) {
		// for all links in this base
		for (cDynamicLink* link : base->m_dynamicLinks) {
			cDynamicMaterial* mat = link->getDynamicMaterial();
			mat->setDynamicFriction(dynamic_friction);
		}
	}
}

void Sai2Simulation::setCoeffFrictionDynamic(const std::string& object_name,
											 double dynamic_friction) {
	setCoeffFrictionDynamic(object_name, "object_link", dynamic_friction);
}

// set co-efficient of dynamic friction: for a named robot and link
void Sai2Simulation::setCoeffFrictionDynamic(const std::string& robot_name,
											 const std::string& link_name,
											 double dynamic_friction) {
	if (!robotExistsInWorld(robot_name, link_name)) {
		throw std::invalid_argument(
			"cannot set dynamic coefficient of friction to link [" + link_name +
			"] of [" + robot_name + "] that doesn't exists in simulation");
	}
	auto robot = _world->getBaseNode(robot_name);
	auto link = robot->getLink(link_name);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	mat->setDynamicFriction(dynamic_friction);
}

const double Sai2Simulation::getCoeffFrictionDynamic(
	const std::string& object_name) const {
	return getCoeffFrictionDynamic(object_name, "object_link");
}

// get co-efficient of dynamic friction: for a named robot and link
const double Sai2Simulation::getCoeffFrictionDynamic(
	const std::string& robot_name, const std::string& link_name) const {
	if (!robotExistsInWorld(robot_name, link_name)) {
		throw std::invalid_argument(
			"cannot get dynamic coefficient of friction to link [" + link_name +
			"] of [" + robot_name + "] that doesn't exists in simulation");
	}
	auto robot = _world->getBaseNode(robot_name);
	auto link = robot->getLink(link_name);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	return mat->getDynamicFriction();
}

// get pose of robot base in the world frame
const Eigen::Affine3d Sai2Simulation::getRobotBaseTransform(
	const std::string& robot_name) const {
	Eigen::Affine3d gToRobotBase;
	const auto base = _world->getBaseNode(robot_name);
	gToRobotBase.translation() = base->getLocalPos().eigen();
	gToRobotBase.linear() = base->getLocalRot().eigen();
	return gToRobotBase;
}

}  // namespace Sai2Simulation
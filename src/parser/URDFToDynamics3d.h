/*
 * URDFToDynamics3d.h
 *
 *  Created on: Dec 23, 2016
 *      Author: Shameek Ganguly
 *  Update: ported to sai-simulation project on Nov 17, 2017
 *    By: Shameek Ganguly
 */

#ifndef URDF_TO_DYNAMICS3D_H
#define URDF_TO_DYNAMICS3D_H

#include <string>

#include "dynamics3d.h"

namespace SaiSimulation
{
	/**
	 * @brief Parse a URDF file and populate a dynamics3d world model from it.
	 * @param filename URDF world model file to parse.
	 * @param world Dynamics3d model to populate from parsed file.
	 * @param verbose To display information about the robot model creation in the
	 * terminal or not.
	 */
	void URDFToDynamics3dWorld(
		const std::string &filename, std::shared_ptr<cDynamicWorld> world,
		std::map<std::string, Eigen::Affine3d> &dyn_objects_pose,
		std::map<std::string, Eigen::Affine3d> &static_objects_pose,
		std::map<std::string, std::string> &robot_filenames, bool verbose);

	/**
	 * @brief Parse a URDF file and populate a single dynamics3d robot model from
	 * it.
	 * @param filename URDF robot model file to parse.
	 * @param model Dynamics3d model to populate from parsed file.
	 * @param verbose To display information about the robot model creation in the
	 * terminal or not.
	 * @param working_dirname Directory path relative to which paths within the
	 * model file are specified.
	 */
	void URDFToDynamics3dRobot(const std::string &filename, cDynamicBase *model,
							   bool verbose,
							   const std::string &working_dirname = "./");
	// TODO: working dir default should be "", but this requires checking
	// to make sure that the directory path has a trailing backslash

} // namespace SaiSimulation

#endif // URDF_TO_DYNAMICS3D_H

/*
 * URDFToDynamics3d.cpp
 *
 *  Created on: Dec 23, 2016
 *      Author: Shameek Ganguly
 *  Update: ported to sai-simulation project on Nov 17, 2017
 *    By: Shameek Ganguly
 */

#include "URDFToDynamics3d.h"

#include <urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>

#include "parser/SaiModelParserUtils.h"

typedef my_shared_ptr<SaiUrdfreader::Link> LinkPtr;
typedef const my_shared_ptr<const SaiUrdfreader::Link> ConstLinkPtr;
typedef my_shared_ptr<SaiUrdfreader::Joint> JointPtr;
typedef my_shared_ptr<SaiUrdfreader::ModelInterface> ModelPtr;
typedef my_shared_ptr<SaiUrdfreader::World> WorldPtr;

#include <Eigen/Core>
using namespace Eigen;

#include <assert.h>

#include <fstream>
#include <iostream>
#include <map>
#include <stack>
#include <vector>
using namespace std;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr> URDFLinkMap;
typedef map<string, JointPtr> URDFJointMap;

using namespace chai3d;
#define CDYN_ERROR_EPSILON 0.0001
#define CDYN_DEFAULT_MAT_RESTITUTION 0.9

namespace SaiSimulation
{

	// load link collision properties from urdf link to dynamics3d link
	// TODO: working dir default should be "", but this requires checking
	// to make sure that the directory path has a trailing backslash
	static void loadLinkCollision(
		cDynamicLink *link,
		const my_shared_ptr<SaiUrdfreader::Collision> &collision_ptr,
		const std::string &working_dirname = "./",
		const Eigen::Matrix3d &joint_axis_alignment_rotation =
			Eigen::Matrix3d::Identity())
	{
		auto tmp_mmesh = new cMultiMesh();
		tmp_mmesh->m_name = std::string("sai_dyn3d_link_mesh");
		auto tmp_mesh = new cMesh();
		const auto geom_type = collision_ptr->geometry->type;
		if (geom_type == SaiUrdfreader::Geometry::MESH)
		{
			// downcast geometry ptr to mesh type
			const auto mesh_ptr = dynamic_cast<const SaiUrdfreader::Mesh *>(
				collision_ptr->geometry.get());
			assert(mesh_ptr);

			// load object
			bool file_load_success = false;

			std::string processed_filepath =
				working_dirname + "/" + mesh_ptr->filename;
			if (SaiModel::ReplaceUrdfPathPrefix(mesh_ptr->filename) !=
				mesh_ptr->filename)
			{
				processed_filepath =
					SaiModel::ReplaceUrdfPathPrefix(mesh_ptr->filename);
			}

			if (processed_filepath.length() < 5)
			{
				cerr << "Couldn't load obj/3ds/STL robot link file, extension not "
						"supported: "
					 << processed_filepath << endl;
				abort();
			}

			std::string extension =
				processed_filepath.substr(processed_filepath.length() - 4);
			std::transform(extension.begin(), extension.end(), extension.begin(),
						   [](unsigned char c)
						   { return std::tolower(c); });

			if (extension == ".stl")
			{
				file_load_success = cLoadFileSTL(tmp_mmesh, processed_filepath);
			}
			else if (extension == ".obj")
			{
				file_load_success = cLoadFileOBJ(tmp_mmesh, processed_filepath);
			}
			else if (extension == ".3ds")
			{
				file_load_success = cLoadFile3DS(tmp_mmesh, processed_filepath);
			}
			if (!file_load_success)
			{
				cerr << "Couldn't load obj/3ds/STL robot link file: "
					 << processed_filepath << endl;
				abort();
			}

			// apply scale
			tmp_mmesh->scaleXYZ(mesh_ptr->scale.x, mesh_ptr->scale.y,
								mesh_ptr->scale.z);
		}
		else if (geom_type == SaiUrdfreader::Geometry::BOX)
		{
			// downcast geometry ptr to box type
			const auto box_ptr = dynamic_cast<const SaiUrdfreader::Box *>(
				collision_ptr->geometry.get());
			assert(box_ptr);
			// create chai box mesh
			chai3d::cCreateBox(tmp_mesh, box_ptr->dim.x, box_ptr->dim.y,
							   box_ptr->dim.z);
			tmp_mmesh->addMesh(tmp_mesh);
		}
		else if (geom_type == SaiUrdfreader::Geometry::SPHERE)
		{
			// downcast geometry ptr to sphere type
			const auto sphere_ptr = dynamic_cast<const SaiUrdfreader::Sphere *>(
				collision_ptr->geometry.get());
			assert(sphere_ptr);
			// create chai sphere mesh
			chai3d::cCreateSphere(tmp_mesh, sphere_ptr->radius);
			tmp_mmesh->addMesh(tmp_mesh);
		}
		else if (geom_type == SaiUrdfreader::Geometry::CYLINDER)
		{
			// downcast geometry ptr to cylinder type
			const auto cylinder_ptr = dynamic_cast<const SaiUrdfreader::Cylinder *>(
				collision_ptr->geometry.get());
			assert(cylinder_ptr);
			// create chai cylinder mesh
			chai3d::cCreateCylinder(tmp_mesh, cylinder_ptr->length,
									cylinder_ptr->radius, 32, 1, 1, true, true,
									cVector3d(0, 0, -cylinder_ptr->length / 2));

			tmp_mmesh->addMesh(tmp_mesh);
		}

		// transfer meshes to combined collision mesh along with local position and
		// orientation info
		auto urdf_q = collision_ptr->origin.rotation;
		Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		Matrix3d mesh_rotation = tmp_q.toRotationMatrix();
		Eigen::Vector3d mesh_position(collision_ptr->origin.position.x,
									  collision_ptr->origin.position.y,
									  collision_ptr->origin.position.z);

		mesh_position = joint_axis_alignment_rotation.transpose() * mesh_position;
		mesh_rotation = joint_axis_alignment_rotation.transpose() * mesh_rotation;

		//-- New code to combine meshes -- This results in multiple collision meshes
		// on the same link
		tmp_mmesh->setLocalPos(cVector3d(mesh_position));
		tmp_mmesh->setLocalRot(cMatrix3d(mesh_rotation));
		link->setCollisionModel(tmp_mmesh);
		// build collision model (currently, only convex hull and box are
		// supported!)
		link->buildCollisionHull(CDYN_ERROR_EPSILON, CDYN_ERROR_EPSILON);
	}

	// load inertial properties from urdf link to dynamics3d link
	static void loadLinkInertial(
		cDynamicLink *link, ConstLinkPtr &urdf_link,
		const Eigen::Matrix3d &joint_axis_alignment_rotation =
			Eigen::Matrix3d::Identity())
	{
		Vector3d link_inertial_rpy;		 // inertial frame rotation in Euler XYZ moving
										 // angles (Roll, Pitch, Yaw)
		Vector3d link_inertial_position; // COM
		Matrix3d link_inertial_inertia;	 // inertia
		double link_inertial_mass;		 // link mass

		if (urdf_link->inertial)
		{
			// copy root mass
			link_inertial_mass = urdf_link->inertial->mass;

			// copy root com
			link_inertial_position << urdf_link->inertial->origin.position.x,
				urdf_link->inertial->origin.position.y,
				urdf_link->inertial->origin.position.z;

			// copy root COM inertia
			link_inertial_inertia(0, 0) = urdf_link->inertial->ixx;
			link_inertial_inertia(0, 1) = urdf_link->inertial->ixy;
			link_inertial_inertia(0, 2) = urdf_link->inertial->ixz;

			link_inertial_inertia(1, 0) = urdf_link->inertial->ixy;
			link_inertial_inertia(1, 1) = urdf_link->inertial->iyy;
			link_inertial_inertia(1, 2) = urdf_link->inertial->iyz;

			link_inertial_inertia(2, 0) = urdf_link->inertial->ixz;
			link_inertial_inertia(2, 1) = urdf_link->inertial->iyz;
			link_inertial_inertia(2, 2) = urdf_link->inertial->izz;

			// copy root inertia rotation (Euler XYZ moving notation)
			// root_inertial_rpy is not used anywhere. It is not supported
			// currently.
			urdf_link->inertial->origin.rotation.getRPY(
				link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

			if (link_inertial_rpy != Vector3d(0., 0., 0.))
			{
				cerr << "Error while processing body '" << urdf_link->name
					 << "': rotation of body frames not yet supported. Please "
						"rotate the joint frame instead."
					 << endl;
				abort();
			}
		}
		else
		{
			cerr << "Error while processing body '" << urdf_link->name
				 << "': no inertial data found. Please specify inertial data."
				 << endl;
		}

		// transform the inertial properties back to the ones
		// defined in the actual frame from urdf if needed
		link_inertial_position =
			joint_axis_alignment_rotation.transpose() * link_inertial_position;
		link_inertial_inertia = joint_axis_alignment_rotation.transpose() *
								link_inertial_inertia *
								joint_axis_alignment_rotation;

		// add inertial properties to link
		link->setMassProperties(link_inertial_mass,
								cMatrix3d(link_inertial_inertia),
								cVector3d(link_inertial_position));
	}

	void URDFToDynamics3dWorld(
		const std::string &filename, std::shared_ptr<cDynamicWorld> world,
		std::map<std::string, Eigen::Affine3d> &dyn_objects_pose,
		std::map<std::string, Eigen::Affine3d> &static_object_pose,
		std::map<std::string, std::string> &robot_filenames, bool verbose)
	{
		// load world urdf file
		std::string resolved_filename = SaiModel::ReplaceUrdfPathPrefix(filename);
		ifstream model_file(resolved_filename);
		if (!model_file)
		{
			cerr << "Error opening file '" << resolved_filename << "'." << endl;
			abort();
		}

		// reserve memory for the contents of the file
		string model_xml_string;
		model_file.seekg(0, std::ios::end);
		model_xml_string.reserve(model_file.tellg());
		model_file.seekg(0, std::ios::beg);
		model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
								std::istreambuf_iterator<char>());

		model_file.close();

		// parse xml to URDF world model
		assert(world);
		WorldPtr urdf_world = SaiUrdfreader::parseURDFWorld(model_xml_string);
		world->m_name = urdf_world->name_;
		if (verbose)
		{
			cout << "URDFToDynamics3dWorld: Starting model conversion to "
					"dynamics3d world."
				 << endl;
			cout << "+ add world: " << world->m_name << endl;
		}

		// parse gravity
		world->setGravity(cVector3d(urdf_world->gravity_.x, urdf_world->gravity_.y,
									urdf_world->gravity_.z));

		// load parsed robots specifications into the dynamics3d world
		for (const auto robot_spec_pair : urdf_world->models_)
		{
			const auto robot_spec = robot_spec_pair.second;

			// get translation
			auto tmp_cvec3 = cVector3d(robot_spec->origin.position.x,
									   robot_spec->origin.position.y,
									   robot_spec->origin.position.z);
			// get rotation
			auto urdf_q = robot_spec->origin.rotation;
			Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
			cMatrix3d tmp_cmat3;
			tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

			// create new robot base
			cDynamicBase *robot = world->newBaseObject(tmp_cvec3, tmp_cmat3);
			assert(robot);
			robot->setLocalPos(tmp_cvec3);
			robot->setLocalRot(tmp_cmat3);

			// load robot from file
			URDFToDynamics3dRobot(
				robot_spec->model_filename, robot, verbose,
				SaiModel::ReplaceUrdfPathPrefix(robot_spec->model_working_dir));
			assert(robot->m_name == robot_spec->model_name);

			// overwrite robot name with custom name for this instance
			robot->m_name = robot_spec->name;
			// fill robot filenames
			auto it = robot_filenames.find(robot->m_name);
			if (it != robot_filenames.end())
			{
				throw std::runtime_error(
					"Different robots cannot have the same name in the world");
			}
			robot_filenames[robot->m_name] =
				SaiModel::ReplaceUrdfPathPrefix(robot_spec->model_working_dir) +
				"/" + robot_spec->model_filename;
		}

		// parse static meshes
		for (const auto object_pair : urdf_world->graphics_.static_objects)
		{
			const auto object_ptr = object_pair.second;
			// get translation
			auto tmp_cvec3 = cVector3d(object_ptr->origin.position.x,
									   object_ptr->origin.position.y,
									   object_ptr->origin.position.z);
			// get rotation
			auto urdf_q = object_ptr->origin.rotation;
			Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
			cMatrix3d tmp_cmat3;
			tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

			// initialize a cGenericObject to represent this object in the world
			cDynamicBase *object = world->newBaseObject(tmp_cvec3, tmp_cmat3);
			assert(object);
			object->setLocalPos(tmp_cvec3);
			object->setLocalRot(tmp_cmat3);
			object->m_name = object_ptr->name;

			// record pose
			static_object_pose[object->m_name] = Eigen::Affine3d(tmp_q);
			static_object_pose.at(object->m_name).translation() = tmp_cvec3.eigen();

			// add a link for collision
			auto default_mat = new cDynamicMaterial();
			// TODO: parse material property from file
			default_mat->setEpsilon(
				CDYN_DEFAULT_MAT_RESTITUTION); // default epsilon
			cDynamicLink *link = object->newLink(default_mat);
			link->m_name = object_link_name;

			// load object graphics, must have atleast one
			for (const auto collision_ptr : object_ptr->collision_array)
			{
				loadLinkCollision(link, collision_ptr);
			}

			// add link to object base
			object->linkChild(link, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
		}

		// parse dynamic meshes
		for (const auto object_pair : urdf_world->graphics_.dynamic_objects)
		{
			const auto object_ptr = object_pair.second;
			// get translation
			auto tmp_cvec3 = cVector3d(object_ptr->origin.position.x,
									   object_ptr->origin.position.y,
									   object_ptr->origin.position.z);
			// get rotation
			auto urdf_q = object_ptr->origin.rotation;
			// chai3d::cQuaternion tmp_q_chai(urdf_q.w, urdf_q.x, urdf_q.y,
			// urdf_q.z);
			Eigen::Quaterniond tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
			cMatrix3d tmp_cmat3;
			tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

			// initialize a cGenericObject to represent this object in the world
			cDynamicBase *object = world->newBaseObject(tmp_cvec3, tmp_cmat3);
			// cDynamicBase* object = world->newBaseObject(cVector3d(0,0,0),
			// cIdentity3d());
			assert(object);
			object->m_name = object_ptr->name;

			// record base pose
			dyn_objects_pose[object->m_name] = Eigen::Affine3d(tmp_q);
			dyn_objects_pose.at(object->m_name).translation() = tmp_cvec3.eigen();

			// add a link for collision
			auto default_mat = new cDynamicMaterial();
			// TODO: parse material property from file
			default_mat->setEpsilon(
				CDYN_DEFAULT_MAT_RESTITUTION); // default epsilon
			cDynamicLink *linkObject = object->newLink(default_mat);
			linkObject->m_name = object_link_name;

			// set mass properties
			Vector3d link_inertial_rpy;		 // inertial frame rotation in Euler XYZ
											 // moving angles (Roll, Pitch, Yaw)
			Vector3d link_inertial_position; // COM
			Matrix3d link_inertial_inertia;	 // inertia
			double link_inertial_mass;		 // object mass

			if (object_ptr->inertial)
			{
				// copy root mass
				link_inertial_mass = object_ptr->inertial->mass;

				// copy root com
				link_inertial_position << object_ptr->inertial->origin.position.x,
					object_ptr->inertial->origin.position.y,
					object_ptr->inertial->origin.position.z;

				// copy root COM inertia
				link_inertial_inertia(0, 0) = object_ptr->inertial->ixx;
				link_inertial_inertia(0, 1) = object_ptr->inertial->ixy;
				link_inertial_inertia(0, 2) = object_ptr->inertial->ixz;

				link_inertial_inertia(1, 0) = object_ptr->inertial->ixy;
				link_inertial_inertia(1, 1) = object_ptr->inertial->iyy;
				link_inertial_inertia(1, 2) = object_ptr->inertial->iyz;

				link_inertial_inertia(2, 0) = object_ptr->inertial->ixz;
				link_inertial_inertia(2, 1) = object_ptr->inertial->iyz;
				link_inertial_inertia(2, 2) = object_ptr->inertial->izz;

				// TODO: abort if linkObject inertia is not symmetric

				// copy root inertia rotation (Euler XYZ moving notation)
				// root_inertial_rpy is not used anywhere. It is not supported
				// currently.
				object_ptr->inertial->origin.rotation.getRPY(link_inertial_rpy[0],
															 link_inertial_rpy[1],
															 link_inertial_rpy[2]);

				if (link_inertial_rpy != Vector3d(0., 0., 0.))
				{
					cerr << "Error while processing body '" << object_ptr->name
						 << "': rotation of body frames not yet supported. Please "
							"rotate the joint frame instead."
						 << endl;
					abort();
				}
			} // else use default

			// add inertial properties to linkObject
			linkObject->setMassProperties(link_inertial_mass,
										  cMatrix3d(link_inertial_inertia),
										  cVector3d(link_inertial_position));

			// add joints
			cDynamicJoint *jointBodyX;
			cDynamicJoint *jointBodyY;
			cDynamicJoint *jointBodyZ;
			cDynamicJoint *jointBodyS;

			// create 3 prismatic joints (x,y,z)
			jointBodyX = linkObject->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_X);
			jointBodyY = linkObject->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Y);
			jointBodyZ = linkObject->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);

			// create 1 spherical joint
			jointBodyS = linkObject->newJoint(DYN_JOINT_SPHERICAL);

			// load object graphics, must have atleast one
			for (const auto collision_ptr : object_ptr->collision_array)
			{
				loadLinkCollision(linkObject, collision_ptr);
			}

			// add linkObject to object base
			object->linkChild(linkObject, cVector3d(0.0, 0.0, 0.0), cIdentity3d());

			// // set initial positions
			// object->m_dynamicJoints[0]->setPos(tmp_cvec3(0));
			// object->m_dynamicJoints[1]->setPos(tmp_cvec3(1));
			// object->m_dynamicJoints[2]->setPos(tmp_cvec3(2));
			// object->m_dynamicJoints[3]->setPosSpherical(tmp_q_chai);
		}
	}

	void URDFToDynamics3dRobot(const std::string &filename, cDynamicBase *model,
							   bool verbose, const std::string &working_dirname)
	{
		// load file
		string filepath = working_dirname + "/" + filename;
		ifstream model_file(filepath);
		if (!model_file)
		{
			cerr << "Error opening file '" << filepath << "'." << endl;
			abort();
		}

		// reserve memory for the contents of the file
		string model_xml_string;
		model_file.seekg(0, std::ios::end);
		model_xml_string.reserve(model_file.tellg());
		model_file.seekg(0, std::ios::beg);
		model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
								std::istreambuf_iterator<char>());

		model_file.close();

		// read and parse xml string to urdf model
		assert(model);
		ModelPtr urdf_model = SaiUrdfreader::parseURDF(model_xml_string);
		model->m_name = urdf_model->getName();
		if (verbose)
		{
			cout
				<< "URDFToDynamics3dRobot: Starting model conversion to dynamics3d."
				<< endl;
			cout << "+ add robot: " << model->m_name << endl;
		}

		// load urdf model to dynamics3D link tree
		LinkPtr urdf_root_link;

		URDFLinkMap link_map; // map<string, LinkPtr >
		link_map = urdf_model->links_;

		URDFJointMap joint_map; // map<string, JointPtr >
		joint_map = urdf_model->joints_;

		vector<string> joint_names;

		stack<LinkPtr> link_stack;
		stack<int> joint_index_stack;

		// add the bodies in a depth-first order of the model tree
		// push the root LinkPtr to link stack. link stack height = 1
		// NOTE: depth first search happens due to use of stack
		link_stack.push(link_map[(urdf_model->getRoot()->name)]);

		// allocate a default material shared by all links
		// TODO: load material properties from file
		auto default_mat = new cDynamicMaterial();
		default_mat->setEpsilon(CDYN_DEFAULT_MAT_RESTITUTION); // default epsilon

		// add the root body
		ConstLinkPtr &root = urdf_model->getRoot();

		// TODO: Not sure if this is ever expected to be true or not
		// 	Mikael's example URDF has it set to false
		if (root->inertial || root->collision)
		{
			// initialize a cDynamics link
			auto root_link = model->newLink(default_mat);
			if (NULL == root_link)
			{
				cerr << "Robot dynamic root link failed to create." << endl;
				abort();
			}

			// set name
			root_link->m_name = root->name;

			// add inertial properties
			loadLinkInertial(root_link, root);

			// add collision properties
			for (const auto collision_ptr : root->collision_array)
			{
				loadLinkCollision(root_link, collision_ptr, working_dirname);
			}

			// Root assumed to be fixed base, so no joint needed in dynamics3d
			// NOTE: to use a floating base, explicitly specify 6 additional joints

			if (verbose)
			{
				cout << "+ Adding Root Body " << endl;
				cout << "  joint position: " << cVector3d(0.0, 0.0, 0.0) << endl;
				cout << "  joint type : fixed" << endl;
				cout << "  body inertia: " << endl;
				cout << root_link->getInertia().getRow(0) << endl;
				cout << root_link->getInertia().getRow(1) << endl;
				cout << root_link->getInertia().getRow(2) << endl;
				cout << "  body com   : " << root_link->getCenterOfMass() << endl;
				cout << "  body mass   : " << root_link->getMass() << endl;
				cout << "  body name   : " << root_link->m_name << endl;
			}

			// append root (joint, frame and link) to the base model.
			// NOTE: by default, root is added to origin in model frame.
			model->linkChild(root_link, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
		} // endif (root->inertial || root->collision)

		if (link_stack.top()->child_joints.size() > 0)
		{
			joint_index_stack.push(0); // SG: what does this do??
		}
		else
		{
			cerr << "Base link has no associated joints!" << endl;
			abort();
		}

		// this while loop is to enumerate all joints in the tree structure by name
		while (link_stack.size() > 0)
		{
			LinkPtr cur_link = link_stack.top();
			unsigned int joint_idx = joint_index_stack.top();

			// if there are unvisited child joints on current link:
			// 	then add link to stack
			if (joint_idx < cur_link->child_joints.size())
			{
				JointPtr cur_joint = cur_link->child_joints[joint_idx];

				// increment joint index
				joint_index_stack.pop();
				joint_index_stack.push(joint_idx + 1);

				// SG: the URDF model structure is:
				//	every non-terminal link has child joint(s)
				// 	every joint has child link (else, we would get an exception
				// right below)
				link_stack.push(link_map[cur_joint->child_link_name]);
				joint_index_stack.push(0);

				if (verbose)
				{
					for (unsigned int i = 1; i < joint_index_stack.size() - 1;
						 i++)
					{
						cout << "  ";
					}
					cout << "joint '" << cur_joint->name << "' child link '"
						 << link_stack.top()->name << "' type = " << cur_joint->type
						 << endl;
				}

				joint_names.push_back(cur_joint->name);
				// SG: this is the only data structure of interest it seems
				// all joints are processed in the for loop below
			}
			else
			{ // else this link has been processed, so pop link
				link_stack.pop();
				joint_index_stack.pop();
			}
		}

		// a map that maps a joint's name to the joint_axis_alignment_rotation_matrix with respect to its parent link  <string, Matrix3d>
		std::map<std::string, Eigen::Matrix3d> joint_axis_alignment_map;

		Matrix3d parent_joint_axis_alignment_rotation = Matrix3d::Identity();
		// iterate over all joints
		for (unsigned int j = 0; j < joint_names.size(); j++)
		{
			JointPtr urdf_joint = joint_map[joint_names[j]];
			LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
			LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

			// debug
			// cout << "Processing joint: " << urdf_joint->name << "\tparent = "
			// 	 << urdf_parent->name << " child = " << urdf_child->name << endl;

			// determine where to add the current joint and child body
			cDynamicLink *parent_link = NULL;
			parent_link = model->getLink(
				urdf_parent->name); // returns NULL if link does not exist

			// create a new link
			auto dyn_link = model->newLink(default_mat);
			if (NULL == dyn_link)
			{
				cerr << "Robot dynamic link failed to create." << endl;
				abort();
			}

			// set name
			dyn_link->m_name = urdf_child->name;

			// find the joint type
			auto urdf_joint_type = urdf_joint->type;
			int dyn3d_joint_type = -1; // -1 will represent fixed joints
			switch (urdf_joint_type)
			{
			case SaiUrdfreader::Joint::REVOLUTE:
				dyn3d_joint_type = DYN_JOINT_REVOLUTE;
				break;
			case SaiUrdfreader::Joint::PRISMATIC:
				dyn3d_joint_type = DYN_JOINT_PRISMATIC;
				break;
			case SaiUrdfreader::Joint::CONTINUOUS:
				dyn3d_joint_type = DYN_JOINT_CONTINUOUS;
				break;
			case SaiUrdfreader::Joint::SPHERICAL:
				dyn3d_joint_type = DYN_JOINT_SPHERICAL;
				break;
			// currently unsupported joint types:
			case SaiUrdfreader::Joint::FLOATING:
			case SaiUrdfreader::Joint::PLANAR:
				cerr << "Unsupported joint type on joint " << joint_names[j]
					 << endl;
				abort();
				break;
			case SaiUrdfreader::Joint::FIXED: // This is true for ground links
			default:
				break;
			}

			// because dynamics3d only supports joint axis aligned with X, Y or Z
			// when it is not the case in the urdf file, we will introduce a
			// rotation to align the joint axis with Z, or X. The inertial and
			// collision properties will be rotated accordingly.
			int axis_type = DYN_AXIS_X;
			Eigen::Matrix3d joint_axis_alignment_rotation =
				Eigen::Matrix3d::Identity();
			if (urdf_joint_type == SaiUrdfreader::Joint::REVOLUTE ||
				urdf_joint_type == SaiUrdfreader::Joint::PRISMATIC ||
				urdf_joint_type == SaiUrdfreader::Joint::CONTINUOUS)
			{
				// determine the joint axis
				Eigen::Vector3d joint_axis = Eigen::Vector3d(
					urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
				if (joint_axis.norm() < 1e-6)
				{
					cerr << "Joint axis is zero for joint " << joint_names[j]
						 << endl;
					abort();
				}
				joint_axis.normalize();
				if (joint_axis.dot(Eigen::Vector3d::UnitX()) > 0.999)
				{
					axis_type = DYN_AXIS_X;
				}
				else if (joint_axis.dot(Eigen::Vector3d::UnitY()) > 0.999)
				{
					axis_type = DYN_AXIS_Y;
				}
				else if (joint_axis.dot(Eigen::Vector3d::UnitZ()) > 0.999)
				{
					axis_type = DYN_AXIS_Z;
				}
				else
				{
					Eigen::Vector3d source_axis = Eigen::Vector3d::UnitZ();
					Eigen::Vector3d dest_axis = joint_axis.normalized();

					Eigen::Vector3d k = source_axis.cross(dest_axis);
					double cos = source_axis.dot(dest_axis);
					double k_norm = k.norm();

					axis_type = DYN_AXIS_Z;
					// Edge Cases (parallel or anti-parallel)
					if (k_norm < 1e-6)
					{
						cout << "edge case" << endl;
						// parallel (joint_axis is approximately +Z)
						if (cos > 0)
						{
							joint_axis_alignment_rotation = Eigen::Matrix3d::Identity();
						}
						// anti-parallel (joint_axis is approximately -Z)
						else
						{
							cout << "anti-parallel case" << endl;
							joint_axis_alignment_rotation = Eigen::Matrix3d(
								Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
							// axis_type = DYN_AXIS_X;
						}
					}
					else
					{
						// Algebraic Rodrigues Formula
						// R = I + ([k]x) + ([k]x)^2 * (1 / (1 + cos))

						Eigen::Matrix3d K;
						K << 0, -k.z(), k.y(),
							k.z(), 0, -k.x(),
							-k.y(), k.x(), 0;

						joint_axis_alignment_rotation = Eigen::Matrix3d::Identity() + K + (K * K) * (1.0 / (1.0 + cos));
					}
				}
			}

			// add inertial properties
			loadLinkInertial(dyn_link, urdf_child, joint_axis_alignment_rotation);

			// load collision properties
			for (const auto collision_ptr : urdf_child->collision_array)
			{
				loadLinkCollision(dyn_link, collision_ptr, working_dirname,
								  joint_axis_alignment_rotation);
			}

			// create the joint
			cDynamicJoint *dyn_joint = NULL;
			if (dyn3d_joint_type != -1)
			{
				dyn_joint = dyn_link->newJoint(dyn3d_joint_type, axis_type);
			}

			if (NULL != dyn_joint)
			{
				// set joint name
				dyn_joint->m_name = joint_names[j];
				// set joint position to zero
				dyn_joint->setPos(0.0);
				// set joint damping
				if (0 != urdf_joint->dynamics)
				{
					dyn_joint->setDamping(urdf_joint->dynamics->damping);
				}
			}

			// compute the joint transformation which acts as the child link
			// transform with respect to the parent
			auto urdf_pos = urdf_joint->parent_to_joint_origin_transform.position;
			// weuse the map to populate the previous_joint_axis_alignment_rotation
			if (NULL == parent_link)
			{
				parent_joint_axis_alignment_rotation =
					Matrix3d::Identity();
			}
			else
			{
				parent_joint_axis_alignment_rotation =
					joint_axis_alignment_map[urdf_parent->name];
			}

			Vector3d joint_translation(urdf_pos.x, urdf_pos.y, urdf_pos.z);
			joint_translation = parent_joint_axis_alignment_rotation.transpose() *
								joint_translation;
			auto urdf_q = urdf_joint->parent_to_joint_origin_transform.rotation;
			Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
			cMatrix3d rot_in_parent;
			rot_in_parent.copyfrom(
				parent_joint_axis_alignment_rotation.transpose() *
				tmp_q.toRotationMatrix() * joint_axis_alignment_rotation);

			// store the current joint_axis_alignment_rotation matrix for the child link
			joint_axis_alignment_map[urdf_child->name] =
				joint_axis_alignment_rotation;

			if (verbose)
			{
				cout << "+ Adding Body " << endl;
				if (NULL == parent_link)
				{
					cout << "  parent_link_name  : NULL" << endl;
				}
				else
				{
					cout << "  parent_link_name  : " << parent_link->m_name << endl;
				}
				cout << "  position in parent: " << joint_translation.transpose()
					 << endl;
				cout << "  orientation in parent: " << tmp_q.coeffs().transpose()
					 << endl;
				if (NULL == dyn_link->getJoint(0))
				{
					cout << "  joint type: fixed " << endl;
				}
				else
				{
					cout << "  joint type (0/P,1/R,2/S): "
						 << dyn_link->getJoint(0)->getJointType() << endl;
					cout << "  joint axis (0/x,1/y,2/z): "
						 << dyn_link->getJoint(0)->getJointAxis() << endl;
					cout << "  joint name: " << dyn_link->m_name << endl;
				}
				cout << "  body inertia: " << endl;
				cout << dyn_link->getInertia().getRow(0) << endl;
				cout << dyn_link->getInertia().getRow(1) << endl;
				cout << dyn_link->getInertia().getRow(2) << endl;
				cout << "  body com   : " << dyn_link->getCenterOfMass() << endl;
				cout << "  body mass   : " << dyn_link->getMass() << endl;
				cout << "  body name   : " << dyn_link->m_name << endl;
			}

			// add link to model
			if (NULL == parent_link)
			{
				// link is located at root
				model->linkChild(dyn_link, cVector3d(joint_translation),
								 rot_in_parent);
			}
			else
			{
				parent_link->linkChild(dyn_link, cVector3d(joint_translation),
									   rot_in_parent);
			}
		}

		if (verbose)
		{
			cout
				<< "URDFToDynamics3dRobot: Finished model conversion to dynamics3d."
				<< endl;
		}
	}

} // namespace SaiSimulation

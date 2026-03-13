//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1247 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynamicLinkH
#define CDynamicLinkH
//---------------------------------------------------------------------------
#include "chai3d/CDynInternal.h"
#include "chai3d/CDynamicJoint.h"
#include "chai3d/CDynamicMaterial.h"
#include "chai3d.h"
//---------------------------------------------------------------------------
const int DYN_COLLISION_NONE = 0;
const int DYN_COLLISION_BBOX = 1;
const int DYN_COLLISION_HULL = 2;
const int DYN_COLLISION_POLY = 3;
//---------------------------------------------------------------------------
class cDynamicBase;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDynamicLink.h

    \brief
    <b> Dynamics Module </b> \n
    Dynamic Link Object.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cDynamicLink
    \ingroup    Dynamics

    \brief
    cDynamicLink models an articulated link of a robot.
*/
//===========================================================================
class cDynamicLink : public chai3d::cGenericObject
{
    friend class cDynamicBase;
    friend struct cDynamicJoint;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:
    //! Constructor of cDynamicLink.
    cDynamicLink(cDynamicMaterial *a_dynamicMaterial);

    //! Destructor of cDynamicLink.
    virtual ~cDynamicLink();

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (DYNAMICS)
    //-----------------------------------------------------------------------

public:
    //! Get number of joints.
    int getNumJoints() { return ((int)(m_dynamicJoints.size())); }

    //! Get pointer to joint by passing index number.
    cDynamicJoint *getJoint(const unsigned int a_index);

    //! Create a new joint.
    cDynamicJoint *newJoint(int a_jointType, int a_axis = 0);

    //! Remove a joint.
    bool removeJoint(cDynamicJoint *a_joint);

    //! Clear all joints.
    void clearAllJoints();

    //! Connect a child link to current one.
    void linkChild(cDynamicLink *a_childLink,
                   const chai3d::cVector3d &a_homePos,
                   const chai3d::cMatrix3d &a_homeRot);

    //! Disconnect a child link from current one.
    void unlinkChild(cDynamicLink *a_childLink);

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (GEOMETRY)
    //-----------------------------------------------------------------------

public:
    //! Create a collision model by passing the desired collision model type
    void buildCollisionModel(int a_collisionModel, double a_radius = 0.0, double a_error = 0.001);

    //! Create a collision model using a bounding box.
    void buildCollisionBox(double a_radius = 0.0, double a_error = 0.001);

    //! Create a collision model using a convex hull.
    void buildCollisionHull(double a_radius = 0.0, double a_error = 0.001);

    //! Create a compound collision model using multiple convex hulls from a multimesh.
    void buildCompoundCollisionHulls(double a_radius = 0.0, double a_error = 0.001);

    //! Create a collision model using triangles.
    void buildCollisionTriangles(double a_radius = 0.0, double a_error = 0.001);

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (HOME POSITION AND ROTATION)
    //-----------------------------------------------------------------------

public:
    //! Set the position of object.
    chai3d::cVector3d getHomePos() { return (m_homePos); }

    //! Set the orientation of object.
    chai3d::cMatrix3d getHomeRot() { return (m_homeRot); }

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (GRAPHICS)
    //-----------------------------------------------------------------------

public:
    //! Set a chai3D body image of object.
    void setImageModel(chai3d::cGenericObject *a_imageModel);

    //! Get body image.
    chai3d::cGenericObject *getImageModel() { return (m_imageModel); }

    //! Render object in OpenGL.
    virtual void render(chai3d::cRenderOptions &a_options);

    //! Update global position frames.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (COLLISION)
    //-----------------------------------------------------------------------

public:
    //! Set a chai3D body image of object.
    void setCollisionModel(chai3d::cMultiMesh *a_collisionModel);

    //! Get collision model.
    chai3d::cGenericObject *getCollisionModel() { return (m_collisionModel); }

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS: (DYNAMICS)
    //-----------------------------------------------------------------------

public:
    //! Dynamic model.
    cDynObject *m_dynObject;

    //! Get mass of link
    inline double getMass() { return (m_mass); }

    //! Get mass of link
    inline chai3d::cVector3d getCenterOfMass() { return (m_centerOfMass); }

    //! Get mass of link
    inline chai3d::cMatrix3d getInertia() { return (m_inertia); }

    //! Set mass properties.
    void setMassProperties(double a_mass,
                           const chai3d::cMatrix3d &a_inertia,
                           const chai3d::cVector3d &a_centerOfMass);

    //! Set mass properties.
    void setMassProperties(double a_mass,
                           const chai3d::cVector3d &a_inertiaPrincipal,
                           const chai3d::cVector3d &a_centerOfMass);

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS: (MATERIAL)
    //-----------------------------------------------------------------------

public:
    //! Get dynamic material properties
    cDynamicMaterial *getDynamicMaterial() { return (m_dynamicMaterial); }

    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS: (COLLISION)
    //-----------------------------------------------------------------------

protected:
    //! Collision mesh model.
    chai3d::cMultiMesh *m_collisionModel;

    //! Material properties.
    cDynamicMaterial *m_dynamicMaterial;

    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS: (GRAPHICS)
    //-----------------------------------------------------------------------

protected:
    //! graphical model of link.
    chai3d::cGenericObject *m_imageModel;

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS: (GENERAL)
    //-----------------------------------------------------------------------

public:
    //! Parent base.
    cDynamicBase *m_dynamicParentBase;

    //! List of joints connecting the current link to its parent link.
    std::vector<cDynamicJoint *> m_dynamicJoints;

    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:
    //! Compute collision with object geometry.
    virtual bool computeOtherCollisionDetection(chai3d::cVector3d &a_segmentPointA,
                                                chai3d::cVector3d &a_segmentPointB,
                                                chai3d::cCollisionRecorder &a_recorder,
                                                chai3d::cCollisionSettings &a_settings);

    //-----------------------------------------------------------------------
    // MEMBERS: (HOME CONFIGURATION)
    //-----------------------------------------------------------------------

public:
    //! Home position.
    chai3d::cVector3d m_homePos;

    //! Home rotation.
    chai3d::cMatrix3d m_homeRot;

    //! Mass of link
    double m_mass;

    //! Inertia of link
    chai3d::cMatrix3d m_inertia;

    //! Center of mass of link
    chai3d::cVector3d m_centerOfMass;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

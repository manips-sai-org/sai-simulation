//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1247 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynamicBaseH
#define CDynamicBaseH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "chai3d/CDynamicLink.h"
#include "chai3d/CDynamicContact.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDynamicBase.h

    \brief
    <b> Dynamics Module </b> \n
    Dynamic Base Object.
*/
//===========================================================================

//---------------------------------------------------------------------------
class cDynamicWorld;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \class      cDynamicBase
    \ingroup    Dynamics

    \brief
    cDynamicBase is a base class for modeling a robot.
*/
//===========================================================================
class cDynamicBase : public chai3d::cGenericObject
{
    friend class cDynamicWorld;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:
    //! Constructor of cDynamicBase.
    cDynamicBase(cDynamicWorld *a_world) { initialize(a_world); }

    //! Destructor of cDynBase.
    virtual ~cDynamicBase() {};

    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:
    //! Enable or disable an object in the dynamic world.
    void enableDynamics(bool a_enabled);

    //! Create a new link for this base.
    cDynamicLink *newLink(cDynamicMaterial *a_dynamicMaterial);

    //! Connect a child link to current one.
    void linkChild(cDynamicLink *a_childLink,
                   const chai3d::cVector3d &a_homePos,
                   const chai3d::cMatrix3d &a_homeRot);

    //! Disconnect a child link from current one.
    void unlinkChild(cDynamicLink *a_childLink);

    //! Show or hide contact normal forces.
    void setShowContactNormalForces(bool a_enabled) { m_dynamicContacts->m_show = a_enabled; }

    //! Get display status of contact normal forces.
    bool getShowContactNormalForces() { return (m_dynamicContacts->m_show); }

    //! Get pointer to link by passing string name.
    cDynamicLink *getLink(std::string a_name);

    //! Get index of link by passing string name.
    int getLinkIndex(std::string a_name);

    //! Get pointer to joint by passing string name.
    cDynamicJoint *getJoint(std::string a_name);

    void removeJoint(std::string a_name);

    //! Get index of joint by passing string name.
    int getJointIndex(std::string a_name);

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:
    //! Parent dynamic world.
    cDynamicWorld *m_dynamicWorld;

    //! Child links.
    std::vector<cDynamicLink *> m_dynamicLinks;

    //! Child joints.
    std::vector<cDynamicJoint *> m_dynamicJoints;

    //! List of contact points.
    cDynamicContactList *m_dynamicContacts;

    //! Dynamic model.
    cDynBaseNode *m_dynBaseNode;

    //! Base object.
    cDynObject *m_dynBaseObject;

    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:
    //! Update global positions
    void updateGlobalPositions(const bool a_frameOnly);

    //! copy frame from dynamics model to graphical model.
    void copyDynamicFrameToGraphicFrame(cDynTime a_time);

    //! Render object in OpenGL.
    virtual void render(chai3d::cRenderOptions &a_options);

    //! Initialize object
    void initialize(cDynamicWorld *a_world);

    //! Compute collision with object geometry.
    virtual bool computeOtherCollisionDetection(chai3d::cVector3d &a_segmentPointA,
                                                chai3d::cVector3d &a_segmentPointB,
                                                chai3d::cCollisionRecorder &a_recorder,
                                                chai3d::cCollisionSettings &a_settings);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1034 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d/CDynamicBase.h"
//---------------------------------------------------------------------------
#include "chai3d/CDynamicWorld.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Initialize Robot

    \param    a_world  World to which this new object belongs to.
*/
//===========================================================================
void cDynamicBase::initialize(cDynamicWorld *a_world)
{
    // store parent world
    m_dynamicWorld = a_world;

    // add body to world
    m_dynamicWorld->m_dynamicObjects.push_back(this);

    // create base object.
    m_dynBaseObject = new cDynObject();

    // create a contact point recorder.
    m_dynamicContacts = new cDynamicContactList();
}

//===========================================================================
/*!
    Create a new link for this base.

    \param    a_dynMaterial  Material property to be applied to link.
    \return   Return pointer to new link.
*/
//===========================================================================
cDynamicLink *cDynamicBase::newLink(cDynamicMaterial *a_dynamicMaterial)
{
    // create new link
    cDynamicLink *newLink = new cDynamicLink(a_dynamicMaterial);
    newLink->m_dynamicParentBase = this;

    // add child to the list of child links
    m_dynamicLinks.push_back(newLink);

    // return new created link
    return (newLink);
}

//===========================================================================
/*!
    Enable or disable an object in the dynamic world

    \param    a_enabled  if true, dynamics is enabled, otherwise not.
*/
//===========================================================================
void cDynamicBase::enableDynamics(bool a_enabled)
{
    if (a_enabled)
    {
        m_dynBaseNode->status(CDYN_ACTIVE);
    }
    else
    {
        m_dynBaseNode->status(CDYN_INACTIVE);
    }
}

//===========================================================================
/*!
    Connect a child link to current one.

    \param    a_childLink  Child link to be attached to current one.
*/
//===========================================================================
void cDynamicBase::linkChild(cDynamicLink *a_childLink,
                             const cVector3d &a_homePos,
                             const cMatrix3d &a_homeRot)
{
    // store home values
    a_childLink->m_homePos = a_homePos;
    a_childLink->m_homeRot = a_homeRot;

    // place frame
    m_dynBaseObject->frame.push();
    m_dynBaseObject->frame.translate(a_homePos.x(),
                                     a_homePos.y(),
                                     a_homePos.z());

    cDynVector3 axis;
    double angle = 0.0;
    cVector3d caxis(0, 0, 0);
    a_homeRot.toAxisAngle(caxis, angle);
    axis.set(caxis.x(),
             caxis.y(),
             caxis.z());
    m_dynBaseObject->frame.rotate(axis, angle);

    // link dynamic object to child dynamic object
    m_dynBaseObject->link(a_childLink->m_dynObject);
    m_dynBaseObject->frame.pop();
}

//===========================================================================
/*!
    Disconnect a child link from current one.

    \param    a_childLink  Child link to be removed.
*/
//===========================================================================
void cDynamicBase::unlinkChild(cDynamicLink *a_childLink)
{
    // link dynamic object of child to current dynamic object
    m_dynBaseObject->unlink(a_childLink->m_dynObject);
}

//===========================================================================
/*!
    Get pointer to link by passing string name.

    \param  a_name  Name of link.

    \return Return pointer to link if found, __NULL__ otherwise.
*/
//===========================================================================
cDynamicLink *cDynamicBase::getLink(std::string a_name)
{
    vector<cDynamicLink *>::iterator i;
    for (i = m_dynamicLinks.begin(); i != m_dynamicLinks.end(); ++i)
    {
        cDynamicLink *nextLink = *i;
        if (nextLink->m_name == a_name)
        {
            return (nextLink);
        }
    }

    return (NULL);
}

//===========================================================================
/*!
    Get link index number by passing string name.

    \param  a_name  Name of link.

    \return Return index number of link if found, -1 otherwise.
*/
//===========================================================================
int cDynamicBase::getLinkIndex(std::string a_name)
{
    int count = 0;
    vector<cDynamicLink *>::iterator i;
    for (i = m_dynamicLinks.begin(); i != m_dynamicLinks.end(); ++i)
    {
        cDynamicLink *nextLink = *i;
        if (nextLink->m_name == a_name)
        {
            return (count);
        }

        count++;
    }

    return (-1);
}

//===========================================================================
/*!
    Get pointer to joint by passing string name.

    \param  a_name  Name of joint.

    \return Return pointer to joint if found, __NULL__ otherwise.
*/
//===========================================================================
cDynamicJoint *cDynamicBase::getJoint(std::string a_name)
{
    vector<cDynamicJoint *>::iterator i;
    for (i = m_dynamicJoints.begin(); i != m_dynamicJoints.end(); ++i)
    {
        cDynamicJoint *nextJoint = *i;
        if (nextJoint->m_name == a_name)
        {
            return (nextJoint);
        }
    }

    return (NULL);
}

void cDynamicBase::removeJoint(std::string a_name)
{
    vector<cDynamicJoint *>::iterator i;
    for (i = m_dynamicJoints.begin(); i != m_dynamicJoints.end(); ++i)
    {
        cDynamicJoint *nextJoint = *i;
        if (nextJoint->m_name == a_name)
        {
            m_dynamicJoints.erase(i);
            break;
        }
    }
}

//===========================================================================
/*!
    Get joint index number by passing string name.

    \param  a_name  Name of joint.

    \return Return index number of joint if found, -1 otherwise.
*/
//===========================================================================
int cDynamicBase::getJointIndex(std::string a_name)
{
    int count = 0;
    vector<cDynamicJoint *>::iterator i;
    for (i = m_dynamicJoints.begin(); i != m_dynamicJoints.end(); ++i)
    {
        cDynamicJoint *nextJoint = *i;
        if (nextJoint->m_name == a_name)
        {
            return (count);
        }

        count++;
    }

    return (-1);
}

//===========================================================================
/*!
    Compute collision detection between a ray and body image.

    \param    a_segmentPointA  Start point of segment.
    \param    a_segmentPointB  End point of segment.
    \param    a_recorder  Stores all collision events.
    \param    a_settings  Contains collision settings information.
    \return   Return \b true if a collision has occurred.
*/
//===========================================================================
bool cDynamicBase::computeOtherCollisionDetection(cVector3d &a_segmentPointA,
                                                  cVector3d &a_segmentPointB,
                                                  cCollisionRecorder &a_recorder,
                                                  cCollisionSettings &a_settings)
{
    bool hit = false;
    vector<cDynamicLink *>::iterator i;
    for (i = m_dynamicLinks.begin(); i != m_dynamicLinks.end(); ++i)
    {
        cDynamicLink *nextItem = *i;
        bool collide = nextItem->computeCollisionDetection(a_segmentPointA,
                                                           a_segmentPointB,
                                                           a_recorder,
                                                           a_settings);

        hit = hit || collide;
    }
    return (hit);
}

//===========================================================================
/*!
    Compute globalPos and globalRot given the localPos and localRot
    of this object and its parents.  Optionally propagates to children.

    If \e a_frameOnly is set to \b false, additional global positions such as
    vertex positions are computed (which can be time-consuming).

     \param    a_frameOnly  If \b true then only the global frame is computed.
*/
//===========================================================================
void cDynamicBase::updateGlobalPositions(const bool a_frameOnly)
{
    vector<cDynamicLink *>::iterator i;
    for (i = m_dynamicLinks.begin(); i != m_dynamicLinks.end(); ++i)
    {
        cDynamicLink *nextItem = *i;
        nextItem->computeGlobalPositions(a_frameOnly,
                                         m_globalPos,
                                         m_globalRot);
    }
};

//===========================================================================
/*!
    Render this deformable mesh in OpenGL.

    \param    a_options  Rendering options.
*/
//===========================================================================
void cDynamicBase::render(cRenderOptions &a_options)
{
    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////

    // render all dynamic link
    vector<cDynamicLink *>::iterator i;
    for (i = m_dynamicLinks.begin(); i != m_dynamicLinks.end(); ++i)
    {
        cDynamicLink *nextItem = *i;
        nextItem->renderSceneGraph(a_options);
    }
}

//===========================================================================
/*!
    Update position and orientation from dynamic model to graphic.
*/
//===========================================================================
void cDynamicBase::copyDynamicFrameToGraphicFrame(cDynTime a_time)
{
    vector<cDynamicLink *>::iterator i;

    for (i = m_dynamicLinks.begin(); i != m_dynamicLinks.end(); ++i)
    {
        cDynamicLink *nextItem = *i;
        cDynFrame f = nextItem->m_dynObject->globalFrame(a_time);

        nextItem->setLocalPos(f.translation()[0], f.translation()[1], f.translation()[2]);
        cQuaternion q(f.rotation()[3],
                      f.rotation()[0],
                      f.rotation()[1],
                      f.rotation()[2]);

        cMatrix3d rot;
        q.toRotMat(rot);

        /*
        cVector3d v0, v1, v2;
        v0 = rot.getCol0();
        v1 = rot.getCol1();
        v0.normalize();
        v0.crossr(v1, v2);
        v2.normalize();
        v2.crossr(v0, v1);
        v2.normalize();
        rot.setCol(v0, v1, v2);
        */
        nextItem->setLocalRot(rot);
    }
}

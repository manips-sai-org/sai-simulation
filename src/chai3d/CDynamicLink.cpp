//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1304 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d/CDynamicLink.h"
#include "chai3d/CDynamicBase.h"
#include "chai3d/CDynamicJoint.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDynamicLink.
*/
//===========================================================================
cDynamicLink::cDynamicLink(cDynamicMaterial *a_dynamicMaterial)
{
    // no body geometry defined yet
    m_imageModel = new cGenericObject();

    // collision mesh model
    m_collisionModel = new cMultiMesh();

    // material
    if (a_dynamicMaterial == NULL)
    {
        m_dynamicMaterial = new cDynamicMaterial();
    }
    else
    {
        m_dynamicMaterial = a_dynamicMaterial;
    }

    // init values
    m_homePos.zero();
    m_homeRot.identity();

    // clear
    m_dynamicJoints.clear();

    // dynamic model
    m_dynObject = new cDynObject();
    m_dynObject->data(this);
}

//===========================================================================
/*!
    Destructor of cDynLink.
*/
//===========================================================================
cDynamicLink::~cDynamicLink()
{
}

//===========================================================================
/*!
    Get pointer to joint by passing index number.

    \param  a_index  Index number of joint.
*/
//===========================================================================
cDynamicJoint *cDynamicLink::getJoint(const unsigned int a_index)
{
    if (a_index < m_dynamicJoints.size())
    {
        return (m_dynamicJoints[a_index]);
    }
    else
    {
        return (NULL);
    }
}

//===========================================================================
/*!
    Create a new joint.

    \param  a_jointType  Values are: DYN_JOINT_PRISMATIC, DYN_JOINT_REVOLUTE, DYN_JOINT_SPHERICAL.
    \param  a_jointAxis  Values are: DYN_AXIS_X, DYN_AXIS_Y, DYN_AXIS_Z.

    \return Return pointer to joint.
*/
//===========================================================================
cDynamicJoint *cDynamicLink::newJoint(int a_jointType, int a_jointAxis)
{
    cDynamicJoint *newJoint = new cDynamicJoint(this, a_jointType, a_jointAxis);
    m_dynamicJoints.push_back(newJoint);
    m_dynamicParentBase->m_dynamicJoints.push_back(newJoint);
    return (newJoint);
}

//===========================================================================
/*!
    Remove a joint.

    \param    a_joint  Joint to remove from list.
*/
//===========================================================================
bool cDynamicLink::removeJoint(cDynamicJoint *a_joint)
{
    // remove joint from list
    vector<cDynamicJoint *>::iterator it;
    for (it = m_dynamicJoints.begin(); it < m_dynamicJoints.end(); it++)
    {
        if (*it == a_joint)
        {
            m_dynamicJoints.erase(it);
        }
    }

    // TODO: remove joint from baseNode joint list.

    return (false);
}

//===========================================================================
/*!
    Clear all joints.

    \fn       void cDynamicLink::clearAllJoints()
*/
//===========================================================================
void cDynamicLink::clearAllJoints()
{
    int num = m_dynamicJoints.size();
    for (int i = 0; i < num; i++)
    {
        cDynamicJoint *nextJoint = m_dynamicJoints[i];
        std::string joint_name = nextJoint->m_name;
        m_dynamicParentBase->removeJoint(joint_name);
        delete nextJoint;
    }
    m_dynamicJoints.clear();

    // TODO: remove joint from baseNode joint list.
    // Done!
}

//===========================================================================
/*!
    Connect a child link to current one.

    \param    a_childLink  Child link to be attached to current one.
*/
//===========================================================================
void cDynamicLink::linkChild(cDynamicLink *a_childLink, const cVector3d &a_homePos, const cMatrix3d &a_homeRot)
{
    // store home values
    a_childLink->m_homePos = a_homePos;
    a_childLink->m_homeRot = a_homeRot;

    // place frame
    m_dynObject->frame.push();
    m_dynObject->frame.translate(a_homePos.x(), a_homePos.y(), a_homePos.z());

    cDynVector3 axis;
    double angle = 0.0;
    cVector3d caxis(0, 0, 0);
    a_homeRot.toAxisAngle(caxis, angle);
    axis.set(caxis.x(),
             caxis.y(),
             caxis.z());
    m_dynObject->frame.rotate(axis, angle);

    // link dynamic object to child dynamic object
    m_dynObject->link(a_childLink->m_dynObject);
    m_dynObject->frame.pop();
}

//===========================================================================
/*!
    Disconnect a child link from current one.

    \param    a_childLink  Child link to be removed.
*/
//===========================================================================
void cDynamicLink::unlinkChild(cDynamicLink *a_childLink)
{
    // link dynamic object of child to current dynamic object
    m_dynObject->unlink(a_childLink->m_dynObject);

    // TODO: remove link from baseNode link list.
    // TODO: remove joints from baseNode joint list.
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
bool cDynamicLink::computeOtherCollisionDetection(cVector3d &a_segmentPointA,
                                                  cVector3d &a_segmentPointB,
                                                  cCollisionRecorder &a_recorder,
                                                  cCollisionSettings &a_settings)
{
    bool hit = false;

    if (m_imageModel != NULL)
    {
        hit = m_imageModel->computeCollisionDetection(a_segmentPointA,
                                                      a_segmentPointB,
                                                      a_recorder,
                                                      a_settings);
    }

    return (hit);
}

//===========================================================================
/*!
    Define a generic object such as a mesh or a shape which is used
    to model the geometry of this current ODE object.

    \param    a_imageModel  CHAI3D graphical image model.
*/
//===========================================================================
void cDynamicLink::setImageModel(cGenericObject *a_imageModel)
{
    // check object
    if (a_imageModel == NULL)
    {
        return;
    }

    // store pointer to body
    m_imageModel = a_imageModel;

    // set external parent of body image to current ODE object.
    // rule applies to children which, in the case a mesh, belong
    // to the same object.
    m_imageModel->setOwner(this);

    // set parent
    m_imageModel->setParent(this);
}

//===========================================================================
/*!
    Define a multimesh object for dynamic collision detection.

    \param    a_collisionModel  CHAI3D multimesh model.
*/
//===========================================================================
void cDynamicLink::setCollisionModel(chai3d::cMultiMesh *a_collisionModel)
{
    // store pointer to body
    m_collisionModel = a_collisionModel;

    // set external parent of body image to current ODE object.
    // rule applies to children which, in the case a mesh, belong
    // to the same object.
    m_collisionModel->setOwner(this);

    // set parent
    m_collisionModel->setParent(this);
}

//===========================================================================
/*!
    Render this deformable mesh in OpenGL.

    \param    a_options  Rendering options.
*/
//===========================================================================
void cDynamicLink::render(cRenderOptions &a_options)
{
    if (m_imageModel != NULL)
    {
        m_imageModel->renderSceneGraph(a_options);
    }

    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
    }
}

//===========================================================================
/*!
    Compute globalPos and globalRot given the localPos and localRot
    of this object and its parents.  Optionally propagates to children.

    If \e a_frameOnly is set to \b false, additional global positions such as
    vertex positions are computed (which can be time-consuming).

    \param    a_frameOnly  If \b true then only the global frame is computed
*/
//===========================================================================
void cDynamicLink::updateGlobalPositions(const bool a_frameOnly)
{
    if (m_imageModel != NULL)
    {
        m_imageModel->computeGlobalPositions(a_frameOnly,
                                             m_globalPos,
                                             m_globalRot);
    }
}

//===========================================================================
/*!
    Create a dynamic model for the object. Use principal inertia co-efficients only.
*/
//===========================================================================
void cDynamicLink::setMassProperties(double a_mass, const cVector3d &a_inertiaPrincipal, const cVector3d &a_centerOfMass)
{
    cMatrix3d inertia;
    inertia.set(a_inertiaPrincipal.x(), 0.0, 0.0,
                0.0, a_inertiaPrincipal.y(), 0.0,
                0.0, 0.0, a_inertiaPrincipal.z());
    setMassProperties(a_mass, inertia, a_centerOfMass);
}

//===========================================================================
/*!
    Create a dynamic model for the object. Use full inertia tensor.
*/
//===========================================================================
void cDynamicLink::setMassProperties(double a_mass, const cMatrix3d &a_inertia, const cVector3d &a_centerOfMass)
{
    m_mass = cAbs(a_mass);
    m_inertia.set(a_inertia(0, 0), a_inertia(0, 1), a_inertia(0, 2),
                  a_inertia(1, 0), a_inertia(1, 1), a_inertia(1, 2),
                  a_inertia(2, 0), a_inertia(2, 1), a_inertia(2, 2));
    m_centerOfMass = a_centerOfMass;

    cDynMatrix3 dyn_inertia;
    dyn_inertia.set(a_inertia(0, 0), a_inertia(0, 1), a_inertia(0, 2),
                    a_inertia(1, 0), a_inertia(1, 1), a_inertia(1, 2),
                    a_inertia(2, 0), a_inertia(2, 1), a_inertia(2, 2));

    m_dynObject->frame.push();
    m_dynObject->frame.translate(m_centerOfMass.x(), m_centerOfMass.y(), m_centerOfMass.z());
    m_dynObject->dynamics.mass(m_mass);
    m_dynObject->dynamics.inertia(dyn_inertia);
    m_dynObject->frame.pop();
}

//===========================================================================
/*!
    Build a collision model from the polygons described in m_collisionModel.

    \param  a_collisionModel  Model type (Bounding Box, Hull, Polygonal)
    \param  a_radius  Radius around object.
    \param  a_error  Tolerance.
*/
//===========================================================================
void cDynamicLink::buildCollisionModel(int a_collisionModel, double a_radius, double a_error)
{
    switch (a_collisionModel)
    {
    case DYN_COLLISION_BBOX:
        buildCollisionBox(a_radius, a_error);
        break;
    case DYN_COLLISION_HULL:
        buildCollisionHull(a_radius, a_error);
        break;
    case DYN_COLLISION_POLY:
        buildCollisionTriangles(a_radius, a_error);
        break;
    }
}

//===========================================================================
/*!
    Create a collision model using a bounding box.

    \param  a_radius  Radius around object.
    \param  a_error  Tolerance.
*/
//===========================================================================
void cDynamicLink::buildCollisionBox(double a_radius, double a_error)
{
    m_collisionModel->computeBoundaryBox(true);

    // create primitive
    cDynPrimitive *p = m_dynObject->geometry.begin(CDYN_HULL);

    // set parameters
    p->radius(a_radius);
    p->error(a_error);
    p->material(m_dynamicMaterial->m_dynMaterial);

    cVector3d pos = m_collisionModel->getLocalPos();
    cMatrix3d rot = m_collisionModel->getLocalRot();

    chai3d::cVector3d min = pos + rot * m_collisionModel->getBoundaryMin();
    chai3d::cVector3d max = pos + rot * m_collisionModel->getBoundaryMax();

    // define volume from bounding box
    p->vertex(min.x(), min.y(), min.z());
    p->vertex(max.x(), min.y(), min.z());
    p->vertex(max.x(), max.y(), min.z());
    p->vertex(min.x(), max.y(), min.z());

    p->vertex(min.x(), min.y(), max.z());
    p->vertex(max.x(), min.y(), max.z());
    p->vertex(max.x(), max.y(), max.z());
    p->vertex(min.x(), max.y(), max.z());

    // end primitive
    p->end();

    // end geometry
    m_dynObject->geometry.end();
}

//===========================================================================
/*!
    Create a collision model using a convex hull.

    \param  a_radius  Radius around object.
    \param  a_error  Tolerance.
*/
//===========================================================================
void cDynamicLink::buildCollisionHull(double a_radius, double a_error)
{
    // create new primitive
    cDynPrimitive *p = m_dynObject->geometry.begin(CDYN_HULL);

    // set parameters
    p->radius(a_radius);
    p->error(a_error);
    p->material(m_dynamicMaterial->m_dynMaterial);

    cVector3d pos = m_collisionModel->getLocalPos();
    cMatrix3d rot = m_collisionModel->getLocalRot();

    int numVertices = m_collisionModel->getNumVertices();

    for (int i = 0; i < numVertices; i++)
    {
        cVector3d posVertex = pos + rot * m_collisionModel->getVertexPos(i);
        p->vertex(posVertex.x(), posVertex.y(), posVertex.z());
    }

    // end primitive
    p->end();

    // end geometry
    m_dynObject->geometry.end();
}

///===========================================================================
/*!
    Create a compound collision model using individual convex hulls for
    each sub-mesh inside the collision model.

    \param  a_radius  Radius around object.
    \param  a_error  Tolerance.
*/
//===========================================================================
void cDynamicLink::buildCompoundCollisionHulls(double a_radius, double a_error)
{
    // Ensure we have a valid multimesh
    if (!m_collisionModel)
        return;

    int numMeshes = m_collisionModel->getNumMeshes();

    // If it's empty or only has 1 mesh, fallback to standard behavior
    if (numMeshes <= 1)
    {
        buildCollisionHull(a_radius, a_error);
        return;
    }

    cVector3d basePos = m_collisionModel->getLocalPos();
    cMatrix3d baseRot = m_collisionModel->getLocalRot();

    // Iterate over each sub-mesh and create a separate hull primitive
    for (int m = 0; m < numMeshes; m++)
    {
        chai3d::cMesh *subMesh = m_collisionModel->getMesh(m);

        // Start a new primitive for this specific hull
        cDynPrimitive *p = m_dynObject->geometry.begin(CDYN_HULL);

        // set parameters
        p->radius(a_radius);
        p->error(a_error);
        p->material(m_dynamicMaterial->m_dynMaterial);

        // Account for any local offsets the sub-mesh might have relative to the multi-mesh
        cVector3d localPos = basePos + baseRot * subMesh->getLocalPos();
        cMatrix3d localRot = baseRot * subMesh->getLocalRot();

        int numVertices = subMesh->getNumVertices();

        for (int i = 0; i < numVertices; i++)
        {
            // FIXED: Access vertex position through the m_vertices array
            cVector3d posVertex = localPos + localRot * subMesh->m_vertices->getLocalPos(i);
            p->vertex(posVertex.x(), posVertex.y(), posVertex.z());
        }

        // end this specific hull primitive
        p->end();
    }

    // end the overall geometry block containing all hulls
    m_dynObject->geometry.end();
}

//===========================================================================
/*!
    Create a collision model using triangles.

    \param  a_radius  Radius around object.
    \param  a_error  Tolerance.
*/
//===========================================================================
void cDynamicLink::buildCollisionTriangles(double a_radius, double a_error)
{
    cDynPrimitive *p;
    double tolerance = a_error * a_error;
    double flag = false;

    cVector3d pos = m_collisionModel->getLocalPos();
    cMatrix3d rot = m_collisionModel->getLocalRot();

    int numTriangles = m_collisionModel->getNumTriangles();
    for (int i = 0; i < numTriangles; i++)
    {
        cMesh *mesh;
        unsigned int index;

        m_collisionModel->getTriangle(i, mesh, index);

        if (mesh->m_triangles->getAllocated(index))
        {
            chai3d::cVector3d pos0 = pos + rot * mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex0(index));
            chai3d::cVector3d pos1 = pos + rot * mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex1(index));
            chai3d::cVector3d pos2 = pos + rot * mesh->m_vertices->getLocalPos(mesh->m_triangles->getVertexIndex2(index));

            if (chai3d::cTriangleArea(pos0, pos1, pos2) > tolerance)
            {
                if (!flag)
                {
                    // create new primitive
                    p = m_dynObject->geometry.begin(CDYN_TRIANGLES);
                    p->radius(a_radius);
                    p->error(a_error);
                    p->material(m_dynamicMaterial->m_dynMaterial);
                }

                // set parameters
                p->vertex(pos0.x(), pos0.y(), pos0.z());
                p->vertex(pos1.x(), pos1.y(), pos1.z());
                p->vertex(pos2.x(), pos2.y(), pos2.z());

                // end primitive
                flag = true;
            }
        }
    }

    // end geometry
    if (flag)
    {
        p->end();
        m_dynObject->geometry.end();
    }
}

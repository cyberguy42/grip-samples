/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "VisualizationTab.h"

// **********************
// STL
#include <wx/wx.h>
#include <iostream>

using namespace std;

// **********************
// GRIP UI stuff
#include <Tabs/AllTabs.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <GRIPApp.h>

// **********************
// Dynamics Stuff
#include <collision/CollisionShapes.h>
#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

// **********************
// Drawing Stuff
#include <wx/glcanvas.h>
#include <GUI/Viewer.h>

// **********************
// To draw CoM
#include <GL/glu.h>
#include <GL/glut.h>
// ***********************


/** UI Control IDs */
enum DynamicSimulationTabEvents {
    id_checkbox_showcontacts = wxID_HIGHEST,
    id_checkbox_showcollmesh,
    id_button_showCoM,
    id_offsetX_Slider,
    id_offsetY_Slider,
    id_offsetZ_Slider
};

/** Handlers for events **/
BEGIN_EVENT_TABLE(VisualizationTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, VisualizationTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, VisualizationTab::OnSlider)
EVT_CHECKBOX(id_checkbox_showcontacts, VisualizationTab::OnCheckShowContacts)
EVT_CHECKBOX(id_checkbox_showcollmesh, VisualizationTab::OnCheckShowCollMesh)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(VisualizationTab, GRIPTab)

/**
 * @function VisualizationTab
 * @brief Constructor
 */
VisualizationTab::VisualizationTab(wxWindow *parent,
                         const wxWindowID id,
                         const wxPoint& pos,
                         const wxSize& size,
                         long style) :
GRIPTab(parent, id, pos, size, style) {
    sizerFull = new wxBoxSizer(wxHORIZONTAL);
    wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Display Options"));
    wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);

    checkShowContacts = new wxCheckBox(this, id_checkbox_showcontacts, wxT("Show Contact Forces"));
    checkShowCollMesh = new wxCheckBox(this, id_checkbox_showcollmesh, wxT("Show Collision Mesh"));
    ss1BoxS->Add(checkShowContacts, 0, wxALL, 1);
    ss1BoxS->Add(checkShowCollMesh, 0, wxALL, 1);

    wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("CoM Display Options"));
    wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);

    ss2BoxS->Add(new wxButton(this, id_button_showCoM, wxT("Show CoM")), 0, wxALL, 1); 
    mOffsetX_Slider = new GRIPSlider("offset X",-1.0,1.0,2000,0,1000,2000,this,id_offsetX_Slider);
    ss2BoxS->Add( mOffsetX_Slider, 1, wxEXPAND | wxALL, 6 );
    mOffsetY_Slider = new GRIPSlider("offset Y",-1.0,1.0,2000,0.5,1000,2000,this,id_offsetY_Slider);
    ss2BoxS->Add( mOffsetY_Slider, 1, wxEXPAND | wxALL, 6 );
    mOffsetZ_Slider = new GRIPSlider("offset Z",-1.0,1.0,2000,0,1000,2000,this,id_offsetZ_Slider);
    ss2BoxS->Add( mOffsetZ_Slider, 1, wxEXPAND | wxALL, 6 );

    sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 1);
    sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 1);
    SetSizer(sizerFull);

    // Initialize
    mDrawCoMFlag = false;
}


/**
 * @function OnButton
 * @brief Handles button events
 */
void VisualizationTab::OnButton(wxCommandEvent & _evt) {
    int slnum = _evt.GetId();
  
    switch( slnum ) {
    case id_button_showCoM: {
      printf("Show CoM \n");
      mDrawCoMFlag = !mDrawCoMFlag;
      if( mDrawCoMFlag ) { printf("CoM drawing ON! \n"); }
      else { printf("CoM drawing OFF! \n"); }
    }
      break;
      
    default: {
      /** Default */
      printf("Default button \n");
      break;
    }

    }
}

void VisualizationTab::OnCheckShowContacts(wxCommandEvent &evt) {
}

void VisualizationTab::OnCheckShowCollMesh(wxCommandEvent &evt) {
}

void VisualizationTab::GRIPEventSceneLoaded() {
    int mGroundIndex = -1;
    for(int i = 0; i < mWorld->getNumObjects(); ++i) {
        if(mWorld->getObject(i)->getName() == "ground") {
            mGroundIndex = i; break;
        }
    }
    if (mGroundIndex != -1) {
        std::cout << "-- Found ground as object %d \n" << std::endl;
    }
    else {
        robotics::Object* ground = new robotics::Object();
        ground->setName("ground");
        ground->addDefaultRootNode();
        dynamics::BodyNodeDynamics* node = new dynamics::BodyNodeDynamics();
        node->setShape(new kinematics::ShapeBox(Eigen::Vector3d(10.0, 10.0, 0.0001), 1.0));
        kinematics::Joint* joint = new kinematics::Joint(ground->getRoot(), node);
        ground->addNode(node);
        ground->initSkel();
        ground->update();
        ground->setImmobileState(true);

        mWorld->addObject(ground);
        mWorld->rebuildCollision();
        mGroundIndex = mWorld->getNumObjects() - 1;
        printf("-- Added ground as object %d\n", mGroundIndex);
    }
    treeView->CreateFromWorld();
}

/**
 * @function GRIPEventSimulationBeforeTimeStep
 * @brief 
 */
void VisualizationTab::GRIPEventSimulationBeforeTimestep() {
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 * @brief
 */
void VisualizationTab::GRIPEventSimulationAfterTimestep() {
}

/**
 * @function GRIPEventSimulationStart
 * @brief
 */
void VisualizationTab::GRIPEventSimulationStart() {
}

/**
 * @function GRIPEventRender
 * @brief
 */
void VisualizationTab::GRIPEventRender() {
    glDisable(GL_FOG);
    glEnable(GL_COLOR_MATERIAL);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    glLineWidth(1.5f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);


    // Draw CoM
    if( mDrawCoMFlag ) {
      glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
      glEnable( GL_COLOR_MATERIAL );
      glColor3f( 1.0f, 0.0f, 0.0f );
      
      // Create a quadric object to store the sphere
      GLUquadricObj *quadric = gluNewQuadric();

      // Get the CoMs
      double offsetY = 0.8;
      double radius = 0.01;
      Eigen::Vector3d cmPos;

      for( int i = 0; i < mWorld->getRobot(0)->getNumNodes(); ++i ) {
	glPushMatrix();
	cmPos = mWorld->getRobot(0)->getNode(i)->getWorldCOM();
	glTranslatef( cmPos(0) +  mCoM_drawOffsetX, 
		      cmPos(1) + mCoM_drawOffsetY, 
		      cmPos(2) + mCoM_drawOffsetZ );
	// Draw it
	gluSphere( quadric, radius, 5, 5 );
	glPopMatrix();
      }
      
      // The CoM of the whole body
      glColor3f( 0.0f, 0.0f, 1.0f );
      cmPos = mWorld->getRobot(0)->getWorldCOM();
      glPushMatrix();
      glTranslatef( cmPos(0) + mCoM_drawOffsetX, 
		    cmPos(1) + mCoM_drawOffsetY, 
		    cmPos(2) + mCoM_drawOffsetZ );
      // Draw it
      gluSphere( quadric, radius, 5, 5 );
      glPopMatrix();
    }

    // draw contact points
    if (checkShowContacts->IsChecked() && mWorld && mWorld->mCollisionHandle) {
        // some preprocessing. calculate vector lengths and find max
        // length, scale down the force measurements, and figure out
        // which contact points involve to the selected body nodes
        int nContacts = mWorld->mCollisionHandle->getCollisionChecker()->getNumContact();
        vector<Eigen::Vector3d> vs(nContacts);
        vector<Eigen::Vector3d> fs(nContacts);
        vector<float> lens(nContacts);
        vector<bool> selected(nContacts);
        float maxl = 0;
        for (int k = 0; k < nContacts; k++) {
            collision_checking::ContactPoint contact = mWorld->mCollisionHandle->getCollisionChecker()->getContact(k);
            vs[k] = contact.point;
            fs[k] = contact.force.normalized() * .1 * log(contact.force.norm());
            lens[k] = (vs[k] - fs[k]).norm();
            if (lens[k] > maxl) maxl = lens[k];
            selected[k] = false;
            if (contact.bd1 == selectedNode || contact.bd2 == selectedNode) {
                selected[k] = true;
            }
        }
        Eigen::Vector3d v;
        Eigen::Vector3d f;
        Eigen::Vector3d vf;
        Eigen::Vector3d arrowheadDir;
        Eigen::Vector3d arrowheadBase;
        glBegin(GL_LINES);
        for (int k = 0; k < nContacts; k++) {
            if (selected[k]) {
                glColor3d(0.0, 1.0, 0.0);
            }
            else {
                glColor3d(lens[k] / (2 * maxl) + .5, 0.0, 0.0);
            }
            v = vs[k];
            f = fs[k];
            vf = v + f;
            arrowheadDir = v.cross(f).normalized() * .0075;
            arrowheadBase = vf - f.normalized() * .02;
            glVertex3f(v[0], v[1], v[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(arrowheadBase[0] + arrowheadDir[0], arrowheadBase[1] + arrowheadDir[1], arrowheadBase[2] + arrowheadDir[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(arrowheadBase[0] - arrowheadDir[0], arrowheadBase[1] - arrowheadDir[1], arrowheadBase[2] - arrowheadDir[2]);
        }
        glEnd();
    }

    // draw collision meshes
    if (checkShowCollMesh->IsChecked() && mWorld && selectedNode && selectedNode->getShape()) {
        renderer::RenderInterface* ri = &viewer->renderer;
        kinematics::BodyNode* cnode = selectedNode;
        kinematics::Shape* shape = selectedNode->getShape();
        const aiScene* sc = shape->getVizMesh();
        if (shape->getCollisionMesh() != NULL) { sc = shape->getCollisionMesh(); }

        if (sc != NULL) {
            int verts = 0;
            const aiNode* nd = sc->mRootNode;

            // put in the proper transform
            glPushMatrix();
            double M[16];
            Eigen::Matrix4d worldTrans = selectedNode->getWorldTransform();
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    M[j*4+i] = worldTrans(i, j);
            glMultMatrixd(M);

            for (unsigned int n = 0; n < nd->mNumMeshes; ++n) {
                const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];
                for (unsigned int t = 0; t < mesh->mNumFaces; ++t) {
                    const struct aiFace* face = &mesh->mFaces[t];
                    glBegin(GL_LINE_STRIP);
                    for(unsigned int i = 0; i < face->mNumIndices; i++) {
                        int index = face->mIndices[i];
                        glColor4d(0.0, 0.0, 1.0, 1.0);
                        if(mesh->mNormals != NULL) 
                            glNormal3fv(&mesh->mNormals[index].x);
                        glVertex3fv(&mesh->mVertices[index].x);
                        verts++;
                    }
                    glEnd();
                }
            }
            glPopMatrix();
        }

        glPopMatrix();
        glEnd();
    }
}

/**
 * @function OnSlider
 * @brief Handles slider changes
 */
void VisualizationTab::OnSlider(wxCommandEvent &evt) {

  int slnum = evt.GetId();
  double pos = *(double*) evt.GetClientData();

  switch( slnum ) {

  case id_offsetX_Slider:
     mCoM_drawOffsetX = pos; 
    break;

  case id_offsetY_Slider:
     mCoM_drawOffsetY = pos; 
    break;

  case id_offsetZ_Slider:
     mCoM_drawOffsetZ = pos; 
    break;

  }

}

// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void VisualizationTab::GRIPStateChange() {
    if(selectedTreeNode==NULL){
        return;
    }

    switch (selectedTreeNode->dType) {
    case Return_Type_Object: {
        robotics::Object* pObject = (robotics::Object*)(selectedTreeNode->data);
        selectedNode = pObject->mRoot;
        break;
    }
    case Return_Type_Robot: {
        robotics::Robot* pRobot = (robotics::Robot*)(selectedTreeNode->data);
        selectedNode = pRobot->mRoot;
        break;
    }
    case Return_Type_Node: {
        dynamics::BodyNodeDynamics* pBodyNode = (dynamics::BodyNodeDynamics*)(selectedTreeNode->data);
        selectedNode = pBodyNode;
        break;
    }
    default: {
        fprintf(stderr, "someone else's problem.");
        assert(0);
        exit(1);
    }
    }
    int type = 0;
    wxCommandEvent evt(wxEVT_GRIP_UPDATE_AND_RENDER,GetId());
    evt.SetEventObject(this);
    evt.SetClientData((void*)&type);
    GetEventHandler()->AddPendingEvent(evt);
}



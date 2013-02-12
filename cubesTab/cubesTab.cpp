/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

/**
 * @file cubesTab.cpp
 * @brief Simulation of cubes falling all over a static floor
 * @author A. Huaman Quispe <ahuaman3@gatech.edu>
 * @date 2013/01/15
 */

#include "cubesTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>
using namespace std;

#include <Tabs/AllTabs.h>
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


/** Events */
enum DynamicSimulationTabEvents {
};

/** Handler for events **/
BEGIN_EVENT_TABLE(cubesTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, cubesTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, cubesTab::OnSlider)
END_EVENT_TABLE()


// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(cubesTab, GRIPTab)

/**
 * @function cubesTab
 * @brief Constructor
 */
cubesTab::cubesTab(wxWindow *parent, const wxWindowID id,
		   const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style) {
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
  // Create Static boxes (outline of your Tab)
  wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Simulation"));
  
  // Create sizers for these static boxes
  wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
  
  // Add the boxes to their respective sizers
  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFull);
   
}

/**
 * @function OnSlider
 * @brief Handles slider changes
 */
void cubesTab::OnSlider(wxCommandEvent &evt) {

}

/**
 * @function OnButton
 * @brief Handles button events
 */
void cubesTab::OnButton(wxCommandEvent & _evt) {
  
  int slnum = _evt.GetId();
  
  switch( slnum ) {
  default: {
    printf("Default button \n");
    }
  }
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 */
void cubesTab::GRIPEventSimulationAfterTimestep() {
  printf("AfterTime step \n");
}

/**
 * @function GRIPStateChange
 * @brief This function is called when an object is selected in the Tree View 
or other global changes to the GRIP world. Use this to capture events 
from outside the tab.
*/
void cubesTab::GRIPStateChange() {
  if(selectedTreeNode==NULL){
    return;
  }

  string statusBuf;
  string buf, buf2;
  switch (selectedTreeNode->dType) {
  case Return_Type_Object:
    statusBuf = " Selected Object: ";
    buf = "You clicked on object: ";
    
    break;
  case Return_Type_Robot:
    statusBuf = " Selected Robot: ";
    buf = "You clicked on robot: ";
    
    break;
  case Return_Type_Node:
    statusBuf = " Selected Link:  of Robot: ";
    buf = " Link:  of Robot: ";
    // Do something here if you want to.  you get the idea...
    
    break;
  default:
    fprintf(stderr, "someone else's problem.");
    assert(0);
    exit(1);
  }
  //frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
  //sizerFull->Layout();
}



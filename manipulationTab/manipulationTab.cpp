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

#include "manipulationTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>

#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/PathFollowingTrajectory.h>
#include "Controller.h"
#include "Grasper.h"


/// Define IDs for buttons
enum DynamicSimulationTabEvents {
    id_button_DoPlanning = 8345,
    id_button_RelocateObjects,
    id_button_SetStart,
    id_button_SetGoal,
    id_button_SetPredefStart,
    id_button_SetPredefGoal,
    id_button_ShowStart,
    id_button_ShowGoal,
    id_button_Grasping,
    id_button_OpenHand,
    id_button_CloseHand,
    id_label_Inst,
    id_checkbox_showcollmesh,
    id_checkbox_UseRRT,
    id_button_next_grasp,
    id_button_FindGrasps,
    id_button_PlanPath,
    id_button_ShowConfig
};
using namespace std;

// Handler for events
BEGIN_EVENT_TABLE(manipulationTab, wxPanel)
EVT_COMMAND(id_button_SetPredefStart, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonSetPredefStart)
EVT_COMMAND(id_button_SetStart, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonSetStart)
EVT_COMMAND(id_button_FindGrasps, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonFindGrasps)
EVT_COMMAND(id_button_ShowStart, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonShowStart)
EVT_COMMAND(id_button_Grasping, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonDoGrasping)
EVT_COMMAND(id_button_OpenHand, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonOpenHand)
EVT_COMMAND(id_button_next_grasp, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonNextGrasp)
EVT_COMMAND(id_button_CloseHand, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonCloseHand)
EVT_COMMAND(id_button_PlanPath, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonPlanPath)
EVT_COMMAND(id_button_ShowConfig, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonShowConfig)
EVT_CHECKBOX(id_checkbox_showcollmesh, manipulationTab::onCheckShowCollMesh)
EVT_CHECKBOX(id_checkbox_UseRRT, manipulationTab::onCheckUseRRT)
END_EVENT_TABLE() 
IMPLEMENT_DYNAMIC_CLASS(manipulationTab, GRIPTab)


manipulationTab::manipulationTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style) {
    wxSizer* sizerFull = new wxBoxSizer(wxHORIZONTAL);

    // Create Static boxes (outline of your Tab)
    wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Setup"));
    wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Manipulation"));
    wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Plan"));
    
    // Create sizers for these static boxes
    wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
    wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
    wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);  
      
    // Start and goal conf button
    ss1BoxS->Add(new wxButton(this, id_button_SetPredefStart, wxT("Set Predef Start")), 0, wxALL, 1);
    ss1BoxS->Add(new wxButton(this, id_button_SetStart, wxT("Set Custom Start")), 0, wxALL, 1);
    ss1BoxS->Add(new wxButton(this, id_button_ShowStart, wxT("Show Start Conf")), 0, wxALL, 1);
    //ss1BoxS->Add(new wxStaticText(this, id_label_Inst, wxT("Instructions:\n[1]Set start conf  [2]Select an object  [3]Click Plan Grasping")), 0, wxEXPAND);
    
    
    ss1BoxS->Add(new wxButton(this, id_button_Grasping, wxT("Execute Grasp")), 0, wxALL, 1);
    // Grasping

    ss2BoxS->Add(new wxButton(this, id_button_next_grasp, wxT("Show next Jacobian grasp")), 0, wxALL, 1);
    ss2BoxS->Add(new wxButton(this, id_button_FindGrasps, wxT("Prepare Grasp Info")), 0, wxALL, 1);
    checkShowCollMesh = new wxCheckBox(this, id_checkbox_showcollmesh, wxT("Show Grasp Target Pose"));
    checkUseRRT = new wxCheckBox(this, id_checkbox_UseRRT, wxT("Use RRT Path"));
    ss3BoxS->Add(checkUseRRT, 0, wxALL, 1);    
    
    ss2BoxS->Add(checkShowCollMesh, 0, wxALL, 1);
    ss2BoxS->Add(new wxButton(this, id_button_CloseHand, wxT("Close Hand")), 0, wxALL, 1);   
    ss2BoxS->Add(new wxButton(this, id_button_OpenHand, wxT("Open Hand")), 0, wxALL, 1);
    ss3BoxS->Add(new wxButton(this, id_button_PlanPath, wxT("Plan RRT Path")), 0, wxALL, 1);
    ss3BoxS->Add(new wxButton(this, id_button_ShowConfig, wxT("Show RRT Config")), 0, wxALL, 1);
    
    // Add the boxes to their respective sizers
    sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
    sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
    sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);
    SetSizer(sizerFull);

    // Additional settings
    mAlreadyReplan = false;
    mPredefStartConf.resize(6);
    mPredefStartConf << -0.858702, -0.674395, 0.0, -0.337896, 0.0, 0.0;
    mController = NULL;
}

/// Setup grasper when scene is loaded as well as populating arm's DoFs
void manipulationTab::GRIPEventSceneLoaded() {
    // Find robot and set initial configuration for the legs
    for(int i = 0; i < mWorld->getNumSkeletons(); i++){
        if(mWorld->getSkeleton(i)->getName() == "GolemHubo"){
            mRobot = (robotics::Robot*) mWorld->getSkeleton(i);
            break;
        }
    }
    assert(mRobot);
    mRobot->getDof(19)->setValue(-10.0 * M_PI / 180.0);
    mRobot->getDof(20)->setValue(-10.0 * M_PI / 180.0);
    mRobot->getDof(23)->setValue(20.0 * M_PI / 180.0);
    mRobot->getDof(24)->setValue(20.0 * M_PI / 180.0);
    mRobot->getDof(27)->setValue(-10.0 * M_PI / 180.0);
    mRobot->getDof(28)->setValue(-10.0 * M_PI / 180.0);
    mRobot->update();

    // Define right arm nodes
    const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"};
    mArmDofs.resize(6);
    for (int i = 0; i < mArmDofs.size(); i++) {
        mArmDofs[i] = mRobot->getNode(armNodes[i].c_str())->getDof(0)->getSkelIndex();
    }

    //Define palm effector name; Note: this is robot dependent!
    eeName = "Body_RWP";
    // Initialize Grasper; done here in order to allow Close and Open Hand buttons!
    grasper = new planning::Grasper(mWorld, mRobot, eeName);
}

/// Handle event for drawing grasp markers
void manipulationTab::onCheckShowCollMesh(wxCommandEvent &evt) {
}

void manipulationTab::onCheckUseRRT(wxCommandEvent &evt) {
}

/// Set start configuration to the configuration the arm is currently in
void manipulationTab::onButtonSetStart(wxCommandEvent& evt){
    if(!mWorld || mRobot == NULL){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }
    mStartConf = mRobot->getConfig(mArmDofs);
    cout << "Start Configuration: " << mStartConf.transpose() << endl;
}

/// Reset start configuration to predefined one
void manipulationTab::onButtonSetPredefStart(wxCommandEvent& evt){
    if(!mWorld || mRobot == NULL){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }
    mStartConf = mPredefStartConf;
}

//now plan a collision free path
void manipulationTab::onButtonPlanPath(wxCommandEvent& evt) {
	if(!mWorld || !mRobot || !grasper ||(grasper->getTargetEEFTransforms().size()==0))
	{
		cout <<"not prepared to plan\n";
		return;
	}
	planPath();
	
	
	
}

/// Show the currently set start configuration
void manipulationTab::onButtonShowStart(wxCommandEvent& evt) {
    if (mStartConf.size()) {
        cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
        mRobot->setConfig(mArmDofs, mStartConf);
        viewer->DrawGLScene();
    } else {
        ECHO("ERROR: Must set start conf for right arm first!");
    }
}

/// Test currently implemented grasping approach
void manipulationTab::onButtonDoGrasping(wxCommandEvent& evt){
    if(!mWorld || mRobot == NULL){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }
    grasp();
}


/// Show next grasp pose
void manipulationTab::onButtonNextGrasp(wxCommandEvent& evt){
    if(!mWorld || mRobot == NULL){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }

	if(grasper)
	{
		
		vector<Eigen::Matrix4d> proposedGraspPoints = grasper->getTargetEEFTransforms();
    	vector<Eigen::VectorXd> proposedGraspPoses = grasper->getGraspJointPoses();
    	cout << "number grasps: " << proposedGraspPoints.size();

    	
    	if(proposedGraspPoints.size()>0)
    	{
    		//cout << "Drawing location\n";
    		shownGraspIndex = (shownGraspIndex+1) % proposedGraspPoints.size();
    		mRobot->setConfig(mArmDofs, proposedGraspPoses.at(shownGraspIndex));
    		mRobot->update();	//? is this needed?)
    	}
    	  cout << "\nshownGraspIndex: " << shownGraspIndex;
    }
	viewer->DrawGLScene();
}


/// Show next RRT config
void manipulationTab::onButtonShowConfig(wxCommandEvent& evt){
    if(!mWorld || mRobot == NULL || rrtConfigs == NULL){
        cout << "No configurations found" << endl;
        return;
    }

	const Eigen::VectorXd aConfig = *(rrtConfigs->at(rrtConfigIndex));
    mRobot->setConfig(mArmDofs, aConfig);
    mRobot->update();
    	  
   	rrtConfigIndex++;
    
	viewer->DrawGLScene();
}

/// Close robot's end effector
void manipulationTab::onButtonOpenHand(wxCommandEvent& evt) {
    if (grasper != NULL && eeName.size()) {
        grasper->openHand();
        viewer->DrawGLScene();
    } else {
        ECHO("ERROR: Must reinitialize Grasper object: Click Grasp Object!")
    }
}

void manipulationTab::onButtonFindGrasps(wxCommandEvent& evt) {
	calculateGrasps();
}

/// Open robot's end effector
void manipulationTab::onButtonCloseHand(wxCommandEvent& evt) {
    if (grasper != NULL && eeName.size()) {
    	//grasper->closeHandGraspNum(shownGraspIndex);
       grasper->closeHandPositionBased(0.1, selectedNode);
        viewer->DrawGLScene();
    } else {
        ECHO("ERROR: Must reinitialize Grasper object: Click Grasp Object!")
    }
}

void manipulationTab::calculateGrasps()
{
     grasper = new planning::Grasper(mWorld, mRobot, eeName);
     grasper->init(mArmDofs, mStartConf, selectedNode, 0.02);


    
    // Perform grasp planning; now really it's just Jacobian translation
    int graspsFound = grasper->tryToPlan();
    cout << "stuff " << endl;
     
}     

void manipulationTab::planPath()
{
	//const vector<Eigen::VectorXd> targetPoses;
	//targetPoses << grasper->getTargetPoses();
	double configStepSize = .1;	//.02
	double jointStepSize = .3;	//.1
	double workspaceThresh = .05;		//.05
	int maxConnectIterations = 100;		//100
	double angularL = .15;//what should it be?
	
	graspRRT = new planning::GraspRRT(mWorld, mRobot, mArmDofs, eeName, mStartConf, grasper->getTargetPoses(), configStepSize, jointStepSize, workspaceThresh, maxConnectIterations, angularL);
	
	if(!graspRRT->plan())
	{
		cout << "Error: unable to plan path\n";
	}
	//success
	rrtConfigs = graspRRT->getConfigurations();
	cout << "Number configurations: " << rrtConfigs->size() << "\n";
}


/// Set initial dynamic parameters and call grasp planner and controller
void manipulationTab::grasp() {
    
    if(grasper == NULL)
    {
    	cout << "Error: must calculate grasps first\n";
    	return;
    }
    
    if(selectedNode == NULL || mStartConf.size() == 0){ECHO("\tERROR: Must select an object to grasp first!!"); return;}
    // Perform memory management to allow for continuous grasping tests
    if(mController != NULL){
        delete mController;
       // delete grasper;
        //re-init grasper

    } 
    // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
    std::vector<int> actuatedDofs(mRobot->getNumDofs() - 6);
    for (unsigned int i = 0; i < actuatedDofs.size(); i++) {
        actuatedDofs[i] = i + 6;
    }
    
    cout << "stored joints. deactivating collisions with ground\n";
    
    // Deactivate collision checking between the feet and the ground during planning
    dynamics::SkeletonDynamics* ground = mWorld->getSkeleton("ground");
    mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mRobot->getNode("Body_LAR"), ground->getNode(1));
    mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mRobot->getNode("Body_RAR"), ground->getNode(1));
    
    // Define PD controller gains
    Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumDofs());
    Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mRobot->getNumDofs());
    Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumDofs());

    // Define gains for the ankle PD
    std::vector<int> ankleDofs(2);
    ankleDofs[0] = 27;
    ankleDofs[1] = 28;
    const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
    const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);
    
    cout << "updating robot pose\n";
    
    // Update robot's pose
    mRobot->setConfig(mArmDofs, mStartConf);
    
    cout << "creating controller\n";
    // Create controller
    mController = new planning::Controller(mRobot, actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);
    
	list<VectorXd> path;
	Eigen::Matrix4d targetGrasp;
	vector<int> mTotalDofs;
	
	//todo: add the rrt here.
	int result=0;
	if(checkUseRRT->IsChecked())	//allow to select between naive and collision free paths
	{
		cout << "Using RRT path\n";
		Eigen::VectorXd targetGraspV;
		result = graspRRT->getPath(path, targetGraspV, mTotalDofs);
		//add path shortening here.
	}
	else
	{
		cout << "Using Jacobian path to selected pose\n";
    	result = grasper->getGrasp(shownGraspIndex, path, targetGrasp, mTotalDofs);
    }
    
	if(result == 0)
	{
		//error
		cout << "No path\n";
		return;
	}

	//cout << "Target Grasp: \n" << targetGrasp;
     
    // CHECK
    cout << "Offline Plan Size: " << (int)path.size();
    cout << ", mTotalDofs: ";
    cout << mTotalDofs.size();
    cout << endl;
    mRobot->update();


    
    // Create trajectory; no need to shorten path here
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    
    cout << "generating path following trajectory, max Velocity = ";
    cout << maxVelocity;
    cout << ", maxAcceleration = ";
    cout << maxAcceleration;
    cout  << "\n";
    planning::Trajectory* trajectory = new planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
    
    std::cout << "Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0, mTotalDofs);
    
    // Reactivate collision of feet with floor Body_LAR Body_RAR
    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mRobot->getNode("Body_LAR"), ground->getNode(1));
    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mRobot->getNode("Body_RAR"), ground->getNode(1));

    printf("Controller time: %f \n", mWorld->mTime);
    
}

/// Replan in the middle of simulation according to accuracy measures
void manipulationTab::retryGrasp(){
    // Deactivate collision checking between the feet and the ground during planning
    dynamics::SkeletonDynamics* ground = mWorld->getSkeleton("ground");
    mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mRobot->getNode("Body_LAR"), ground->getNode(1));
    mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mRobot->getNode("Body_RAR"), ground->getNode(1));
    
    // Setup grasper by updating startConfig to be current robot's config
    grasper->init(mArmDofs, mRobot->getConfig(mArmDofs), selectedNode, 0.02);
    
    // Perform grasp planning; now really it's just Jacobian translation
    std::list<Eigen::VectorXd> path;
    std::vector<int> mTotalDofs;
    grasper->plan(path, mTotalDofs);
    
    // CHECK
    cout << "\tReplanned Path Size: " << path.size()<< endl;
    mRobot->update();
     
    // Create trajectory; no need to shorten path here
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    planning::Trajectory* trajectory = new planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
    
    cout << "\tReplanned Trajectory Duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0, mTotalDofs);
    
    // Reactivate collision of feet with floor Body_LAR Body_RAR
    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mRobot->getNode("Body_LAR"), ground->getNode(1));
    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mRobot->getNode("Body_RAR"), ground->getNode(1));

    printf("\tReplanned Controller Time: %f \n", mWorld->mTime);
}

/// Before each simulation step we set the torques the controller applies to the joints and check for plan's accuracy
void manipulationTab::GRIPEventSimulationBeforeTimestep() {


	
    Eigen::VectorXd positionTorques = mController->getTorques(mRobot->getPose(), mRobot->getQDotVector(), mWorld->mTime);
    
/*
	cout << "torques:\n" << positionTorques << "\n\n";
	
    int closeHand = 0;
    if(closeHand)
	{
		grasper->closeHandTorqueBased(&positionTorques);
	}

	cout << "torques after:\n" << positionTorques << "\n\n";
	*/

    // section here to control the fingers for force-based grasping
    // instead of position-based grasping

    mRobot->setInternalForces(positionTorques);
    
    //check object position and replan only if it hasnt been done already to save computing power
    if (!mAlreadyReplan && 0) {
        grasper->findClosestGraspingPoint(currentGraspPoint, selectedNode);
        Vector3d diff = currentGraspPoint - grasper->getGraspingPoint();
        
        // Note: Error bound must be < 0.09 as Jacobian translation fails to reach when it's too close to target
        if (diff.norm() >= 0.006) {
            ECHO("\tNote: Re-planning grasp!");
            this->retryGrasp();
            mAlreadyReplan = true;
        }
    }
}

/// Handle simulation events after timestep
void manipulationTab::GRIPEventSimulationAfterTimestep() {
}

/// Handle simulation start events
void manipulationTab::GRIPEventSimulationStart() {
}

/// Store selected node in tree-view data as grasper's objective
void manipulationTab::GRIPStateChange() {
    if (!selectedTreeNode) {
        return;
    }
    switch (selectedTreeNode->dType) {
    case Return_Type_Object:
    case Return_Type_Robot:
        selectedNode = ((kinematics::Skeleton*)selectedTreeNode->data)->getRoot();
        break;
    case Return_Type_Node:
        selectedNode = (kinematics::BodyNode*)selectedTreeNode->data;
        break;
    default:
        fprintf(stderr, "someone else's problem.");
        assert(0);
        exit(1);
    }
}

/// Render grasp' markers such as grasping point
void manipulationTab::GRIPEventRender() {

    if(checkShowCollMesh->IsChecked() && grasper && mWorld)
    {
        drawAxesWithOrientation(grasper->getGCPTransform(), 0.08);	//where actual GCP is
        drawAxesWithOrientation(grasper->getEEFTransform(), 0.12);	//where actual wrist is
   //     cout << "\n\n GCP:\n" << grasper->getGCPTransform();
    //    cout <<"\n\n eef:\n" << grasper ->getEEFTransform();
        
	//	cout << "\nCurrent wrist location:\n" << grasper->getOrientationVector(grasper->getEEFTransform());

    	if(grasper->getTargetPalmTransforms().size() > 0)
    	{
    		Matrix4d aGrasp = grasper->getTargetPalmTransforms().at(shownGraspIndex);

    		drawAxesWithOrientation(aGrasp, .06);		//desired location of gcp
    		
    		Matrix4d EEFTarget = grasper->getTargetEEFTransforms().at(shownGraspIndex);	//where wrist needs to be
    		drawAxesWithOrientation(EEFTarget, .1);
    		
    	//	cout << "\nTarget wrist location: \n" << grasper->getOrientationVector(EEFTarget);
    		
    		
    		Eigen::VectorXd orientationDiff(6);
    		orientationDiff << grasper->getOrientationVector(grasper->getEEFTransform()) - grasper->getOrientationVector(EEFTarget);
    		
    	//	cout << "\nDiff: \n" << orientationDiff;
    	//	cout << "\nMagnitude:\n" << orientationDiff.norm();
    	}
    	
    	glFlush();
    }
}

/// Method to draw XYZ axes
void manipulationTab::drawAxes(Eigen::VectorXd origin, double size, tuple<double,double,double> color){
    glBegin(GL_LINES);
    glColor3f(get<0>(color), get<1>(color), get<2>(color));
    glVertex3f(origin(0) - size, origin(1), origin(2));
    glVertex3f(origin(0) + size, origin(1), origin(2));

    //glColor3f(0, 0, 1);
    glVertex3f(origin(0), origin(1) - size, origin(2));
    glVertex3f(origin(0), origin(1) + size, origin(2));

    //glColor3f(0, 1, 0);
    glVertex3f(origin(0), origin(1), origin(2) - size);
    glVertex3f(origin(0), origin(1), origin(2) + size);
    glEnd();
}

/// Method to draw XYZ axes with proper orientation. Collaboration with Justin Smith
//red: x, blue: y, green: z		long: positive, short: negative
void manipulationTab::drawAxesWithOrientation(const Eigen::Matrix4d& transformation, double size ) {

    Eigen::Matrix4d basis1up, basis1down, basis2up, basis2down;
    basis1up << size/2, 0.0, 0.0, 0,
            0.0, size/2, 0.0, 0,
            0.0, 0.0, size/2, 0,
            1.0, 1.0, 1.0, 1;

    basis1down << -size, 0.0, 0.0, 0,
            0.0, -size, 0.0, 0,
            0.0, 0.0, -size, 0,
            1.0, 1.0, 1.0, 1;

    basis2up = transformation * basis1up;
    basis2down = transformation * basis1down;


    glBegin(GL_LINES);

    glColor3f(1, 0, 0);
    glVertex3f(basis2down(0, 0), basis2down(1, 0), basis2down(2, 0));
    glVertex3f(basis2up(0, 0), basis2up(1, 0), basis2up(2, 0));

    glColor3f(0, 0, 1);
    glVertex3f(basis2down(0, 1), basis2down(1, 1), basis2down(2, 1));
    glVertex3f(basis2up(0, 1), basis2up(1, 1), basis2up(2, 1));

    glColor3f(0, 1, 0);
    glVertex3f(basis2down(0, 2), basis2down(1, 2), basis2down(2, 2));
    glVertex3f(basis2up(0, 2), basis2up(1, 2), basis2up(2, 2));
    glEnd();
}

/// Method to draw XYZ axes with proper orientation. Collaboration with Justin Smith
void manipulationTab::drawAxesWithOrientation(const Eigen::Matrix4d& transformation, double size, tuple<double,double,double> color) {
    Eigen::Matrix4d basis1up, basis1down, basis2up, basis2down;
    basis1up << size, 0.0, 0.0, 0,
            0.0, size, 0.0, 0,
            0.0, 0.0, size, 0,
            1.0, 1.0, 1.0, 1;

    basis1down << -size, 0.0, 0.0, 0,
            0.0, -size, 0.0, 0,
            0.0, 0.0, -size, 0,
            1.0, 1.0, 1.0, 1;

    basis2up = transformation * basis1up;
    basis2down = transformation * basis1down;


    glBegin(GL_LINES);
    glColor3f(get<0>(color), get<1>(color), get<2>(color));
    glVertex3f(basis2down(0, 0), basis2down(1, 0), basis2down(2, 0));
    glVertex3f(basis2up(0, 0), basis2up(1, 0), basis2up(2, 0));

    glColor3f(0, 0, 1);
    glVertex3f(basis2down(0, 1), basis2down(1, 1), basis2down(2, 1));
    glVertex3f(basis2up(0, 1), basis2up(1, 1), basis2up(2, 1));

    glColor3f(0, 1, 0);
    glVertex3f(basis2down(0, 2), basis2down(1, 2), basis2down(2, 2));
    glVertex3f(basis2up(0, 2), basis2up(1, 2), basis2up(2, 2));
    glEnd();
}

// Local Variables:
// c-basic-offset: 4
// End:

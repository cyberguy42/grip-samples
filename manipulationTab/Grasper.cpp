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

/** @file Grasper.cpp
 *  @author Juan C. Garcia
 */

#include "Grasper.h"
#include "robotics/World.h"
#include "robotics/Robot.h"
#include "kinematics/Dof.h"
#include <GUI/Viewer.h>
#include <Eigen/LU>
#include <set>
#include <dart/dynamics/ContactDynamics.h>
#include <dart/collision/CollisionSkeleton.h>
#include <planning/PathShortener.h>
#include "manipulationTab.h"

using namespace std;
using namespace Eigen;
using namespace robotics;
using namespace collision_checking;

namespace planning {
    
    Grasper::Grasper(World* w, robotics::Robot* r, string mEEName) {
        world = w;
        robot = r;
        EEName = mEEName;
        gcpVirtualLoc << 0.004047, 0.0128156, -0.07162959;
    }
    
    Grasper::~Grasper() {
        delete jm;
        delete shortener;
    }
    
    /// Finish initialization of grasper by providing end effector's links ids, start configuration and target object
    void Grasper::init(std::vector<int> d, Eigen::VectorXd start, kinematics::BodyNode* node, double step){
        dofs = d; 
        objectNode = node;
        this->setStartConfig(start);
        
        //initialize path shortener
        shortener = new planning::PathShortener(world, 0, dofs);
        
        //initialize JointMover with end effector and arm's DoFs
        jm = new JointMover(*world, robot, dofs, EEName, step);
        
        gcpTransform = Eigen::Matrix4d::Identity();
        
        //good enough for now
        Eigen::Matrix4d palmTranslate; palmTranslate << 1,0,0,.016,0,1,0,-.01,0,0,1,-.075,0,0,0,1;
		
		float palmAngle = .75;
        Eigen::Matrix4d palmTilt; palmTilt << 1,0,0,0,0,cos(palmAngle),-sin(palmAngle),0,0,sin(palmAngle),cos(palmAngle),0,0,0,0,1;     
       	
       	Eigen::Matrix4d palmRotate; 
       	palmRotate << 	-1, 0, 0, 0, 
       					0, -1, 0, 0, 
       					0, 0,  1, 0, 
       					0, 0,  0, 1;
       	palmTransformation = palmTranslate*palmTilt*palmRotate;// for some reason, order is backwards: first the translation, then tilt, then rotate. 
       	palmInverse = palmTransformation.inverse();
       	
        //gcpTransform = robot->getNode(EEName.c_str())->getLocalInvTransform();
        
        //gcpTransform.topLeftCorner(3,3) = Eigen::Matrix3f(AngleAxisf(3.14159/2, Vector3f::UnitZ()) * AngleAxisf(0, Vector3f::UnitY()) * AngleAxisf(0, Vector3f::UnitZ()));
        
    }
    
    /// Attempt a grasp at a target object
    void Grasper::plan(list<VectorXd> &path, vector<int> &totalDofs) {
        //find closest point in target object; grasp target point
        int min = findClosestGraspingPoint(graspPoint, objectNode);
        
        //perform translation Jacobian towards grasping point computed
        VectorXd goalPose(6);
   //     jm->GoToXYZRPY(startConfig, graspPoint, goalPose, path);
        
        //shorten path
        shortener->shortenPath(path);
        
        //try to close hand;
        totalDofs = dofs;
        closeHandPositionBased(0.1, objectNode);
 
        //merge DoFs to include hand closure configs in path
        totalDofs.insert(totalDofs.end(), hand_dofs.begin(), hand_dofs.end());
        
        //increase size of every vector in path
        for(list<VectorXd>::iterator it = path.begin(); it != path.end(); it++){
            VectorXd & v (*it);
            v.conservativeResize(totalDofs.size()); 
            v.segment(dofs.size(),hand_dofs.size()) = VectorXd::Zero(hand_dofs.size(), 1);
        }
        path.push_back(robot->getConfig(totalDofs));
        ECHO("Note: Added closed hand config to path!");
        
        //move grasped object around
        list<VectorXd> targetPoints; 
        
        //For all objects except for the life saver and the driving wheel 
        VectorXd v(3); v << 0.33,-0.10, 1.0; 
        VectorXd w(3); w << 0.33,-0.16, 1.0; 
        
        //For life saver
        //VectorXd v(3); v << 0.33, -0.27, 1.0;
        //VectorXd w(3); w << 0.33, -0.16, 1.2;
        
        //For driving wheel
        //VectorXd v(3); v << 0.30, 0.04, 0.7;
        //VectorXd w(3); w << 0.30, 0.04, 0.9;
        
        targetPoints.push_back(v);
        targetPoints.push_back(w);
       
        VectorXd backPose(6);
        list<VectorXd> path_back;
       
        // move to as many target points as wished and store paths
        for(list<VectorXd>::iterator loc = targetPoints.begin(); loc != targetPoints.end(); loc++){
            VectorXd & t(*loc);
            path_back.clear();
           
         //   jm->GoToXYZRPY(robot->getConfig(dofs), t, backPose, path_back);
            shortener->shortenPath(path_back);
            for(list<VectorXd>::iterator it = path_back.begin(); it != path_back.end(); it++){
                    VectorXd & v (*it);
                    v.conservativeResize(totalDofs.size()); 
                    v.segment(dofs.size(),hand_dofs.size()) = robot->getConfig(hand_dofs);

                    //merge lists
                    path.push_back(v);
            }
        }
        //open hand at the end and store such configuration
        openHand();
        path.push_back(robot->getConfig(totalDofs));
        
        //reset robot to start configuration
        robot->setConfig(dofs, startConfig);
    }
    
    /// Find closest point in target object to be grasped
    double Grasper::findClosestGraspingPoint(Vector3d &closest, kinematics::BodyNode* object){
        //1. get collision meshes and vertices
    	kinematics::ShapeMesh* shapeMesh = dynamic_cast<kinematics::ShapeMesh *>(object->getCollisionShape());

    	if(!shapeMesh) {
    		return -1;
        }
        const aiScene* sc = shapeMesh->getMesh();

        double min_distance = -1;
         
        if(sc != NULL){
            const aiNode* nd = sc->mRootNode;
            Matrix4d worldTrans = object->getWorldTransform();
            const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[0]];
           
            //2. find closest vertex
            for(int j=0; j<mesh->mNumVertices; j++){
                
                VectorXd vertices(4); vertices <<  mesh->mVertices[j].x, mesh->mVertices[j].y , mesh->mVertices[j].z, 1;
                vertices = worldTrans * vertices; //now vertices turned into 4x1 matrix

                //calculate distance between current vertex and GCP
                Vector3d diff; diff << vertices(0,0), vertices(1,0), vertices(2,0);
                Vector3d GCP;
                GCP = robot->getNode(EEName.c_str())->getWorldCOM();
                diff = diff - GCP;
                
                if(min_distance == -1 || diff.norm() < min_distance){
                    min_distance = diff.norm();
                    closest << vertices(0,0), vertices(1,0), vertices(2,0);
                }    
            }
        }
        
        //offset grasping point by virtual grasp center point (GCP) in robot's end effector
        Eigen::Matrix4d effTrans = robot->getNode(EEName.c_str())->getLocalInvTransform();
        VectorXd prod(4); prod << gcpVirtualLoc, 1;
        prod = effTrans * prod;
        VectorXd temp(3); temp << prod(0), prod(1), prod(2);
        closest = closest - temp;
        
        return min_distance;
    }



	void Grasper::closeHandTorqueBased(Eigen::VectorXd* torques) {
        int fingers = robot->getNode(EEName.c_str())->getNumChildJoints();
        vector<int> jointDirections;
        
        //first build list of joints
        this->populateEndEffIds(fingers, joints, jointDirections);
  
        int jointID = 0;
        float torqueValue = -.0001;
			
			//iterate through each finger
				//iterate through each joint and check collisions, apply torque appropriately
            //iterate through each end-effector joint i.e. 15 joints = 5 x 3 joint per finger
 	
 		int j = 0;
        for(int i = 0; i < hand_dofs.size(); i++, j = i % 3)
        {
        	int index = hand_dofs.at(i);
        	float eachTorque;
        	if(j==0)
        		eachTorque = -.001; //torqueValue;
        	else if(j==1)
        		eachTorque = 0; //torqueValue/2;
        	else
        		eachTorque = 0;//torqueValue/4;
        		
        	cout << index << "\n";
        	(*torques)[index] = torqueValue;
        }

    }
    
    /// Modifications of idea provided by Asfour et. al. GraspRRT on Robotics and Automation Magazine, 2012
    vector<ContactPoint> Grasper::closeHandPositionBased(double step, kinematics::BodyNode* target) {
        
        vector<ContactPoint> resulting_contacts(100); 
        if (target == NULL) {
            ECHO("ERROR: Must select object to grasp first!");
            return resulting_contacts;
        }
        int fingers = robot->getNode(EEName.c_str())->getNumChildJoints();
        vector<int> jointDirections;
        
        //first build list of joints
        this->populateEndEffIds(fingers, joints, jointDirections);
  
        vector<bool> colliding_link(joints.size(), false);
        bool grasped = false;
        int jointID = 0;
        int iteration = 0;
        while (!grasped) {
            iteration++;
            grasped = true;
            //iterate through each end-effector joint i.e. 15 joints = 5 x 3 joint per finger
            int link = 0;
            for (list<kinematics::Joint*>::iterator it = joints.begin(); it != joints.end(); it++, link++) {
                //check for collision status
                if (!colliding_link[link]) {
                    grasped = false;
                    kinematics::Joint *j(*it);
                    
                    // check for collision and set as colliding if so; set corresponding 
                    // link collision status to true or false; disregard for thumb
                    // QUICK FIX for thumb bug: don't check for collision initially
                    colliding_link[link] = (link > 11) ? moveLinkWithCollisionChecking(step, jointDirections[link], j, target, resulting_contacts, false) : 
                    moveLinkWithCollisionChecking(step, jointDirections[link], j, target, resulting_contacts, true);
                }
            }
            //perform additional check to allow for better grasping only before more than half the links are already in contact
            if (grasped && iteration > 2 && colliding_link.size() < 7) {
                iteration = 0; 
                grasped = false;

                for (int i = 0; i < colliding_link.size(); i++) {
                    colliding_link[i] = false;
                }
            }
        }
        return resulting_contacts;
    }
    
    /// Open robot's hand by setting all fingers joints values to 0
    void Grasper::openHand(){
        int fingers = robot->getNode(EEName.c_str())->getNumChildJoints();
        int jointID = 0;
        for(int i=0; i < fingers; i++){
            //set all fingers'joint values to 0
            kinematics::Joint *fingerJoint = robot->getNode(EEName.c_str())->getChildJoint(i);
            fingerJoint->getDof(0)->setValue(0);
            fingerJoint->getChildNode()->getChildJoint(jointID)->getDof(0)->setValue(0);
            fingerJoint->getChildNode()->getChildJoint(jointID)->getChildNode()->getChildJoint(0)->getDof(0)->setValue(0);
            robot->update();
        }
    }
    
    /// Increase a joint value with collision checking
    bool Grasper::moveLinkWithCollisionChecking(double step, int direction, kinematics::Joint* joint, kinematics::BodyNode* target, vector<ContactPoint> contacts, bool checkCollisions){
        bool ret = true;
        
        double oldJointValue = joint->getDof(0)->getValue();
        double newJointValue = oldJointValue + step*direction;
        
        if((newJointValue <= (joint->getDof(0)->getMax()*0.4)) && (newJointValue >= (joint->getDof(0)->getMin()*0.4))){
            joint->getDof(0)->setValue(newJointValue);
            robot->update();
           
            CollisionSkeletonNode* other = world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(target);
            
            //check collision against child BodyNode
            if(!checkCollisions || !world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(joint->getChildNode())->checkCollision(other, &contacts, contacts.size())){
                ret = false;
            }
            else{
                joint->getDof(0)->setValue(oldJointValue);
                robot->update();
            }
        }
        return ret;
    }
    
    /// Print contents of std::vector
    void Grasper::printVectorContents(std::vector<int> v){
        for(int i = 0; i < v.size(); i++)
            cout << "vector[" << i << "]--" << v.at(i) << endl;
    }
    
    /// Check how many of the links are colliding with target node
    int Grasper::checkHandCollisionCount(){
        vector<ContactPoint> contacts(10);
        int count = 0;
        CollisionSkeletonNode* other = world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(objectNode);
        for(list<kinematics::Joint*>::iterator loc = joints.begin(); loc != joints.end(); loc++){
             kinematics::Joint *j(*loc);
            count += (world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(j->getChildNode())->checkCollision(other, &contacts, contacts.size()) > 0);
        }
        return count;
    }
    
    /// Get current grasping point
    Eigen::Vector3d Grasper::getGraspingPoint(){
        return graspPoint;
    }
    
    /// Return the list of skeleton indices for the hand
    std::vector<int> Grasper::getHandDofs(){
        assert(hand_dofs.size() > 0);
        return hand_dofs;
    }
    
    /// Return the GCP virtual location in the end-effector (in world's coordinates)
    Eigen::Vector3d Grasper::getGCPXYZ(){
        Eigen::Matrix4d transformation = robot->getNode(EEName.c_str())->getWorldTransform();
        Vector4d res; res << gcpVirtualLoc, 1;
        res = transformation*res;
        Vector3d ret; ret << res(0), res(1), res(2);
        return ret;
    }
    
    /// Return the virtual GCP transform; mainly for drawing. want this to match targetGraspTransform
    Eigen::Matrix4d Grasper::getGCPTransform(){
        Eigen::Matrix4d eefTransf = getEEFTransform();
        
       	return eefTransf *palmTransformation;
    }
    
    Eigen::Matrix4d Grasper::getEEFTransform()
    {
    	return robot->getNode(EEName.c_str())->getWorldTransform();
    }
    
    vector<Eigen::Matrix4d> Grasper::getTargetEEFTransforms(){		//target coordinate system for EEF
    	return targetWristTransforms;
    }
    
    vector<Eigen::Matrix4d> Grasper::getTargetPalmTransforms(){	//relative to object in real world	
    	return targetPalmTransforms;
    }
    
    vector<Eigen::Matrix4d> Grasper::getLocalGraspTransforms(){	//relative to object in real world	
    	return localGraspTransforms;
    }
    
    vector<Eigen::VectorXd> Grasper::getGraspJointPoses(){		//joint angles for grasp
    	return targetJointPoses;
    
    }
    
    /// Set start config for grasper planner
    void Grasper::setStartConfig(Eigen::VectorXd start){
        startConfig = start;
    }
    

    /// Method creates a list of joints and a vector with their respective directions-robot dependent-; populates the hand_dofs vector
    void Grasper::populateEndEffIds(int fingers, list<kinematics::Joint*> &js, vector<int> &jointDirections){
        //Clear if already has been called
        if(hand_dofs.size()){
            hand_dofs.clear();
            js.clear();
            jointVec.clear();
            jointDirections.clear();
            successfulGraspIndex.clear();
        }
        for (int i = 0; i < fingers; i++) {
            //populate list of end-effector joints
            kinematics::Joint* fingerJoint = robot->getNode(EEName.c_str())->getChildJoint(i);
            js.push_back(fingerJoint);
            js.push_back(fingerJoint->getChildNode()->getChildJoint(0));
            js.push_back(fingerJoint->getChildNode()->getChildJoint(0)->getChildNode()->getChildJoint(0));
			
			jointVec.push_back(fingerJoint);
			jointVec.push_back(fingerJoint->getChildNode()->getChildJoint(0));
            jointVec.push_back(fingerJoint->getChildNode()->getChildJoint(0)->getChildNode()->getChildJoint(0));
			
            //populate list of joint directions; finger joint grows - ,while distal and medial grow +
            jointDirections.push_back(-1);
            jointDirections.push_back(1);
            jointDirections.push_back(1);

            //populate end-effector's DoF vector
            hand_dofs.push_back(fingerJoint->getDof(0)->getSkelIndex());
            hand_dofs.push_back(fingerJoint->getChildNode()->getChildJoint(0)->getDof(0)->getSkelIndex());
            hand_dofs.push_back(fingerJoint->getChildNode()->getChildJoint(0)->getChildNode()->getChildJoint(0)->getDof(0)->getSkelIndex());
        }
    }
    
    
    int Grasper::tryToPlan()
    {
    	cout << "Try to plan...\n";
    	if(!loadGrasps())
    		return 0;
    	
    	cout << "loaded grasps. proceeding to test each one:\n";
    	
    	int result = 0;
    	for(int i = 0; i < objectGrasps.size(); i++)
    	{
    		//cout << "trying grasp # " << i << "\n";
    		
    		result += tryGrasp(&objectGrasps.at(i), i);
    
    	}
    	cout << "done\n\n";
    
    	return result;
    }
    
    int Grasper::getGrasp(int graspNum, list<VectorXd> &path, Eigen::Matrix4d &targetGrasp, vector<int> &dofs)
    {
    //	if(objectGrasps.size()>0)
    //	{
    		path = allPaths[graspNum];
    		dofs = allTotalDofs[graspNum];
    		targetGrasp = targetPalmTransforms[graspNum];
    		return 1;
    //	}
    //	else
    //	{
    //		return 0;
    //	}
    }
    		
    
    /// Attempt a grasp at a target object
    int Grasper::tryGrasp(graspStruct* grasp, int graspNum) {
	//	path.clear();
	//	totalDofs.clear();
		
	//	cout << "this grasp: x " << grasp->xCoord << ", y " << grasp->yCoord << ", r0 " << grasp->r0 << ", thumb0 " << grasp->thumb0 << "\n";
		
		vector<int> totalDofs;
		list<VectorXd> path;
		
		
		//need to change this
		//start with gcp offset:
		//offset grasping point by virtual grasp center point (GCP) in robot's end effector
        Eigen::Matrix4d effTrans = robot->getNode(EEName.c_str())->getLocalInvTransform();
        VectorXd prod(4); prod << gcpVirtualLoc, 1;
        prod = effTrans * prod;
        //VectorXd gcpOff(6); gcpOff << prod(0), prod(1), prod(2),0,0,0;
        Eigen::Matrix4d gcpOff = Eigen::Matrix4d::Identity();
     //   gcpOff.col(3) = prod;
        
        
        
		
		//get transformation matrix for pose of hand relative to object
		Eigen::Quaterniond quatAng(grasp->r0, grasp->r1, grasp->r2, grasp->r3);
		
		Eigen::Matrix3d rotMat = Eigen::Matrix3d(quatAng);
		Eigen::Matrix4d relTransf = Eigen::Matrix4d::Identity();
		relTransf.topLeftCorner(3,3) = rotMat;
		relTransf(0,3) = grasp->xCoord;
		relTransf(1,3) = grasp->yCoord;
		relTransf(2,3) = grasp->zCoord;
		
		
		
		

		
		Eigen::Matrix4d globalObjectTransf = objectNode->getWorldTransform();
		
		//now transform gcp offset to relative hand to object pose to object to world.
		Eigen::Matrix4d globalGraspPose = globalObjectTransf * relTransf*palmInverse;	//proper order, somehow
		

		
		Eigen::Matrix3d rotationM = globalGraspPose.topLeftCorner(3,3);
		Eigen::VectorXd rotation = rotationM.eulerAngles(0,1,2);
		//get pose
		Eigen::VectorXd graspPose(6); graspPose << globalGraspPose(0,3), globalGraspPose(1,3), globalGraspPose(2,3), rotation(0), rotation(1), rotation(2);
        
        //perform translation Jacobian towards grasping point computed
        VectorXd goalPose(6);
        
   //     cout << "\nGlobal transformation of object\n" << globalObjectTransf;
   //     cout << "\n\nRelative Transformation Hand to Object:\n" << relTransf;
        
   //     cout << "\n\nPose:\n" << globalGraspPose;
                
        double angularL =palmTransformation.block(0,3,3,1).norm();        
        
        double distance = jm->GoToXYZRPY(startConfig, graspPose, goalPose, path, angularL);
        bool goodJoints = jm->jointsValid(goalPose);
        
        cout << "\npath size: " << (int)path.size() << ", distance: " << distance << ", valid: " << goodJoints << "\n";
        
        if(distance > .05 || !goodJoints)
        {
         	//	cout << "\nNot good enough\n";
        	return 0;
        }
        
        totalDofs = dofs;
        targetJointPoses.push_back(goalPose);
        allPaths.push_back(path);
    	allTotalDofs.push_back(totalDofs);
    	cout << "dofs: " << totalDofs.size();
    	cout << "\n";
    	
    	
    	targetPalmTransforms.push_back(globalObjectTransf * relTransf);
		targetWristTransforms.push_back(globalGraspPose);
        localGraspTransforms.push_back(relTransf);
        successfulGraspIndex.push_back(graspNum);
        
        cout << "added another path (#" << (int)targetJointPoses.size() << ")\n";
        
        //closeHandGraspFile(grasp);	//testing purposes- does the hand close right?
        
        return 1;
        //shorten path
        shortener->shortenPath(path);
        
        //try to close hand;
        totalDofs = dofs;
        //closeHandPositionBased(0.1, objectNode);
 		closeHandGraspFile(grasp);
 		
        //merge DoFs to include hand closure configs in path
        totalDofs.insert(totalDofs.end(), hand_dofs.begin(), hand_dofs.end());
        
        //increase size of every vector in path
        for(list<VectorXd>::iterator it = path.begin(); it != path.end(); it++){
            VectorXd & v (*it);
            v.conservativeResize(totalDofs.size()); 
            v.segment(dofs.size(),hand_dofs.size()) = VectorXd::Zero(hand_dofs.size(), 1);
        }
        path.push_back(robot->getConfig(totalDofs));
        ECHO("Note: Added closed hand config to path!");
        
        //move grasped object around
        list<VectorXd> targetPoints; 
        
        //For all objects except for the life saver and the driving wheel 
        VectorXd v(3); v << 0.33,-0.10, 1.0; 
        VectorXd w(3); w << 0.33,-0.16, 1.0; 
        
        //For life saver
        //VectorXd v(3); v << 0.33, -0.27, 1.0;
        //VectorXd w(3); w << 0.33, -0.16, 1.2;
        
        //For driving wheel
        //VectorXd v(3); v << 0.30, 0.04, 0.7;
        //VectorXd w(3); w << 0.30, 0.04, 0.9;
        
        targetPoints.push_back(v);
        targetPoints.push_back(w);
       
        VectorXd backPose(6);
        list<VectorXd> path_back;
       
        // move to as many target points as wished and store paths
        for(list<VectorXd>::iterator loc = targetPoints.begin(); loc != targetPoints.end(); loc++){
            VectorXd & t(*loc);
            path_back.clear();
           
            jm->GoToXYZRPY(robot->getConfig(dofs), t, backPose, path_back, angularL);
            shortener->shortenPath(path_back);
            for(list<VectorXd>::iterator it = path_back.begin(); it != path_back.end(); it++){
                    VectorXd & v (*it);
                    v.conservativeResize(totalDofs.size()); 
                    v.segment(dofs.size(),hand_dofs.size()) = robot->getConfig(hand_dofs);

                    //merge lists
                    path.push_back(v);
            }
        }
        //open hand at the end and store such configuration
        openHand();
        path.push_back(robot->getConfig(totalDofs));
        
        //reset robot to start configuration
        robot->setConfig(dofs, startConfig);
        return 1;	//better, return quality
    }
    
    
    /*
	void collisionCheck(int activate)
	{
	/
		int fingers = robot->getNode(EEName.c_str())->getNumChildJoints();
        vector<int> jointDirections;
    	this->populateEndEffIds(fingers, joints, jointDirections);
    	
    	vector<int> collisionDofs = dofs;
        collisionDofs.insert(collision.end(), hand_dofs.begin(), hand_dofs.end());
        
        for(int i = 0; i < collisionDofs.size(); i++)
        {
        	int index = hand_dofs.at(i);
        	
        }/
        
        const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"};
        
        for(int i = 0; i < ; i++)
        {
        	world->mCollisionHandle->getCollisionChecker()->deactivatePair(mRobot->getNode(armNodes[i]), ground->getNode(1));
        }
	}
	*/
	
	Eigen::VectorXd Grasper::getOrientationVector(Eigen::Matrix4d transformation)
	{
		Eigen::Matrix3d rotM= transformation.topLeftCorner(3,3);
		Eigen::VectorXd rotation = rotM.eulerAngles(0,1,2);

		Eigen::VectorXd orientation(6); 
		orientation << transformation(0,3), transformation(1,3), transformation(2,3), rotation(0), rotation(1), rotation(2);
		
		return orientation;
	}
	
	void Grasper::closeHandGraspNum(int graspNum)
	{
		int graspIndex = successfulGraspIndex.at(graspNum);
		graspStruct* grasp = &objectGrasps.at(graspIndex);
		closeHandGraspFile(grasp);
	}
	
	void Grasper::closeHandGraspFile(graspStruct* grasp)
	{
	    int fingers = robot->getNode(EEName.c_str())->getNumChildJoints();
        vector<int> jointDirections;
        
        //first build list of joints
        this->populateEndEffIds(fingers, joints, jointDirections);
               
        jointVec.at(0)->getDof(0)->setValue(jointDirections[0] * grasp->point0 *180/3.14);
        jointVec.at(1)->getDof(0)->setValue(jointDirections[1] * grasp->point1 *180/3.14);
        jointVec.at(2)->getDof(0)->setValue(jointDirections[2] * grasp->point2 *180/3.14);
        
        jointVec.at(3)->getDof(0)->setValue(jointDirections[3] * grasp->middle0 *180/3.14);
        jointVec.at(4)->getDof(0)->setValue(jointDirections[4] * grasp->middle1 *180/3.14);
        jointVec.at(5)->getDof(0)->setValue(jointDirections[5] * grasp->middle2 *180/3.14);
        
        jointVec.at(6)->getDof(0)->setValue(jointDirections[6] * grasp->pinky0 *180/3.14);
        jointVec.at(7)->getDof(0)->setValue(jointDirections[7] * grasp->pinky1 *180/3.14);
        jointVec.at(8)->getDof(0)->setValue(jointDirections[8] * grasp->pinky2 *180/3.14);
        
        jointVec.at(9)->getDof(0)->setValue(jointDirections[9] * grasp->ring0 *180/3.14);
        jointVec.at(10)->getDof(0)->setValue(jointDirections[10] * grasp->ring1 *180/3.14);
        jointVec.at(11)->getDof(0)->setValue(jointDirections[11] * grasp->ring2 *180/3.14);
        
        jointVec.at(12)->getDof(0)->setValue(jointDirections[12] * grasp->thumb0 *180/3.14);
        jointVec.at(13)->getDof(0)->setValue(jointDirections[13] * grasp->thumb1 *180/3.14);
        jointVec.at(14)->getDof(0)->setValue(jointDirections[14] * grasp->thumb2 *180/3.14);
        robot->update();
	}
    
    int Grasper::loadGrasps()
    {
        //todo: auto load name from object
		float f;
		FILE * pFile;
		
		cout << "load grasps\n";
		
		pFile = fopen ("grasps/drill.txt","r");
		if(pFile==NULL)
		{
			cout << "\nerror, invalid file\n";
			return 0;
		}
		int unknown;
		
		objectGrasps.clear();
		
		int keepGoing = 1;
		while(keepGoing)
		{
			graspStruct newGrasp;
			keepGoing = fscanf(pFile, "%u %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n%u %f %f %f %f %f %f %f\n", &newGrasp.numFingers, &newGrasp.thumb0, &newGrasp.thumb1, &newGrasp.thumb2, &newGrasp.point0, &newGrasp.point1, &newGrasp.point2, &newGrasp.middle0, &newGrasp.middle1, &newGrasp.middle2, &newGrasp.ring0, &newGrasp.ring1, &newGrasp.ring2, &newGrasp.pinky0, &newGrasp.pinky1, &newGrasp.pinky2, &unknown, &newGrasp.xCoord, &newGrasp.yCoord, &newGrasp.zCoord, &newGrasp.r0, &newGrasp.r1, &newGrasp.r2, &newGrasp.r3);
			
			newGrasp.xCoord/=1000;
			newGrasp.yCoord/=1000;
			newGrasp.zCoord/=1000;
			
			if(keepGoing==24)
			{
				objectGrasps.push_back(newGrasp);
	//			cout << "keep going: " << keepGoing << " New grasp (x, y): " << newGrasp.xCoord <<", " << newGrasp.yCoord << "\n";
			}
			else
			{
	//			cout << "bad one\n";
				keepGoing = 0;
			}
		}
		
		fclose (pFile);
		return objectGrasps.size();
	}
}

/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
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
 *
 */

/** @file GraspRRT.cpp
 *  @author Juan C. Garcia
 */

#include "GraspRRT.h"
#include "robotics/World.h"
#include "robotics/Robot.h"
#include <robotics/Object.h>
#include "kinematics/Dof.h"
#include <GUI/Viewer.h>
#include <Eigen/LU>
#include "planningTab.h"
// Helpful macros
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;


using namespace std;
using namespace Eigen;
using namespace robotics;

Eigen::Vector3d GCP;
Eigen::Vector3d graspPoint;

namespace planning {

    GraspRRT::GraspRRT(World* world, const std::vector<int> &dofs, int r, const Eigen::VectorXd &start, 
            const Eigen::VectorXd &objectLocation, kinematics::BodyNode* objectNode, std::string mEEName) :
    world(world),
    robot(r),
    dofs(dofs),
    startConfig(start),
    objectNode(objectNode),
    index(flann::KDTreeSingleIndexParams()),
    gcp_offset(0.03),
    cycles(0),
    forceStop(false),
    foundSolution(false),
    EEName(mEEName)
    {
        srand(time(NULL));
        jm = new JointMover(*world, robot, dofs, EEName, 0.02);

        Eigen::VectorXd handcoord(3);
        world->getRobot(robot)->getBodyNodePositionXYZ(EEName, handcoord[0], handcoord[1], handcoord[2]);
        GCP << handcoord[0], handcoord[1], handcoord[2];
        
        //performance variables
        smallGap = std::numeric_limits<double>::infinity();
        max_cycles = 8000;
    }

    GraspRRT::~GraspRRT() {
    }

    void GraspRRT::plan(std::list<Eigen::VectorXd> &path) {
        bool stopLoop = false;
        
        //init RRT
        rrt = new planning::RRT(world, robot, dofs, startConfig, 0.01);
        typename RRT::StepResult progress = RRT::STEP_PROGRESS;
        
        //try to grasp an object until max_cycle count reached, forced stop or a solution has been found
        do {
            progress = doCycle();
            cycles++;
            //PRINT(cycles);
            
            if (cycles > max_cycles || forceStop || foundSolution) {
                stopLoop = true;
                ECHO("\tMAX_CYCLES REACHED: forced stop!!");
            }
            
        } while (progress != RRT::STEP_REACHED && !stopLoop);
        
        rrt->tracePath(rrt->activeNode, path, false);
    }
    
    RRT::StepResult GraspRRT::doCycle() {
        int i;
        float randMult = (float) (1.0 / (double) (RAND_MAX));
        float probabGrasp = 0.1f;
        
        
         //update GCP location
        Eigen::VectorXd handcoord(3);
        world->getRobot(robot)->getBodyNodePositionXYZ(EEName, handcoord[0], handcoord[1], handcoord[2]);
        GCP << handcoord[0], handcoord[1], handcoord[2];
        //PRINT(GCP);
               
        //int graspResult = tryObjectGrasp();
        Eigen::VectorXd goalPose(6);
        //find grasping pose
        bool poseFound = calculateGraspPose(goalPose);
        
        double gap = rrt->getGap(goalPose);
        if (gap < smallGap) {
            smallGap = gap;
            std::cout << "Gap: " << smallGap << ", tree size: " << rrt->configVector.size() << std::endl;
        }
        
        //check whether to do random RRT extension or to go for goal
        float r = (float) rand() * randMult;
        if (r <= probabGrasp) {
            //ECHO("GRASPING");
             return rrt->tryStep(goalPose);
            //pending: check return types
            
        }else {
            //call regular RRT with random goal
            rrt->tryStep();
            return rrt->tryStep(goalPose);
        }
    }
    
    bool GraspRRT::calculateGraspPose(Eigen::VectorXd &pose){
        //1. get target object's location closest to hand's GCP
        double dist = calculateMinDistance(graspPoint);
       
        //2. calculate target orientation
        //first, change graspPoint from global coordinate to hand's coordinate system
        Eigen::Transform<double,3,2> graspTrans;
        world->getRobot(robot)->getBodyNodeTransform(EEName, graspTrans);
        
        //PRINT(graspPoint);
        Eigen::Vector3d targetPoint = graspTrans.linear() * graspPoint;
        //PRINT(targetPoint);
        
        Eigen::Vector3d z_local; z_local << 0.0, 0.0, 1.0;
        targetPoint.normalize();
        //get angle between z axis and target grasp point
        double angle = findAngle(targetPoint, z_local);
        //PRINT(angle);
        
        //get local rotation axis & matrix; get matrix in global coord. system
        Eigen::Vector3d rotAxisLocal; 
        rotAxisLocal = z_local.cross(targetPoint); 
        
        Eigen::Matrix4d rotMat = axisangle2eigen4d(rotAxisLocal, angle);
        Eigen::Matrix4d mat = world->getRobot(robot)->getNode(EEName.c_str())->getWorldTransform() * rotMat;
        mat.block(0,3,3,1) = targetPoint;
        
        
        //NEED TO CHECK BELOW HERE:
        Eigen::VectorXd p(6);
	// The translational error is just the vector  between the actual and the target position
	p.segment(0,3) = mat.block(0,3,3,1);
        //ECHO("RETURN pose1");
        
	Eigen::AngleAxis<double> aa(mat.block<3,3>(0,0));
	p.segment(3,3) = aa.axis()*aa.angle();
        
	pose = p;
        
        //PRINT(pose);
       //PRINT(targetPoint);
        //PRINT(graspPoint);
         
        return true;
    }
    
    double GraspRRT::calculateMinDistance(Eigen::Vector3d &closest){
        //1. get collision meshes and vertices
        //ECHO("CALCULATING CLOSEST POINT");
        
        kinematics::Shape* shape = objectNode->getShape();
        const aiScene* sc = shape->getVizMesh();
        
        if (shape->getCollisionMesh() != NULL) { sc = shape->getCollisionMesh(); }
        double min_distance = -1;
         
        if(sc != NULL){
            const aiNode* nd = sc->mRootNode;
            Eigen::Matrix4d worldTrans = objectNode->getWorldTransform();
            const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[0]];
            
            //find closest vertex
            for(int j=0; j< mesh->mNumVertices; j++){
                
                Eigen::VectorXd vertices(4); vertices <<  mesh->mVertices[j].x, mesh->mVertices[j].y , mesh->mVertices[j].z, 1;
                vertices = worldTrans * vertices; //now vertices turned into 4x1 matrix

                //calculate distance between current vertex and GCP
                Eigen::Vector3d diff; diff << vertices(0,0), vertices(1,0), vertices(2,0);
                diff = diff - GCP;
                
                if(min_distance == -1 || diff.norm() < min_distance){
                    min_distance = diff.norm();
                    closest << vertices(0,0), vertices(1,0), vertices(2,0);
                }
                 
            }
            //ECHO("FOUND CLOSEST VERTEX");
            //PRINT(closest);
        }
        return min_distance;
    }
    
    Eigen::Matrix3d axisangle2eigen3d( const Eigen::Vector3d &axis, double angle ){
	Eigen::AngleAxis<double> aa(angle, axis);
	return aa.matrix();
    }

    Eigen::Matrix4d axisangle2eigen4d(const Eigen::Vector3d &axis, double angle) {
        Eigen::Matrix4d res4 = Eigen::Matrix4d::Identity();
        res4.block(0, 0, 3, 3) = axisangle2eigen3d(axis, angle);
        return res4;
    }
    
    double GraspRRT::findAngle(Eigen::Vector3d a, Eigen::Vector3d b){
        //calculate angle between a & b
        double angle = acos( (a.dot(b))/(a.norm() * b.norm()) );
        return (angle * 180.0 / PI);
    }
    
}

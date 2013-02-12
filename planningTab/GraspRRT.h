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

/** @file GraspRRT.h
 *  @author Juan C. Garcia
 */

#ifndef GraspRRT_H
#define GraspRRT_H

#include <vector>
#include <list>
#include <iostream>
#include <Eigen/Core>
#include <flann/flann.hpp>
#include <planning/RRT.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include "JointMover.h"
namespace robotics {
    class World;
}

//Markers for OpenGL
extern Eigen::Vector3d GCP;
extern Eigen::Vector3d graspPoint;

namespace planning {

    class GraspRRT {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GraspRRT(robotics::World* world, const std::vector<int> &dofs, int robot, const Eigen::VectorXd &start, 
                const Eigen::VectorXd &objectLocation, kinematics::BodyNode* objectNode, std::string mEEName);
        virtual ~GraspRRT();

        void plan(std::list<Eigen::VectorXd> &path);
        RRT::StepResult doCycle();
        int tryObjectGrasp();
        int approachObject(Eigen::VectorXd startConfig, Eigen::VectorXd goalConfig, int loops);
        bool calculateGraspPose(Eigen::VectorXd &pose);
        double calculateGraspScore(Eigen::VectorXd &grasp);
        double calculateMinDistance(Eigen::Vector3d &closest);
        double findAngle(Eigen::Vector3d a, Eigen::Vector3d b);
        
    protected:
        robotics::World* world;
        int robot;
        std::vector<int> dofs;
        Eigen::VectorXd startConfig;
        Eigen::VectorXd objectConfig;
        kinematics::BodyNode* objectNode;
         kinematics::BodyNode* handNode;
        
        flann::Index<flann::L2<double> > index;
        double gcp_offset;
        int cycles;
        bool forceStop;
        bool foundSolution;
        planning::RRT* rrt;
        JointMover* jm;
        std::string EEName;
        double smallGap;
        int max_cycles;
    };
    
    Eigen::Matrix3d axisangle2eigen3d( const Eigen::Vector3d &axis, double angle );
    Eigen::Matrix4d axisangle2eigen4d(const Eigen::Vector3d &axis, double angle);
}

#endif /* GraspRRT_H */

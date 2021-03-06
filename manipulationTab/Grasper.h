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

/** @file Grasper.h
 *  @author Juan C. Garcia
 */

#ifndef GRASPER_H
#define GRASPER_H

#include <vector>
#include <list>
#include <iostream>
#include <Eigen/Core>
#include <flann/flann.hpp>
#include <planning/RRT.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <collision/CollisionSkeleton.h>
#include "JointMover.h"
namespace robotics {
    class World;
}

//Markers for OpenGL Drawing
extern Eigen::Vector3d graspPoint;

namespace planning {

    class Grasper {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Grasper(robotics::World* world, int r, std::string mEEName);
        virtual ~Grasper();
        
        void init(std::vector<int> &dofs, Eigen::VectorXd &start, kinematics::BodyNode* objectNode);
        
        void plan(std::list<Eigen::VectorXd> &path, std::vector<int> &dofs);
        double calculateMinDistance(Eigen::Vector3d &closest);
        bool closeHand(double stepSize, kinematics::BodyNode* target);
        void openHand();
        
    protected:
        robotics::World* world;
        int robot;
        std::vector<int> dofs;
        std::vector<int> hand_dofs;
        Eigen::VectorXd startConfig;
        Eigen::VectorXd objectConfig;
        kinematics::BodyNode* objectNode;
        
        planning::RRT* rrt;
        JointMover* jm;
        std::string EEName;
    };
}

#endif /* Grasper_H */

/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved
 * Author(s): Ana C. Huaman Quispe (modified by Juan C. Garcia) <ahuaman3@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */	

#ifndef _JointMover_H_
#define _JointMover_H_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <dynamics/BodyNodeDynamics.h>
#include <robotics/World.h>

#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

using namespace std;
using namespace Eigen;

// The speed of each joint, note that the joint values are between -120 to
// 120 on the robot arm
const double jointSpeeds = 5.0; // degrees/second

class JointMover {
  private:
    /// Member variables
    double mConfigStep;
    robotics::World &mWorld;
    int mRobotId;
    std::vector<int> mLinks;
    double mWorkspaceThresh;

    dynamics::BodyNodeDynamics* mEENode;
    int mMaxIter;

    

  public:
    JointMover( robotics::World &_world, int _robotId, const std::vector<int> &_links,  std::string _EEName,
        double _configStep = 0.1 ); // 0.046 = 1_degree * sqrt(7)
    MatrixXd GetPseudoInvJac();
    
    // True if could reach "in time". No actual movement!
    bool GoToXYZ( VectorXd _qStart, VectorXd _targetXYZ, VectorXd &_qResult);

    // Returns new configuration q. No actual movement!
    VectorXd OneStepTowardsXYZ( VectorXd _q, VectorXd _targetXYZ);

    // Returns the workspace coordinate for a jointspace coordinate
    VectorXd GetXYZ( VectorXd _q );
    
    double jointSpaceDistance(VectorXd _q1, VectorXd _q2);
    
    VectorXd jointSpaceMovement(VectorXd _qStart, VectorXd _qGoal);

};

#endif


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
 * @file JointMover.cpp
 * @brief Jacobian-based planning handling both translations and rotations
 * @author Juan C. Garcia, Justin Smith in collaboration with Arash Rouhani made modifications 
 *         to original code provided by Ana C. Huaman Quispe 
 */
#include <iostream>
#include <stdlib.h>
#include <robotics/Robot.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <vector>
#include <Eigen/LU>
#include <list>
#include <dart/dynamics/ContactDynamics.h>
#include <dart/collision/CollisionSkeleton.h>
#include <dart/dynamics/SkeletonDynamics.h>
#include "JointMover.h"

using namespace std;
using namespace Eigen;

const double dt = 0.1;
JointMover::JointMover( robotics::World &_world, robotics::Robot* robot, const std::vector<int> &_links,  std::string _EEName,
		       double _configStep)
  : mConfigStep(_configStep), mWorld(_world), mRobot(robot) {

  mLinks = _links;
  mMaxIter = 100;
  mWorkspaceThresh = 0.02;	//was .02
  mEENode = (dynamics::BodyNodeDynamics*)mRobot->getNode(_EEName.c_str());
}

// Method returns either a translational Jacobian, or a full trans+rot Jacobian
MatrixXd JointMover::GetPseudoInvJac() {
  MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size());

  // handle both translations and rotations
  MatrixXd Jacang = mEENode->getJacobianAngular().topRightCorner(3, mLinks.size());
  MatrixXd Jac(Jaclin.rows() + Jacang.rows(), Jaclin.cols());
  Jac << Jaclin, Jacang;
//  std::cout<< "\nJaclin: \n"<<Jaclin << "\nJacang: \n" << Jacang << std::endl;
//  cout << "\ncombined: \n" << Jac;
  
  MatrixXd JacT = Jac.transpose();
  MatrixXd JJt = (Jac*JacT);
  FullPivLU<MatrixXd> lu(JJt);
  MatrixXd Jt = JacT*( lu.inverse() );
   
 // std::cout<< "\nJaclin pseudo inverse: \n"<<Jt << std::endl;  
  return Jt;
}


// Method performs a Jacobian towards specified target; such target could  
// be a 3D vector (X,Y,Z) or a 6D vector (X,Y,Z,R,P,Y)
double JointMover::GoToXYZRPY( VectorXd _qStart, VectorXd _targetXYZRPY, VectorXd &_qResult, std::list<Eigen::VectorXd> &path, double angularL) {
  _qResult = _qStart;
  mRobot->update();

  // GetXYZ also updates the config to _qResult, so Jaclin use an updated value
  VectorXd delta = _targetXYZRPY - GetXYZRPY(_qResult); 
  delta.tail(3) = delta.tail(3) * angularL;
  
 // cout << "\nangularL: " << angularL << "\n";
  
 // cout << "Mconfigstep: " << mConfigStep << "\n";
  mConfigStep = .1;
  int iter = 0;
  while( delta.norm() > mWorkspaceThresh && iter < mMaxIter ) {
	//delta = .005 * delta/delta.head(3).norm();		//alternative way to limit xyz motion, same result
	VectorXd dConfig = GetPseudoInvJac()*delta;
  	
  	double n = dConfig.norm();
  //	cout << "Iter: " << iter << ", dConfig norm: " << n << "\n";
  	
  	if( n > mConfigStep ) {
      
      dConfig = dConfig *(mConfigStep/n);
  //    cout << "mconfigstep: " << mConfigStep << ", new dConfig (other too big): \n" << dConfig << "\n";
    }
    
    applyJointLimits(_qResult, dConfig);
    
    _qResult = _qResult + dConfig;
    
    path.push_back(_qResult);
    //mRobot->update();
    delta = (_targetXYZRPY - GetXYZRPY(_qResult) );
    
  //  cout << "\nDelta:\n" << delta << "\n\n";    
    
    //convert angular portion into meters by multiplying by length of translation vector between wrist and grasp point. Prevents angular part form dominating and rejecting good solutions. Courtesy of Michael Grey
  	delta.tail(3) = delta.tail(3) * angularL;		
    
 //   cout << "New Delta:\n" << delta << "\n\n";
    //PRINT(delta.norm());
    iter++;
  }
  mRobot->update();
  return delta.norm();
}

// Method to compute location of given a vector of joint configurations: 6D(X,Y,Z,R,P,Y)
VectorXd JointMover::GetXYZRPY( VectorXd _q) {
  // Get current XYZ position
  mRobot->setConfig(mLinks, _q);
  mRobot->update();
  
  MatrixXd qTransform = mEENode->getWorldTransform();
  VectorXd qXYZRPY(6);
  
  //return a 6D vector if both x,y,z and r,p,y must be computed
      Matrix3d rotM = qTransform.topLeftCorner(3,3);
      VectorXd rot = rotM.eulerAngles(0,1,2);
      qXYZRPY << qTransform(0,3), qTransform(1,3), qTransform(2,3), rot(0), rot(1), rot(2);

  return qXYZRPY;
}

void JointMover::applyJointLimits(VectorXd qCurr, VectorXd &qStep)
{
	for(int i = 0; i < mLinks.size(); i++)
	{
		int jointNum = mLinks.at(i);
		//cout << "qstep[" << i << "]: " << qStep[i] << endl;
		
		if(qStep[i] > 0)
		{
			double qMax = mRobot->getDof(jointNum)->getMax();
			double maxStep = qMax - qCurr[i];

			if(qStep[i] > maxStep)
			{
			cout << "qMax: " << qMax << ", current: " << qCurr[i] << "=maxStep: " << maxStep << endl;
				if(i == 1)
				{
					cout << "qMax = " << qMax << ", current: " << qCurr[i] << ", old qStep: " << qStep[i] << ", new: " << maxStep << "\n";
				}
				qStep[i] = maxStep;
			}
		}
		else if(qStep[i] < 0)
		{
			double qMin = mRobot->getDof(jointNum)->getMin();
			double maxStep = qMin - qCurr[i];
			if(qStep[i] < maxStep)
				qStep[i] = maxStep;
		} 
	}
}


// Method to compute distance between configs in joint space
double JointMover::jointSpaceDistance(VectorXd _q1, VectorXd _q2) {
  // This is the infinite norm
  return (_q2-_q1).cwiseAbs().maxCoeff();
}

// Method to achieve simple joints movements
VectorXd JointMover::jointSpaceMovement(VectorXd _qStart, VectorXd _qGoal) {
  VectorXd diff = (_qGoal-_qStart);
  for(int i = 0; i < diff.size(); i++){
    diff[i] = max(-jointSpeeds*dt, min(jointSpeeds*dt, diff[i]));
  }
  return _qStart + diff;
}

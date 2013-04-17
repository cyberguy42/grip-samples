/**
 * @file JT_Follower.cpp
 * @brief Read the .h heading for details :)
 * @author A.H.Q.
 * @date March 07th, 2012
 */

#include <robotics/Robot.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "JTFollower.h"

#include <Eigen/LU>

/**
 * @function JTFollower
 * @brief Constructor
 */
JTFollower::JTFollower() {
  mCopyWorld = false;
  mWorld = NULL;
}

/**
 * @function JTFollower
 * @brief Constructor
 */
JTFollower::JTFollower( robotics::World &_world, 
                        bool _copyWorld, 
			double _configStep ) {
  
  mCopyWorld = _copyWorld;
  
  if( mCopyWorld ) {
    printf( "Not implemented yet. Sorry -- achq \n" );
  } else {
    mWorld = &_world;
  }
  
  mConfigStep = _configStep;
}

/**
 * @function ~JTFollower
 * @brief Destructor
 */
JTFollower::~JTFollower() {
  
  if( mCopyWorld ) {
    delete mWorld;
  }
}

/**
 * @function init
 */
void JTFollower::init( int _robotId,
		       const std::vector<int> &_links,
		       std::string _EEName,
		       int _EEId,
		       double _res ) {
  
  mRobotId = _robotId;
  mLinks = _links;
  
  mMaxIter = 200;
  mWorkspaceThresh = _res; // An error of half the resolution
  mEENode = (dynamics::BodyNodeDynamics*) mWorld->getRobot(mRobotId)->getNode( _EEName.c_str() );
  mEEId = _EEId;  
}

/**
 * @function planPath
 * @brief Main function
 */
std::vector< Eigen::VectorXd > JTFollower::PlanPath( const Eigen::VectorXd &_start,  
						     const std::vector<Eigen::VectorXd> &_workspacePath ) {
  

  //-- Follow the path
  std::vector< Eigen::VectorXd > configPath;
  Eigen::VectorXd q;
  
  int numPoints = _workspacePath.size();
  
  //-- Initialize	
  q = _start;
  
  for( size_t i = 1; i < numPoints; ++i ) { // start from 1 since 0 is the current start position
    if( GoTo6D( q, _workspacePath[i], configPath ) == false ) {
      printf(" --(x) An error here, stop following path \n"); break;
    }
  } 
  
  printf("End of Plan Path \n");
  return configPath;
  
}

/**
 * @function GetPseudoInvJac   
 */
Eigen::MatrixXd JTFollower::GetPseudoInvJac( Eigen::VectorXd _q ) {
  printf("Num Dependent DOF minus 6D0F is : %d \n", mEENode->getNumDependentDofs() - 6 );
  Eigen::MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
  Eigen::MatrixXd Jacang = mEENode->getJacobianAngular().topRightCorner( 3, mLinks.size() );
  std::cout<< "Jaclin: \n"<<Jaclin << "Jacang: \n" << Jacang << std::endl;
  
  Eigen::MatrixXd Jac(Jaclin.rows() + Jacang.rows(), Jaclin.cols());
  Jac << Jaclin, Jacang;
 
  Eigen::MatrixXd JacT = Jac.transpose();
  Eigen::MatrixXd Jt;
  Eigen::MatrixXd JJt = (Jac*JacT);
  Eigen::FullPivLU<Eigen::MatrixXd> lu(JJt);
  Jt = JacT*( lu.inverse() );
  std::cout<< "Jaclin pseudo inverse: \n"<<Jt << std::endl;  
  return Jt;
}

/**
 * @function GoToXYZ
 */
bool JTFollower::GoToXYZ( Eigen::VectorXd &_q, 
			  Eigen::VectorXd _targetXYZ, 
			  std::vector<Eigen::VectorXd> &_workspacePath ) {

  Eigen::VectorXd dXYZ;
  Eigen::VectorXd dConfig;
  int iter;
  mWorld->getRobot(mRobotId)->update();
  
  //-- Initialize
  dXYZ = ( _targetXYZ - GetXYZ(_q) ); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  iter = 0;
  printf("New call to GoToXYZ: dXYZ: %f  \n", dXYZ.norm() );
  while( dXYZ.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    printf("XYZ Error: %f \n", dXYZ.norm() );
    Eigen::MatrixXd Jt = GetPseudoInvJac(_q);
    dConfig = Jt*dXYZ;
    printf("dConfig : %.3f \n", dConfig.norm() );
    if( dConfig.norm() > mConfigStep ) {
      double n = dConfig.norm();
      dConfig = dConfig *(mConfigStep/n);
      printf("NEW dConfig : %.3f \n", dConfig.norm() );
    }
    _q = _q + dConfig;
    _workspacePath.push_back( _q );
    
    dXYZ = (_targetXYZ - GetXYZ(_q) );
    iter++;
  }
  
  if( iter >= mMaxIter ) { return false; }
  else { return true; }
  
}

/**
 * @function GoTo6D
 */
bool JTFollower::GoTo6D( Eigen::VectorXd &_q, 
			  Eigen::VectorXd _target6D, 
			  std::vector<Eigen::VectorXd> &_workspacePath ) {

  Eigen::VectorXd d6D;
  Eigen::VectorXd dConfig;
  int iter;
  mWorld->getRobot(mRobotId)->update();
  
  //-- Initialize
  d6D = ( _target6D - Get6D(_q) ); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  iter = 0;
  printf("New call to GoTo6D: d6D: %f  \n", d6D.norm() );
  while( d6D.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    printf("6D Error: %f \n", d6D.norm() );
    Eigen::MatrixXd Jt = GetPseudoInvJac(_q);
    dConfig = Jt*d6D;
    printf("dConfig : %.3f \n", dConfig.norm() );
    if( dConfig.norm() > mConfigStep ) {
      double n = dConfig.norm();
      dConfig = dConfig *(mConfigStep/n);
      printf("NEW dConfig : %.3f \n", dConfig.norm() );
    }
    _q = _q + dConfig;
    _workspacePath.push_back( _q );
    
    d6D = (_target6D - Get6D(_q) );
    iter++;
  }
  
  if( iter >= mMaxIter ) { return false; }
  else { return true; }
  
}


/**
 * @function GetXYZ
 */
Eigen::VectorXd JTFollower::GetXYZ( Eigen::VectorXd _q ) {

	
  // Get current XYZ position
  mWorld->getRobot(mRobotId)->setConfig( mLinks, _q );
  mWorld->getRobot(mRobotId)->update();
  
  Eigen::MatrixXd qTransform = mEENode->getWorldTransform();
  Eigen::VectorXd qXYZ(3); qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3);
  
  return qXYZ;
}
 


/**
 * @function Get6D
 */
Eigen::VectorXd JTFollower::Get6D( Eigen::VectorXd _q ) {

	
  // Get current XYZ position and orientation
  mWorld->getRobot(mRobotId)->setConfig( mLinks, _q );
  mWorld->getRobot(mRobotId)->update();
  
  Eigen::MatrixXd qTransform = mEENode->getWorldTransform();
  Eigen::Matrix3d rotationM = qTransform.topLeftCorner(3,3);
  Eigen::VectorXd rotation = rotationM.eulerAngles(0,1,2);
  Eigen::VectorXd q6D(6); q6D << qTransform(0,3), qTransform(1,3), qTransform(2,3), rotation(0), rotation(1), rotation(2);
  
  return q6D;
}
 

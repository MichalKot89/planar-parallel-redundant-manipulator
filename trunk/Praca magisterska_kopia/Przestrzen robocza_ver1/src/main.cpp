#include "modul.h"

using namespace std;

int main()
{
  
  Manipulator M(30, 40, 50, 30, 30, 
		M_PI/3, 0, 2*M_PI/3, 
		M_PI/2, 0, M_PI,
		2*M_PI/3, M_PI/3, M_PI,
		-2*M_PI/3, -M_PI, 0);
  
  //M.DoublePendulumInverseKinematics(50, 60);
  /*DoublePendulum DPa, DPb;
  DPa.q1=M_PI/3;
  DPa.q2=M_PI/2;
  DPb.q1=2*M_PI/3;
  DPb.q2=-2*M_PI/3;
  Position P = M.ForwardKinematics(DPa, DPb);
  cout << P.x << " " << P.y << " " << P.alpha*180/M_PI << endl;
  
  M.InverseKinematics(P, 2*M_PI/3);
  cout << M.Qb().q1*180/M_PI << " " << M.Qb().q2*180/M_PI << endl; 
  cout << M.Qa().q1*180/M_PI << " " << M.Qa().q2*180/M_PI << endl; */
  
  M.WorkSpaceUsingInverseKinematics();
  M.WorkSpaceUsingForwardKinematics();
  
  return (0);
}
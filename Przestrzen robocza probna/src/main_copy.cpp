#include "modul.h"
#define BAR_STEP 5
#define L_MIN 0
#define L_MAX 50
//#define l1_MIN 10
#define l1_MIN 50
#define l1_MAX 50
#define l2_MIN 50
//#define l2_MIN 10
#define l2_MAX 50
#define l3_MIN 50
// #define l3_MIN 10
#define l3_MAX 50
// #define l4_MIN 10
#define l4_MIN 50
#define l4_MAX 50
#define SUM_OF_ARMS 100

using namespace std;

void FindMaximumCapacity()
{
  double Qa1q = M_PI/3;
  double Qa1qmin = 0;
  double Qa1qmax = 2*M_PI/3;
  
  double Qa2q = M_PI/2;
  double Qa2qmin = 0;
  double Qa2qmax = M_PI;
  
  double Qb1q = 2*M_PI/3;
  double Qb1qmin = M_PI/3;
  double Qb1qmax = M_PI;
  
  double Qb2q = -2*M_PI/3;
  double Qb2qmin = -M_PI;
  double Qb2qmax = 0;
  
  unsigned int Capacity=0;
  unsigned int MaxCapacityInv=0;
  unsigned int MaxCapacityFor=0;
  Manipulator* MaxCapacityManipulatorInv=NULL;
  Manipulator* MaxCapacityManipulatorFor=NULL;
  bool flague=true;
  
  
  for(double i=L_MIN; i<=L_MAX; i+=BAR_STEP)
  {
    for(double j=l1_MIN; j<=l1_MAX; j+=BAR_STEP)
    {
      for(double k=l2_MIN; k<=l2_MAX; k+=BAR_STEP)
      {
	for(double m=l3_MIN; m<=l3_MAX; m+=BAR_STEP)
	{
	  for(double n=l4_MIN; n<=l4_MAX; n+=BAR_STEP)
	  {
	    flague=true;
	    
	    Manipulator* M = new Manipulator(i, j, k, m, n, 
					     Qa1q, Qa1qmin, Qa1qmax,
					     Qa2q, Qa2qmin, Qa2qmax,
					     Qb1q, Qb1qmin, Qb1qmax,
					     Qb2q, Qb2qmin, Qb2qmax);
	   
	    Capacity = M->WorkSpaceUsingForwardKinematics();
	    std::cout << i << " " << Capacity << std::endl;/*
	    
	    Capacity = M->WorkSpaceUsingInverseKinematics();
	    if(Capacity > MaxCapacityInv)
	    {
	      // only if both kinematics point on different objects (manipulators)
	      if(MaxCapacityManipulatorInv != MaxCapacityManipulatorFor)
		delete MaxCapacityManipulatorInv;
	      
	      MaxCapacityManipulatorInv = M;
	      MaxCapacityInv = Capacity;
//std::cout << "MaxCapacityInv: " << MaxCapacityInv << std::endl;
	      flague=false;
	    }
	    
	    Capacity = M->WorkSpaceUsingForwardKinematics();
	    if(Capacity > MaxCapacityFor)
	    {
	      // only if both kinematics point on different objects (manipulators)
	      if(MaxCapacityManipulatorInv != MaxCapacityManipulatorFor)
		delete MaxCapacityManipulatorFor;
	      
	      MaxCapacityManipulatorFor = M;
	      MaxCapacityFor = Capacity;
//std::cout << "MaxCapacityFor: " << MaxCapacityFor << std::endl;
	      flague=false;
	    }*/
	    if(flague)
	      delete M;
	    
	  }
	}  	
      }        
    }    
    
    
  }
  
}


int main()
{
  /* Invoke Manipulator constructor:
   * Arguments:
   * L - the distance from the arm beginning to the beginning of the workspace
   * l1 - length of the first arm of double pendulum
   * l2 - length of the second arm of double pendulum
   * l3 - length of the joining arm
   * l4 - length of the last bar
   * Qa1q, Qa1qmin, Qa1qmax - actual, minimum and maximum value of first right joint
   * Qa2q, Qa2qmin, Qa2qmax - actual, minimum and maximum value of second right joint
   * Qb1q, Qb1qmin, Qb1qmax - actual, minimum and maximum value of first left joint
   * Qb2q, Qb2qmin, Qb2qmax - actual, minimum and maximum value of second left joint
   */
  
  Manipulator M(30, 30, 50, 50, 30, 
		M_PI/3, 0, 2*M_PI/3, 
		M_PI/2, 0, M_PI,
		2*M_PI/3, M_PI/3, M_PI,
		-2*M_PI/3, -M_PI, 0);
  
  //M.WorkSpaceUsingInverseKinematics();
  //M.WorkSpaceUsingForwardKinematics();
  
  FindMaximumCapacity();
  
  return (0);
}
#include "modul.h"
#define BAR_STEP 10
#define BAR_STEP_L 1
#define L_MIN 0
#define L_MAX 50
#define l1_MIN 0
#define l1_MAX 50
#define l2_MIN 0
#define l2_MAX 50
#define l3_MIN 0
#define l3_MAX 50
#define l4_MIN 0
#define l4_MAX 50
#define SUM_OF_ARMS 100

using namespace std;

void FindMaximumCapacity(double L)
{ 
  unsigned int Capacity=0;
  unsigned int MaxCapacityInv=0;
  unsigned int MaxCapacityFor=0;
  Manipulator* MaxCapacityManipulatorInv=NULL;
  Manipulator* MaxCapacityManipulatorFor=NULL;
  bool flague=true;

    for(double j=l1_MIN; j<=l1_MAX; j+=BAR_STEP)
    {
      for(double k=l2_MIN; k<=l2_MAX; k+=BAR_STEP)
      {
	for(double m=l3_MIN; m<=l3_MAX; m+=BAR_STEP)
	{
	  for(double n=l4_MIN; n<=l4_MAX; n+=BAR_STEP)
	  {
	    if(j+k+m+n != SUM_OF_ARMS) // to make the limit
	      break;
	    
	    flague=true;
	    
	    Manipulator* M = new Manipulator(L, j, k, m, n);
	    /*
	    Capacity = M->WorkSpaceUsingInverseKinematics();
	    
	    if(Capacity > MaxCapacityInv)
	    {
	      // only if both kinematics point on different objects (manipulators)
	      if(MaxCapacityManipulatorInv != MaxCapacityManipulatorFor)
		delete MaxCapacityManipulatorInv;
	      
	      MaxCapacityManipulatorInv = M;
	      MaxCapacityInv = Capacity;
std::cout << "MaxCapacityInv: " << MaxCapacityInv << std::endl;
	      flague=false;
	    }
	    */
	    Capacity = M->WorkSpaceUsingForwardKinematics();
	    if(Capacity > MaxCapacityFor)
	    {
	      // only if both kinematics point on different objects (manipulators)
	      if(MaxCapacityManipulatorInv != MaxCapacityManipulatorFor)
		delete MaxCapacityManipulatorFor;
	      
	      MaxCapacityManipulatorFor = M;
	      MaxCapacityFor = Capacity;
std::cout << "MaxCapacityFor: " << MaxCapacityFor << std::endl;
	      flague=false;
	    }
	    if(flague)
	      delete M;
	    
	  }
	}  	
      }        
    }/*
std::cout << "Inverse kinematics" << std::endl;
std::cout << "L1: " << MaxCapacityManipulatorInv->l1() << std::endl;
std::cout << "L2: " << MaxCapacityManipulatorInv->l2() << std::endl;
std::cout << "L3: " << MaxCapacityManipulatorInv->l3() << std::endl;
std::cout << "L4: " << MaxCapacityManipulatorInv->l4() << std::endl;
*/
std::cout << "Forward kinematics" << std::endl;
std::cout << "L1: " << MaxCapacityManipulatorFor->l1() << std::endl;
std::cout << "L2: " << MaxCapacityManipulatorFor->l2() << std::endl;
std::cout << "L3: " << MaxCapacityManipulatorFor->l3() << std::endl;
std::cout << "L4: " << MaxCapacityManipulatorFor->l4() << std::endl;

delete MaxCapacityManipulatorInv;
delete MaxCapacityManipulatorFor;

}

void FindBestL(double l1, double l2, double l3, double l4)
{
  unsigned int Capacity=0;
  
  for(double i=L_MIN; i<=L_MAX; i+=BAR_STEP_L)
  {
    Manipulator* M = new Manipulator(i, l1, l2, l3, l4);

    Capacity = M->WorkSpaceUsingForwardKinematics();
    std::cout << i << " " << Capacity << ";" << std::endl;
    delete M;
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
  
  /*Manipulator M(30, 30, 50, 50, 30, 
		M_PI/3, 0, 2*M_PI/3, 
		M_PI/2, 0, M_PI,
		2*M_PI/3, M_PI/3, M_PI,
		-2*M_PI/3, -M_PI, 0);
  
  std::cout << M.WorkSpaceUsingInverseKinematics() << std::endl;
  std::cout << M.WorkSpaceUsingForwardKinematics() << std::endl;
  */
  FindMaximumCapacity(30);
  //FindBestL(20, 30, 50, 1);
  
  return (0);
}
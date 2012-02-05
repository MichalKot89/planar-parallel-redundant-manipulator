#include "modul.h"
#include <cmath>
#define CONSTANT_ARM_TO_EFFECTOR_ANGLE 2*M_PI/3
#define STEP 0.1
#define SPACE_GRID 5
#define SPACE_X_MIN -200
#define SPACE_X_MAX 200
#define SPACE_Y_MAX 200
#define SPACE_Y_MIN 0
#define MIN_EFF_ORIENT 0
#define MAX_EFF_ORIENT M_PI
#define INVERSE_ANGLE_STEP 0.05
#define MIN_FORCE_ANGLE M_PI/2
#define MAX_FORCE_ANGLE M_PI

using namespace std;

/*
 * Method: DoublePendulumInverseKinematics
 *
 * This method is used to calculate inverse kinematics of two double
 * pendulums, which are both arms of the robot. As an argument it takes
 * X,Y coordinates of pendulums endings and returns true or false value,
 * whether the operation succeded or not
 * It also stores the joints values resulted in Inverse Kinematics
 * in manipulators Joints object
 *
 */


bool Manipulator::DoublePendulumInverseKinematics(double dxa, double dya, double dxb, double dyb)
{
  double Odl=sqrt((dxa-L())*(dxa-L()) + dya*dya);
  
  if(Odl>=l1()+l2())
    return false;
  double alpha=atan2(dya, dxa-L());
  Q().Qa2().q(M_PI-acos((l1()*l1()+l2()*l2()-Odl*Odl)/(2*l1()*l2())));
  Q().Qa1().q(alpha-acos((l1()*l1()-l2()*l2()+Odl*Odl)/(2*l1()*Odl)));

  Odl=sqrt((dxb+L())*(dxb+L()) + dyb*dyb);
  if(Odl>=l1()+l2())
    return false;
  alpha=atan2(dyb, dxb+L());
  Q().Qb2().q(acos((l1()*l1()+l2()*l2()-Odl*Odl)/(2*l1()*l2()))-M_PI);
  Q().Qb1().q(alpha+acos((l1()*l1()-l2()*l2()+Odl*Odl)/(2*l1()*Odl)));
  
  return true;
}

/*
 * Method: InverseKinematics
 *
 * This method calculates endings of double pendulums 
 * being given as an arguments
 * a position of an effector and the angle between
 * effector arm and right double pendulum
 * It starts method DoublePendulumInverseKinematics 
 * and returns whether it returned true or false
 *
 */

bool Manipulator::InverseKinematics(Position P, double force_angle)
{

  P.x = P.x-l4()*cos(P.alpha);
  P.y = P.y-l4()*sin(P.alpha);
  
  double alpha=3*M_PI/2-CONSTANT_ARM_TO_EFFECTOR_ANGLE-P.alpha;
  double beta=force_angle-P.alpha;

  double dxa=P.x+l3()*cos(beta);
  double dya=P.y-l3()*sin(beta);
  double dxb=P.x-l3()*sin(alpha);
  double dyb=P.y-l3()*cos(alpha);  
  

  
  if(!DoublePendulumInverseKinematics(dxa, dya, dxb, dyb))
    return false;
					 
  if(!ValidateConfiguration(dxa, dya, dxb, dyb, P.x, P.y))
    return false;
  
  return true;
}

/*
 * Method: ForwardKinematics
 *
 * This method calculates X,Y coordinates and orientation
 * of an effector for given Joint values
 * The calculation is done using simple geometry and
 * the position is returned by the function
 *
 */


bool Manipulator::ForwardKinematics(Position& P) {
  double dxa=L()+l1()*cos(Q().Qa1().q())+l2()*cos(Q().Qa1().q()+Q().Qa2().q());
  double dya=l1()*sin(Q().Qa1().q())+l2()*sin(Q().Qa1().q()+Q().Qa2().q());
  double dxb=-L()+l1()*cos(Q().Qb1().q())+l2()*cos(Q().Qb1().q()+Q().Qb2().q());
  double dyb=l1()*sin(Q().Qb1().q())+l2()*sin(Q().Qb1().q()+Q().Qb2().q());
  double alpha=atan2(dya-dyb, dxa-dxb);

  P.x=(dxa+dxb)/2;
  P.y=(dya+dyb)/2;

  double dist=sqrt(pow(dxa-dxb, 2)+pow(dya-dyb, 2)); // we count the distance between two pendulums
  if(l3()<=dist/2) // we check if the distance between pendulums ends is not too big
    return false;

  dist=sqrt(pow(l3(), 2)-pow(dist/2, 2)); // we count the distance from pendulums middle to the conjunction point
  P.x=P.x+dist*cos(M_PI/2+alpha);
  P.y=P.y+dist*sin(M_PI/2+alpha);

  if(!ValidateConfiguration(dxa, dya, dxb, dyb, P.x, P.y))
    return false;
  
  // this part should be changed when manipulator will be simetric
  alpha=atan2(P.x-dxb, P.y-dyb);
  P.alpha=3*M_PI/2-CONSTANT_ARM_TO_EFFECTOR_ANGLE-alpha;
  //

  P.x=P.x+cos(P.alpha)*l4();
  P.y=P.y+sin(P.alpha)*l4();

  return true;

}

bool Manipulator::ValidateConfiguration(double dxa, double dya, double dxb, double dyb, double cnx, double cny)
{
  double alfa, beta;
  
  if(Q().Qa1().q()<Q().Qa1().q_min() || Q().Qa1().q()>Q().Qa1().q_max())
    return false;
  if(Q().Qa2().q()<Q().Qa2().q_min() || Q().Qa2().q()>Q().Qa2().q_max())
    return false;
  
  if(Q().Qb1().q()<Q().Qb1().q_min() || Q().Qb1().q()>Q().Qb1().q_max())
    return false;
  if(Q().Qb2().q()<Q().Qb2().q_min() || Q().Qb2().q()>Q().Qb2().q_max())
    return false;
  
  // here we check if joint values are not NaN
  if(Q().Qb2().q()!=Q().Qb2().q() || Q().Qb1().q()!=Q().Qb1().q() || Q().Qa1().q()!=Q().Qa1().q() || Q().Qa2().q()!=Q().Qa2().q())
    return false;
  
  if(dxa<=dxb)
    return false;
  
  if(l3()<=sqrt(pow(dxa-dxb, 2)+pow(dya-dyb, 2))/2) // we check if the distance between pendulums ends is not too big
    return false;
  
  alfa=atan2(cny-dya, dxa-cnx);
  beta=atan2(cny-dya, cnx-dxb);
  if(alfa+beta<=0)
    return false;
  if((M_PI-alfa)-(Q().Qa1().q()+Q().Qa2().q())<=0)
    return false;
  if((Q().Qb1().q()+Q().Qb2().q())-beta<=0)
    return false; 
  
  
  return true;
}


/*
 * Method: WorkSpaceUsingForwardKinematics
 *
 * This method finds manipulators workspace by
 * searching through entire configuration space
 * It checks each possible joint configuration
 * and marks spots which manipulator is able to achieve
 *
 */

unsigned int Manipulator::WorkSpaceUsingForwardKinematics()
{
  unsigned int Space_Size_X = ceil((SPACE_X_MAX - SPACE_X_MIN) / SPACE_GRID);
  unsigned int Space_Size_Y = ceil((SPACE_Y_MAX - SPACE_Y_MIN) / SPACE_GRID);
  double** Space = new double*[Space_Size_Y];
  for(unsigned int i=0; i<Space_Size_Y; i++)
  {
    Space[i] = new double[Space_Size_X];
    for(unsigned int j=0; j<Space_Size_X; j++)
      Space[i][j]=0;
  }

  Position P;

  Q().Qa1().q(Q().Qa1().q_min());
  Q().Qa2().q(Q().Qa2().q_min());
  Q().Qb1().q(Q().Qb1().q_min());
  Q().Qb2().q(Q().Qb2().q_min());
  
  for(Q().Qa1().q(Q().Qa1().q_min()); Q().Qa1().q()<Q().Qa1().q_max(); Q().Qa1().q(Q().Qa1().q()+STEP))
  {
    for(Q().Qa2().q(Q().Qa2().q_min()); Q().Qa2().q()<Q().Qa2().q_max(); Q().Qa2().q(Q().Qa2().q()+STEP))
    {
      for(Q().Qb1().q(Q().Qb1().q_min()); Q().Qb1().q()<Q().Qb1().q_max(); Q().Qb1().q(Q().Qb1().q()+STEP))
      {
	for(Q().Qb2().q(Q().Qb2().q_min()); Q().Qb2().q()<Q().Qb2().q_max(); Q().Qb2().q(Q().Qb2().q()+STEP))
	{
	  Position P;
	  if(!ForwardKinematics(P))
	    continue;
	  
	  int x = ((int)ceil(P.x)-SPACE_X_MIN)/SPACE_GRID;
	  int y = ((int)ceil(P.y)-SPACE_Y_MIN)/SPACE_GRID;
	  Space[y][x] = 1;
	    /*if(y==0 && x<40)
	      cout << endl << Q().Qa1().q()*180/M_PI << " " << Q().Qa2().q()*180/M_PI << " " << 
	      Q().Qb1().q()*180/M_PI << " " << Q().Qb2().q()*180/M_PI << " " << 
	      l1()*sin(Q().Qa1().q())+l2()*sin(Q().Qa1().q()+Q().Qa2().q()) << " " << l1()*sin(Q().Qb1().q())+l2()*sin(Q().Qb1().q()+Q().Qb2().q()) 
	      << " " << P.y << endl;*/
	}
      }
    }
  }
  
  //PrintWorkSpace(Space, Space_Size_Y, Space_Size_X);
  
  unsigned int Capacity = FindWorkspaceOutline(Space, Space_Size_Y, Space_Size_X);
  
  for(unsigned int i=0; i<Space_Size_Y; i++)
    delete [] Space[i];
  delete [] Space;
  
  return Capacity;
}

/*
 * Method: WorkSpaceUsingInverseKinematics
 *
 * This method finds manipulators workspace by
 * searching through entire given workspace
 * and marking only these locations for which
 * inverse kinematics has successfully found a solution
 * Checking is done for a range of possible orientations
 * and a range of possible force angles
 *
 */

unsigned int Manipulator::WorkSpaceUsingInverseKinematics()
{
  unsigned int Space_Size_X = ceil((SPACE_X_MAX - SPACE_X_MIN) / SPACE_GRID);
  unsigned int Space_Size_Y = ceil((SPACE_Y_MAX - SPACE_Y_MIN) / SPACE_GRID);
  double** Space = new double*[Space_Size_Y];
  
  for(unsigned int i=0; i<Space_Size_Y; i++) {
    Space[i] = new double[Space_Size_X];
    for(unsigned int j=0; j<Space_Size_X; j++)
      Space[i][j]=0;
  }

  Position P;
  bool flague;
  for(unsigned int i=0; i<Space_Size_Y; i++) {
    P.y=((double)i)*SPACE_GRID+SPACE_Y_MIN;
    for(unsigned int j=0; j<Space_Size_X; j++) {
      P.x=((double)j)*SPACE_GRID+SPACE_X_MIN;
      flague=false;
      for(double k=MIN_EFF_ORIENT; k<MAX_EFF_ORIENT; k+=INVERSE_ANGLE_STEP) {
	P.alpha = k;
	for(double l=MIN_FORCE_ANGLE; l<MAX_FORCE_ANGLE; l+=INVERSE_ANGLE_STEP) { 
	  flague=InverseKinematics(P, l);
	  if(flague) {
	    Space[i][j]=1;
	    break;
	  }
	}
	if(flague)
	  break;
      }
    }
  }
  
  //PrintWorkSpace(Space, Space_Size_Y, Space_Size_X);

  unsigned int Capacity = FindWorkspaceOutline(Space, Space_Size_Y, Space_Size_X);
  for(unsigned int i=0; i<Space_Size_Y; i++)
    delete [] Space[i];
  delete [] Space;
  
  return Capacity;
}

/*
 * Method: PrintWorkSpace
 *
 * This method prints found workspace for given
 * matrix Space
 *
 */

void Manipulator::PrintWorkSpace(double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X)
{
  std::cout << std::endl << std::endl;
  for(unsigned int i=0; i<Space_Size_Y; i++)
  {
    for(unsigned int j=0; j<Space_Size_X; j++)
    {
      std::cout << Space[i][j];
    }
    std::cout << std::endl;
  }
}


/*
 * Method: FindWorkspaceOutlinee
 *
 * This method finds outline of a given space by finding 
 * and choosing only the margin ones
 * It also saves data to a output file
 *
 */

unsigned int Manipulator::FindWorkspaceOutline(double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X)
{
  ofstream outfile;
  outfile.open("output2.txt");
  
  bool flague=false;
  unsigned int i;
  unsigned int j;
  unsigned int k;
  unsigned int starti=0;
  unsigned int startj=0;
  
  double** OutlineSpace = new double*[Space_Size_Y];
  
  for(unsigned int l=0; l<Space_Size_Y; l++) {
    OutlineSpace[l] = new double[Space_Size_X];
    for(unsigned int m=0; m<Space_Size_X; m++)
      OutlineSpace[l][m]=0;
  }

  for(j=0; j<Space_Size_X; j++)
  {
    for(i=0; i<Space_Size_Y; i++)
    {
      if(Space[i][j]>0)
      {
	if(Space[i-1][j-1]>0 || Space[i-1][j]>0 || Space[i-1][j+1]>0 
	  || Space[i+1][j-1]>0 || Space[i+1][j]>0 || Space[i+1][j+1]>0 
	  || Space[i][j-1]>0 || Space[i][j+1]>0)
	{
	  starti=i;
	  startj=j;
	  OutlineSpace[i][j]=1;
	  flague=true;
	  break;
	}
      }
    }
    if(flague)
      break;
  } 
 
  if(flague==false)
    return 0;
  outfile << SPACE_X_MIN + (double)j * SPACE_GRID << " " << SPACE_Y_MIN + i * SPACE_GRID << endl;
  flague=false;
  do {
    if(flague==false) // we go right
    {
      if(Space[i][j+1]==1)
      {
	do {
	  if(i<=0)
	    break;
	  i--;
	}
	while(Space[i][j+1]==1);
	if(i>0)
	  i++;
	j++;
	OutlineSpace[i][j]=1;
	outfile << SPACE_X_MIN + (double)j * SPACE_GRID << " " << SPACE_Y_MIN + i * SPACE_GRID << endl;
	if(j>=Space_Size_X-1)
	  flague=true;
      }
      else
      {
	k=i;
	do {
	  if(k>=Space_Size_Y-1)
	  { // if there is no 1 below
	    flague=true;
	    break;	
	  }
	  k++;
	}
	while(Space[k][j+1]==0);
	if(flague)
	{ // we search for row with last 1 in last column with ones
	  while(i<Space_Size_Y-1 && Space[i+1][j]==1)
	    i++;
	  if(i<Space_Size_Y-1)
	  {
	    OutlineSpace[i][j]=1;
	    outfile << SPACE_X_MIN + (double)j * SPACE_GRID << " " << SPACE_Y_MIN + i * SPACE_GRID << endl;
	  }
	}
	else
	{
	  i=k;
	  j++;
	  OutlineSpace[i][j]=1;
	  outfile << SPACE_X_MIN + (double)j * SPACE_GRID << " " << SPACE_Y_MIN + i * SPACE_GRID << endl;
	}
      }
    }
    else // we go left
    {
      if(Space[i][j-1]==1)
      { 
	do {
	  if(i>=Space_Size_Y-1)
	    break;
	  i++;
	}
	while(Space[i][j-1]==1);
	if(i<Space_Size_Y-1)
	  i--;
	j--;
	OutlineSpace[i][j]=1;
	outfile << SPACE_X_MIN + (double)j * SPACE_GRID << " " << SPACE_Y_MIN + i * SPACE_GRID << endl;
	if(j<=0)
	  break;
      }
      else
      {
	k=i;
	do {
	  if(k<=0)
	  { // the end of this method
	    i=starti;
	    j=startj;
	    break;	
	  }
	  k--;
	}
	while(Space[k][j-1]==0);
	if(i!=starti && j!=startj)
	{
	  i=k;
	  j--;
	  OutlineSpace[i][j]=1;
	  outfile << SPACE_X_MIN + (double)j * SPACE_GRID << " " << SPACE_Y_MIN + i * SPACE_GRID << endl;
	}
      }
    }
  } while(i!=starti || j!=startj);
  
  outfile.close();
  //PrintWorkSpace(OutlineSpace, Space_Size_Y, Space_Size_X);
  PrintSpaceToFile("output.txt", OutlineSpace, Space_Size_Y, Space_Size_X);
  
  unsigned int Capacity = CountCapacity(OutlineSpace, Space_Size_Y, Space_Size_X, startj);
  
  for(unsigned int i=0; i<Space_Size_Y; i++)
    delete [] OutlineSpace[i];
  delete [] OutlineSpace;
  
  return Capacity;
}

/*
 * Method: PrintSpaceToFile
 *
 * This method prints given space to an output file
 * which name is given as an argument
 *
 */

void Manipulator::PrintSpaceToFile(std::string filename, double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X)
{
  ofstream outfile;
  outfile.open(filename.c_str());
  
  for(unsigned int i=0; i<Space_Size_Y; i++)
  {
    for(unsigned int j=0; j<Space_Size_X; j++)
    {
      if(Space[i][j]>0)
	outfile << SPACE_X_MIN + (double)j * SPACE_GRID << " " << SPACE_Y_MIN + i * SPACE_GRID << endl;
    }
  }
  outfile.close();
}


/*
 * Method: CountCapacity
 *
 * This method counts the capacity of given outline of the space
 *
 */

unsigned int Manipulator::CountCapacity(double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X, 
				unsigned int startj)
{
  unsigned int Capacity=0;
  unsigned int starti;
  bool SpaceEndFlague;
  
  for(unsigned int j=startj; j<Space_Size_X; j++)
  {
    SpaceEndFlague=true;
    
    // we search for first 1 in column
    for(unsigned int i=0; i<Space_Size_Y; i++)
    {
      if(Space[i][j]>0)
      {
	starti=i;
	SpaceEndFlague=false;
	break;
      }      
    }
    
    // if it is the last column of space we finish
    if(SpaceEndFlague==true)
      break;
    
    // we search for second 1 in column
    for(unsigned int i=starti+1; i<Space_Size_Y; i++)
    {
      if(Space[i][j]>0)
      {
	Capacity+=SPACE_GRID*(i-starti);
	break;
      }
    }
  }
  
  return Capacity;
}

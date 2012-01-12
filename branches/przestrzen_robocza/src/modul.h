#ifndef _LIBS
#define _LIBS

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <vector>
#include <iostream>
#include <string>
#endif

/*
 * Structure: Position
 *
 * It is used to define an position and orientation in 2D space
 * Angle alpha starts from X axis in first quarter
 *
 */
struct Position
{
  public:
    double x;
    double y;
    double alpha;
    Position(): x(0), y(0), alpha(0) { }
};

/*
 * Class: Joint
 *
 * It is used to define one joint of a manipulator,
 * by its current value and also minimum and maximum ones
 * All values have to be defined in constructor and
 * minimum and maximum joint values cannot be changed
 *
 */

class Joint
{
  private:
    double _q;
    const double _q_min;
    const double _q_max;
    
  public:
    Joint(double q, double q_min, double q_max): _q(q), _q_min(q_min), _q_max(q_max) { }
    double q() { return _q; }
    //double& q() { return _q; }
    double q_min() { return _q_min; }
    double q_max() { return _q_max; }
    void q(double q) { _q=q; }
    //void q_min(double q_min) { _q_min=q_min; }
    //void q_max(double q_max) { _q_min=q_max; }
};

/*
 * Class: Joints
 *
 * Class used to gather four objects of class Joint
 *
 */

class Joints
{
  private:
    Joint _Qa1;
    Joint _Qa2;
    Joint _Qb1;
    Joint _Qb2;
    
  public:
    Joints(double Qa1q, double Qa1qmin, double Qa1qmax, 
	    double Qa2q, double Qa2qmin, double Qa2qmax, 
	    double Qb1q, double Qb1qmin, double Qb1qmax, 
	    double Qb2q, double Qb2qmin, double Qb2qmax):  
	    _Qa1(Qa1q, Qa1qmin, Qa1qmax), _Qa2(Qa2q, Qa2qmin, Qa2qmax), 
	    _Qb1(Qb1q, Qb1qmin, Qb1qmax), _Qb2(Qb2q, Qb2qmin, Qb2qmax){ }
    Joint& Qa1() { return _Qa1; }
    Joint& Qa2() { return _Qa2; }
    Joint& Qb1() { return _Qb1; }
    Joint& Qb2() { return _Qb2; }
};

/*
 * Class: Manipulator
 *
 * Main class which gathers all manipulators data.
 * It includes joints as an object of class Joints and 
 * all methods used to calculate forward and inverse kinematics.
 *
 */

class Manipulator
{
  private:
    const double _L;
    const double _l1;
    const double _l2;
    const double _l3;
    const double _l4;
    Position _effector;
    Joints _Q;

  public:
    Manipulator(double L, double l1, double l2, double l3, double l4, 
		double Qa1q = M_PI/3, double Qa1qmin = 0, double Qa1qmax = 2*M_PI/3, 
		double Qa2q = M_PI/2, double Qa2qmin = 0, double Qa2qmax = M_PI, 
		double Qb1q = 2*M_PI/3, double Qb1qmin = M_PI/3, double Qb1qmax = M_PI, 
		double Qb2q = -2*M_PI/3, double Qb2qmin = -M_PI, double Qb2qmax = 0): 
			    _L(L), _l1(l1), _l2(l2), _l3(l3), _l4(l4),
			    _Q(Qa1q, Qa1qmin, Qa1qmax, Qa2q, Qa2qmin, Qa2qmax, 
				Qb1q, Qb1qmin, Qb1qmax, Qb2q, Qb2qmin, Qb2qmax) { }
    double L() { return _L; }
    double l1() { return _l1; }
    double l2() { return _l2; }
    double l3() { return _l3; }
    double l4() { return _l4; }
    Position effector() { return _effector; }
    Joints& Q() { return _Q; }

    bool DoublePendulumInverseKinematics(double dxa, double dya, double dxb, double dyb);
    bool InverseKinematics(Position P, double force_angle);
    bool ForwardKinematics(Position& P);
    bool ValidateConfiguration(double dxa, double dya, double dxb, double dyb, double cnx, double cny);
    unsigned int WorkSpaceUsingForwardKinematics();
    unsigned int WorkSpaceUsingInverseKinematics();
    void PrintWorkSpace(double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X);
    unsigned int FindWorkspaceOutline(double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X);
    void PrintSpaceToFile(std::string filename, double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X);
    unsigned int CountCapacity(double** Space, unsigned int Space_Size_Y, unsigned int Space_Size_X, unsigned int startj);
};

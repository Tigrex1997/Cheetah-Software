#ifndef CHEETAH_SOFTWARE_SPARSECMPC_H
#define CHEETAH_SOFTWARE_SPARSECMPC_H

#include "GraphSearch.h"
#include "cppTypes.h"
#include "../../../third-party/JCQP/SparseMatrixMath.h"
#include "sophus/geometry.hpp"

struct BblockID {
  u32 foot;
  u32 timestep;
};

class SparseCMPC {
public:
  SparseCMPC();
  // origin
  void run();
  // errorState (1st version)
  void run_ES_1();
  // errorState (2nd version)
  void run_ES_2();
  // errorState (3rd version)
  void run_ES_3();


  // setup methods
  template<typename T>
  void setRobotParameters(Mat3<T>& inertia, T mass, T maxForce) {
    _Ibody = inertia.template cast<double>();
    _mass = mass;
    _maxForce = maxForce;
  }

  void setFriction(double mu) {
    _mu = mu;
  }

  template<typename T>
  void setWeights(Vec12<T>& weights, T alpha) {
    _weights = weights.template cast<double>();
    _alpha = alpha;
  }

  template<typename T>
  void setX0(Vec3<T> p, Vec3<T> v, Vec4<T> q, Vec3<T> w) {
    _p0 = p.template cast<double>();
    _v0 = v.template cast<double>();
    _q0 = q.template cast<double>();
    _w0 = w.template cast<double>();
  }

  void setContactTrajectory(ContactState* contacts, std::size_t length) {
    _contactTrajectory.resize(length);
    for(std::size_t i = 0; i < length; i++) {
      _contactTrajectory[i] = contacts[i];
    }
  }

  void setStateTrajectory(vectorAligned<Vec12<double>>& traj) {
    _stateTrajectory = traj;
  }

  template<typename T>
  void setDtTrajectory(std::vector<T>& traj) {
    _dtTrajectory.clear();
    _dtTrajectory.reserve(traj.size());
    for(auto pt : traj)
      _dtTrajectory.push_back(pt);
  }

  template<typename T>
  void setFeet(Vec12<T>& feet) {
    _pFeet = feet.template cast<double>();
  }

//  Eigen::Matrix<float, Eigen::Dynamic, 1>& getResult() {
//    return _result;
//  }

  Vec12<float> getResult();


private:
  // origin
  void buildX0();
  void buildCT();
  void buildDT();
  // errorState (1st version)
  void buildX0_ES_1();
  void buildCT_ES_1();
  void buildDT_ES_1();
  // errorState (2nd version)
  void buildX0_ES_2();
  void buildCT_ES_2();
  void buildDT_ES_2();
  // errorState (3rd version)
  void buildX0_ES_3();
  void buildCT_ES_3();
  void buildDT_ES_3();
  void c2d(u32 trajIdx, u32 bBlockStartIdx, u32 block_count);

  // errorState get comp term (1st version)
  Vec12<double> getCompTerm_1(int k);
  // errorState get comp term (2nd version)
  Vec12<double> getCompTerm_2(int k);
  // errorState get comp term (3rd version)
  Vec12<double> getCompTerm_3(int k);

  u32 getStateIndex(u32 trajIdx);
  u32 getControlIndex(u32 bBlockIdx);
  u32 addConstraint(u32 size);
  void addConstraintTriple(double value, u32 row, u32 col);

  // origin
  void addX0Constraint();
  void addDynamicsConstraints();
  void addForceConstraints();
  void addFrictionConstraints();
  void addQuadraticStateCost();
  void addLinearStateCost();
  void addQuadraticControlCost();
  // errorState (1st version)
  void addX0Constraint_ES_1();
  void addDynamicsConstraints_ES_1();
  void addForceConstraints_ES_1();
  void addFrictionConstraints_ES_1();
  void addQuadraticStateCost_ES_1();
  void addLinearStateCost_ES_1();
  void addQuadraticControlCost_ES_1();
  // errorState (2nd version)
  void addX0Constraint_ES_2();
  void addDynamicsConstraints_ES_2();
  void addForceConstraints_ES_2();
  void addFrictionConstraints_ES_2();
  void addQuadraticStateCost_ES_2();
  void addLinearStateCost_ES_2();
  void addQuadraticControlCost_ES_2();
  // errorState (3rd version)
  void addX0Constraint_ES_3();
  void addDynamicsConstraints_ES_3();
  void addForceConstraints_ES_3();
  void addFrictionConstraints_ES_3();
  void addQuadraticStateCost_ES_3();
  void addLinearStateCost_ES_3();
  void addQuadraticControlCost_ES_3();


  void runSolver();
  void runSolverOSQP();

  // inputs
  Mat3<double> _Ibody;
  Vec12<double> _weights;
  double _mass, _maxForce, _mu, _alpha;
  Vec3<double> _p0, _v0, _w0, _rpy0;
  Vec4<double> _q0;
  Vec12<double> _x0;
  Vec12<double> _pFeet, _g;

  // input trajectories
  std::vector<ContactState> _contactTrajectory;
  vectorAligned<Vec12<double>> _stateTrajectory;
  std::vector<double> _dtTrajectory;

  // intermediates
  vectorAligned<Mat12<double>> _aMat;
  std::vector<BblockID> _bBlockIds;
  vectorAligned<Eigen::Matrix<double,12,3>> _bBlocks;
  std::vector<u32> _contactCounts;
  std::vector<u32> _runningContactCounts;

  std::vector<SparseTriple<double>> _constraintTriples, _costTriples;
  std::vector<double> _lb, _ub, _linearCost;

  Eigen::Matrix<float, Eigen::Dynamic, 1> _result;


  u32 _trajectoryLength;
  u32 _bBlockCount;
  u32 _constraintCount;

  // errorState params
  Mat3<double> _R;
  Vec3<double> _wb, _vb;
  Vec6<double> _xib;
  Mat4<double> _errX;
  Vec6<double> _errxi;
  Vec3<double> _errp, _errr;
  Vec12<double> _err0;
  Mat6<double> _coadj;
  Mat6<double> _adj;
  Mat12<double> _Ac;

  // Inertia matrix and inverse
  Mat6<double> _M;
  Mat6<double> _Minv;
};

#endif //CHEETAH_SOFTWARE_SPARSECMPC_H

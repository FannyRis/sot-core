/*
 * Copyright 2010,
 * Fanny Risbourg
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>

#include <sot/core/sot.hh>
#include <sot/core/feature-point6d.hh>
#include <sot/core/feature-visual-point.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/debug.hh>
#include <sot/core/task.hh>
#include <sot/core/gain-adaptive.hh>
#include <dynamic-graph/linear-algebra.h>
using namespace std;
using namespace dynamicgraph::sot;

int main( void )
{
  dynamicgraph::Matrix Jq(6,6); Jq.setIdentity();
  dynamicgraph::Vector p1xy(2); p1xy(0)=1.; p1xy(1)=-2;

  sotDEBUGF("Create feature");
  FeaturePoint6d feature = new FeaturePoint6d("feature");

  // Plug the signals (Not the Jacobian?)
  Eigen::Matrix<double, 4, 4> position;
  Eigen::VectorXd velocity(6);
  Eigen::MatrixXd jacobian(6, 6);
  feature.positionSIN.value = position;
  feature.velocitySIN.value = velocity;
  feature.articularJacobianSIN.value = jacobian;

  // Test getDimension
  int dim = 3;
  int time = 2;
  int dimension = getDimension(dim, time);

  // test computeError
  dg::Vector res;
  int time = 2;
  dg::Vector result = computeError(res, time);

  // test computeErrordot
  dg::Vector res;
  int time = 2;
  dg::Vector result = computeErrordot(res, time);

  // test computeJacobian
  dg::Matrix res;
  int time = 2;
  dg::Matrix result = computeJacobian(res, time);

  servoCurrentPosition();

  return 0;
}

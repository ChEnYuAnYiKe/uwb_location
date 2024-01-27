#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include "trilateration.h"


void autopositioning(int *range, int aid, Eigen::MatrixXd& anchorArray);

void mds(Eigen::MatrixXd twrdistance, int nNodes, int viewNode, Eigen::MatrixXd& transCoord);

void angleRotation(Eigen::MatrixXd transCoord, int nNodes, Eigen::MatrixXd& estCoord);


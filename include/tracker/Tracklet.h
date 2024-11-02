#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getX() { return kf_.getX(); }
  double getY() { return kf_.getY(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  Eigen::MatrixXd getSMatrix() { return kf_.getSMatrix(); }
  Eigen::VectorXd getMeasureDifferenceY(const Eigen::VectorXd &z){ return kf_.getMeasureDifferenceY(z); }
  int getLossCount() { return loss_count_; }
  int getId() { return id_; }
  double getDistanceTraveled() { return dist_traveled_; }

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;

  //  total distance traveled by tracklet
  double dist_traveled_;

  // used to calculate distance traveled
  Eigen::Vector2d last_position_;
};

#endif // TRACKLET_H_

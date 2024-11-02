#include "tracker/Tracklet.h"

Tracklet::Tracklet(int idTrack, double x, double y)
{
  // set id
  id_ = idTrack;

  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);

  // set loss count to 0
  loss_count_ = 0;

  dist_traveled_ = 0;
  last_position_ = kf_.getPosition();
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
  loss_count_++;
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);
  Eigen::Vector2d curr_position = kf_.getPosition();
  
  // measurement update
  if (lidarStatus)
  {
    dist_traveled_ = std::hypot(curr_position.x() - last_position_.x(), curr_position.y() - last_position_.y());
    last_position_ = curr_position;
    raw_measurements_ << x, y;
    kf_.update(raw_measurements_);
    loss_count_ = 0;
  }
}

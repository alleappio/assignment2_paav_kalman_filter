#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>
#include <iostream>
#include <map>

class Tracker
{
public:
  Tracker();
  ~Tracker();

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  void calcTrackletsInArea();  

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }
  int getNumTrackletsInArea(){ return tracklets_in_area_.size(); }
  std::vector<int> getIdsTracletsInArea();
  std::pair<int,double> getLongestPath();

  //setter
  void setArea(double min_x,double max_x,double min_y,double max_y); 

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // association
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // thresholds
  double distance_threshold_;
  double covariance_threshold;
  int loss_threshold;
  bool mahalanobis_dist;

  // area
  std::map<std::string, double> area_;
  std::vector<Tracklet> tracklets_in_area_;
};

#endif // TRACKER_H_

/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <set>

#include <visnav/common_types.h>

#include <visnav/calibration.h>

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

using namespace std;
using namespace opengv;

namespace visnav {

void project_landmarks(
    const Sophus::SE3d& current_pose,  // T_w_c
    const std::shared_ptr<AbstractCamera<double>>& cam,
    const Landmarks& landmarks, const double cam_z_threshold,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>&
        projected_points,
    std::vector<TrackId>& projected_track_ids) {
  projected_points.clear();
  projected_track_ids.clear();

  // TODO SHEET 5: project landmarks to the image plane using the current
  // locations of the cameras. Put 2d coordinates of the projected points into
  // projected_points and the corresponding id of the landmark into
  // projected_track_ids.
  UNUSED(current_pose);
  UNUSED(cam);
  UNUSED(landmarks);
  UNUSED(cam_z_threshold);

  for (const auto& landmark : landmarks) {
    Eigen::Vector3d landmark_c = current_pose.inverse() * landmark.second.p;
    Eigen::Vector2d projection = cam->project(landmark_c);
    if (projection[0] < cam->width() && projection[0] > 0 &&
        projection[1] < cam->height() && projection[1] > 0 &&
        landmark_c[2] > cam_z_threshold) {
      projected_points.push_back(projection);
      projected_track_ids.push_back(landmark.first);
    }
  }
}

struct Candidates {
  /// track ids of landmarks within the searching range of the current keypoint
  std::vector<TrackId> track_ids;
  /// feature ids of the landmark with the minimal descriptor distance
  // std::vector<FeatureId> feature_ids;
  /// distances between landmarks and the keypoint
  std::vector<int> dists;
};

void find_matches_landmarks(
    const KeypointsData& kdl, const Landmarks& landmarks,
    const Corners& feature_corners,
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>>&
        projected_points,
    const std::vector<TrackId>& projected_track_ids,
    const double match_max_dist_2d, const int feature_match_threshold,
    const double feature_match_dist_2_best, LandmarkMatchData& md) {
  md.matches.clear();

  // TODO SHEET 5: Find the matches between projected landmarks and detected
  // keypoints in the current frame. For every detected keypoint search for
  // matches inside a circle with radius match_max_dist_2d around the point
  // location. For every landmark the distance is the minimal distance between
  // the descriptor of the current point and descriptors of all observations of
  // the landmarks. The feature_match_threshold and feature_match_dist_2_best
  // should be used to filter outliers the same way as in exercise 3. You should
  // fill md.matches with <featureId,trackId> pairs for the successful matches
  // that pass all tests.
  UNUSED(kdl);
  UNUSED(landmarks);
  UNUSED(feature_corners);
  UNUSED(projected_points);
  UNUSED(projected_track_ids);
  UNUSED(match_max_dist_2d);
  UNUSED(feature_match_threshold);
  UNUSED(feature_match_dist_2_best);

  // here the feature_id to be added to md.match is the id of the keypoit in
  // kdl, but not the one in obs
  for (size_t i = 0; i < kdl.corners.size(); i++) {
    Eigen::Vector2d kp = kdl.corners[i];
    bitset<256> kp_descriptor = kdl.corner_descriptors[i];
    FeatureId fid_kp = i;

    // find projections within match_max_dist_2d, for each landmark, pick the
    // feature_id with min distance and storet the feature_id and distance in
    // candidates
    Candidates candidates;
    for (size_t j = 0; j < projected_points.size(); j++) {
      double distance_2d = (projected_points[j] - kp).norm();

      // within the range, then get the feature_id, track_id, descriptor
      if (distance_2d < match_max_dist_2d) {
        TrackId track_id = projected_track_ids[j];

        // find the minimal descriptor distance for this landmark
        // unordered_map<FeatureId, bitset<256>>
        TrackId min_feature_id;
        int min_distance = 256;
        for (const auto& obs : landmarks.at(track_id).obs) {
          bitset<256> descriptor =
              feature_corners.at(obs.first).corner_descriptors[obs.second];
          int distance = (kp_descriptor ^ descriptor).count();
          if (distance < min_distance) {
            min_feature_id = obs.second;
            min_distance = distance;
          }
        }
        candidates.track_ids.push_back(track_id);
        candidates.dists.push_back(min_distance);
      }
    }
    // now we have all candidate landmarks for one keypoint, pick the best and
    // filter outliers

    auto min_index_ptr =
        min_element(candidates.dists.begin(), candidates.dists.end());
    size_t min_index = distance(candidates.dists.begin(), min_index_ptr);

    sort(candidates.dists.begin(), candidates.dists.end());

    if (candidates.dists.size() > 1) {
      if (candidates.dists[0] < feature_match_threshold &&
          candidates.dists[0] * feature_match_dist_2_best < candidates.dists[1])
        md.matches.push_back(
            make_pair(fid_kp, candidates.track_ids[min_index]));
    } else if (candidates.dists.size() == 1) {
      if (candidates.dists[0] < feature_match_threshold)
        md.matches.push_back(
            make_pair(fid_kp, candidates.track_ids[min_index]));
    }
  }
}

void localize_camera(const Sophus::SE3d& current_pose,
                     const std::shared_ptr<AbstractCamera<double>>& cam,
                     const KeypointsData& kdl, const Landmarks& landmarks,
                     const double reprojection_error_pnp_inlier_threshold_pixel,
                     LandmarkMatchData& md) {
  md.inliers.clear();

  // default to previous pose if not enough inliers
  md.T_w_c = current_pose;

  if (md.matches.size() < 4) {
    return;
  }

  // TODO SHEET 5: Find the pose (md.T_w_c) and the inliers (md.inliers) using
  // the landmark to keypoints matches and PnP. This should be similar to the
  // localize_camera in exercise 4 but in this exercise we don't explicitly have
  // tracks.
  UNUSED(cam);
  UNUSED(kdl);
  UNUSED(landmarks);
  UNUSED(reprojection_error_pnp_inlier_threshold_pixel);

  bearingVectors_t bVectors;
  points_t points;
  for (size_t i = 0; i < md.matches.size(); i++) {
    FeatureId feature_id = md.matches[i].first;
    TrackId track_id = md.matches[i].second;
    bVectors.push_back(cam->unproject(kdl.corners[feature_id]));
    points.push_back(landmarks.at(track_id).p);
  }

  // construct adapter and ransac problem
  absolute_pose::CentralAbsoluteAdapter adapter(bVectors, points);
  sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  std::shared_ptr<sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr(
          new sac_problems::absolute_pose::AbsolutePoseSacProblem(
              adapter,
              sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ =
      1.0 - cos(atan(sqrt(2.0) * reprojection_error_pnp_inlier_threshold_pixel /
                     500.0));  // 500 is the focus length, threshold_pix is the
                               // error you can accept in pixels
  ransac.max_iterations_ = 100;
  ransac.computeModel();

  transformation_t best_transformation = ransac.model_coefficients_;

  // non-linear optimize the transformation using all inliers
  adapter.setR(best_transformation.block(0, 0, 3, 3));
  adapter.sett(best_transformation.block(0, 3, 3, 1));
  transformation_t new_transformation =
      absolute_pose::optimize_nonlinear(adapter, ransac.inliers_);

  Eigen::Matrix3d R = new_transformation.block(0, 0, 3, 3);
  Eigen::Matrix<double, 3, 1> t = new_transformation.block(0, 3, 3, 1);
  Sophus::SE3d T(R, t);
  md.T_w_c = T;

  // filter inliers again using refined transformation
  vector<int> inliers_indices;
  ransac.sac_model_->selectWithinDistance(new_transformation, ransac.threshold_,
                                          inliers_indices);
  for (size_t i = 0; i < inliers_indices.size(); i++) {
    md.inliers.push_back(md.matches[inliers_indices[i]]);
  }
}

void add_new_landmarks(const FrameCamId fcidl, const FrameCamId fcidr,
                       const KeypointsData& kdl, const KeypointsData& kdr,
                       const Calibration& calib_cam, const MatchData& md_stereo,
                       const LandmarkMatchData& md, Landmarks& landmarks,
                       TrackId& next_landmark_id) {
  // input should be stereo pair
  assert(fcidl.cam_id == 0);
  assert(fcidr.cam_id == 1);

  const Sophus::SE3d T_0_1 = calib_cam.T_i_c[0].inverse() * calib_cam.T_i_c[1];
  const Eigen::Vector3d t_0_1 = T_0_1.translation();
  const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();

  // TODO SHEET 5: Add new landmarks and observations. Here md_stereo contains
  // stereo matches for the current frame and md contains feature to landmark
  // matches for the left camera (camera 0). For all inlier feature to landmark
  // matches add the observations to the existing landmarks. If the left
  // camera's feature appears also in md_stereo.inliers, then add both
  // observations. For all inlier stereo observations that were not added to the
  // existing landmarks, triangulate and add new landmarks. Here
  // next_landmark_id is a running index of the landmarks, so after adding a new
  // landmark you should always increase next_landmark_id by 1.
  UNUSED(fcidl);
  UNUSED(fcidr);
  UNUSED(kdl);
  UNUSED(kdr);
  UNUSED(calib_cam);
  UNUSED(md_stereo);
  UNUSED(md);
  UNUSED(landmarks);
  UNUSED(next_landmark_id);
  UNUSED(t_0_1);
  UNUSED(R_0_1);

  unordered_map<FeatureId, FeatureId> stereo_inliers_map(
      md_stereo.inliers.begin(), md_stereo.inliers.end());

  for (size_t i = 0; i < md.inliers.size(); i++) {
    // add all inlier matches to the obs of the left cam
    FeatureId feature_idl = md.inliers[i].first;
    TrackId track_id = md.inliers[i].second;
    landmarks.at(track_id).obs[fcidl] = feature_idl;

    // if fidl appears in md_stereo.inlier, then add the corresponding fidr to
    // the obs of the right cam
    if (stereo_inliers_map.count(feature_idl) > 0) {
      FeatureId feature_idr = stereo_inliers_map.at(feature_idl);
      landmarks.at(track_id).obs[fcidr] = feature_idr;
      stereo_inliers_map.erase(feature_idl);
    }
  }

  // the left pairs in stereo_inliers_map were not used, now use them to do
  // triangulation to creat new landmark
  for (const auto& pair : stereo_inliers_map) {
    FeatureId fid0 = pair.first;
    FeatureId fid1 = pair.second;
    TrackId new_track_id = next_landmark_id;
    next_landmark_id++;

    // construct adapter
    bearingVectors_t bVector0, bVector1;
    bVector0.push_back(calib_cam.intrinsics[0]->unproject(kdl.corners[fid0]));
    bVector1.push_back(calib_cam.intrinsics[1]->unproject(kdr.corners[fid1]));
    relative_pose::CentralRelativeAdapter adapter(bVector0, bVector1, t_0_1,
                                                  R_0_1);
    // new landmark
    Landmark new_landmark;
    new_landmark.p = md.T_w_c * triangulation::triangulate(adapter, 0);
    new_landmark.obs[fcidl] = fid0;
    new_landmark.obs[fcidr] = fid1;

    landmarks[new_track_id] = new_landmark;
  }
}

void remove_old_keyframes(const FrameCamId fcidl, const int max_num_kfs,
                          Cameras& cameras, Landmarks& landmarks,
                          Landmarks& old_landmarks,
                          std::set<FrameId>& kf_frames) {
  kf_frames.emplace(fcidl.frame_id);

  // TODO SHEET 5: Remove old cameras and observations if the number of keyframe
  // pairs (left and right image is a pair) is larger than max_num_kfs. The ids
  // of all the keyframes that are currently in the optimization should be
  // stored in kf_frames. Removed keyframes should be removed from cameras and
  // landmarks with no left observations should be moved to old_landmarks.
  UNUSED(max_num_kfs);
  UNUSED(cameras);
  UNUSED(landmarks);
  UNUSED(old_landmarks);

  std::vector<TrackId> to_be_removed_landmarks;

  // sort(kf_frames.begin(), kf_frames.end());
  while (int(kf_frames.size()) > max_num_kfs) {
    FrameCamId fcid0, fcid1;
    fcid0.cam_id = 0;
    fcid1.cam_id = 1;
    fcid0.frame_id = *kf_frames.begin();
    fcid1.frame_id = *kf_frames.begin();

    // remove old cameras
    cameras.erase(fcid0);
    cameras.erase(fcid1);

    for (auto& landmark : landmarks) {
      // remove old observations
      landmark.second.obs.erase(fcid0);
      landmark.second.obs.erase(fcid1);

      // if no observation anymore for this landmark, move it to old_landmark
      if (landmark.second.obs.size() == 0) {
        old_landmarks.emplace(landmark);
        // landmarks.erase(landmark.first);   cannot do this in the loop
        to_be_removed_landmarks.push_back(landmark.first);
      }
    }

    // remove old keyframes
    kf_frames.erase(kf_frames.begin());
  }

  for (size_t i = 0; i < to_be_removed_landmarks.size(); i++)
    landmarks.erase(to_be_removed_landmarks[i]);
}
}  // namespace visnav

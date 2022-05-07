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

#include <fstream>
#include <thread>

#include <ceres/ceres.h>

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

#include <visnav/common_types.h>
#include <visnav/serialization.h>

#include <visnav/reprojection.h>
#include <visnav/local_parameterization_se3.hpp>

#include <visnav/tracks.h>

using namespace std;
using namespace opengv;

namespace visnav {

// save map with all features and matches
void save_map_file(const std::string& map_path, const Corners& feature_corners,
                   const Matches& feature_matches,
                   const FeatureTracks& feature_tracks,
                   const FeatureTracks& outlier_tracks, const Cameras& cameras,
                   const Landmarks& landmarks) {
  {
    std::ofstream os(map_path, std::ios::binary);

    if (os.is_open()) {
      cereal::BinaryOutputArchive archive(os);
      archive(feature_corners);
      archive(feature_matches);
      archive(feature_tracks);
      archive(outlier_tracks);
      archive(cameras);
      archive(landmarks);

      size_t num_obs = 0;
      for (const auto& kv : landmarks) {
        num_obs += kv.second.obs.size();
      }
      std::cout << "Saved map as " << map_path << " (" << cameras.size()
                << " cameras, " << landmarks.size() << " landmarks, " << num_obs
                << " observations)" << std::endl;
    } else {
      std::cout << "Failed to save map as " << map_path << std::endl;
    }
  }
}

// load map with all features and matches
void load_map_file(const std::string& map_path, Corners& feature_corners,
                   Matches& feature_matches, FeatureTracks& feature_tracks,
                   FeatureTracks& outlier_tracks, Cameras& cameras,
                   Landmarks& landmarks) {
  {
    std::ifstream is(map_path, std::ios::binary);

    if (is.is_open()) {
      cereal::BinaryInputArchive archive(is);
      archive(feature_corners);
      archive(feature_matches);
      archive(feature_tracks);
      archive(outlier_tracks);
      archive(cameras);
      archive(landmarks);

      size_t num_obs = 0;
      for (const auto& kv : landmarks) {
        num_obs += kv.second.obs.size();
      }
      std::cout << "Loaded map from " << map_path << " (" << cameras.size()
                << " cameras, " << landmarks.size() << " landmarks, " << num_obs
                << " observations)" << std::endl;
    } else {
      std::cout << "Failed to load map from " << map_path << std::endl;
    }
  }
}

// Create new landmarks from shared feature tracks if they don't already exist.
// The two cameras must be in the map already.
// Returns the number of newly created landmarks.
int add_new_landmarks_between_cams(const FrameCamId& fcid0,
                                   const FrameCamId& fcid1,
                                   const Calibration& calib_cam,
                                   const Corners& feature_corners,
                                   const FeatureTracks& feature_tracks,
                                   const Cameras& cameras,
                                   Landmarks& landmarks) {
  // shared_track_ids will contain all track ids shared between the two images,
  // including existing landmarks
  std::vector<TrackId> shared_track_ids;

  // find shared feature tracks
  const std::set<FrameCamId> fcids = {fcid0, fcid1};
  // if no common tracks, GetTracksInImages return 0
  if (!GetTracksInImages(fcids, feature_tracks, shared_track_ids)) {
    return 0;
  }

  // at the end of the function this will contain all newly added track ids
  std::vector<TrackId> new_track_ids;

  // TODO SHEET 4: Triangulate all new features and add to the map
  UNUSED(calib_cam);
  UNUSED(feature_corners);
  UNUSED(cameras);
  UNUSED(landmarks);
  // *cameras contains all cameras that have been added in the map
  // *shared_track_ids contains all track ids in common
  // *feature track is a collection of all images that observed this feature
  // and the corresponding feature index in that image

  // pick common fid from shared_track_ids, use fid to pick keypoints from
  // feature_corner, reproject it to 3d and then triangulate. Add all fcid to
  // obs but remember to filter the camera that not yet exists in map

  CamId cam_id0 = fcid0.cam_id;
  CamId cam_id1 = fcid1.cam_id;
  // check if both cameras are already in the map?
  if (cameras.count(fcid0) > 0 && cameras.count(fcid1) > 0) {
    for (size_t i = 0; i < shared_track_ids.size(); i++) {
      TrackId track_id = shared_track_ids[i];
      // check if the landmark already exists
      if (landmarks.count(track_id) == 0) {
        // pick 2d keypoints
        FeatureId feature_id0 = feature_tracks.at(track_id).at(fcid0);
        FeatureId feature_id1 = feature_tracks.at(track_id).at(fcid1);
        Eigen::Vector2d keypoint0 =
            feature_corners.at(fcid0).corners[feature_id0];
        Eigen::Vector2d keypoint1 =
            feature_corners.at(fcid1).corners[feature_id1];

        // construct adapter to do triangulation
        bearingVectors_t bVector0, bVector1;
        bVector0.push_back(calib_cam.intrinsics[cam_id0]->unproject(keypoint0));
        bVector1.push_back(calib_cam.intrinsics[cam_id1]->unproject(keypoint1));
        // give the transformation from c1 to c0 i.e.T_c0_c1
        translation_t trans =
            (cameras.at(fcid0).T_w_c.inverse() * cameras.at(fcid1).T_w_c)
                .translation();
        rotation_t rot =
            (cameras.at(fcid0).T_w_c.inverse() * cameras.at(fcid1).T_w_c)
                .rotationMatrix();

        relative_pose::CentralRelativeAdapter adapter(bVector0, bVector1, trans,
                                                      rot);
        point_t landmark_w =
            cameras.at(fcid0).T_w_c * triangulation::triangulate(adapter, 0);

        // new landmark
        Landmark new_landmark;
        new_landmark.p = landmark_w;
        // for obs, filter cameras that are not yet added to the map
        for (const auto& kv : feature_tracks.at(track_id)) {
          if (cameras.count(kv.first) > 0) new_landmark.obs.emplace(kv);
        }
        // new_landmark.obs = feature_tracks.at(track_id);
        landmarks[track_id] = new_landmark;
        new_track_ids.push_back(track_id);
      }
    }
  }

  return new_track_ids.size();
}

// Initialize the scene from a stereo pair, using the known transformation from
// camera calibration. This adds the inital two cameras and triangulates shared
// landmarks.
// Note: in principle we could also initialize a map from another images pair
// using the transformation from the pairwise matching with the 5-point
// algorithm. However, using a stereo pair has the advantage that the map is
// initialized with metric scale.
bool initialize_scene_from_stereo_pair(const FrameCamId& fcid0,
                                       const FrameCamId& fcid1,
                                       const Calibration& calib_cam,
                                       const Corners& feature_corners,
                                       const FeatureTracks& feature_tracks,
                                       Cameras& cameras, Landmarks& landmarks) {
  // check that the two image ids refer to a stereo pair
  if (!(fcid0.frame_id == fcid1.frame_id && fcid0.cam_id != fcid1.cam_id)) {
    std::cerr << "Images " << fcid0 << " and " << fcid1
              << " don't form a stereo pair. Cannot initialize." << std::endl;
    return false;
  }

  // TODO SHEET 4: Initialize scene (add initial cameras and landmarks)
  UNUSED(calib_cam);
  UNUSED(feature_corners);
  UNUSED(feature_tracks);
  UNUSED(cameras);
  UNUSED(landmarks);

  Camera cam0, cam1;
  cam0.T_w_c = calib_cam.T_i_c[0];
  cam1.T_w_c = calib_cam.T_i_c[1];
  if (fcid0.cam_id == 0) {
    cameras[fcid0] = cam0;
    cameras[fcid1] = cam1;
  } else {
    cameras[fcid0] = cam1;
    cameras[fcid1] = cam0;
  }

  add_new_landmarks_between_cams(fcid0, fcid1, calib_cam, feature_corners,
                                 feature_tracks, cameras, landmarks);

  return true;
}

// Localize a new camera in the map given a set of observed landmarks. We use
// pnp and ransac to localize the camera in the presence of outlier tracks.
// After finding an inlier set with pnp, we do non-linear refinement using all
// inliers and also update the set of inliers using the refined pose.
//
// shared_track_ids already contains those tracks which the new image shares
// with the landmarks (but some might be outliers).
//
// We return the refined pose and the set of track ids for all inliers.
//
// The inlier threshold is given in pixels. See also the opengv documentation on
// how to convert this to a ransac threshold:
// http://laurentkneip.github.io/opengv/page_how_to_use.html#sec_threshold
void localize_camera(
    const FrameCamId& fcid, const std::vector<TrackId>& shared_track_ids,
    const Calibration& calib_cam, const Corners& feature_corners,
    const FeatureTracks& feature_tracks, const Landmarks& landmarks,
    const double reprojection_error_pnp_inlier_threshold_pixel,
    Sophus::SE3d& T_w_c, std::vector<TrackId>& inlier_track_ids) {
  inlier_track_ids.clear();

  // TODO SHEET 4: Localize a new image in a given map
  UNUSED(fcid);
  UNUSED(shared_track_ids);
  UNUSED(calib_cam);
  UNUSED(feature_corners);
  UNUSED(feature_tracks);
  UNUSED(landmarks);
  UNUSED(T_w_c);
  UNUSED(reprojection_error_pnp_inlier_threshold_pixel);
  // use shared_track_ids to pick the 2d points of the landmarks in the new img,
  // then use 3D-2D ransac to compute, after that use only inliers to refine the
  // pose
  bearingVectors_t bVectors;
  points_t points;
  for (size_t i = 0; i < shared_track_ids.size(); i++) {
    TrackId track_id = shared_track_ids[i];
    FeatureId feature_id = feature_tracks.at(track_id).at(fcid);
    bVectors.push_back(calib_cam.intrinsics[fcid.cam_id]->unproject(
        feature_corners.at(fcid).corners[feature_id]));
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
  ransac.max_iterations_ = 50;
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
  T_w_c = T;

  // filter inliers again using refined transformation
  vector<int> inliers_indices;
  ransac.sac_model_->selectWithinDistance(new_transformation, ransac.threshold_,
                                          inliers_indices);
  for (size_t i = 0; i < inliers_indices.size(); i++) {
    inlier_track_ids.push_back(shared_track_ids[inliers_indices[i]]);
  }
}

struct BundleAdjustmentOptions {
  /// 0: silent, 1: ceres brief report (one line), 2: ceres full report
  int verbosity_level = 1;

  /// update intrinsics or keep fixed
  bool optimize_intrinsics = false;

  /// use huber robust norm or squared norm
  bool use_huber = true;

  /// parameter for huber loss (in pixel)
  double huber_parameter = 1.0;

  /// maximum number of solver iterations
  int max_num_iterations = 20;
};

// Run bundle adjustment to optimize cameras, points, and optionally intrinsics
void bundle_adjustment(const Corners& feature_corners,
                       const BundleAdjustmentOptions& options,
                       const std::set<FrameCamId>& fixed_cameras,
                       Calibration& calib_cam, Cameras& cameras,
                       Landmarks& landmarks) {
  ceres::Problem problem;

  // TODO SHEET 4: Setup optimization problem
  UNUSED(feature_corners);
  UNUSED(options);
  UNUSED(fixed_cameras);
  UNUSED(calib_cam);
  UNUSED(cameras);
  UNUSED(landmarks);

  ceres::LossFunction* loss;
  if (options.use_huber == true) {
    loss = new ceres::HuberLoss(options.huber_parameter);
  } else {
    loss = NULL;
  }

  for (const auto& landmark : landmarks) {
    double* p_3d_w = const_cast<double*>(landmark.second.p.data());
    for (const auto& feature_track : landmark.second.obs) {
      FrameCamId fcid = feature_track.first;
      FeatureId feature_id = feature_track.second;

      Eigen::Vector2d p_2d = feature_corners.at(fcid).corners[feature_id];
      string cam_model = calib_cam.intrinsics[fcid.cam_id]->name();
      double* T_w_c = cameras.at(fcid).T_w_c.data();
      double* intrinsics = calib_cam.intrinsics[fcid.cam_id]->data();

      problem.AddParameterBlock(T_w_c, 7,
                                new Sophus::test::LocalParameterizationSE3);

      ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<
          BundleAdjustmentReprojectionCostFunctor, 2, 7, 3, 8>(
          new BundleAdjustmentReprojectionCostFunctor(p_2d, cam_model));

      problem.AddResidualBlock(cost_function, loss, T_w_c, p_3d_w, intrinsics);

      if (fixed_cameras.count(fcid) > 0)
        problem.SetParameterBlockConstant(T_w_c);

      if (options.optimize_intrinsics == false)
        problem.SetParameterBlockConstant(intrinsics);
    }
  }

  // Solve
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = options.max_num_iterations;
  ceres_options.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_options.num_threads = std::thread::hardware_concurrency();
  ceres::Solver::Summary summary;
  Solve(ceres_options, &problem, &summary);
  switch (options.verbosity_level) {
    // 0: silent
    case 1:
      std::cout << summary.BriefReport() << std::endl;
      break;
    case 2:
      std::cout << summary.FullReport() << std::endl;
      break;
  }
}

}  // namespace visnav

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

#include <CLI/CLI.hpp>

#include <visnav/common_types.h>

#include <visnav/calibration.h>

#include <visnav/keypoints.h>
#include <visnav/map_utils.h>
#include <visnav/matching_utils.h>
#include <visnav/vo_utils.h>

#include <visnav/gui_helper.h>
#include <visnav/tracks.h>

#include <visnav/serialization.h>

#include <dirent.h>
#include "opencv2/opencv.hpp"

using namespace visnav;
using namespace std;
using namespace cv;

///////////////////////////////////////////////////////////////////////////////
/// Declarations
///////////////////////////////////////////////////////////////////////////////
void load_data(const std::string& path, const std::string& calib_path);
void draw_image_overlay(pangolin::View& v, size_t view_id);
void change_display_to_image(const FrameCamId& fcid);
void convert_to_gray(const string& dataset_path, const string& gray_path);

///////////////////////////////////////////////////////////////////////////////
/// Constants
///////////////////////////////////////////////////////////////////////////////

constexpr int UI_WIDTH = 200;
constexpr int NUM_CAMS = 1;

///////////////////////////////////////////////////////////////////////////////
/// Variables
///////////////////////////////////////////////////////////////////////////////

/// timestamps for all stereo pairs
std::vector<Timestamp_d> timestamps;

/// loaded images
tbb::concurrent_unordered_map<FrameId, std::string> images;

/// intrinsic calibration
Calibration calib_cam;
Calibration calib_cam_opt;

///////////////////////////////////////////////////////////////////////////////
/// GUI parameters
///////////////////////////////////////////////////////////////////////////////

pangolin::Var<bool> ui_show_hidden("ui.show_extra_options", false, true);

//////////////////////////////////////////////
/// Image display options

pangolin::Var<int> show_frame1("ui.show_frame1", 0, 0, 1500);
pangolin::Var<int> show_cam1("ui.show_cam1", 0, 0, NUM_CAMS - 1);
pangolin::Var<int> show_frame2("ui.show_frame2", 0, 0, 1500);
pangolin::Var<int> show_cam2("ui.show_cam2", 1, 0, NUM_CAMS - 1);
pangolin::Var<bool> lock_frames("ui.lock_frames", true, true);

///////////////////////////////////////////////////////////////////////////////
/// Main
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  bool show_gui = true;
  std::string dataset_path = "data/rgbd_dataset_freiburg1_xyz";
  std::string cam_calib = "opt_calib.json";

  CLI::App app{"RGB-D slam"};

  app.add_option("--show-gui", show_gui, "Show GUI");
  app.add_option("--dataset-path", dataset_path,
                 "Dataset path. Default: " + dataset_path);
  app.add_option("--cam-calib", cam_calib,
                 "Path to camera calibration. Default: " + cam_calib);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  // check if the rgb images are converted to gray, if not, do it
  string gray_path = dataset_path + "/gray";
  cout << 222 << endl;
  // DIR* dir;
  //  if ((dir = opendir(gray_path.c_str())) == NULL) {
  if (1 == 1) {
    cout << 111 << endl;
    system("mkdir -p ./data/rgbd_dataset_freiburg1_xyz/gray/");
    cout << 333 << endl;
    convert_to_gray(dataset_path, gray_path);
  }

  // load gray images
  load_data(dataset_path, cam_calib);
  /*
  if (show_gui) {
    pangolin::CreateWindowAndBind("Main", 1800, 1000);

    glEnable(GL_DEPTH_TEST);

    // main parent display for images and 3d viewer
    pangolin::View& main_view =
        pangolin::Display("main")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
            .SetLayout(pangolin::LayoutEqualVertical);

    // parent display for images
    pangolin::View& img_view_display =
        pangolin::Display("images").SetLayout(pangolin::LayoutEqual);
    main_view.AddDisplay(img_view_display);

    // main ui panel
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    // extra options panel
    pangolin::View& hidden_panel = pangolin::CreatePanel("hidden").SetBounds(
        0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH),
        pangolin::Attach::Pix(2 * UI_WIDTH));
    ui_show_hidden.Meta().gui_changed = true;

    // 2D image views
    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
    while (img_view.size() < NUM_CAMS) {
      std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

      size_t idx = img_view.size();
      img_view.push_back(iv);

      img_view_display.AddDisplay(*iv);
      iv->extern_draw_function =
          std::bind(&draw_image_overlay, std::placeholders::_1, idx);
    }

    while (!pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (ui_show_hidden.GuiChanged()) {
        hidden_panel.Show(ui_show_hidden);
        const int panel_width = ui_show_hidden ? 2 * UI_WIDTH : UI_WIDTH;
        main_view.SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_width), 1.0);
      }

      // display3D.Activate(camera);
      // glClearColor(0.95f, 0.95f, 0.95f, 1.0f);  // light gray background

      // draw_scene();

      img_view_display.Activate();

      if (lock_frames) {
        // in case of locking frames, chaning one should change the other
        if (show_frame1.GuiChanged()) {
          change_display_to_image(FrameCamId(show_frame1, 0));
          change_display_to_image(FrameCamId(show_frame1, 1));
        } else if (show_frame2.GuiChanged()) {
          change_display_to_image(FrameCamId(show_frame2, 0));
          change_display_to_image(FrameCamId(show_frame2, 1));
        }
      }

      if (show_frame1.GuiChanged() || show_cam1.GuiChanged()) {
        auto frame_id = static_cast<FrameId>(show_frame1);
        if (images.find(frame_id) != images.end()) {
          pangolin::TypedImage img = pangolin::LoadImage(images[frame_id]);
          img_view[0]->SetImage(img);
        } else {
          img_view[0]->Clear();
        }
      }

      if (show_frame2.GuiChanged() || show_cam2.GuiChanged()) {
        auto frame_id = static_cast<FrameId>(show_frame2);
        if (images.find(frame_id) != images.end()) {
          pangolin::GlPixFormat fmt;
          fmt.glformat = GL_LUMINANCE;
          fmt.gltype = GL_UNSIGNED_BYTE;
          fmt.scalable_internal_format = GL_LUMINANCE8;

          pangolin::TypedImage img = pangolin::LoadImage(images[frame_id]);
          img_view[1]->SetImage(img);
        } else {
          img_view[1]->Clear();
        }
      }

      pangolin::FinishFrame();
    }
  }
  */
}

// convert rgb to gray images
void convert_to_gray(const string& dataset_path, const string& gray_path) {
  const string timestams_path = dataset_path + "/rgb.txt";
  ifstream rgb_images(timestams_path);
  cout << 444 << endl;
  while (rgb_images) {
    string line;
    getline(rgb_images, line);
    string rgb_name = line.substr(18, line.size() - 18);

    cout << dataset_path << rgb_name << endl;

    Mat src = imread(dataset_path + rgb_name, 0);
    if (!src.data) cout << "could not read rgb" << endl;
  }
}

void load_data(const std::string& dataset_path, const std::string& calib_path) {
  const std::string timestams_path = dataset_path + "/rgb.txt";

  {
    std::ifstream times(timestams_path);

    FrameId id = 0;

    while (times) {
      std::string line;
      std::getline(times, line);

      if (line.size() < 20 || line[0] == '#' || id > 2700) continue;

      {
        // read the first part in the file as timestamp
        std::string timestamp_str = line.substr(0, 17);
        std::istringstream ss(timestamp_str);
        Timestamp_d timestamp;
        ss >> timestamp;
        timestamps.push_back(timestamp);
      }

      // image name: rgb/xxx.png, load into images
      std::string img_name = line.substr(18, line.size() - 18);
      std::stringstream ss;
      ss << dataset_path << img_name;
      images[id] = ss.str();

      // next frame id
      id++;
    }

    std::cerr << "Loaded " << id << " images" << std::endl;
  }

  {
    std::ifstream os(calib_path, std::ios::binary);

    if (os.is_open()) {
      cereal::JSONInputArchive archive(os);
      archive(calib_cam);
      std::cout << "Loaded camera from " << calib_path << " with models ";
      for (const auto& cam : calib_cam.intrinsics) {
        std::cout << cam->name() << " ";
      }
      std::cout << std::endl;
    } else {
      std::cerr << "could not load camera calibration " << calib_path
                << std::endl;
      std::abort();
    }
  }

  show_frame1.Meta().range[1] = images.size() / NUM_CAMS - 1;
  show_frame1.Meta().gui_changed = true;
  show_frame2.Meta().range[1] = images.size() / NUM_CAMS - 1;
  show_frame2.Meta().gui_changed = true;
}

// Update the image views to a given image id
void change_display_to_image(const FrameCamId& fcid) {
  if (0 == fcid.cam_id) {
    // left view
    show_cam1 = 0;
    show_frame1 = fcid.frame_id;
    show_cam1.Meta().gui_changed = true;
    show_frame1.Meta().gui_changed = true;
  } else {
    // right view
    show_cam2 = fcid.cam_id;
    show_frame2 = fcid.frame_id;
    show_cam2.Meta().gui_changed = true;
    show_frame2.Meta().gui_changed = true;
  }
}

void draw_image_overlay(pangolin::View& v, size_t view_id) {}

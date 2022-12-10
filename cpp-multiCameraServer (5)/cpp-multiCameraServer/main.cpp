// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <math.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTable.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "include/cscore_raw.h"

#include "cameraserver/CameraServer.h"

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
   */

static const char* configFile = "/boot/frc.json";

namespace {

    // Class I Created to handel processing
    // Creating a class is not necessary however helpfully organized workflow
    class Processing
    {
    public:
        Processing() {}
        // Function that does all the processing
        cv::Mat ProcessImg(cv::Mat img);

        //Public variables that were useful to to be displayed
        int m_totalWidth = -1;
        int m_distanceFromCenter = -1;
        double m_fov = -1;
        double m_turnAngle;
        double m_percent;
        double m_centerX;
        double m_contourCols;
        cv::Point m_fullCenter;
        cv::Mat m_contourImg;

    protected:
        // Image variables
        cv::Mat m_hsv;
        cv::Mat m_kernel;
        cv::Mat m_range;
        cv::Mat m_dilate;

        // Setting Variables
            //HSV
        int m_highHue = 97;
        int m_lowHue = 64;
        int m_highSat = 255;
        int m_lowSat = 208;
        int m_highVal = 255;
        int m_lowVal = 134;
        //Kernel
        int m_kernelSize = 10;

        double m_centroidX = -1;
        double m_centroidY = -1;

        std::vector<std::vector<cv::Point>> m_contours;
        std::vector<cv::Vec4i> m_hierarchy;
    };

    cv::Mat Processing::ProcessImg(cv::Mat img)
    {
        // Convert to HSV
        cv::cvtColor(img, m_hsv, cv::COLOR_BGR2HSV);
        // Create kernel
        m_kernel = cv::Mat::ones(cv::Size(m_kernelSize, m_kernelSize), cv::MORPH_DILATE);
        // InRange
        cv::Scalar lowValue(m_lowHue, m_lowSat, m_lowVal);
        cv::Scalar highValue(m_highHue, m_highSat, m_highVal);
            // Creates a black-white image displaying the selected HSV values in white
        cv::inRange(m_hsv, lowValue, highValue, m_range);
        // Dilate
        cv::dilate(m_range, m_dilate, m_kernel);
        // contours
        cv::findContours(m_dilate, m_contours, m_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        m_contourImg = cv::Mat::zeros(m_range.rows, m_range.cols, CV_8UC3);

        // If no contours Stop the function
        if (m_contours.size() <= 0)
        {
            m_turnAngle = -666.0;
            m_contourCols = m_contourImg.cols;
            return m_contourImg;
        }

        // loop through all contours
        for (int i = 0; i < m_contours.size(); i++)
        {
            //Create rectangle based on the contour
            cv::Rect boundRect = cv::boundingRect(m_contours[i]);
            m_centroidX = boundRect.x + (double)(boundRect.width / 2);
            m_centroidY = boundRect.y + (double)(boundRect.height / 2);
            m_totalWidth = boundRect.width;

            //sqaure around contours
            cv::Scalar color(0, 255, 0);
            cv::Point topLeft(m_centroidX - (boundRect.width / 2), m_centroidY + boundRect.height / 2);
            cv::Point topRight(m_centroidX + (boundRect.width / 2), m_centroidY + (boundRect.height / 2));
            cv::Point bottomLeft(m_centroidX - (boundRect.width / 2), m_centroidY - (boundRect.height / 2));
            cv::Point bottomRight(m_centroidX + (boundRect.width / 2), m_centroidY - (boundRect.height / 2));
            cv::line(m_contourImg, topLeft, topRight, color, 3);
            cv::line(m_contourImg, topLeft, bottomLeft, color, 3);
            cv::line(m_contourImg, bottomLeft, bottomRight, color, 3);
            cv::line(m_contourImg, bottomRight, topRight, color, 3);
            // draw contours
            cv::drawContours(m_contourImg, m_contours, i, cv::Scalar(255, 0, 0));
        }
        // Center of all images
        cv::Moments moms = cv::moments(m_dilate, true);
        if (moms.m00 != 0)
        {
            m_fullCenter = cv::Point2f(moms.m10 / moms.m00, moms.m01 / moms.m00);
            cv::line(m_contourImg, cv::Point(m_fullCenter.x, 0), cv::Point(m_fullCenter.x, m_contourImg.cols), cv::Scalar(0, 0, 255));
            cv::line(m_contourImg, cv::Point(0, m_fullCenter.y), cv::Point(m_contourImg.cols, m_fullCenter.y), cv::Scalar(0, 0, 255));

            m_distanceFromCenter = (m_contourImg.cols/2) - m_fullCenter.x;

        }

        //How many pixels are between one cornor to the opposite corner of the image
        double pixelDiag = sqrt(pow(m_contourImg.cols, 2) + pow(m_contourImg.rows, 2));
        // Find horizontal fov
        m_fov = 68.5 * (m_contourImg.cols / pixelDiag);
        // What percentage away from image center is the center of the contours
        m_percent = (m_fullCenter.x - (m_contourImg.cols / 2)) / (double)(m_contourImg.cols / 2);
        // Output to PI console
        wpi::outs() << m_percent << '\n';
        m_centerX = m_fullCenter.x;
        m_contourCols = m_contourImg.cols;
        //The angle we tuen is half of the horizontal fov multiplied by the percentage off we are
        m_turnAngle = (m_fov / 2) * m_percent;
        return m_contourImg;
    }

    // Create class object
    Processing processing;

    // output
    cs::CvSource output;

    //Network Entries
    nt::NetworkTableEntry angle;
    nt::NetworkTableEntry fromCenter;
    nt::NetworkTableEntry centerX;
    nt::NetworkTableEntry contourCols;
    //Network table
    std::shared_ptr<nt::NetworkTable> table;

unsigned int team;
bool server = false;

struct CameraConfig {
  std::string name;
  std::string path;
  wpi::json config;
  wpi::json streamConfig;
};

struct SwitchedCameraConfig {
  std::string name;
  std::string key;
};

std::vector<CameraConfig> cameraConfigs;
std::vector<SwitchedCameraConfig> switchedCameraConfigs;
std::vector<cs::VideoSource> cameras;

wpi::raw_ostream& ParseError() {
  return wpi::errs() << "config error in '" << configFile << "': ";
}

bool ReadCameraConfig(const wpi::json& config) {
  CameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read camera name: " << e.what() << '\n';
    return false;
  }

  // path
  try {
    c.path = config.at("path").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "camera '" << c.name
                 << "': could not read path: " << e.what() << '\n';
    return false;
  }

  // stream properties
  if (config.count("stream") != 0) c.streamConfig = config.at("stream");

  c.config = config;

  cameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadSwitchedCameraConfig(const wpi::json& config) {
  SwitchedCameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read switched camera name: " << e.what() << '\n';
    return false;
  }

  // key
  try {
    c.key = config.at("key").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "switched camera '" << c.name
                 << "': could not read key: " << e.what() << '\n';
    return false;
  }

  switchedCameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadConfig() {
  // open config file
  std::error_code ec;
  wpi::raw_fd_istream is(configFile, ec);
  if (ec) {
    wpi::errs() << "could not open '" << configFile << "': " << ec.message()
                << '\n';
    return false;
  }

  // parse file
  wpi::json j;
  try {
    j = wpi::json::parse(is);
  } catch (const wpi::json::parse_error& e) {
    ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
    return false;
  }

  // top level must be an object
  if (!j.is_object()) {
    ParseError() << "must be JSON object\n";
    return false;
  }

  // team number
  try {
    team = j.at("team").get<unsigned int>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read team number: " << e.what() << '\n';
    return false;
  }

  // ntmode (optional)
  if (j.count("ntmode") != 0) {
    try {
      auto str = j.at("ntmode").get<std::string>();
      wpi::StringRef s(str);
      if (s.equals_lower("client")) {
        server = false;
      } else if (s.equals_lower("server")) {
        server = true;
      } else {
        ParseError() << "could not understand ntmode value '" << str << "'\n";
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read ntmode: " << e.what() << '\n';
    }
  }

  // cameras
  try {
    for (auto&& camera : j.at("cameras")) {
      if (!ReadCameraConfig(camera)) return false;
    }
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read cameras: " << e.what() << '\n';
    return false;
  }

  // switched cameras (optional)
  if (j.count("switched cameras") != 0) {
    try {
      for (auto&& camera : j.at("switched cameras")) {
        if (!ReadSwitchedCameraConfig(camera)) return false;
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read switched cameras: " << e.what() << '\n';
      return false;
    }
  }

  return true;
}

cs::UsbCamera StartCamera(const CameraConfig& config) {
  wpi::outs() << "Starting camera '" << config.name << "' on " << config.path
              << '\n';
  auto inst = frc::CameraServer::GetInstance();
  cs::UsbCamera camera{config.name, config.path};
  auto server = inst->StartAutomaticCapture(camera);

  camera.SetConfigJson(config.config);
  camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

  if (config.streamConfig.is_object())
    server.SetConfigJson(config.streamConfig);

  return camera;
}

cs::MjpegServer StartSwitchedCamera(const SwitchedCameraConfig& config) {
  wpi::outs() << "Starting switched camera '" << config.name << "' on "
              << config.key << '\n';
  auto server =
      frc::CameraServer::GetInstance()->AddSwitchedCamera(config.name);

  nt::NetworkTableInstance::GetDefault()
      .GetEntry(config.key)
      .AddListener(
          [server](const auto& event) mutable {
            if (event.value->IsDouble()) {
              int i = event.value->GetDouble();
              if (i >= 0 && i < cameras.size()) server.SetSource(cameras[i]);
            } else if (event.value->IsString()) {
              auto str = event.value->GetString();
              for (int i = 0; i < cameraConfigs.size(); ++i) {
                if (str == cameraConfigs[i].name) {
                  server.SetSource(cameras[i]);
                  break;
                }
              }
            }
          },
          NT_NOTIFY_IMMEDIATE | NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

  return server;
}

// example pipeline
class MyPipeline : public frc::VisionPipeline {
 public:

  void Process(cv::Mat& mat) override {
      if (mat.empty())
      {
          wpi::outs() << "MAT IS EMPTY" << '\n';
      }
      else
      {
          // Process the image that is gathered
         mat = processing.ProcessImg(mat);

         //Output the processed image
	    output.PutFrame(mat);

        //Output networktable entries
        angle.SetDouble(processing.m_turnAngle);
        centerX.SetDouble(processing.m_centerX);
        contourCols.SetDouble(processing.m_contourCols);
        fromCenter.SetDouble(processing.m_percent);
      }
  }
};
}  // namespace

int main(int argc, char* argv[]) {
  if (argc >= 2) configFile = argv[1];

  // read configuration
  if (!ReadConfig()) return EXIT_FAILURE;

  // start NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  if (server) {
    wpi::outs() << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } else {
    wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
    ntinst.StartDSClient();
  }

  //NETWORK TABLES-----
  table = ntinst.GetTable("datatable");
  //Get Entries
  angle = table->GetEntry("Angle");
  fromCenter = table->GetEntry("FromCenter");
  centerX = table->GetEntry("CenterX");
  contourCols = table->GetEntry("ContourCols");

  //OutPut Video
  output = frc::CameraServer::GetInstance()->PutVideo("Display", 160, 120);

  // start cameras
  for (const auto& config : cameraConfigs)
    cameras.emplace_back(StartCamera(config));

  // start switched cameras
  for (const auto& config : switchedCameraConfigs) StartSwitchedCamera(config);


  // start image processing on camera 0 if present
  if (cameras.size() >= 1) {
    std::thread([&] {
      frc::VisionRunner<MyPipeline> runner(cameras[0], new MyPipeline(),
                                           [&](MyPipeline &pipeline) {
        // do something with pipeline results
      });
      /* something like this for GRIP:
      frc::VisionRunner<MyPipeline> runner(cameras[0], new grip::GripPipeline(),
                                           [&](grip::GripPipeline& pipeline) {
        ...
      });
       */
      runner.RunForever();
    }).detach();
  }

  // loop forever
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(10));
}

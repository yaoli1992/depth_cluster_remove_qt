// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdio.h>

#include <string>
#include <thread>

#include "clusterers/image_based_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"

#include "ground_removal/depth_ground_remover.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

//add by yaoli    to show the image or point cloud
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//using namespace cv;   //不能使用这个命名空间　　因为当我们在包含下面的cloud_viewer.h的文件的时候
//使用了ＶＴＫ的库函数，该库函数中与opencv命名规则重复　　导致编译失败
//
 #include <pcl/visualization/cloud_viewer.h>
 #include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


//#include "tclap/CmdLine.h"

using std::string;
using std::to_string;
using namespace depth_clustering;

boost::mutex updateModelMutex;


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
        while (!viewer->wasStopped ())
        {
         viewer->spinOnce (200);
         boost::this_thread::sleep (boost::posix_time::microseconds (200));
        }
}

void ReadData(const string& in_path)
 {
  // delay reading for one second to allow GUI to load
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // now load the data
  fprintf(stderr, "INFO: running on Moosman data\n");

  int min_cluster_size = 20;
  int max_cluster_size = 100000;

  int smooth_window_size = 5;
  Radians ground_remove_angle = 9_deg;
 //add by yaoli
  Radians angle_tollerance = 5_deg;
  //读取png图像
  auto image_reader = FolderReader(in_path, ".png", FolderReader::Order::SORTED);
  //深度图与点云图之间的转换关系的参数读取
  auto config_reader = FolderReader(in_path, "img.cfg");
  //
  auto proj_params_ptr = ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

  //根据获取的深度图　　
  //输入图像以及一些基本的参数　　其中包含了地面角度以及平滑的窗口的大小
  auto depth_ground_remover = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle, smooth_window_size);

  //这个函数是根据上一步去除了地面之后，再进行聚类处理的结果，输入的参数主要是角度约束，最小和最大的聚类的点的个数
  ImageBasedClusterer<LinearImageLabeler<> > clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  //ImageBasedClusterer<LinearImageLabeler<> > clusterer(_rad(0.5), min_cluster_size, max_cluster_size);

  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  depth_ground_remover.AddClient(&clusterer);
  //clusterer.AddClient(visualizer->object_clouds_client());

   pcl::PointCloud<pcl::PointXYZ>::Ptr Point_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer=simpleVis(Point_XYZ);
   boost::thread vthread(&viewerRunner,viewer);

   //boost::mutex::scoped_lock updateLock(updateModelMutex);

  for (const auto& path : image_reader.GetAllFilePaths())
  {
    auto depth_image = MatFromDepthPng(path);
    auto cloud_ptr = Cloud::FromImage(depth_image, *proj_params_ptr);//实现从图像到点云的转化
    time_utils::Timer timer;

    //add by yaoli      use opencv visualization the depth iamge
    //  cv::imshow("image",depth_image);
    //  cv::waitKey(10);




// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
// viewer = simpleVis(Point_XYZ);
// while (!viewer->wasStopped())
// {
//    // pcl::visualization::PointCloudColorHandlerCustom<PointT> add_color1(Point_XYZ, rand() % 255, rand() % 255, rand() % 255);//添加随机颜色
//     viewer->updatePointCloud<pcl::PointXYZ>(Point_XYZ,"cloud");
//     viewer->spinOnce(100);
// }
   //add by yaoli   可以实现了可视化的效果
  //  pcl::visualization::CloudViewer viewer("Cloud VIewer");
  //  viewer.showCloud(Point_XYZ);
  //  while(!viewer.wasStopped())
  //  {

  //  }
    //显示点云数据
    //visualizer->OnNewObjectReceived(*cloud_ptr, 0);

    //这是一个将点云去除地面之后聚类的结果
    depth_ground_remover.OnNewObjectReceived(*cloud_ptr, 0);


    //add by yaoli to visualize the point cloud
    std::cout << cloud_ptr->size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZL>::Ptr pcl_point=cloud_ptr->ToPcl();
    pcl::copyPointCloud(*pcl_point,*Point_XYZ);
    viewer->updatePointCloud<pcl::PointXYZ>(Point_XYZ,"cloud");


    auto current_millis = timer.measure(time_utils::Timer::Units::Milli);

    fprintf(stderr, "INFO: It took %lu ms to process and show everything.\n",
            current_millis);
    uint max_wait_time = 100;
    if (current_millis > max_wait_time)
    {
      continue;
    }
    auto time_to_wait = max_wait_time - current_millis;
    fprintf(stderr, "INFO: Waiting another %lu ms.\n", time_to_wait);
    std::this_thread::sleep_for(std::chrono::milliseconds(time_to_wait));
  }
   // updateLock.unlock();
  //  boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    vthread.join();
}

int main(int argc, char* argv[]) {
  // TCLAP::CmdLine cmd(
  //     "Loads clouds from Frank Moosmann's data and clusters each of them.", ' ',
  //     "1.0");
  // TCLAP::ValueArg<int> angle_arg(
  //     "", "angle",
  //     "Threshold angle. Below this value, the objects are separated", false, 10,
  //     "int");
  // TCLAP::ValueArg<string> path_to_data_arg(
  //     "", "path", "Path to folder that stores the data", true, "", "string");

  // cmd.add(angle_arg);
  // cmd.add(path_to_data_arg);
  // cmd.parse(argc, argv);

 // Radians angle_tollerance = Radians::FromDegrees(angle_arg.getValue());
  string in_path ="/home/baidu/Yaoli/demo/depth_cluster_remove_qt/build/demo/data/scenario1"; //path_to_data_arg.getValue();
  fprintf(stderr, "INFO: Reading from: %s \n", in_path.c_str());

  // QApplication application(argc, argv);
  // // visualizer should be created from a gui thread
  // Visualizer visualizer;
  // visualizer.show();

  // create and run loader thread
  std::thread loader_thread(ReadData, in_path);

  // if we close the qt application we will be here
  // auto exit_code = application.exec();

  // join thread after the application is dead
   loader_thread.join();
 // return exit_code;
  return 0;
}

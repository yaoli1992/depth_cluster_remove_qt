/*
说明：这里读取的数据无论是深度图还是bin的点云图 流程中首先是将这些数据转换到点云

并且根据输入点云数据的激光线数等基本条件生成或者决定这里面的角度图的宽和高等图像的基本

信息，然后用这些信息将点云数据转换到图像下，进行一定的搜索地面的算法。



*/
#include <stdio.h>
#include <qapplication.h>
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

//using namespace cv;   //不能使用这个命名空间　　因为当我们在包含下面的cloud_viewer.h的文件的时候
//使用了ＶＴＫ的库函数，该库函数中与opencv命名规则重复　　导致编译失败

 #include <pcl/visualization/cloud_viewer.h>
 #include <boost/thread/thread.hpp>
 #include <pcl/visualization/pcl_visualizer.h>

using std::string;
using namespace depth_clustering;

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



void ReadData(const Radians& angle_tollerance, const string& in_path)
 {
  // delay reading for one second to allow GUI to load
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // now load the data
  fprintf(stderr, "INFO: running on kitti data\n");

  int min_cluster_size = 20;//最小的点的个数
  int max_cluster_size = 100000;

  int smooth_window_size = 7;//
  Radians ground_remove_angle = 5_deg;//7_deg

  auto cloud_reader = FolderReader(in_path, ".bin", FolderReader::Order::SORTED);//读取的文件资源的位置

  auto proj_params_ptr = ProjectionParams::HDL_32();//设置该输入的激光雷达的信息是多少线的

  auto depth_ground_remover = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle, smooth_window_size);//将根据输入的窗口大小以及我们设置的地面角度阈值  从而生成映射成图片的参数

  // ImageBasedClusterer<LinearImageLabeler<>> clusterer(
  //     angle_tollerance, min_cluster_size, max_cluster_size);
  
  //delete by yaoli 暂时不考虑聚类的问题
  //clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  //depth_ground_remover.AddClient(&clusterer);
  //clusterer.AddClient(visualizer->object_clouds_client());

   //add by yaoli 增加一个使用PCL可视化的线程
   pcl::PointCloud<pcl::PointXYZ>::Ptr Point_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer=simpleVis(Point_XYZ);
   boost::thread vthread(&viewerRunner,viewer);

 //理解以上是我们手动输入的一些关于数据的基本信息，生成一些基本的生成图像信息的参数。
  fprintf(stderr, "INFO: everything initialized\n");

  for (auto path : cloud_reader.GetAllFilePaths()) {
    time_utils::Timer timer;
    auto cloud = ReadKittiCloud(path);//深度图和bin的数据转换到点云再从点云数据转换到这里所描述的角度形成的图像
    
    cloud->InitProjection(*proj_params_ptr);

   // visualizer->OnNewObjectReceived(*cloud, 0);
    depth_ground_remover.OnNewObjectReceived(*cloud, 0);

    //add by yaoli 2019-12-12 为了将我们提取出来的平面的图像点再转换成点云
    cv::Mat noground=cloud->projection_ptr()->depth_image();

    //  cv::imshow("image",noground);
    //  cv::waitKey(500);
  //auto cloud_ptr = Cloud::FromImage(noground, *proj_params_ptr);//实现从图像到点云的转化
  //auto cloud_ptr =  CloudProjection::UnprojectPoint(noground,noground.rows,noground.cols);


   //add by yaoli to visualize the point cloud
    std::cout << cloud->size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZL>::Ptr pcl_point=cloud->ToPcl();
    pcl::copyPointCloud(*pcl_point,*Point_XYZ);
    viewer->updatePointCloud<pcl::PointXYZ>(Point_XYZ,"cloud");


    uint max_wait_time = 100;
    auto current_millis = timer.measure(time_utils::Timer::Units::Milli);
    fprintf(stderr, "INFO: It took %lu ms to process.\n", current_millis);
    if (current_millis > max_wait_time) {
      continue;
    }
    auto time_to_wait = max_wait_time - current_millis;
    fprintf(stderr, "INFO: Waiting another %lu ms.\n", time_to_wait);
    std::this_thread::sleep_for(std::chrono::milliseconds(time_to_wait));
  }
}




int main(int argc, char* argv[]) {
  string in_path ="/home/baidu/Yaoli/demo/depth_cluster_remove_qt/bin2pcd/build/bin1"; //path_to_data_arg.getValue();
  fprintf(stderr, "INFO: Reading from: %s \n", in_path.c_str());
   Radians angle_tollerance = 5_deg;
  // create and run loader thread
  std::thread loader_thread(ReadData, angle_tollerance, in_path);
  loader_thread.join();

}

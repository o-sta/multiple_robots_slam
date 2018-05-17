#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <limits>

#include <new_exploration_programs/segmented_cloud.h>

//#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h>

//#include <tf/transform_listener.h>

//#include <pcl_ros/transforms.h>

//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
//#include <geometry_msgs/TransformStamped.h>

#include <pcl/registration/icp.h>

#include <pcl/io/pcd_io.h>

class ScanMatching
{
private:
  ros::NodeHandle psm;

	ros::NodeHandle ssd;
  ros::NodeHandle sod;

  ros::Subscriber sd_sub;
  ros::Subscriber od_sub;

  ros::Publisher mc_pub;

  //ros::Publisher t1_pub;
  //ros::Publisher t2_pub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr pre_input_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud;

  nav_msgs::Odometry input_odom;
  nav_msgs::Odometry pre_input_odom;

  //nav_msgs::Odometry estimate_odom;

  sensor_msgs::PointCloud2 merged_cloud_r;

  //tf::TransformListener tflistener;
  //geometry_msgs::TransformStamped transform;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr input_tf_cloud;
  //sensor_msgs::PointCloud2 input_msg;
  //sensor_msgs::PointCloud2 output_msg;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr tfin_cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr tfout_cloud;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr icpout_cloud;

  pcl::VoxelGrid<pcl::PointXYZ> vg;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr del_unval_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr del_ceiling_cloud;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_input_cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr re_voxeled_input_cloud;

  float camera_position_y;
  float ceiling_position_y;
  float floor_position_y;
  float per_range_z;

  float x;
  float z;
  float move_x;
  float move_z;
  float move_th;

  Eigen::Matrix3f R_estimate;
  Eigen::Vector3f T_estimate;

public:
  ros::CallbackQueue sd_queue;
  ros::CallbackQueue od_queue;
  bool input_c;
  bool input_o;

  ScanMatching()
  :input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //pre_input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  merged_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //input_tf_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //tfin_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //tfout_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  icpout_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  //del_unval_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  //merged_cloud2(new pcl::PointCloud<pcl::PointXYZ>)
  //del_ceiling_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  voxeled_input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  re_voxeled_input_cloud(new pcl::PointCloud<pcl::PointXYZ>)
  {
    ssd.setCallbackQueue(&sd_queue);
    sd_sub = ssd.subscribe("/camera/depth_registered/points",1,&ScanMatching::input_pointcloud,this);
    sod.setCallbackQueue(&od_queue);
    od_sub = sod.subscribe("/odom",1,&ScanMatching::input_odometry,this);
    input_c = false;
    input_o = false;
    mc_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/merged_cloud", 1);

    //t1_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/test_cloud1", 1);
    //t2_pub = psm.advertise<sensor_msgs::PointCloud2>("scan_matching/test_cloud2", 1);

    //transform.header.frame_id = "odom";
    //transform.child_frame_id = "camera_rgb_optical_frame";

    //vg.setLeafSize (0.1f, 0.1f, 0.1f);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.10);
    // Set the maximum number of iterations (criterion 1)
    //icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    //icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (1);

    camera_position_y = 0.41;
    ceiling_position_y = 2.3;
    floor_position_y = 0.3;
    per_range_z = 6.0;

    R_estimate = Eigen::Matrix3f::Identity();
    T_estimate = Eigen::Vector3f::Zero();
  };
  ~ScanMatching(){};

  void input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
  void input_odometry(const nav_msgs::Odometry::ConstPtr& od_msg);
  void voxeling(void);
  void remove_unrelialbe(void);
  void convert3dto2d(void);
  void odomove_cloud(void);
  void icp_test(void);
  void myICP(void);
  float calc_cost(const float dx,const float dz,const float dth);
  void publish_mergedcloud(void);
};

void ScanMatching::input_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
  //input_msg = *pc_msg;
	pcl::fromROSMsg (*pc_msg, *input_cloud);
	std::cout << "input_pointcloud" << std::endl;
	input_c = true;
}

void ScanMatching::input_odometry(const nav_msgs::Odometry::ConstPtr& od_msg)
{
  // double dth;
  // double dth2;
  // double dth3;
  // double diff;

  input_odom = *od_msg;
  std::cout << "input_odometry" << std::endl;
  // if(input_o)
  // {
  //   dth = 2*asin(input_odom.pose.pose.orientation.z) - 2*asin(pre_input_odom.pose.pose.orientation.z);
  //   dth2 = 2*asin(input_odom.pose.pose.orientation.z - pre_input_odom.pose.pose.orientation.z);
  //   dth3 = 2*asin(input_odom.pose.pose.orientation.z*pre_input_odom.pose.pose.orientation.w - pre_input_odom.pose.pose.orientation.z*input_odom.pose.pose.orientation.w);
  //   diff = (dth2 - dth)*100/dth;
  // }

  input_o = true;
  // std::cout << "now : " << 2*asin(input_odom.pose.pose.orientation.z)*180/M_PI << ", pre : " << 2*asin(pre_input_odom.pose.pose.orientation.z)*180/M_PI << '\n';
  // std::cout << "bara : " << dth*180/M_PI << ", mato : " << dth2*180/M_PI << ", dai : " << dth3*180/M_PI << '\n' << '\n';
  // pre_input_odom = input_odom;
}

void ScanMatching::voxeling(void)
{
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.setInputCloud (input_cloud);
	//vg.filter (*del_unval_cloud);
  vg.filter (*voxeled_input_cloud);
}

void ScanMatching::remove_unrelialbe(void)
{
  // float camera_position_y = 0.41;
  // float ceiling_position_y = 2.3;
  // float floor_position_y = 0.3;
  // float per_range_z = 6.0;
  //float nan_c = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for(int i=0;i<voxeled_input_cloud->points.size();i++)
	{
		if(voxeled_input_cloud->points[i].y > camera_position_y - ceiling_position_y && voxeled_input_cloud->points[i].y < camera_position_y - floor_position_y) //&& voxeled_input_cloud->points[i].z < per_range_z)
		{
      remove_cloud->points.push_back(voxeled_input_cloud->points[i]);
		}
	}
  *voxeled_input_cloud = *remove_cloud;
  //vg.setLeafSize (0.1f, 0.1f, 0.1f);
  //vg.setInputCloud (del_unval_cloud);
	//vg.filter (*del_unval_cloud);
  //vg.filter (*merged_cloud);
  //*merged_cloud = *del_unval_cloud;
  voxeled_input_cloud -> width = voxeled_input_cloud -> points.size();
  voxeled_input_cloud -> height = 1;
  voxeled_input_cloud -> is_dense = false;
}

void ScanMatching::convert3dto2d(void)
{
  for(int i=0;i<voxeled_input_cloud->points.size();i++)
	{
    voxeled_input_cloud->points[i].y = camera_position_y;
	}

  vg.setLeafSize (0.2f, 0.2f, 0.2f);
  vg.setInputCloud (voxeled_input_cloud);
  vg.filter (*re_voxeled_input_cloud);

  *voxeled_input_cloud = *re_voxeled_input_cloud;
}

void ScanMatching::odomove_cloud(void)
{
  move_x = -input_odom.pose.pose.position.y;
  move_z = input_odom.pose.pose.position.x;
  move_th = 2*asin(input_odom.pose.pose.orientation.z);

  // float move_x = input_odom.pose.pose.position.x;
  // float move_y = input_odom.pose.pose.position.y;
  // float move_s = 2*asin(input_odom.pose.pose.orientation.z);

  //std::cout << "move_x: " << move_x;
  //std::cout << ", move_z: " << move_z;
  //std::cout << ", move_s: " << move_s*180/M_PI << '\n';

  for(int i=0;i<voxeled_input_cloud->points.size();i++)
  {
    x = voxeled_input_cloud->points[i].x;
    z = voxeled_input_cloud->points[i].z;

    x = x*cos(move_th)-z*sin(move_th);
    z = x*sin(move_th)+z*cos(move_th);

    voxeled_input_cloud->points[i].x = x + move_x;
    voxeled_input_cloud->points[i].z = z + move_z;
  }
}

void ScanMatching::icp_test(void)
{
  if(merged_cloud->points.size()!=0)
  {
    std::cout << "icp" << '\n';
    //IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    // Set the input source and target
    //icp.setInputSource (del_unval_cloud);
    icp.setInputSource (voxeled_input_cloud);
    icp.setInputTarget (merged_cloud);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    //icp.setMaxCorrespondenceDistance (0.10);
    // Set the maximum number of iterations (criterion 1)
    //icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    //icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (1);
    // Perform the alignment
    icp.align (*icpout_cloud);
    //icp.computeTransformation
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();
    transformation_matrix = icp.getFinalTransformation ().cast<float>();
    std::cout << transformation_matrix << '\n';
    /*
    R
    matrix (0, 0), matrix (0, 1), matrix (0, 2);
    matrix (1, 0), matrix (1, 1), matrix (1, 2)
    matrix (2, 0), matrix (2, 1), matrix (2, 2);

    T
    matrix (0, 3), matrix (1, 3), matrix (2, 3)
    */
  }
  else
  {
    *icpout_cloud = *voxeled_input_cloud;
  }

  *merged_cloud += *icpout_cloud;
}


void ScanMatching::myICP(void)
{
  //voxeled_input_cloud

  if(merged_cloud->points.size() > 0)
  {
    /*推定用に点群をコピー*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr clone_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clone_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*voxeled_input_cloud, *clone_input_cloud);
    pcl::copyPointCloud(*merged_cloud, *clone_merged_cloud);

    /*オドメトリの変化値*/
    float initX = -input_odom.pose.pose.position.y + pre_input_odom.pose.pose.position.y;
    float initZ = input_odom.pose.pose.position.x - pre_input_odom.pose.pose.position.x;
    float initTH = 2*asin(input_odom.pose.pose.orientation.z*pre_input_odom.pose.pose.orientation.w - pre_input_odom.pose.pose.orientation.z*input_odom.pose.pose.orientation.w);

    if(input_odom.pose.pose.orientation.z*pre_input_odom.pose.pose.orientation.z < 0)
    {
      initTH *= -1;
    }

    std::cout << "dx: " << initX << ", dz: " << initZ << ", dth: " << initTH << '\n';

    /*init_move*/
    Eigen::Matrix3f R0_estimate;
    Eigen::Vector3f T0_estimate;
    Eigen::Vector3f point_vector;
    Eigen::Vector3f move_point_vector;

    Eigen::Matrix3f c_R_estimate;
    Eigen::Vector3f c_T_estimate;


    /*初期値を代入*/
    R0_estimate << cos(-initTH),0,sin(-initTH),0,1,0,-sin(-initTH),0,cos(-initTH);
    T0_estimate << initX,0,initZ;

    //std::cout << R0_estimate << '\n';
    //std::cout << T0_estimate << '\n' << '\n';


    /*前回の推定値に変化値を足して初期値とする*/
    //R_estimate = R_estimate + R0_estimate;
    R_estimate = R0_estimate*R_estimate;
    T_estimate = T_estimate + T0_estimate;

    //std::cout << R_estimate << '\n';
    //std::cout << T_estimate << '\n' << '\n';


    /*最初に一回動かして、そのあと対応付けをする*/
    for (int i=0;i<clone_input_cloud->points.size();i++)
    {
      point_vector << clone_input_cloud->points[i].x,clone_input_cloud->points[i].y,clone_input_cloud->points[i].z;
      move_point_vector = R_estimate*point_vector + T_estimate;
      clone_input_cloud->points[i].x = move_point_vector(0);
      clone_input_cloud->points[i].y = move_point_vector(1);
      clone_input_cloud->points[i].z = move_point_vector(2);
    }


    // for (int i=0;i<voxeled_input_cloud->points.size();i++)
    // {
    //   point_vector << voxeled_input_cloud->points[i].x,voxeled_input_cloud->points[i].y,voxeled_input_cloud->points[i].z;
    //   move_point_vector = R_estimate*point_vector + T_estimate;
    //   voxeled_input_cloud->points[i].x = move_point_vector(0);
    //   voxeled_input_cloud->points[i].y = move_point_vector(1);
    //   voxeled_input_cloud->points[i].z = move_point_vector(2);
    // }

    /*この時点での最近傍点で対応付け*/

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud (merged_cloud);
    kdtree->setEpsilon(0.10);

    pcl::PointXYZ searchPoint;
    std::vector<int> v_nearest_point_index(1);
    int nearest_point_index;
    std::vector<int> index_list;

    std::vector<float> v_point_distance(1);
    float point_distance;
    std::vector<float> distance_list;


    /*最近傍点を検索して対応付け*/
    for(int i=0;i<voxeled_input_cloud->points.size();i++)
    {
      searchPoint.x = voxeled_input_cloud->points[i].x;
      searchPoint.y = voxeled_input_cloud->points[i].y;
      searchPoint.z = voxeled_input_cloud->points[i].z;

      if ( kdtree->nearestKSearch (searchPoint, 1, v_nearest_point_index, v_point_distance) > 0 )
      {
        nearest_point_index = v_nearest_point_index[0];
        point_distance = v_point_distance[0];
        index_list.push_back(nearest_point_index);
        distance_list.push_back(point_distance);
      }
      else
      {
        index_list.push_back(-1);
        distance_list.push_back(-1);
      }
    }




    /*対応点間の誤差の二乗平均の初期値を計算*/
    float ave_squr_distance = 0;

    for(int i=0; i<distance_list.size();i++)
    {
      ave_squr_distance += distance_list[i];
    }
    ave_squr_distance /= (float)distance_list.size();

    /*誤差の二乗平均が最小になるようなRTを推定する*/
    /*変化できる値　x,z,th*/

    float pre_ave_squr_distance;
    float dx;
    float dz;
    float dth;

    /*過去の移動を含めてを推定する場合*/
    dx = T_estimate(0);
    dz = T_estimate(2);
    dth = acos(R_estimate(0.0));

    /*現在の移動のみを推定する場合*/
    dx = initX;
    dz = initZ;
    dth = initTH;



    new_ave_squr_distance = calc_cost(dx,dz,dth);


    for (int i=0;i<voxeled_input_cloud->points.size();i++)
    {
      point_vector << voxeled_input_cloud->points[i].x,voxeled_input_cloud->points[i].y,voxeled_input_cloud->points[i].z;
      move_point_vector = R_estimate*point_vector + T_estimate;
      voxeled_input_cloud->points[i].x = move_point_vector(0);
      voxeled_input_cloud->points[i].y = move_point_vector(1);
      voxeled_input_cloud->points[i].z = move_point_vector(2);
    }

  }

  pre_input_odom = input_odom;


  *merged_cloud += *voxeled_input_cloud;

}

void ScanMatching::icp_main(void)
{
  if(merged_cloud->points.size() > 0)
  {
    /*初期設定*/


    while(true)
    {



      /*dx.dz.dthを変化*/

      /*このときのコストを計算*/
      ave_sqrt_distance = calc_cost(dx,dz,dth);

      /*コストの計算結果が今までで最小であれば更新*/
      if(ave_sqrt_distance < min_ave_sqrt_distance)
      {
        min_ave_sqrt_distance = ave_sqrt_distance;
        min_dx = dx;
        min_dz = dz;
        min_dth = dth;
      }

      if(true/*繰り返し終了条件を満たしていたら*/)
      {
        /*繰り返しの終了条件を満たしたら推定値を保存*/
        R_estimate = c_R_estimate;
        T_estimate = c_T_estimate;

        /*最後に推定した結果で点群を動かす*/
        for (int i=0;i<voxeled_input_cloud->points.size();i++)
        {
          point_vector << voxeled_input_cloud->points[i].x,voxeled_input_cloud->points[i].y,voxeled_input_cloud->points[i].z;
          move_point_vector = R_estimate*point_vector + T_estimate;
          voxeled_input_cloud->points[i].x = move_point_vector(0);
          voxeled_input_cloud->points[i].y = move_point_vector(1);
          voxeled_input_cloud->points[i].z = move_point_vector(2);
        }
        break;
      }
    }
  }

  pre_input_odom = input_odom;


  *merged_cloud += *voxeled_input_cloud;
}

float ScanMatching::calc_cost(const float dx,const float dz,const float dth)
{
  /*受け取った引数でコストを計算する*/

  /*cloudコピー*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr clone_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr clone_merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*voxeled_input_cloud, *clone_input_cloud);
  //pcl::copyPointCloud(*merged_cloud, *clone_merged_cloud);

  R0_estimate << cos(-dth),0,sin(-dth),0,1,0,-sin(-dth),0,cos(-dth);
  T0_estimate << dx,0,dz;

  c_R_estimate = Eigen::Matrix3f::Identity();
  c_T_estimate = Eigen::Vector3f::Zero();

  c_R_estimate = R0_estimate * R_estimate;
  c_T_estimate = T_estimate + T0_estimate;

  for (int i=0;i<clone_input_cloud->points.size();i++)
  {
    point_vector << clone_input_cloud->points[i].x,clone_input_cloud->points[i].y,clone_input_cloud->points[i].z;
    move_point_vector = c_R_estimate*point_vector + c_T_estimate;
    clone_input_cloud->points[i].x = move_point_vector(0);
    clone_input_cloud->points[i].y = move_point_vector(1);
    clone_input_cloud->points[i].z = move_point_vector(2);
  }

  /*if初回の計算だったらツリーを構築する*/
  if(true/*初回計算だったら*/)
  {
    kdtree->setInputCloud (merged_cloud);
    kdtree->setEpsilon(0.10);

    pcl::PointXYZ searchPoint;
    std::vector<int> v_nearest_point_index(1);
    int nearest_point_index;
    std::vector<int> index_list;

    std::vector<float> v_point_distance(1);
    float point_distance;
    std::vector<float> distance_list;


    /*最近傍点を検索して対応付け*/
    for(int i=0;i<voxeled_input_cloud->points.size();i++)
    {
      searchPoint.x = voxeled_input_cloud->points[i].x;
      searchPoint.y = voxeled_input_cloud->points[i].y;
      searchPoint.z = voxeled_input_cloud->points[i].z;

      if ( kdtree->nearestKSearch (searchPoint, 1, v_nearest_point_index, v_point_distance) > 0 )
      {
        nearest_point_index = v_nearest_point_index[0];
        point_distance = v_point_distance[0];
        index_list.push_back(nearest_point_index);
        distance_list.push_back(point_distance);
      }
      else
      {
        index_list.push_back(-1);
        distance_list.push_back(-1);
      }
    }

  }

  float diff_x;
  float diff_y;
  float diff_z;
  /*対応点間の距離を計算*/
  for(int i=0;i<index_list.size();i++)
  {
    if(index_list[i] >= 0)
    {
      diff_x = clone_input_cloud->points[i].x - merged_cloud->points[index_list[i]].x;
      diff_y = clone_input_cloud->points[i].y - merged_cloud->points[index_list[i]].y;
      diff_x = clone_input_cloud->points[i].z - merged_cloud->points[index_list[i]].z;
      ave_sqrt_distance += pow(diff_x,2) + pow(diff_y,2) + pow(diff_z,2);
    }
  }
  ave_squr_distance /= (float)index_list.size();

  return ave_sqrt_distance;

}


void ScanMatching::publish_mergedcloud(void)
{
  //*merged_cloud = *voxeled_input_cloud;
  pcl::toROSMsg (*merged_cloud, merged_cloud_r);
  merged_cloud_r.header.frame_id = "camera_rgb_optical_frame";
  //merged_cloud_r.header.frame_id = "odom";
  mc_pub.publish(merged_cloud_r);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_matching");
	ScanMatching sm;

  ros::Rate rate(2);

  while(ros::ok()){
    sm.sd_queue.callOne(ros::WallDuration(1));
    sm.od_queue.callOne(ros::WallDuration(1));
    if(sm.input_c && sm.input_o)
    {
      sm.voxeling();
      sm.remove_unrelialbe();
      //sm.convert3dto2d();
      //sm.odomove_cloud();
      //sm.icp_test();
      sm.myICP();
      sm.publish_mergedcloud();
    }
    else
    {
      std::cout << '\n' << "not input" << '\n';
    }
    sm.input_c = false;
    sm.input_o = false;
    rate.sleep();
  }
	return 0;
}

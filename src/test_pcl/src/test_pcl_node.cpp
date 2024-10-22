#include <iostream>
#include <math.h>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <CSF.h>
#include <time.h>
#include "test_pcl_node.h"
#include <unordered_set>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/boundary.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/search/organized.h>
#include <pcl/features/don.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/conditional_removal.h>


using namespace std;
const char* config_file_path = "/home/bjersgen2004/pc_new/src/test_pcl/src/params.cfg";
float distance_threshold = 1.0;
std::ostringstream os;
pcl::PCDWriter writer;

// ros::Publisher pub_ply_map;

void PointCloud2Vector2d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::on_nurbs::vector_vec2d &data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		pcl::PointXYZRGB &p = cloud->at(i);
		if (!pcl_isnan(p.x) && !pcl_isnan(p.y))
			data.push_back(Eigen::Vector2d(p.x, p.y));
	}
}
void PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{

  for (unsigned i = 0; i < cloud->size (); i++)

  {

    pcl::PointXYZ &p = cloud->at (i);

    if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))

      data.push_back (Eigen::Vector3d (p.x, p.y, p.z));

  }

}
int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float re,float reforn, int k) 
{ 

    pcl::PointCloud<pcl::Boundary> boundaries; 
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; 
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>); 
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
    normEst.setRadiusSearch(reforn); 
    normEst.compute(*normals); 

    boundEst.setInputCloud(cloud); 
    boundEst.setInputNormals(normals); 
    boundEst.setRadiusSearch(re); 
    boundEst.setAngleThreshold(M_PI/4); 
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); 
    boundEst.compute(boundaries); 

    for(int i = 0; i < cloud->points.size(); i++) 
    { 
        
        if(boundaries[i].boundary_point > 0) 
        { 
            cloud_boundary->push_back(cloud->points[i]); 
        } 
    }

    pcl::PCDWriter writer;
    cloud_boundary->height=1;
    cloud_boundary->width=cloud_boundary->points.size();
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << k;
    writer.write<pcl::PointXYZ>("/home/bjersgen2004/pc_new/cloud_boundary/Cloud_boundary_" + ss.str()+".pcd", *cloud_boundary, false);

    return 0; 
} 
void CSF_addPointCloud(const vector<int>& index_vec, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
    auto& points = cloud_filtered->points;
    const auto& pointclouds = cloud->points;

    for_each(index_vec.begin(), index_vec.end(), [&](const auto& index) {
        pcl::PointXYZRGB pc;
        pc.x = pointclouds[index].x;
        pc.y = pointclouds[index].y;
        pc.z = pointclouds[index].z;
        // pc.intensity = pointclouds[index].intensity;
        pc.b = 127;

        points.push_back(pc);
    });

    cloud_filtered->height = 1;
    cloud_filtered->width = cloud_filtered->points.size();
}
void clothSimulationFilter(const vector< csf::Point >& pc,vector<int> &groundIndexes,vector<int> & offGroundIndexes)
{
    //step 1 read point cloud
    Cfg cfg;
    std::string slop_smooth;
    cfg.readConfigFile(config_file_path, "slop_smooth", slop_smooth);
    bool ss = false;
    if (slop_smooth == "true" || slop_smooth == "True")
    {
        	ss = true;
    }
    else if (slop_smooth == "false" || slop_smooth == "False")
    {
        	ss = false;
    }
    else{
        if (atoi(slop_smooth.c_str()) == 0){	
            ss = false;
        }
        else
        {
            ss = true;
        }
    }
    std::string class_threshold;
    cfg.readConfigFile(config_file_path, "class_threshold", class_threshold);
    std::string cloth_resolution;
    cfg.readConfigFile(config_file_path, "cloth_resolution", cloth_resolution);
    std::string interations;
    cfg.readConfigFile(config_file_path, "interations", interations);
    std::string rigidness;
    cfg.readConfigFile(config_file_path, "rigidness", rigidness);
    std::string time_step;
    cfg.readConfigFile(config_file_path, "time_step", time_step);

    std::cout<< "class_threshold" << class_threshold << endl;
    CSF csf;
    csf.setPointCloud(pc);// or csf.readPointsFromFile(pointClouds_filepath); 
    //pc can be vector< csf::Point > or PointCloud defined in point_cloud.h

    //step 2 parameter settings
    //Among these paramters:  
    //time_step  interations class_threshold can remain as defualt in most cases.
    csf.params.bSloopSmooth = ss;
    csf.params.cloth_resolution = atof(cloth_resolution.c_str());
    csf.params.rigidness = atoi(rigidness.c_str());

    csf.params.time_step = atof(time_step.c_str());
    csf.params.class_threshold = atof(class_threshold.c_str());
    csf.params.interations = atoi(interations.c_str());

    //step 3 do filtering
    //result stores the index of ground points or non-ground points in the original point cloud
    
    csf.do_filtering(groundIndexes, offGroundIndexes);
    //csf.do_filtering(groundIndexes, offGroundIndexes,true); 
    //if the third parameter is set as true, then a file named "cloth_nodes.txt" will be created, 
    //it respresents the cloth nodes.By default, it is set as false

}
void projectToXYPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud) {
    for (const auto& point : input_cloud->points) {
        pcl::PointXYZ projected_point;
        projected_point.x = point.x;
        projected_point.y = point.y;
        projected_point.z = 0;  // Project onto XY-plane

        output_cloud->points.push_back(projected_point);
    }
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
}
void projectToXYPlaneRGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud) {
    for (const auto& point : input_cloud->points) {
        pcl::PointXYZRGB projected_point;
        projected_point.x = point.x;
        projected_point.y = point.y;
        projected_point.z = 0;  // Project onto XY-plane
        projected_point.r = point.r;
        projected_point.b = point.b;
        projected_point.g = point.g;
        output_cloud->points.push_back(projected_point);
    }
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
}

// 合并地面点云与每个凹包点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeGroundAndConcaveHulls(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ground_cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &concave_hull_clouds)
{
    // 创建一个新的点云对象用于合并
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 将地面点云添加到合并的点云中
    for (const auto& point : ground_cloud->points) {
        merged_cloud->points.push_back(point);
    }

    // 遍历所有凹包点云，将每个聚类的凹包点云添加到合并的点云中
    for (size_t i = 0; i < concave_hull_clouds.size(); ++i) {
        const auto& hull_cloud = concave_hull_clouds[i];
        
        // 为每个凹包设置颜色，以区分不同的聚类
        for (const auto& point : hull_cloud->points) {
            pcl::PointXYZRGB color_point;
            color_point.x = point.x;
            color_point.y = point.y;
            color_point.z = point.z;
            
            // 设置颜色，这里可以根据聚类 ID 来设置不同颜色
            color_point.r = (i * 50) % 256;
            color_point.g = (i * 80) % 256;
            color_point.b = (i * 110) % 256;

            merged_cloud->points.push_back(color_point);
        }
    }

    // 设置合并点云的属性
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    return merged_cloud;
}

void visualizeMergedPointCloudWithBoundaries(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &merged_cloud, 
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &concave_hull_clouds)
{
    // 创建 PCLVisualizer 对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Merged Cloud with Boundaries"));
    viewer->setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色

    // 为点云设置颜色处理器
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(merged_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(merged_cloud, rgb, "merged_cloud");

    // 设置显示属性
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "merged_cloud");

    // 添加凹包的边界线
    for (size_t cluster_id = 0; cluster_id < concave_hull_clouds.size(); ++cluster_id) {
        const auto& hull_cloud = concave_hull_clouds[cluster_id];

        // 循环连接每个凹包点形成闭合轮廓
        for (size_t i = 0; i < hull_cloud->points.size(); ++i) {
            pcl::PointXYZ p1 = hull_cloud->points[i];
            pcl::PointXYZ p2 = hull_cloud->points[(i + 1) % hull_cloud->points.size()];  // 连接最后一个点到第一个点
            std::stringstream line_id;
            line_id << "line_" << cluster_id << "_" << i;
            viewer->addLine(p1, p2, 1.0, 0.0, 0.0, line_id.str());  // 设置线的颜色为红色
        }
    }

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // 启动可视化
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);  // 每100毫秒更新一次
    }
}

void publishMultipleConcaveHulls(ros::Publisher& pub, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& hulls, const std::string& frame_id) {
    for (size_t i = 0; i < hulls.size(); ++i) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = frame_id;
        line_strip.header.stamp = ros::Time::now();
        line_strip.id = i;
        line_strip.ns = "concave_hull";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // 设置线的颜色，每个凹包不同颜色
        line_strip.scale.x = 0.05;  // 线宽
        line_strip.color.r = (i * 50) % 256 / 255.0;
        line_strip.color.g = (i * 80) % 256 / 255.0;
        line_strip.color.b = (i * 110) % 256 / 255.0;
        line_strip.color.a = 1.0;

        // 添加凹包边界点
        for (const auto& point : hulls[i]->points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            line_strip.points.push_back(p);
        }

        // 闭合凹包边界
        if (!line_strip.points.empty()) {
            line_strip.points.push_back(line_strip.points.front());
        }

        // 发布 Marker
        pub.publish(line_strip);
    }
}

int main (int argc, char **argv)
{


    std::cout << "BEGIN BEGIN" << std::endl;
    clock_t start, end;
    start = clock();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_nonground (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ground (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_nearby (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr after_sege (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr after_sege_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_test_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/bjersgen2004/Documents/new.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}

    std::cerr << "Cloud before filtering for ground : " << std::endl;
    std::cerr << *cloud << std::endl;

    //using csf
    vector<csf::Point> pc;
    const auto& pointclouds = cloud->points;
    pc.resize(cloud->size());
    transform(pointclouds.begin(), pointclouds.end(), pc.begin(), [&](const auto& p)->csf::Point {
        csf::Point pp;
        pp.x = p.x;
        pp.y = p.y;
        pp.z = p.z;
        return pp;
        });

    std::vector<int> groundIndexes, offGroundIndexes;
    clothSimulationFilter(pc, groundIndexes, offGroundIndexes);
    
    CSF_addPointCloud(groundIndexes, cloud,cloud_filtered_ground);
    writer.write<pcl::PointXYZRGB>("groundPointCloud.pcd", *cloud_filtered_ground, false);

    CSF_addPointCloud(offGroundIndexes, cloud,cloud_filtered_nonground);
    writer.write<pcl::PointXYZRGB>("nonGroundPointCloud.pcd", *cloud_filtered_nonground, false);

    cout<<"cloud_filtered_ground points:"<<cloud_filtered_ground->points.size() 
    << "cloud_filtered_nonground points:"<<cloud_filtered_nonground->points.size()<<endl;


    //Create a KD-Tree for the cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearby_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree4;
    kdtree4.setInputCloud(cloud_filtered_nonground);

    // Use a set to store indices to avoid duplicates
    std::unordered_set<int> all_nearby_indices;

    // Loop through each ground point and find the nearby points within the threshold
    for (const auto& ground_point : cloud_filtered_ground->points) {
        std::vector<int> point_indices;
        std::vector<float> point_squared_distances;

        if (kdtree4.radiusSearch(ground_point, distance_threshold, point_indices, point_squared_distances) > 0) {
            all_nearby_indices.insert(point_indices.begin(), point_indices.end());
        }
    }

    // Reserve space for flatpak run org.cloudcompare.CloudComparenearby_points to improve memory allocation efficiency
    nearby_points->reserve(all_nearby_indices.size());

    // Extract the nearby points using the indices
    for (const auto& idx : all_nearby_indices) {
        nearby_points->points.push_back(cloud_filtered_nonground->points[idx]);
    }

    nearby_points->width = nearby_points->points.size();
    nearby_points->height = 1;
    nearby_points->is_dense = true;

    cout<<"groundnearby points:" <<nearby_points->points.size()<<endl;
    writer.write<pcl::PointXYZRGB>("GroundNearbyPointCloud.pcd", *nearby_points, false);


    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/bjersgen2004/pc_new/GroundNearbyPointCloud.pcd", *ground_nearby) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
    std::cout<< "1" << std::endl;
    //EuclideanClusterExtraction

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);


    //**************************************StatisticalOutlierRemoval Filter************************
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *ground_nearby << std::endl;
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (ground_nearby);
    sor.setMeanK (20);
    sor.setStddevMulThresh (2.0);
    sor.filter (*cloud_src_test_filter);
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_src_test_filter << std::endl;

    writer.write<pcl::PointXYZRGB> ("Filtered_ground_nearby_pointcloud.pcd", *cloud_src_test_filter, false);
    /***********************************************************************************************/
    ///The smallest scale to use in the DoN filter.
    double scale1 = 0.1;

    ///The largest scale to use in the DoN filter.
    double scale2 = 0.5;

    ///The minimum DoN magnitude to threshold by
    double threshold = 0.2;

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius = 0.1;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree_search;

    if (cloud->isOrganized ())
    {
        tree_search.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
    }
    else
    {
        tree_search.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
    }
    // Set the input pointcloud for the search tree
    tree_search->setInputCloud (ground_nearby);

    if (scale1 >= scale2)
    {
        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point

    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setInputCloud (ground_nearby);
    ne.setSearchMethod (tree_search);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    std::cout << "Calculating normals for scale..." << scale1 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    std::cout << "Calculating normals for scale..." << scale2 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
    copyPointCloud (*ground_nearby, *doncloud);
    std::cout << "Calculating DoN... " << std::endl;

    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (ground_nearby);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);

    // Save DoN features
    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

    // Filter by magnitude
    std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionOr<pcl::PointNormal> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));

    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

    // Apply filter
    condrem.filter (*doncloud_filtered);
    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->size () << " data points." << std::endl;
    writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;

    pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
    segtree->setInputCloud (doncloud);

    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    ec.setClusterTolerance (segradius); // 10cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (250000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);
    cout<< "cluster_indices:" << cluster_indices.size() << endl;
    int j = 0;
    for (const auto& cluster : cluster_indices){

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_colored(new pcl::PointCloud<pcl::PointXYZRGB>());

        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*doncloud)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        for(int i = 0; i<cloud_cluster->points.size();i++){

            pcl::PointXYZRGB pc2;
            pc2.x = cloud_cluster->points[i].x;
            pc2.y = cloud_cluster->points[i].y;
            pc2.z = cloud_cluster->points[i].z;
            pc2.rgb  = j;
            // cloud_cluster->points[i].rgb = j;
            after_sege->points.push_back(pc2);
            cloud_cluster_colored->points.push_back(pc2);

        }

        cloud_cluster_colored->height=1;
        cloud_cluster_colored->width=cloud_cluster_colored->points.size();
    
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write<pcl::PointXYZRGB> ("/home/bjersgen2004/pc_new/cloud_cluster/cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster_colored, false); //*
        j++;
    }
    after_sege->height=1;
    after_sege->width=after_sege->points.size();
    writer.write<pcl::PointXYZRGB>("AfterSege.pcd", *after_sege, false);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr after_sege_xy(new pcl::PointCloud<pcl::PointXYZRGB>());
    projectToXYPlaneRGB(after_sege,after_sege_xy);
    writer.write<pcl::PointXYZRGB>("AfterSegeXY.pcd", *after_sege_xy, false);

	srand(time(NULL));
    Cfg cfg;
    std::string re_read;
    cfg.readConfigFile(config_file_path, "re", re_read);
    std::string reforn_read;
    cfg.readConfigFile(config_file_path, "reforn", reforn_read);
    std::cout<< "re" << re_read << "reforn" << reforn_read << std::endl;
    float re, reforn;
    re=atof(re_read.c_str());
    reforn=atof(reforn_read.c_str());

    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PolygonMesh combined_mesh;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> concave_hull_clouds;
    for(int cluster_id = 0; cluster_id < cluster_indices.size(); cluster_id++) {

         std::cout << "Processing with the "<< cluster_id << "Cluster" << std::endl;
        // 加载当前聚类点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        std::stringstream cluster_filename;
        cluster_filename << "/home/bjersgen2004/pc_new/cloud_cluster/cloud_cluster_" << std::setw(4) << std::setfill('0') << cluster_id << ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(cluster_filename.str(), *cluster) == -1)
        {
            PCL_ERROR("Couldn't read cluster file \n");
            continue;
        }

        // 提取边界点
        std::vector<int> boundary_indices;
        std::vector<int> non_boundary_indices;
        estimateBorders(cluster, re, reforn, cluster_id);

        // 加载边界点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        std::stringstream boundary_filename;
        boundary_filename << "/home/bjersgen2004/pc_new/cloud_boundary/Cloud_boundary_" << std::setw(4) << std::setfill('0') << cluster_id << ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(boundary_filename.str(), *cluster_boundary) == -1)
        {
            PCL_ERROR("Couldn't read boundary file \n");
            continue;
        }

        // 将边界点投影到XY平面
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);
        projectToXYPlane(cluster_boundary, output_cloud_xy);

        // 计算凹包（Concave Hull）
        pcl::PointCloud<pcl::PointXYZ>::Ptr concave_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConcaveHull<pcl::PointXYZ> chull;
        
        
        // 读取配置文件中的 alpha 参数
        Cfg cfg;
        std::string alpha_read;
        cfg.readConfigFile(config_file_path, "alpha", alpha_read);
        float alpha = atof(alpha_read.c_str());

        chull.setInputCloud(output_cloud_xy);
        chull.setAlpha(alpha);
        chull.reconstruct(*concave_hull);


        *combined_cloud += *concave_hull;

        //保存凹包点云
        std::stringstream concave_filename;
        concave_filename << "/home/bjersgen2004/pc_new/concave_hull/Concave_" << std::setw(4) << std::setfill('0') << cluster_id << ".pcd";
        writer.write<pcl::PointXYZ>(concave_filename.str(), *concave_hull, false);
        concave_hull_clouds.push_back(concave_hull);

    }

    pcl::io::savePLYFile("combined_hulls.ply", *combined_cloud);

    //visulization

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud = mergeGroundAndConcaveHulls(cloud_filtered_ground, concave_hull_clouds);
    // visualizeMergedPointCloudWithBoundaries(merged_cloud, concave_hull_clouds);

/*********************************************triangles************************************************************************/
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::io::loadPCDFile("/home/bjersgen2004/pc_new/cloud_cluster/cloud_cluster_0001.pcd", *cluster);
    // // pcl::io::loadPCDFile("/home/bjersgen2004/Downloads/pcl-pcl-1.10.0/test/bunny.pcd", *cluster);

    // estimateBorders(cluster,re,reforn,2);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_boarder(new pcl::PointCloud<pcl::PointXYZ>);
    // // pcl::io::loadPCDFile("/home/bjersgen2004/Downloads/pcl-pcl-1.10.0/test/bunny.pcd", *cluster_boarder);
    // pcl::io::loadPCDFile("/home/bjersgen2004/pc_new/CLoud_boundary_0002.pcd", *cluster_boarder);


    // // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_xy_1(new pcl::PointCloud<pcl::PointXYZRGB>);

    // projectToXYPlane(cluster_boarder,output_cloud_xy);

    // writer.write<pcl::PointXYZ>("output_cloud_xy.pcd", *output_cloud_xy, false);

    // std::cerr << "Cloud : " << std::endl;
    // std::cerr << *cluster_boarder << std::endl;

    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    // pcl::PointCloud<pcl::Normal>::Ptr normals_2 (new pcl::PointCloud<pcl::Normal>);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fuck (new pcl::search::KdTree<pcl::PointXYZ>);
    // tree_fuck->setInputCloud (output_cloud_xy);
    // n.setInputCloud (output_cloud_xy);
    // n.setSearchMethod (tree_fuck);
    // n.setKSearch (20);
    // n.compute (*normals_2);
    // //* normals should not contain the point normals + surface curvatures
    // // Concatenate the XYZ and normal fields*

    // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    // pcl::concatenateFields (*output_cloud_xy, *normals_2, *cloud_with_normals);

    // //* cloud_with_normals = cloud + normals


    // // Create search tree*
    // pcl::search::KdTree<pcl::PointNormal>::Ptr tree_fuck2 (new pcl::search::KdTree<pcl::PointNormal>);
    // tree_fuck2->setInputCloud (cloud_with_normals);

    // // Initialize objects
    // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    // pcl::PolygonMesh triangles;

    // // Set the maximum distance between connected points (maximum edge length)
    // gp3.setSearchRadius (0.25);

    // // Set typical values for the parameters
    // gp3.setMu (10);
    // gp3.setMaximumNearestNeighbors (500);
    // gp3.setMaximumSurfaceAngle(3*M_PI/4); // 45 degrees
    // gp3.setMinimumAngle(M_PI/18); // 10 degrees
    // gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    // gp3.setNormalConsistency(false);

    // // Get result
    // gp3.setInputCloud (cloud_with_normals);
    // gp3.setSearchMethod (tree_fuck2);
    // gp3.reconstruct (triangles);

    // // Additional vertex information
    // std::vector<int> parts = gp3.getPartIDs();
    // std::vector<int> states = gp3.getPointStates();

    // for(int i = 0; i < parts.size(); ++i) {
    //     std::cout << parts[i] << " ";
    // }
    // std::cout << std::endl;

    // for(int i = 0; i < states.size(); ++i) {
    //     std::cout << states[i] << " ";
    // }
    // std::cout << std::endl;
    // pcl::io::saveVTKFile ("mesh1.vtk", triangles);


    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("groundPointCloud.pcd", *ground_cloud);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_xy_hull(new pcl::PointCloud<pcl::PointXYZ>);
    // projectToXYPlane(ground_cloud,ground_cloud_xy);
    // pcl::ConcaveHull<pcl::PointXYZ> chull2;
    // chull2.setInputCloud(ground_cloud_xy);
    // chull2.setAlpha(3.0);
    // chull2.reconstruct(*ground_cloud_xy_hull);
    // concave_hull_clouds.push_back(ground_cloud_xy_hull);

    // estimateBorders(ground_cloud,1, 1, 9000);

    end = clock();
    double dur = (double)(end - start);
    printf("Use Time:%f\n", (dur / CLOCKS_PER_SEC));
    //convert to ros and pub to rviz
    ros::init (argc, argv, "multiple_concave_hulls_publisher");
    ros::NodeHandle nh;


    ros::Publisher cloud_pub =nh.advertise<sensor_msgs::PointCloud2> ("ground_cloud", 1);
    ros::Publisher hull_pub = nh.advertise<visualization_msgs::Marker>("concave_hulls_marker", 1);


    sensor_msgs::PointCloud2 output;
    //Convert the cloud to ROS message
    pcl::toROSMsg(*ground_cloud, output);
    output.header.frame_id = "odom";

    //循环每秒发送一次
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        cloud_pub.publish(output);
        publishMultipleConcaveHulls(hull_pub, concave_hull_clouds, "odom");

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

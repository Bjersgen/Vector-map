#include <iostream>
#include <math.h>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <CSF.h>
#include <time.h>
#include "test_pcl_node.h"
#include <unordered_set>

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


using namespace std;
const char* config_file_path = "/home/bjersgen2004/pc_new/src/test_pcl/src/params.cfg";
float distance_threshold = 1.0;
std::ostringstream os;

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
int estimateBorders(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,float re,float reforn, int k) 
{ 

    pcl::PointCloud<pcl::Boundary> boundaries; 
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> boundEst; 
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normEst; 
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZRGB>); 
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud)); 
    normEst.setRadiusSearch(reforn); 
    normEst.compute(*normals); 

    boundEst.setInputCloud(cloud); 
    boundEst.setInputNormals(normals); 
    boundEst.setRadiusSearch(re); 
    boundEst.setAngleThreshold(M_PI/4); 
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>)); 
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
    writer.write<pcl::PointXYZRGB>("CLoud_boundary_" + ss.str()+".pcd", *cloud_boundary, false);


    // boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("BoundaryEstimation"));

    // int v1(0); 
    // MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
    // MView->setBackgroundColor (0.3, 0.3, 0.3, v1); 
    // MView->addText ("Raw point clouds", 10, 10, "v1_text", v1); 
    // int v2(0); 
    // MView->createViewPort (0.5, 0.0, 1, 1.0, v2); 
    // MView->setBackgroundColor (0.5, 0.5, 0.5, v2); 
    // MView->addText ("Boudary point clouds", 10, 10, "v2_text", v2); 

    // MView->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud",v1);
    // MView->addPointCloud<pcl::PointXYZRGB> (cloud_boundary, "cloud_boundary",v2);
    // MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample cloud",v1);
    // MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "cloud_boundary",v2);
    // MView->addCoordinateSystem (1.0);
    // MView->initCameraParameters ();

    // MView->spin();

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
        // projected_point.r = point.r;
        // projected_point.b = point.b;
        // projected_point.g = point.g;
        output_cloud->points.push_back(projected_point);
    }
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
}
void visualizeCurve (ON_NurbsCurve &curve,
                    ON_NurbsSurface &surface,
                    pcl::visualization::PCLVisualizer &viewer)
{
    // 将曲线转换为点云数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud (curve, surface, curve_cloud, 4);//该函数会将曲线上的点均匀地采样，并将采样点作为点云数据的点。
 
    // 可视化
    for(std::size_t i=0; i< curve_cloud->size() - 1; i++)
    {
        pcl::PointXYZRGB &p1 = curve_cloud->at(i);
        pcl::PointXYZRGB &p2 = curve_cloud->at(i+1);
        os << "line" << i;
        viewer.removeShape(os.str());
        viewer.addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
    }
 
    //
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i< curve.CVCount(); i++)
    {
        ON_3dPoint p1;
        curve.GetCV(i, p1); // 曲线的一个控制点
 
        double pnt[3];
        surface.Evaluate(p1.x ,p1.y, 0, 3, pnt); // 加usn曲面上对应的点的坐标
        pcl::PointXYZRGB p2;
        p2.x = float (pnt[0]);
        p2.y = float (pnt[1]);
        p2.z = float (pnt[2]);
 
        p2.r = 255;
        p2.g = 0;
        p2.b = 0;
 
        curve_cps->push_back (p2);
    }
    viewer.removePointCloud ("cloud_cps");
    viewer.addPointCloud (curve_cps, "cloud_cps");
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
    pcl::PCDWriter writer;
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
    auto& after_sege_points = after_sege->points;

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

    // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // ec.setClusterTolerance (0.10); // 10cm
    // ec.setMinClusterSize (200);
    // ec.setMaxClusterSize (250000);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (ground_nearby);
    // ec.extract (cluster_indices);
    // cout<< "cluster_indices:" << cluster_indices.size() << endl;
    // int j = 0;
    // for (const auto& cluster : cluster_indices){
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    //     for (const auto& idx : cluster.indices) {
    //         cloud_cluster->push_back((*ground_nearby)[idx]);
    //     } //*
    //     cloud_cluster->width = cloud_cluster->size ();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;
    //     for(int i = 0; i<cloud_cluster->points.size();i++){

    //         pcl::PointXYZRGB pc2;
    //         pc2.x = cloud_cluster->points[i].x;
    //         pc2.y = cloud_cluster->points[i].y;
    //         pc2.z = cloud_cluster->points[i].z;
    //         pc2.rgb  = j;
    //         // cloud_cluster->points[i].rgb = j;
    //         after_sege_points.push_back(pc2);

    //     }
    
    //     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    //     std::stringstream ss;
    //     ss << std::setw(4) << std::setfill('0') << j;
    //     writer.write<pcl::PointXYZRGB> ("/home/bjersgen2004/pc_new/cloud_cluster/cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
    //     j++;
    // }
    // after_sege->height=1;
    // after_sege->width=after_sege->points.size();
    // writer.write<pcl::PointXYZRGB>("AfterSege.pcd", *after_sege, false);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (ground_nearby);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);


    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*ground_nearby, *indices);


    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (30);
    reg.setMaxClusterSize (300000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (50);
    reg.setInputCloud (ground_nearby);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (8.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (8.0);
    reg.extract (cluster_indices);


    std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
    std::cout << "First cluster has " << cluster_indices[0].indices.size () << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
    int j = 0;
    for (const auto& cluster : cluster_indices){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*ground_nearby)[idx]);
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
            after_sege_points.push_back(pc2);

        }

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write<pcl::PointXYZRGB> ("/home/bjersgen2004/pc_new/cloud_cluster/cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster, false); //*
        j++;
}
    after_sege->height=1;
    after_sege->width=after_sege->points.size();
    writer.write<pcl::PointXYZRGB>("cluster_combine.pcd", *after_sege, false);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    writer.write<pcl::PointXYZRGB>("AfterSege.pcd", *colored_cloud, false);


    // //EstimateBorders
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



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/bjersgen2004/pc_new/cloud_cluster/cloud_cluster_0001.pcd", *cluster);
    // pcl::io::loadPCDFile("/home/bjersgen2004/Downloads/pcl-pcl-1.10.0/test/bunny.pcd", *cluster);

    estimateBorders(cluster,re,reforn,2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_boarder(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("/home/bjersgen2004/Downloads/pcl-pcl-1.10.0/test/bunny.pcd", *cluster_boarder);
    pcl::io::loadPCDFile("/home/bjersgen2004/pc_new/CLoud_boundary_0002.pcd", *cluster_boarder);


    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_xy_1(new pcl::PointCloud<pcl::PointXYZRGB>);

    projectToXYPlane(cluster_boarder,output_cloud_xy);

    writer.write<pcl::PointXYZ>("output_cloud_xy.pcd", *output_cloud_xy, false);

    std::cerr << "Cloud : " << std::endl;
    std::cerr << *cluster_boarder << std::endl;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals_2 (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fuck (new pcl::search::KdTree<pcl::PointXYZ>);
    tree_fuck->setInputCloud (output_cloud_xy);
    n.setInputCloud (output_cloud_xy);
    n.setSearchMethod (tree_fuck);
    n.setKSearch (20);
    n.compute (*normals_2);
    //* normals should not contain the point normals + surface curvatures
    // Concatenate the XYZ and normal fields*

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*output_cloud_xy, *normals_2, *cloud_with_normals);

    //* cloud_with_normals = cloud + normals


    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_fuck2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree_fuck2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.25);

    // Set typical values for the parameters
    gp3.setMu (10);
    gp3.setMaximumNearestNeighbors (500);
    gp3.setMaximumSurfaceAngle(3*M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree_fuck2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    // for(int i = 0; i < parts.size(); ++i) {
    //     std::cout << parts[i] << " ";
    // }
    // std::cout << std::endl;

    // for(int i = 0; i < states.size(); ++i) {
    //     std::cout << states[i] << " ";
    // }
    // std::cout << std::endl;
    pcl::io::saveVTKFile ("mesh1.vtk", triangles);

    //****************************convex & concave */********************************************** */

    // pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ConvexHull<pcl::PointXYZ> hull;
    // hull.setInputCloud(output_cloud_xy);
    // hull.setDimension(2);
    // hull.reconstruct(*convex_hull);
    // writer.write<pcl::PointXYZ>("convex.pcd", *convex_hull, false);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);

    std::string alpha_read;
    cfg.readConfigFile(config_file_path, "alpha", alpha_read);
    std::cout<< "alpha" << alpha_read << std::endl;
    float alpha;
    alpha=atof(alpha_read.c_str());


    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(output_cloud_xy);
    chull.setAlpha(alpha);
    chull.reconstruct(*cloud_hull);

    std::cout<< "concave hull has" << cloud_hull->size() << "points" <<std::endl;
    writer.write<pcl::PointXYZ>("concave.pcd", *cloud_hull, false);


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Curve Fitting 2D"));
    viewer->setBackgroundColor(255,255,255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(output_cloud_xy,0,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(output_cloud_xy,cloud_color,"cloud23");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> hull_color(cloud_hull,0,255,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_hull,hull_color,"cloud45");

    for(size_t i =0; i< cloud_hull->points.size(); ++i){

        pcl::PointXYZ p1 = cloud_hull->points[i];
        pcl::PointXYZ p2 = cloud_hull->points[(i+1)%cloud_hull->points.size()];
        viewer->addLine<pcl::PointXYZ>(p1,p2,0,255,0,"line"+std::to_string(i));
    }

    // viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
    }

//     viewer.setSize(800, 600);

    end = clock();
    double dur = (double)(end - start);
    printf("Use Time:%f\n", (dur / CLOCKS_PER_SEC));


//convert to ros and pub to rviz
    ros::init (argc, argv, "pcl_example");
    ros::NodeHandle nh;
    ros::Publisher pub_ply_map=nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    sensor_msgs::PointCloud2 output;
    //Convert the cloud to ROS message
    // pcl::toROSMsg(*cloud_filtered_ground, output);
    output.header.frame_id = "odom";

    //循环每秒发送一次
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub_ply_map.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

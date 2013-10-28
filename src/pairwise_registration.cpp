/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

double voxel_leaf_size = 0.005;

double normal_radius = 0.02;

double feature_radius = 0.1;

double ia_min_sample_distance = 0.1;
float ia_max_distance = 0.05f;
int ia_iterations = 500;

double icp_epsilon = 1e-4;
float icp_maxdistance = 0.5f;
int icp_iterations = 500;

//convenient structure to handle our pointclouds
struct PCD {
    PointCloud::Ptr cloud;
    std::string f_name;

    PCD() :
            cloud(new PointCloud) {
    }
    ;
};

struct PCDComparator {
    bool operator ()(const PCD& p1, const PCD& p2) {
        return (p1.f_name < p2.f_name);
    }
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation: public pcl::PointRepresentation<PointNormalT> {
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation() {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray(const PointNormalT &p, float * out) const {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target,
        const PointCloud::Ptr cloud_source) {
    p->removePointCloud("vp1_target");
    p->removePointCloud("vp1_source");

    PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 255, 0, 0);
    PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 0, 255, 0);
    p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO("Press q to begin the registration.\n");
    p->spin();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target,
        const PointCloudWithNormals::Ptr cloud_source) {
    p->removePointCloud("source");
    p->removePointCloud("target");

    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(
            cloud_target, "curvature");
    if (!tgt_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!");

    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(
            cloud_source, "curvature");
    if (!src_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!");

    p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

    p->spinOnce(30, true);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
 * \param argc the number of arguments (pass from main ())
 * \param argv the actual command line arguments (pass from main ())
 * \param models the resultant vector of point cloud datasets
 */
void loadData(int argc, char **argv,
        std::vector<PCD, Eigen::aligned_allocator<PCD> > &models) {
    std::string extension(".pcd");
    // Suppose the first argument is the actual test model
    for (int i = 1; i < argc; i++) {
        std::string fname = std::string(argv[i]);
        // Needs to be at least 5: .plot
        if (fname.size() <= extension.size())
            continue;

        std::transform(fname.begin(), fname.end(), fname.begin(),
                (int (*)(int))tolower);

                //check that the argument is a pcd file
if(        fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
        {
            // Load the cloud and saves it into the global list of models
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile (argv[i], *m.cloud);
            //remove NAN points from the cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

            models.push_back (m);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt,
        PointCloud::Ptr output, Eigen::Matrix4f &final_transform,
        bool downsample = true) {
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample) {
		grid.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);
        std::cout << "Source cloud size before " << cloud_src->size() << " after " << src->size() << std::endl;
        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
        std::cout << "Target cloud size before " << cloud_tgt->size() << " after " << tgt->size() << std::endl;
    } else {
        src = cloud_src;
        tgt = cloud_tgt;
    }

    // Compute surface normals and curvature
    std::cout << "Compute Normals" << std::endl;
    PointCloudWithNormals::Ptr points_with_normals_src(
            new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(
            new PointCloudWithNormals);

    pcl::NormalEstimationOMP<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZRGBA>());
    norm_est.setSearchMethod(tree);
	//    norm_est.setKSearch(30);
	norm_est.setRadiusSearch(normal_radius);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = { 1.0, 1.0, 1.0, 2.0 };
    point_representation.setRescaleValues(alpha);
//    pcl::PointRepresentation<pcl::FPFHSignature33> point_representation;

    // ### Feature Estimation ########################################
    std::cout << "Estimate Features" << std::endl;
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimationOMP<PointNormalT, PointNormalT, pcl::FPFHSignature33> fpfh;

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointNormalT>::Ptr tree2(
            new pcl::search::KdTree<PointNormalT>);

    fpfh.setSearchMethod(tree2);


    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(
            new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(
            new pcl::PointCloud<pcl::FPFHSignature33>());
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch(feature_radius);

    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);
    fpfh.setInputCloud(points_with_normals_src);
    fpfh.setInputNormals(points_with_normals_src);
    // Compute the features
    fpfh.compute(*source_features);

    fpfh.setInputCloud(points_with_normals_tgt);
    fpfh.setInputNormals(points_with_normals_tgt);
    // Compute the features
    fpfh.compute(*target_features);

    // ############################################################################
    std::cout << "Compute initial alignment" << std::endl;
    // TODO: Initial Allign from pcl::Registration Paper
    pcl::SampleConsensusInitialAlignment<PointNormalT, PointNormalT,
            pcl::FPFHSignature33> sac;
	sac.setMinSampleDistance(ia_min_sample_distance);
	sac.setMaxCorrespondenceDistance(ia_max_distance);
	sac.setMaximumIterations(ia_iterations);
    sac.setInputSource(points_with_normals_src);
    sac.setSourceFeatures(source_features);
    sac.setInputTarget(points_with_normals_tgt);
    sac.setTargetFeatures(target_features);
    pcl::PointCloud<PointNormalT>::Ptr pre_aligned_source(
            new pcl::PointCloud<PointNormalT>);
    sac.align(*pre_aligned_source);
    Eigen::Matrix4f initial_T = sac.getFinalTransformation();

    showCloudsRight(points_with_normals_tgt, pre_aligned_source);
    std::cout << "Initial Alignment done " << std::endl << initial_T
            << std::endl;

    // #############################################################################
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;

	reg.setTransformationEpsilon(icp_epsilon);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(icp_maxdistance);

    // Set the point representation
    reg.setPointRepresentation(
            boost::make_shared<const MyPointRepresentation>(
                    point_representation));

    reg.setInputSource(pre_aligned_source);
//    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    Ti = Ti * initial_T;
    PointCloudWithNormals::Ptr reg_result = pre_aligned_source;
//    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

    reg.setMaximumIterations(2);
    bool done = false;
	for (int i = 0; i < icp_iterations; ++i) {

        // save cloud for visualization purpose
        pre_aligned_source = reg_result;
//		points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(pre_aligned_source);
//		reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum())
                < reg.getTransformationEpsilon()) {
            double max_cor = reg.getMaxCorrespondenceDistance();
            double diff = 0.000001;
            if (max_cor > 0.05) {
                diff = 0.008;
            } else if (max_cor > 0.005) {
                diff = 0.0008;
            } else if (max_cor > 0.0005) {
                diff = 0.00008;
            } else if (max_cor > 0.00005) {
                done = true;
            }
            reg.setMaxCorrespondenceDistance(max_cor - diff);
            PCL_INFO("Iteration Nr. %d.\n", i);
            std::cout << "New MaxCorrespondenceDistance: "
                    << reg.getMaxCorrespondenceDistance() << std::endl;
        }
        prev = reg.getLastIncrementalTransformation();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, pre_aligned_source);
//        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
        if (done)
            break;
    }

    //
    // Get the transformation from target to source
	// TODO: consider doing the transformation in the other direction
	// to keep difference between consecutive models small
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
//    pcl::copyPointCloud(*pre_aligned_source, *output);

    std::cout << "Fine Alignment done " << std::endl << Ti << std::endl;

    p->removePointCloud("source");
    p->removePointCloud("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 255, 0, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 0, 255, 0);
    p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO("Press q to continue the registration.\n");
    p->spin();

    p->removePointCloud("source");
    p->removePointCloud("target");

    //add the source to the transformed target
    *output += *cloud_src;

//    final_transform = initial_T * Ti;
    final_transform = Ti;
}

/* ---[ */
int main(int argc, char** argv) {
    // Load data
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
    loadData(argc, argv, data);

    // Check user input
    if (data.empty()) {
        PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
        PCL_ERROR(
                "[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
        return (-1);
    }
    PCL_INFO("Loaded %d datasets.", (int)data.size ());

    // Create a PCLVisualizer object
    p = new pcl::visualization::PCLVisualizer(argc, argv,
            "Pairwise Incremental Registration example");
    p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

    PointCloud::Ptr result(new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(),
            pairTransform;


    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.5);

    pcl::VoxelGrid<PointT> vox_grid;
    vox_grid.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

    pcl::search::KdTree<PointT>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    PointCloud::Ptr temp(new PointCloud);

    for (size_t i = 1; i < data.size(); ++i) {


        if (i == 1) {
            source = data[i - 1].cloud;
        } else {
            source = temp;
        }
        target = data[i].cloud;

        // Add visualization data
        showCloudsLeft(source, target);

//        PointCloud::Ptr temp(new PointCloud);

        temp.reset(new PointCloud);

        PCL_INFO(
                "Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        pairAlign(target, source, temp, pairTransform, true);

        //transform current pair into the global transform
        pcl::transformPointCloud(*temp, *result, GlobalTransform);

        std::cout << "Apply voxelgrid filter. Points before filtering: " << temp->size() << std::endl;
        vox_grid.setInputCloud(temp);
//        vox_grid.filter (*temp); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>);
        vox_grid.filter(*tempCloud);

        std::cout << "Apply statistical outlier removal. Points before filtering: " << tempCloud->size() << std::endl;
        sor.setInputCloud (tempCloud);
        sor.filter (*temp);


        std::cout << "Apply smoothing filter. Points before filtering: " << temp->size() << std::endl;
        mls.setInputCloud (temp);
        mls.process (mls_points);
        std::cout << "Points after filtering: " << mls_points.size() << std::endl;

        pcl::copyPointCloud(mls_points, *temp);

        //update the global transform
        GlobalTransform = pairTransform * GlobalTransform;

        //save aligned pair, transformed into the first cloud's frame
        std::stringstream ss;
        ss << i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *temp, true);

    }
}
/* ]--- */

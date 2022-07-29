// SIFT3D.cpp : Defines the entry point for the console application.
//

// ISS_detector.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/features/normal_3d.h>

void write_ply_file(std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints_ptr, const pcl::PointCloud<pcl::Normal>::Ptr& keypoints_normal_ptr)
{
	int nb_vertex = (int)keypoints_ptr->size();
	// write header
	std::ofstream output_file;
	output_file.open(filename, std::ios::out);
	output_file << "ply" << std::endl;
	output_file << "format ascii 1.0" << std::endl;
	output_file << "element vertex " << std::to_string(nb_vertex).c_str() << std::endl;
	output_file << "property float x" << std::endl;
	output_file << "property float y" << std::endl;
	output_file << "property float z" << std::endl;
	output_file << "property float nx" << std::endl;
	output_file << "property float ny" << std::endl;
	output_file << "property float nz" << std::endl;
	output_file << "end_header" << std::endl;

	// write points from kpairs_queue
	std::set<int> used_index;
	for (int i = 0; i < keypoints_ptr->size(); i++)
		output_file << (*keypoints_ptr)[i].x << " " << (*keypoints_ptr)[i].y << " " << (*keypoints_ptr)[i].z << " " << (*keypoints_normal_ptr)[i].normal_x << " " << (*keypoints_normal_ptr)[i].normal_y << " " << (*keypoints_normal_ptr)[i].normal_z << std::endl;

	output_file.close();
}


int main(int argc, const char** argv)
{

	// check the input
	if (argc < 4) {
		PCL_ERROR("you must enter first the filepath of the mesh"); // take as input a pcd file
		PCL_ERROR("you must enter secondly the resolution of the mesh (mean of edge lengths");
		PCL_ERROR("you must enter thirdly the filepath where to save keypoints (could be text file or ply file)");
		return (-1);
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>());

	// Fill in the model cloud
	// read ply file
	std::string pc_filepath = argv[1];
	std::cout << "file path : " << pc_filepath << std::endl;

	if (pcl::io::loadPLYFile<pcl::PointNormal>(pc_filepath, *model) == -1) //* load the file
																		   //if (pcl::io::loadPCDFile<pcl::PointXYZ>(pc_filepath, *model) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read ply file \n");
		return (-1);
	}

	std::cout << "Loaded "
		<< model->width * model->height
		<< " data points from ply file "
		<< std::endl;

	float model_resolution = std::stof(argv[2]);
	std::string output_filename = argv[3];


	pcl::PointCloud<pcl::PointXYZ>::Ptr model_points(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr model_normal(new pcl::PointCloud<pcl::Normal>());
	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*model, *model_points);
	pcl::copyPointCloud<pcl::PointNormal, pcl::Normal>(*model, *model_normal);

	// Parameters for sift computation
	const float min_scale = model_resolution;
	const int n_octaves = 6;
	const int n_scales_per_octave = 5;
	const float min_contrast = 0.5f;


	// Estimate the sift interest points using Intensity values from RGB values
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints(new pcl::PointCloud<pcl::PointWithScale>());
	pcl::PointCloud<pcl::Normal>::Ptr keypoints_normal(new pcl::PointCloud<pcl::Normal>());
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(model);
	sift.compute(*keypoints);

	//get respective normals of the keypoints point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_pts(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud<pcl::PointWithScale, pcl::PointXYZ>(*keypoints, *keypoints_pts);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_tree(new pcl::search::KdTree<pcl::PointXYZ>());
	std::vector<int> normal_indices;
	std::vector<float> k_sqr_distances;
	normal_tree->setInputCloud(model_points);
	for (int i = 0; i < keypoints_pts->size(); i++)
	{
		std::vector<int> k_indices;
		normal_tree->nearestKSearch((*keypoints_pts)[i], 1, k_indices, k_sqr_distances);
		normal_indices.insert(normal_indices.end(), k_indices.begin(), k_indices.end());
	}

	for (int i = 0; i < normal_indices.size(); i++)
		keypoints_normal->push_back((*model_normal)[normal_indices[i]]);

	// write ply file of keypoints
	write_ply_file(output_filename, keypoints_pts, keypoints_normal);
	//pcl::io::savePLYFileASCII<pcl::PointXYZ>(output_filename, *keypoints_pts);


	return 0;
}



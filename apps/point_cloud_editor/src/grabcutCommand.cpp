//
//  grabcut.cpp
//  pcl
//
//  Created by Matthew Marzin on 11/17/14.
//  Copyright (c) 2014 Matthew Marzin. All rights reserved.
//

/// @file grabcutCommand.cpp
/// @details the implementation of the class GrabcutCommand
/// @author Matthew Marzin

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/grabcut_segmentation.h>
#include <pcl/apps/point_cloud_editor/grabcutCommand.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/cloud.h>

void GrabcutCommand::execute()
{
  pcl::GrabCut<pcl::PointXYZRGBA> grabcutter;
  std::vector<pcl::PointIndices> clusters;
  Selection inverted_selection = *selection_ptr_;
  inverted_selection.invertSelect();
  pcl::PointIndices cluster;
  unsigned int index = 0;
  unsigned int j = 1;
  pcl::PointIndicesPtr background_points;
  Selection::const_iterator it;
  Selection::iterator jt;
  
  
  grabcutter.setInputCloud(cloud_ptr_->getInternalCloud().makeShared());
  grabcutter.setK(k_);
  grabcutter.setLambda(lambda_);
  pcl::console::print_info("K: %f\n",k_);
  pcl::console::print_info("Lambda: %f\n",lambda_);
  
  background_points.reset (new pcl::PointIndices);

  
  for(it = selection_ptr_->begin(); it != selection_ptr_->end(); ++it)
  {
    index = *it;
    background_points->indices.push_back (index);
  }
  grabcutter.setBackgroundPointsIndices(background_points);
  pcl::console::print_info("Set Background Points\n");
  
  grabcutter.refine();
  pcl::console::print_info("Done Running Refine\n");
  
  //Get the clusters from the grabcut algorithm
  grabcutter.extract(clusters);
  pcl::console::print_info("Done Running Extract\n");

  pcl::console::print_info("Object found! Contains %ul points\n", clusters[0].indices.size());
  //Convert each cluster into a selection and remove it from the cloud
  pcl::console::print_info("Making list of indicies not in object\n");
  for(unsigned int j = 1; j < clusters.size(); j++ )
  {
    pcl::console::print_info("Adding points from cluster %ul to remove list\n",j);
    cluster = clusters[j];
    pcl::console::print_info("Removing %u points\n",cluster.indices.size());
    for(unsigned int k = 0; k < cluster.indices.size(); k++)
    {
      removed_indices_.addIndex(static_cast<unsigned int>(cluster.indices[k]));
    }
  }
  pcl::console::print_info("Removing Non-Object Points\n");
  pcl::console::print_info("Going to remove %u points\n",removed_indices_.size());
  cloud_ptr_->remove(removed_indices_);
  
  pcl::console::print_info("Removing Inverted Selection\n");
  pcl::console::print_info("Going to remove %u points\n",inverted_selection.size());
  cloud_ptr_->remove(inverted_selection);
}

void
GrabcutCommand::undo ()
{
  pcl::console::print_info("Sorry! I didn't Implement an \"Undo\"...\n");
}


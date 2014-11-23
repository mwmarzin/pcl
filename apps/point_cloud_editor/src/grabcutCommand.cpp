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
  pcl::PointIndices background;
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
  
  //Get the clusters from the grabcut algorithm
  grabcutter.extract(clusters);
  pcl::console::print_info("Done Running Extract\n");

  pcl::console::print_info("Object found! Contains %ul points\n", clusters[1].indices.size());
  
  //Convert background cluster into a selection and remove it from the cloud
  pcl::console::print_info("Making list of indicies not in object\n");

  pcl::console::print_info("Adding points from cluster %u to remove list\n",j);
  background = clusters[0];
  pcl::console::print_info("Removing %u points\n",background.indices.size());
  for(unsigned int k = 0; k < background.indices.size(); k++)
  {
    removed_indices_.addIndex(static_cast<unsigned int>(background.indices[k]));
  }
  
  pcl::console::print_info("Removing Non-Object Points\n");
  pcl::console::print_info("Going to remove %u points\n",removed_indices_.size());
  cloud_ptr_->remove(removed_indices_);
  
  selection_ptr_->clear();
}

void
GrabcutCommand::undo ()
{
  pcl::console::print_info("Sorry! I didn't Implement an \"Undo\"...\n");
}


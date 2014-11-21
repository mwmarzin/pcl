//
//  grabcut.h
//  pcl
//
//  Created by Matthew Marzin on 11/17/14.
//  Copyright (c) 2014 Matthew Marzin. All rights reserved.
//

/// @file grabcutCommand.h
/// @details A GrabcutCommand object provides functionalities for segmenting the
/// point cloud using the grabcut algorithm described in the paper
/// "GrabCut — Interactive Foreground Extraction using Iterated Graph Cuts"
/// by Carsten Rother, Vladimir Kolmogorov and Andrew Blake.
/// See http://docs.pointclouds.org/trunk/classpcl_1_1_grab_cut.html
/// @author Matthew Marzin
#ifndef GRABCUT_COMMAND_H_
#define GRABCUT_COMMAND_H_


#include <pcl/apps/point_cloud_editor/command.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>


class GrabcutCommand : public Command
{
public:
    /// @brief Constructor
    /// @param selection_ptr a shared pointer pointing to the selection object.
    /// @param cloud_ptr a shared pointer pointing to the cloud object.
    /// @param mean the number of points to use for mean distance estimation.
    /// @param threshold the standard deviation multiplier threshold
    GrabcutCommand (SelectionPtr selection_ptr, CloudPtr cloud_ptr,
                    float k, float lambda)
    : selection_ptr_(selection_ptr), cloud_ptr_(cloud_ptr), k_(k),
    lambda_(lambda), removed_indices_(cloud_ptr)
    {
    }
    
    /// @brief Destructor
    ~GrabcutCommand ()
    {
    }
    
protected:
    /// @brief Runs the grabcut algorithm to segment the cloud.
    void
    execute ();
    
    /// @brief Adds the removed noisy points back to the cloud
    void
    undo ();
    
private:
    /// @brief Default Constructor
    GrabcutCommand () : removed_indices_(CloudPtr())
    {
    }
    
    /// @brief Copy constructor - commands are non-copyable
    GrabcutCommand (const GrabcutCommand&)
    : removed_indices_(CloudPtr())
    {
        assert(false);
    }
    
    /// @brief Equal operator - commands are non-copyable
    GrabcutCommand&
    operator= (const GrabcutCommand&)
    {
        assert(false); return (*this);
    }
    
    /// A shared pointer pointing to the selection object of the widget
    SelectionPtr selection_ptr_;
    
    /// A pointer pointing to the cloud of the widget
    CloudPtr cloud_ptr_;
    
    /// The number of points to use for mean distance estimation.
    float k_;
    
    /// The standard deviation multiplier threshold
    float lambda_;
    
    /// A copy buffer which backs up the noisy point removed after denoising.
    CopyBuffer removed_points_;
    
    /// A selection object which backs up the indices of the noisy points removed.
    Selection removed_indices_;
};

#endif // GRABCUT_COMMAND_H_ 

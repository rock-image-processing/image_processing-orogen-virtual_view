/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <frame_helper/Calibration.h>
#include <frame_helper/FrameHelper.h>

using namespace virtual_view;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}


void Task::addCam( const base::Affine3d& cam2plane, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame >& frame )
{
    // get calibration matrix
    frame_helper::CameraCalibration calib = 
	frame_helper::CameraCalibration::fromFrame( *frame );

    if( !calib.isValid() )
	throw std::runtime_error("No valid calibration matrix embedded in frame");

    Eigen::Matrix3f camMatrix = 
	calib.getCameraMatrix().cast<float>();

    // get cv image
    cv::Mat img = frame_helper::FrameHelper::convertToCvMat( *frame );

    // project image in homography
    hom.addImage( img, Eigen::Isometry3f( cam2plane.matrix().cast<float>() ), camMatrix );
}

void Task::cam1TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam1_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( base::Time(), cam2plane ) )
	addCam( cam2plane, cam1_sample );

    // TODO for now, sync on cam1, but do something smarter later
    frame_helper::FrameHelper::copyMatToFrame( hom.getVirtualImage(), viewFrame );
    _virtual_cam.write( &viewFrame );
    hom.clearVirtualImage();
}

void Task::cam2TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam2_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( base::Time(), cam2plane ) )
	addCam( cam2plane, cam2_sample );
}

void Task::cam3TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam3_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( base::Time(), cam2plane ) )
	addCam( cam2plane, cam3_sample );
}

void Task::cam4TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam4_sample)
{
    Eigen::Affine3d cam2plane;
    if( _cam12plane.get( base::Time(), cam2plane ) )
	addCam( cam2plane, cam4_sample );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    Eigen::Affine3d vcam2plane;
    if( _virtual_cam2plane.get( base::Time(), vcam2plane ) )
    {
	hom.init( _width.value(), _height.value(), _focal_length.value(), 
		Eigen::Isometry3f( vcam2plane.matrix().cast<float>() ) );
    }
    else
    {
	LOG_ERROR_S << "Could not get vcam2plane transformation chain" << std::endl;
	return false;
    }

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

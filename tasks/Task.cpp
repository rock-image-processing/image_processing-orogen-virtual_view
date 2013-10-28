/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

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

void Task::cam1TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam1_sample)
{
    throw std::runtime_error("Transformer callback for cam1 not implemented");
}

void Task::cam2TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam2_sample)
{
    throw std::runtime_error("Transformer callback for cam2 not implemented");
}

void Task::cam3TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam3_sample)
{
    throw std::runtime_error("Transformer callback for cam3 not implemented");
}

void Task::cam4TransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &cam4_sample)
{
    throw std::runtime_error("Transformer callback for cam4 not implemented");
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

/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/transports/corba/TaskContextProxy.hpp>
#include <base-logging/Logging.hpp>
#include <random>

using namespace trajectory_follower;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    lastMotionCommand.translation = 0;
    lastMotionCommand.rotation    = 0;
    lastMotionCommand.heading     = 0;
}

Task::~Task()
{
}

std::string Task::printState(const TaskBase::States& state)
{
    switch(state)
    {
        case FINISHED_TRAJECTORIES:
            return "FINISHED_TRAJECTORIES";
        case FOLLOWING_TRAJECTORY:
            return "FOLLOWING_TRAJECTORY";
        case SLAM_POSE_INVALID:
            return "SLAM_POSE_INVALID";
        case LATERAL:
            return "LATERAL";
        case TURN_ON_SPOT:
            return "TURN_ON_SPOT";
        case STABILITY_FAILED:
            return "STABILITY_FAILED";
        default:
            return "UNKNOWN_STATE";
    }
}

bool Task::isMotionCommandZero(const Motion2D& mc)
{
    return ( mc.translation == 0 &&
             mc.rotation    == 0 &&
             mc.heading     == 0 );
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    previous_distance_closest_obstacle = -1.0;
    if (! TaskBase::configureHook())
        return false;

    if (_enable_evasive_behavior.get())
    {
        LOG_INFO_S << "Evasive behavior is " << _enable_evasive_behavior.get();
        if (_traversability_map_provider_task.get().empty()){
            LOG_ERROR_S << "No task name provided for the traversability map provider!";
            throw std::runtime_error("No task name provided for the traversability map provider!");
        }

        planner_task  = RTT::corba::TaskContextProxy::Create(_traversability_map_provider_task.get(), false);

        if(!planner_task->ready()){
            LOG_ERROR_S << "ERROR: " << _traversability_map_provider_task.get() << " is not ready!!";
            throw std::runtime_error("ERROR: " + _traversability_map_provider_task.get() + " is not ready!!");
        }

        this->addPeer(planner_task);
        isTraversable = planner_task->getOperation("isTraversable");
    }
    turning_left = false;
    turning_right = false;
    return true;
}
bool Task::startHook()
{
    trajectoryFollower = TrajectoryFollower( _follower_config.value() );

    current_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::getNextPose(const Motion2D& mc){

    const double time_step_in_future = 0.5;

    Motion2D motionCommand = mc;

    //new_heading = current_heading + (motionCommand.rotation * std::abs((previous_time-current_time).toSeconds()));

    if(mc.rotation < 0){
        motionCommand.rotation += 2*M_PI;
    }
    if(current_heading < 0){
        current_heading += 2*M_PI;
    }
    new_heading = current_heading + (motionCommand.rotation * time_step_in_future);

    //where will I be after 0.5 seconds
    double delta = motionCommand.translation * time_step_in_future;

    new_position.x() = current_position.x() + (delta * cos(new_heading));
    new_position.y() = current_position.y() + (delta * sin(new_heading));

    LOG_DEBUG_S << "Current Position" << current_position.transpose();
    LOG_DEBUG_S << "New Position" << new_position.transpose();

}

Eigen::Vector3d Task::getClosestObjectCentroid(std::vector<pointcloud_obstacle_detection::Box> dynamic_objects){

    double distance = std::numeric_limits< double >::max();

    int object_index{0};

    for (int i{0}; i < dynamic_objects.size(); ++i){

        Eigen::Vector3d object_in_robot_frame((dynamic_objects.at(i).x_min+dynamic_objects.at(i).x_max)/2,
                                              (dynamic_objects.at(i).y_min+dynamic_objects.at(i).y_max)/2,
                                              (dynamic_objects.at(i).z_min+dynamic_objects.at(i).z_max)/2);
        Eigen::Vector3d object_in_world_frame = robot2map * object_in_robot_frame;

        double dist = std::abs((object_in_world_frame - robot2map.translation()).norm());

        if (dist < distance){
            distance = dist;
            object_index = i;
            LOG_DEBUG_S << "Closest object is " << distance << " meters away.";
        }
    }

    pointcloud_obstacle_detection::Box closest_box = dynamic_objects.at(object_index);
    Eigen::Vector3d object_in_robot_frame((closest_box.x_min+closest_box.x_max)/2,
                                          (closest_box.y_min+closest_box.y_max)/2,
                                          (closest_box.z_min+closest_box.z_max)/2);
    return object_in_robot_frame;
}

void Task::turnLeft(Motion2D& mc){
    //adapt the mc to turn left for evasive behavior
    LOG_DEBUG_S << "Turning Left ";

    if (turning_left == true){
        mc = lastMotionCommand;
        return;
    }

    //if (mc.rotation < 0.01 && mc.rotation > -0.01){
    //    mc.rotation = 0.2;
    //}
    //else{
    //    mc.rotation = std::abs(mc.rotation) + 0.5;
    //}

    mc.rotation = std::abs(mc.rotation) + 0.2*(std::abs(mc.rotation));

    turning_left = true;
}

void Task::turnRight(Motion2D& mc){
    //adapt the mc to turn left for evasive behavior
    LOG_DEBUG_S << "Turning Right ";

    if (turning_right== true){
        mc = lastMotionCommand;
        return;
    }

    //if (mc.rotation < 0.01 && mc.rotation > -0.01){
    //    mc.rotation = -0.2;
    //}
    //else{
    //   mc.rotation = -std::abs(mc.rotation) - 0.5;
    //}
    mc.rotation = -(std::abs(mc.rotation) + 0.2*(std::abs(mc.rotation)));
    turning_right = true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    motionCommand.translation = 0;
    motionCommand.rotation    = 0;
    motionCommand.heading     = 0;

    if(!_robot2map.get(current_time, robot2map, false))
    {
        LOG_ERROR_S << "Could not get robot pose!";
        trajectoryFollower.removeTrajectory();
        _motion_command.write(motionCommand.toBaseMotion2D());
        return;
    }

    base::Pose robotPose(robot2map);

    // in the current usage if we get a new set of trajectories means we planned a new path
    // from the current pose and should skip the old set of trajectories.
    //if (_trajectory.readNewest(trajectories, false) == RTT::NewData && !trajectories.empty()) {

    if (_trajectory.readNewest(trajectories, false) == RTT::NewData) {
        if(trajectories.empty()) {
            _motion_command.write(motionCommand.toBaseMotion2D());
            state(FINISHED_TRAJECTORIES);
        }
        else {
            bool last = false;
            if(trajectories.size() < 3) last = true;
            trajectoryFollower.setNewTrajectory(trajectories.front(), robotPose, last);
            _current_trajectory.write(trajectoryFollower.getData().currentTrajectory);
            trajectories.erase(trajectories.begin());
            //emit following once, to let the outside know we got the trajectory
            state(FOLLOWING_TRAJECTORY);
        }
    }

    SubTrajectory subTrajectory;
    if (_holonomic_trajectory.readNewest(subTrajectory, false) == RTT::NewData) {
        trajectoryFollower.setNewTrajectory(subTrajectory, robotPose);
        _current_trajectory.write(trajectoryFollower.getData().currentTrajectory);
        //emit following once, to let the outside know we got the trajectory
        state(FOLLOWING_TRAJECTORY);
    }

    FollowerStatus status = trajectoryFollower.traverseTrajectory(motionCommand, robotPose);
    current_heading = robotPose.getYaw();
    current_position = robotPose.position.head<2>();

    if (_enable_evasive_behavior.get() && status == TRAJECTORY_FOLLOWING){
        Motion2D copy = motionCommand;
        _dynamic_objects.readNewest(dynamic_objects,false);
        Eigen::Vector3d closest_object_in_robot_frame;
        bool execute_evasive_action = false;
        if (dynamic_objects.size() > 0){
            closest_object_in_robot_frame = getClosestObjectCentroid(dynamic_objects);

            const double current_distance_closest_obstacle = closest_object_in_robot_frame.norm();
            if (previous_distance_closest_obstacle < 0){ //first run
                previous_distance_closest_obstacle = current_distance_closest_obstacle;
            }
            else{
                const double movement = current_distance_closest_obstacle-previous_distance_closest_obstacle;
                if (std::abs(movement) < 0.01){ //ignore small jerks maybe due to sensor errors
                    previous_distance_closest_obstacle = current_distance_closest_obstacle;
                }
                else{
                    //TODO: use robot dimensions for the clearance check
                    execute_evasive_action = (movement < 0) ? true : false;
                    if (execute_evasive_action && closest_object_in_robot_frame.y() < 0.0 && closest_object_in_robot_frame.y() > -1.0){
                        turnLeft(copy);
                        turning_right = false;
                    }
                    //TODO: use robot dimensions for the clearance check
                    else if (execute_evasive_action && closest_object_in_robot_frame.y() >= 0.0 && closest_object_in_robot_frame.y() < 1.0){
                        turnRight(copy);
                        turning_left = false;
                    }
                    else{
                        turning_left = false;
                        turning_right = false;
                    }
                    getNextPose(copy);
                    Eigen::Vector3d future_position(new_position.x(), new_position.y(),0);
                    if (isTraversable(future_position)){
                        motionCommand = copy;
                    }
                }
                previous_distance_closest_obstacle = current_distance_closest_obstacle;
            }
        }
        else{
            previous_distance_closest_obstacle = -1;
            turning_left = false;
            turning_right = false;
        }
    }

    switch(status)
    {
    case TRAJECTORY_FINISHED:
        if(!trajectories.empty())
        {
            bool last = false;
            if(trajectories.size() < 3) last = true;
            trajectoryFollower.setNewTrajectory(trajectories.front(), robotPose, last);
            _current_trajectory.write(trajectoryFollower.getData().currentTrajectory);
            trajectories.erase(trajectories.begin());
        }
        else
            new_state = FINISHED_TRAJECTORIES;
        break;
    case TRAJECTORY_FOLLOWING:
        new_state = FOLLOWING_TRAJECTORY;
        break;
    case SLAM_POSE_CHECK_FAILED:
        new_state = SLAM_POSE_INVALID;
        break;
    case EXEC_TURN_ON_SPOT:
        new_state = TURN_ON_SPOT;
        break;
    case EXEC_LATERAL:
        new_state = LATERAL;
        break;
    case INITIAL_STABILITY_FAILED:
        if(current_state != new_state)
        {
            LOG_ERROR_S << "update TrajectoryFollowerTask state to STABILITY_FAILED.";
        }
        new_state = STABILITY_FAILED;
        break;
    default:
        std::runtime_error("Unknown TrajectoryFollower state");
    }

    _follower_data.write(trajectoryFollower.getData());


    if ( not ( isMotionCommandZero(lastMotionCommand) &&
               isMotionCommandZero(motionCommand)     &&
               _send_zero_cmd_once.value() )
       )
    {
        lastMotionCommand = motionCommand;
        _motion_command.write(motionCommand.toBaseMotion2D());
    }

    // update task state
    if(current_state != new_state)
    {
        LOG_INFO_S << "update TrajectoryFollowerTask state to " << printState(new_state);
        current_state = new_state;
        state(new_state);
    }

    previous_time = current_time;
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    motionCommand.translation = 0;
    motionCommand.rotation    = 0;
    motionCommand.heading     = 0;
    _motion_command.write(motionCommand.toBaseMotion2D());

    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

// 1 0.5 0.1 0.01
ActionModel::ActionModel(void)
: initialized_(false)
, alpha1_(1.0f)   //0.01
, alpha2_(0.8f)    //0.1
, alpha3_(0.1f) //0.0025
, alpha4_(0.001f) //0.0001
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // numberGenerator_ = generator;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_)
    {
        previousOdometry_ = odometry;
        initialized_ = true;
    }

    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    float deltaTheta = angle_diff(odometry.theta, previousOdometry_.theta);

    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    trans_ = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    // std::cout << "trans_: " << trans_ << std::endl;
    // if (std::abs(trans_) < 0.0001f)
    // {
    //     rot1_ = 0.0f;
    // }
    rot2_ = angle_diff(deltaTheta, rot1_);
    moved_ = false;
    moved_ = (deltaX != 0.0f) || (deltaY != 0.0f) || (deltaTheta != 0.0f);

    rot1Std_ = std::sqrt(alpha1_ * rot1_*rot1_ + alpha2_ * trans_*trans_);
    transStd_ = std::sqrt(alpha3_ * trans_*trans_ + alpha4_ * (rot1_*rot1_ + rot2_*rot2_));
    rot2Std_ = std::sqrt(alpha2_ * trans_*trans_ + alpha1_ * rot2_*rot2_);

    utime_ = odometry.utime;
    previousOdometry_ = odometry;

    return moved_;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    particle_t newSample = sample;
    newSample.pose.utime = utime_;

    if (moved_)
    {

        std::random_device rd;
        std::mt19937 numberGenerator(rd());

        float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator);
        // std::cout << sampleRot1 << std::endl;
        float sampleTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator);
        // std::cout << sampleTrans << std::endl;
        float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator);
        // std::cout << sampleRot2 << std::endl << std::endl;
        // std::cout << newSample.pose.x << " + " << sampleTrans << " * " << std::cos(newSample.pose.theta + sampleRot1) << std::endl;
        newSample.pose.x += sampleTrans * std::cos(newSample.pose.theta + sampleRot1);
        // std::cout << "=" << newSample.pose.x << std::endl;
        
        newSample.pose.y += sampleTrans * std::sin(newSample.pose.theta + sampleRot1);
        newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
        newSample.parent_pose = sample.pose;
    }
    else
    {
        // do nothing
    }  

    return newSample;
}

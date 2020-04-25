#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/angle_functions.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.001);
    // std::normal_distribution<> dist_theta(0.0, 0.001);

    for (auto& p : posterior_)
    {
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator));
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
    posterior_.back().pose = pose;

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);  // action model
        posterior_ = computeNormalizedPosterior(proposal, laser, map);  // sensor model
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }   
    
    posteriorPose_.utime = odometry.utime;
    // posteriorPose_ = odometry;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
        // std::cout << "pppppppppppppppppppp" << posterior_[0].pose.x << std::endl;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0 / kNumParticles_);
    double r = dist(generator);
    double c = posterior_[0].weight;
    int i = 0;

    for (int m = 1; m <= kNumParticles_; m++)
    {
        double U = r + (m - 1) * (1.0 / kNumParticles_);
        while (U > c)
        {
            c += posterior_[++i].weight;
        }
        prior.push_back(posterior_[i]);
    }

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    for (auto& p : prior)
    {
        proposal.push_back(actionModel_.applyAction(p));
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;

    posterior = proposal;

    double sumWeight = 0;
    for (auto& p : posterior)
    {
        p.weight = sensorModel_.likelihood(p, laser, map);
        std::cout << p.weight << std::endl;
        sumWeight += p.weight;
    }
    std::cout << "********************************************\n";
    for (auto& p : posterior)
    {
        p.weight = p.weight / sumWeight;
        std::cout << p.weight << std::endl;
    }
    std::cout<<"---------------------------------------------"<<std::endl;

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    pose.x = pose.y = pose.theta = 0.0;
    float sumSin, sumCos;
    sumSin = sumCos = 0.0;

    for (auto&p : posterior)
    {
        pose.x += p.weight * p.pose.x;
        pose.y += p.weight * p.pose.y;
        sumSin += p.weight * std::sin(p.pose.theta);
        sumCos += p.weight * std::cos(p.pose.theta);
    }

    pose.theta = std::atan2(sumSin, sumCos);

    return pose;
}

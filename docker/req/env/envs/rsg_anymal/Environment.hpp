//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include <time.h>
#include "../../RaisimGymEnv.hpp"

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "raisim/OgreVis.hpp"
#include "visSetupCallback.hpp"

#include "visualizer/raisimKeyboardCallback.hpp"
#include "visualizer/helper.hpp"
#include "visualizer/guiState.hpp"
#include "visualizer/raisimBasicImguiPanel.hpp"

#include "visualizer/robot/robot_imgui_render_callback.hpp"
#include "visualizer/robot/gaitLogger.hpp"
#include "visualizer/robot/jointSpeedTorqueLogger.hpp"
#include "visualizer/robot/RewardLogger.hpp"
#include "visualizer/robot/videoLogger.hpp"
#include "visualizer/robot/frameVisualizer.hpp"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDistX_(1.0, 3.0), normDistY_(-0.1, 0.1), normDistZ_(-0.1, 0.1)  {

    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
    robot_ = world_->addArticulatedSystem(resourceDir_+"/anymal/urdf/anymal.urdf");

    robot_->setName("robot");
    robot_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    auto ground = world_->addGround();

    /// get robot data
    gcDim_ = robot_->getGeneralizedCoordinateDim();
    gvDim_ = robot_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    jointTorque.setZero(nJoints_); jointSpeed.setZero(nJoints_); 
    deltaTorque_.setZero(nJoints_); deltaSpeed_.setZero(nJoints_);

    prevGaitLogger_ = {false,false,false,false};

    prev_gc_ = gc_init_.tail(nJoints_);

    // command_ << normDistX_(gen_), normDistY_(gen_), 0;
    // command_ << normDistX_(gen_), 0, 0;
    command_ << 3, 0, 0;



    gravityConst_ << 0, 0, -9.8;

    airTimeArray_.setZero(4);
    bodyLinearVelTemp_.setZero(3);
    angleRotation_.setZero(3);
    oldAction_.setZero(12);
    errorJointPos_.setZero(12);

    
    /// this is nominal configuration of anymal
    gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8; // ANYMAL


    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(50.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.2);




    robot_->setPdGains(jointPgain, jointDgain);
    robot_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 85; //34;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    double action_std;
    READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
    actionStd_.setConstant(action_std);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.insert(robot_->getBodyIdx("LF_SHANK"));
    footIndices_.insert(robot_->getBodyIdx("RF_SHANK"));
    footIndices_.insert(robot_->getBodyIdx("LH_SHANK"));
    footIndices_.insert(robot_->getBodyIdx("RH_SHANK"));



    /// visualize if it is the first environment
    if (visualizable_) {

      auto vis = raisim::OgreVis::get();
      
      robot_gui::init({robot_gui::gait::init(100),
                    robot_gui::joint_speed_and_torque::init(100),
                    robot_gui::reward::init({}),
                    robot_gui::frame::init()});


      vis->setContactVisObjectSize(0.4, 0.3);
      /// these method must be called before initApp
      vis->setWorld(world_.get());
      vis->setWindowSize(1920, 1080);
      vis->setImguiSetupCallback(imguiSetupCallback);
      vis->setImguiRenderCallback(raisim::robot_gui::robotImguiRenderCallBack);
      vis->setKeyboardCallback(raisimKeyboardCallback);
      vis->setSetUpCallback(setupCallback);
      vis->setAntiAliasing(2);

      /// starts visualizer thread
      vis->initApp();
      robotVisual_ = vis->createGraphicalObject(robot_, "robot");
      vis->createGraphicalObject(ground, 40, "floor", "checkerboard_green");
      desired_fps_ = 50.0;
      vis->setDesiredFPS(desired_fps_);
      vis->select(robotVisual_->at(0), false);
      vis->getCameraMan()->setYawPitchDist(Ogre::Radian(-0.0), Ogre::Radian(-1.3), 4);

    }

  }


  void init() final { }

  void reset() final {
    robot_->setState(gc_init_, gv_init_);
    updateObservation();

    double normDistX_d, normDistY_d;

    normDistX_d = generate_random_double(1.0, 3.0);
    normDistY_d = generate_random_double(-0.1, 0.1);

    // command_ << normDistX_d, normDistY_d, 0;
    // command_ << normDistX_d, 0, 0;
    command_ << 3, 0, 0;

    energy_ = 0;


    if(visualizeThisStep_) 
      // std::cout<<"Command: "<< command_ <<std::endl;

    if(visualizable_)
    {
      raisim::robot_gui::reward::clear();
    }
  }

  double generate_random_double(double min, double max) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> distrib(min, max);

    double random_number = std::round(distrib(gen) * 100.0) / 100.0;

    return random_number;
}


double logisticKernel(double& x) {
  return -((1)/(exp(x) + 2 + exp(-x)));
}

double angularVelocityCost() {
 
  double c_w = -6 * world_->getTimeStep();
  Eigen::Vector3d angularVelocitySet(3);

  angularVelocitySet << 0, 0, command_[2];
  double temp = (bodyAngularVel_ - angularVelocitySet).lpNorm<1>();

  return c_w * logisticKernel(temp);

  
}

double linearVelocityCost() {
  double c_v1 = -10 * world_->getTimeStep();
  double c_v2 = -4 * world_->getTimeStep();
  Eigen::Vector3d linearVelocitySet(3);

  linearVelocitySet << command_[0], command_[1], 0;
  // double temp = (c_v2 * (bodyLinearVel_ - linearVelocitySet)).lpNorm<1>();
  double temp = (bodyLinearVel_ - linearVelocitySet).lpNorm<1>();
  // std::cout<<"body Lin.vel. = "<<logisticKernel(temp)<<std::endl;

  return c_v1 * logisticKernel(temp);
  }

double torqueCost() {
  double k_c = 0.1;
  double c_tau = 0.005 * world_->getTimeStep();
  return k_c * c_tau * robot_->getGeneralizedForce().squaredNorm();
}

double jointSpeedCost() {
  double k_c = 0.1;
  double c_js = 0.03 * world_->getTimeStep();
  return k_c * c_js * robot_->getGeneralizedVelocity().squaredNorm();
}

// double orientationCost(raisim::Mat<3,3> rot) {
//   double k_c = 0.1;
//   double c_o = 0.4 * world_->getTimeStep();
//   Eigen::Vector3d angular, phi;
//   angular << 0, 0, -1;
//   phi = rot.e().row(2).transpose();
//   angular = angular - phi;
//   return k_c * c_o * angular.norm();
// }

double smoothnessCost(Eigen::VectorXd tau_t1, Eigen::VectorXd velocity_t1) {
  double k_c = 0.1;
  double c_s = 0.4 * world_->getTimeStep();
  Eigen::VectorXd tau_t(12), velocity_t(12);

  tau_t = robot_->getGeneralizedForce().e().tail(12);
  tau_t1 = tau_t1 - tau_t;
  // std::cout << tau_t1 << std::endl;

  velocity_t = robot_->getGeneralizedVelocity().e().tail(12);
  velocity_t1 = velocity_t1 - velocity_t;

  return k_c * c_s * tau_t1.squaredNorm() + k_c * c_s * velocity_t1.squaredNorm();
}

double calcFootClearanceAndSlipCost() {
    double c_f = 0.1 * world_->getTimeStep();
    double c_fv = 2.0 * world_->getTimeStep();
    double p_f_hat = 0.07;
    double k_c_= 0.1;

    std::set<long> contactedFoot;
    for(auto& contact: robot_->getContacts()) {
      contactedFoot.insert(contact.getlocalBodyIndex());
      if (footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end()) {
        return true;
      }
    }

    double footClearanceCost = 0.0;
    double footSlipCost = 0.0;
    raisim::Vec<3> pos, vel;

    for (auto footIndex : footIndices_) {
      robot_->getPosition(footIndex, pos);
      robot_->getVelocity(footIndex, vel);
      if (contactedFoot.find(footIndex) == contactedFoot.end())
        footClearanceCost += k_c_ * c_f * (p_f_hat - pos[2]) * (p_f_hat - pos[2]) * vel.norm();
        // auto sphere = world.addSphere(pos[0], 1.0);
      else
        footSlipCost += k_c_ * c_fv * vel.norm();
    }

    return footClearanceCost + footSlipCost;
  }

double bodyOrientationCost(double& robot_height, double& pitch, double& roll, double& yaw) {
    
    double k_c = 0.1;
    double c_o = 0.4 * world_->getTimeStep();

    // Инициализируем желаемую высоту робота
    const double desired_height = gc_init_[2]; // Можно изменить на нужное значение

    // Вычисляем ошибку высоты
    double height_error = std::abs(robot_height - desired_height);

    // Вычисляем ошибки тангажа, крена и рысканья
    double pitch_error = std::abs(pitch);
    double roll_error = std::abs(roll);
    // double yaw_error = std::abs(yaw);

    // Суммируем ошибки и возвращаем результат
    double total_error = height_error + pitch_error + roll_error;
    return exp(total_error) * k_c * c_o;
}

double airTimeCost()
{
  // world_->getTimeStep();
  double c_c = 2.0 * world_->getTimeStep();
  int size = airTimeArray_.size();

  for (int i = 0; i <size; ++i)
  {
    airTimeArray_[i] = airTimeArray_[i] + world_->getTimeStep();
  }

  for(auto& contact: robot_->getContacts())
  {
    if (footIndices_.find(contact.getlocalBodyIndex()) != footIndices_.end())
    {
      int index = std::distance(footIndices_.begin(), footIndices_.find(contact.getlocalBodyIndex()));
      airTimeArray_[index] = 0;
    }
  }


  double reward = 0;

  for (int i = 0; i <size; ++i)
  {
    reward = reward + (airTimeArray_[i] - 0.5);
    if (airTimeArray_[i] > 1.5)
    {
      reward = reward - exp(airTimeArray_[i]);
    } 
  }

  return reward * c_c;
}

double accelerationCost()
{

  double c_v1 = -10 * world_->getTimeStep();
  double c_v2 = -4 * world_->getTimeStep();

  Eigen::Vector3d linearAccelerationSet(3), acceleration(3);
  
  linearAccelerationSet << 0, 0, 0;

  acceleration = (bodyLinearVel_ - bodyLinearVelTemp_)/world_->getTimeStep();
  bodyLinearVelTemp_ = bodyLinearVel_;

  
  // double temp = (c_v2 * (acceleration - linearAccelerationSet)).lpNorm<1>();
  // return (c_v1 * logisticKernel(temp));
  return abs(bodyLinearVel_[2]);


}

double gaitGenerationCost()
{

    // std::string gaitMode_ = "canter"; 
    // std::string gaitMode_ = "half_bound";
    // std::string gaitMode_ = "pace";
    // std::string gaitMode_ = "rotary_gallop";
    // std::string gaitMode_ = "trot";
    std::string gaitMode_ = "no_gait";

    double reward = 0;
    std::array<bool, 4> phase1_1, phase1_2, phase2_1, phase2_2, phase3_1, phase3_2, phase4_1, phase4_2, flyPhase;
    flyPhase = {false, false, false, false};

    if (gaitMode_ == "canter")
    {
      phase1_1 = {false, false, true, false}; phase1_2 = {false, false, true, false};
      phase2_1 = {true, false, true, true};   phase2_2 = {true, false, false, true};  
      phase3_1 = {true, true, false, true};   phase3_2 = {false, true, false, false};
    
      if ((gaitLogger_ == phase1_1 && prevGaitLogger_ == flyPhase) || (gaitLogger_ == phase1_1 && prevGaitLogger_ == phase1_1) || // 1 phase (start)
          (gaitLogger_ == phase1_2 && prevGaitLogger_ == phase1_1) || (gaitLogger_ == phase1_2 && prevGaitLogger_ == phase1_2) || // 1 phase (end)
          (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase1_1) || (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase2_1) || // 2 phase (start)
          (gaitLogger_ == phase2_2 && prevGaitLogger_ == phase2_1) || (gaitLogger_ == phase2_2 && prevGaitLogger_ == phase2_2) || // 2 phase (end)
          (gaitLogger_ == phase3_1 && prevGaitLogger_ == phase2_2) || (gaitLogger_ == phase3_1 && prevGaitLogger_ == phase3_1) || // 3 phase (start)
          (gaitLogger_ == phase3_2 && prevGaitLogger_ == phase3_1) || (gaitLogger_ == phase3_2 && prevGaitLogger_ == phase3_2) || // 3 phase (end)
          (gaitLogger_ == flyPhase && prevGaitLogger_ == phase3_2) || (gaitLogger_ == flyPhase && prevGaitLogger_ == flyPhase))   // Flight phase
      {
        reward = reward + 1;
      }
    }

    if (gaitMode_ == "pace")
    {
      phase1_1 = {true, false, true, false}; phase1_2 = {true, true, true, true};
      phase2_1 = {false, true, false, true}; phase2_2 = {true, true, true, true};

      if ((gaitLogger_ == phase1_1 && prevGaitLogger_ == phase2_1) || (gaitLogger_ == phase1_1 && prevGaitLogger_ == phase1_1) || // 1 phase (start)
          (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase1_1) || (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase2_1))   // Flight phase
      {
          reward = reward + 1;
      }
    }

    if (gaitMode_ == "rotary_gallop")
    {
      phase1_1 = {false, false, false, true}; phase1_2 = {false, false, false, true};
      phase2_1 = {false, false, true, true}; phase2_2 = {false, false, true, true};
      phase3_1 = {true, false, true, true}; phase3_2 = {true, false, true, false};
      phase4_1 = {true, true, true, false}; phase4_2 = {true, true, false, false};


      if ((gaitLogger_ == phase1_1 && prevGaitLogger_ == flyPhase) || (gaitLogger_ == phase1_1 && prevGaitLogger_ == phase1_1) || // 1 phase (start)
          (gaitLogger_ == phase1_2 && prevGaitLogger_ == phase1_1) || (gaitLogger_ == phase1_2 && prevGaitLogger_ == phase1_2) || // 1 phase (end)
          (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase1_2) || (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase2_1) || // 2 phase (start)
          (gaitLogger_ == phase2_2 && prevGaitLogger_ == phase2_1) || (gaitLogger_ == phase2_2 && prevGaitLogger_ == phase2_2) || // 2 phase (end)
          (gaitLogger_ == phase3_1 && prevGaitLogger_ == phase2_2) || (gaitLogger_ == phase3_1 && prevGaitLogger_ == phase3_1) || // 3 phase (start)
          (gaitLogger_ == phase3_2 && prevGaitLogger_ == phase3_1) || (gaitLogger_ == phase3_2 && prevGaitLogger_ == phase3_2) || // 3 phase (end)
          (gaitLogger_ == phase4_1 && prevGaitLogger_ == phase3_2) || (gaitLogger_ == phase4_1 && prevGaitLogger_ == phase4_1) || // 4 phase (start)
          (gaitLogger_ == phase4_2 && prevGaitLogger_ == phase4_1) || (gaitLogger_ == phase4_2 && prevGaitLogger_ == phase4_2) || // 4 phase (end)
          (gaitLogger_ == flyPhase && prevGaitLogger_ == phase4_2) || (gaitLogger_ == flyPhase && prevGaitLogger_ == flyPhase))   // Flight phase
      {
        reward = reward + 1;
      }

    }

    if (gaitMode_ == "half_bound")
    {

      phase1_1 = {false, false, true, true}; phase1_2 = {false, false, true, true};
      phase2_1 = {true, false, true, true};  phase2_2 = {true, false, false, false};
      phase3_1 = {true, true, false, false};  phase3_2 = {false, true, false, false};

      if ((gaitLogger_ == phase1_1 && prevGaitLogger_ == flyPhase) || (gaitLogger_ == phase1_1 && prevGaitLogger_ == phase1_1) || // 1 phase (start)
          (gaitLogger_ == phase1_2 && prevGaitLogger_ == phase1_1) || (gaitLogger_ == phase1_2 && prevGaitLogger_ == phase1_2) || // 1 phase (end)
          (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase1_2) || (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase2_1) || // 2 phase (start)
          (gaitLogger_ == phase2_2 && prevGaitLogger_ == phase2_1) || (gaitLogger_ == phase2_2 && prevGaitLogger_ == phase2_2) || // 2 phase (end)
          (gaitLogger_ == phase3_1 && prevGaitLogger_ == phase2_2) || (gaitLogger_ == phase3_1 && prevGaitLogger_ == phase3_1) || // 3 phase (start)
          (gaitLogger_ == phase3_2 && prevGaitLogger_ == phase3_1) || (gaitLogger_ == phase3_2 && prevGaitLogger_ == phase3_2) || // 3 phase (end)
          (gaitLogger_ == flyPhase && prevGaitLogger_ == phase4_2) || (gaitLogger_ == flyPhase && prevGaitLogger_ == flyPhase))
      {
        reward = reward + 1;
      }
    }

    if (gaitMode_ == "trot")
    {
      phase1_1 = {true, false, false, true}; phase1_2 = {true, true, true, true};
      phase2_1 = {false, true, true, false}; phase2_2 = {true, true, true, true};

      if ((gaitLogger_ == phase1_1 && prevGaitLogger_ == phase2_1) || (gaitLogger_ == phase1_1 && prevGaitLogger_ == phase1_1) || // 1 phase (start)
          (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase1_1) || (gaitLogger_ == phase2_1 && prevGaitLogger_ == phase2_1))   // Flight phase
      {
          reward = reward + 1;
      }
    }

    // if (mode == "walk")
    // {
    // }

    phases_ = {phase1_1, phase1_2, phase2_1, phase2_2, phase3_1, phase3_2, phase4_1, phase4_2, flyPhase};

    return reward;
}



double rad2deg(double x) {
  return x*180/3.14;
}

double deg2rad(double x) {
  return x*3,14/180;
}

double sqr(double x) {
  return x*x;
}

inline void quaternion_to_euler_angles(const Vec<4> &q, Eigen::Vector3d &angles) {
  float x = q[1]; float y = q[2]; float z = q[3]; float w = q[0];
  float roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
  float pitch = asin(2*(w*y - z*x));
  float yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));

  angles << roll, pitch, yaw;
}

void printSet(const std::set<size_t>& s) {
    std::cout << "Set content: { ";
    for (const auto& element : s) {
        std::cout << element << ", ";
    }
    std::cout << "}" << std::endl;
}

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling


    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;


    // // pTarget12_ = actionMean_;
    // pTarget12_[0] = gc_init_[7];
    // pTarget12_[3] = gc_init_[10];
    // pTarget12_[6] = gc_init_[13];
    // pTarget12_[9] = gc_init_[16];


    pTarget_.tail(nJoints_) = pTarget12_;

    robot_->setPdTarget(pTarget_, vTarget_);

    auto visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);

    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      world_->integrate();
      if (visualizable_ && visualizeThisStep_ && visualizationCounter_ % visDecimation == 0)
        raisim::OgreVis::get()->renderOneFrame();
      visualizationCounter_++;
    }

    updateObservation();

    gaitLogger_ = {false, false, false, false};
    for(auto& contact: robot_->getContacts())
    {
      if (footIndices_.find(contact.getlocalBodyIndex()) != footIndices_.end())
      {
        int index = std::distance(footIndices_.begin(), footIndices_.find(contact.getlocalBodyIndex()));
        gaitLogger_[index] = true;
      }
    }


    oldAction_ = pTarget12_;


    deltaTorque_ << jointTorque.tail(12);
    deltaSpeed_ << jointSpeed.tail(12);

    jointTorque = robot_->getGeneralizedForce().e().tail(12);
    jointSpeed = robot_->getGeneralizedVelocity().e().tail(12);


    rewards_.record("linearVelocityCost", linearVelocityCost()); 
    rewards_.record("angularVelocityCost", angularVelocityCost()); 
    rewards_.record("torqueCost", torqueCost()); 
    rewards_.record("jointSpeedCost", jointSpeedCost()); 
    rewards_.record("ClearAndSlip", calcFootClearanceAndSlipCost()); 
    rewards_.record("smoothnessCost", smoothnessCost(deltaTorque_, deltaSpeed_)); 
    rewards_.record("bodyOrientationCost",  bodyOrientationCost(gc_[2], pitch_, roll_, yaw_)); 
    rewards_.record("airTimeCost",  airTimeCost()); 
    rewards_.record("zAcceleration", accelerationCost()); 
    rewards_.record("gaitGenerationCost", gaitGenerationCost());


    if(visualizeThisStep_) 
      {
        robot_gui::joint_speed_and_torque::push_back(world_->getWorldTime(), jointSpeed, jointTorque);
        robot_gui::gait::push_back(gaitLogger_);
        robot_gui::reward::log("linAndAngVelocityCost", linearVelocityCost() * rewards_.getCoef("linearVelocityCost") + angularVelocityCost() * rewards_.getCoef("angularVelocityCost"));
        robot_gui::reward::log("torqueCost", torqueCost() * rewards_.getCoef("torqueCost"));
        robot_gui::reward::log("jointSpeedCost", jointSpeedCost() * rewards_.getCoef("jointSpeedCost"));
        robot_gui::reward::log("ClearAndSlip", calcFootClearanceAndSlipCost() * rewards_.getCoef("ClearAndSlip"));
        robot_gui::reward::log("smoothnessCost",  smoothnessCost(deltaTorque_, deltaSpeed_) * rewards_.getCoef("smoothnessCost"));
        robot_gui::reward::log("bodyOrientationCost",  bodyOrientationCost(gc_[2], pitch_, roll_, yaw_) * rewards_.getCoef("bodyOrientationCost"));
        robot_gui::reward::log("airTimeCost",  airTimeCost() * rewards_.getCoef("airTimeCost"));
        robot_gui::reward::log("zAcceleration", accelerationCost() * rewards_.getCoef("zAcceleration"));
        robot_gui::reward::log("gaitGenerationCost", gaitGenerationCost() * rewards_.getCoef("gaitGenerationCost"));


        updateVisual();
      }
    
    prevGaitLogger_ = gaitLogger_;


    return rewards_.sum();
  }


inline void updateVisual() 
{


void energy_from_cost_of_transport() 
{
  
  energy_ = energy_ + ((robot_->getGeneralizedForce().e().tail(12)).lpNorm<1>() * (gc_.tail(nJoints_) - prev_gc_.tail(nJoints_))).lpNorm<1>();
  // energy_ = energy_ +
  distance_ = sqrt(sqr(gc_[0]) + sqr(gc_[1]));
}

  void updateObservation() {

    double CoT;

    robot_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    // raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);


    errorJointPos_ = gc_.tail(12) - pTarget12_;

    quaternion_to_euler_angles(quat, angleRotation_);
    roll_ = angleRotation_[0]; pitch_ = angleRotation_[1]; yaw_ = angleRotation_[2];


    // std::cout << "Energy: " << energy_ << std::endl;
    if(visualizeThisStep_) 
    {
      energy_from_cost_of_transport();
      CoT = -energy_/(robot_->getTotalMass() * gravityConst_[2] * distance_);
      std::cout << "CoT: " << CoT << std::endl;
    }


    obDouble_ << 
        command_, // 3
        gravityConst_, // 3
        bodyLinearVel_, // 3
        bodyAngularVel_, // 3
        gc_.tail(12), /// joint angles 12
        gv_.tail(12), /// joint velocity 12
        errorJointPos_, /// 12
        oldAction_, // 12
        gaitLogger_, /// 4
        prevGaitLogger_, /// 4
        gc_[2], /// body height 1
        phases_,
        angleRotation_; /// body orientation 3


        /**
      current state =
       *      [command (horizontal velocity, yawrate)                      n =  3, si =  3
       *       gravity vector                                              n =  3, si =  6
       *       body Linear velocities,                                     n =  3, si =  9
       *       body Angular velocities,                                    n =  3, si =  12
       *       joint position                                              n = 12, si =  24
       *       joint velocity                                              n = 12, si =  36
       *       joint position err                                          n = 12, si =  48
       *       previous action                                             n = 16, si =  64
       *       contact state                                               n =  4, si =  68
       *       previous contact state                                      n =  4, si =  72
       *       body height                                                 n =  1, si =  73
       *       phases                                                      n =  9, si =  82
       *       body orientation                                            n =  3, si =  85
       *       ]
*/
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: robot_->getContacts())
    {
      // If we find a leg in contacts, we don't enter the condition (no penalty).
      // If we don't find a leg in contacts, we enter the condition (get a penalty).
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end()) 
      {
        if(visualizeThisStep_) 
        {
          std::cout<<"ABORT wrong contact"<<std::endl;
          // std::cout<<contact.getlocalBodyIndex()<<std::endl;
          // printSet(footIndices_);
        }
        return true;
      }

      // if(contact.isSelfCollision() == true) // self collision
      // {
      //   if(visualizeThisStep_)
      //   {
      //     std::cout<<"ABORT Self Collision"<<std::endl;
      //   }
      //   return true;
      // }
    
    }

    // if ((gc_[0] < -0.3) || (abs(gc_[1]) > 1.0)) // Coordinats
    // {
    //   if(visualizeThisStep_) 
    //     std::cout<<"ABORT X or Y coordinate: "<<gc_[0]<<" , "<<gc_[1]<<std::endl;
    //   return true;
    // }

    if ((gc_[2] > (gc_init_[2]+0.10)) || (gc_[2] < (gc_init_[2]-0.20))) // height
    { 
      if(visualizeThisStep_) 
        std::cout<<"ABORT HEIGHT: "<<gc_[2]<<std::endl;
      return true;
    }

    if (abs(rad2deg(yaw_)) > 60) // 40
    {
      if(visualizeThisStep_) 
        std::cout<<"ABORT yaw"<<std::endl;
      return true;
    }

    if(visualizeThisStep_) 
      // std::cout<<"Command: "<< command_ <<std::endl;
      // std::cout<<"Roll: "<< roll_ <<" Pitch: "<< pitch_<<" Yaw: "<< yaw_ <<std::endl;
    

    terminalReward = 0.f;
  
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_, visualizationCounter_ = 0;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* robot_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_, deltaTorque_, deltaSpeed_, prev_gc_;
  double terminalRewardCoeff_ = -10.0, desired_fps_ = 50.0;
  double roll_ = 0.00, pitch_ = 0.00, yaw_ = 0.00, energy_ = 0.00, distance_ = 0.00;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_, jointTorque, jointSpeed, airTimeArray_, oldAction_, errorJointPos_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_, angleRotation_, bodyLinearVelTemp_, command_, gravityConst_;
  std::set<size_t> footIndices_;
  std::array<bool, 4> gaitLogger_, prevGaitLogger_;
  std::vector<GraphicObject> * robotVisual_;
  raisim::Mat<3,3> rot;

  std::array<std::array<bool, 4>, 9> phases_;
  std::string gaitMode_;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDistX_, normDistY_, normDistZ_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}


#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/transport/Node.hh>

#include "crowd_simulator_common.hpp"


namespace crowd_simulation_ign {

//=================================================================================
template<typename... Args>
using Task = std::function<bool(Args... args)>;

template<typename TaskType>
class TaskManager
{
public:
  using TaskPtr = std::shared_ptr<TaskType>;

  void AddTask(const TaskType& task);

  template<typename... TaskArgs>
  void RunAllTasks(TaskArgs... args);

  size_t GetTasksCount();

private:
  std::list<TaskPtr> _tasks;
};

template<typename TaskType>
void TaskManager<TaskType>::AddTask(const TaskType& task)
{
  this->_tasks.emplace_back(std::make_shared<TaskType>(task));
}

template<typename TaskType>
template<typename... TaskArgs>
void TaskManager<TaskType>::RunAllTasks(TaskArgs... args)
{
  std::list<TaskPtr> done;
  for (const auto& task : this->_tasks)
  {
    if ((*task)(args...))
    {
      done.emplace_back(task);
    }
  }

  for (const auto& task : done)
  {
    this->_tasks.remove(task);
  }
}

template<typename TaskType>
size_t TaskManager<TaskType>::GetTasksCount()
{
  return this->_tasks.size();
}

//============================================================

class IGNITION_GAZEBO_VISIBLE CrowdSimulatorPlugin 
    : public ignition::gazebo::System, 
    public ignition::gazebo::ISystemConfigure, 
    public ignition::gazebo::ISystemPreUpdate
{   
public:
    CrowdSimulatorPlugin();

    // inherit from ISystemConfigure 
    void Configure(const ignition::gazebo::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        ignition::gazebo::EntityComponentManager& ecm, ignition::gazebo::EventManager& eventMgr) override;

    // inherit from ISystemPreUpdate
    void PreUpdate(const ignition::gazebo::UpdateInfo& info, ignition::gazebo::EntityComponentManager& ecm) override;

private:

    std::shared_ptr<ignition::gazebo::Model> _world;
    std::string _worldName;

    std::string _resourcePath;
    std::string _behaviorFile;
    std::string _sceneFile;
    double _simTimeStep; // load "update_time_step" tag from world file
    std::vector<std::string> _externalAgents; //store the name of all "external_agent"

    std::chrono::steady_clock::duration _lastSimTime{0};
    std::chrono::steady_clock::duration _lastAnimTime{0};

    // map for <model_name, object_id>, contains both external (models) and internal agents (actors)
    std::unordered_map<std::string, size_t> _objectDic;
    // map for <model_name, entity_id> contains external and internal agents
    std::unordered_map<std::string, ignition::gazebo::Entity> _entityDic;

    std::shared_ptr<crowd_simulator::ModelTypeDatabase> _modelTypeDBPtr;
    std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowdSimInterface;
    std::shared_ptr<rclcpp::Node> _nodePtr;
    std::shared_ptr<ignition::transport::Node> _transportNodePtr;

    using UpdateObjectTask = Task<double, double>;
    TaskManager<UpdateObjectTask> _updateTaskManager;

    volatile bool _pluginInitialized = false; // disable compiler optimization
    volatile bool _agentInitialized = false;

    bool _LoadParams(const std::shared_ptr<const sdf::Element>& sdf);

    bool _LoadModelInitPose(const sdf::ElementPtr& modelTypeElement, crowd_simulator::AgentPose3d& result) const;
    
    bool _LoadCrowdSim();
    
    bool _LoadAgents(ignition::gazebo::EntityComponentManager& ecm);
   
    bool _CreateEntity(ignition::gazebo::EntityComponentManager& ecm, 
      const std::string& modelName, 
      const crowd_simulator::ModelTypeDatabase::Record* modelTypePtr, 
      const crowd_simulator::AgentPtr agentPtr);
    
    bool _CheckSpawnedAgents(ignition::gazebo::EntityComponentManager& ecm);

    void _UpdateObject(double deltaTime, double deltaSimTime, ignition::gazebo::EntityComponentManager& ecm);

    void _UpdateObject(double deltaTime, double deltaSimTime, ignition::gazebo::EntityComponentManager& ecm, ignition::gazebo::Entity entity, crowd_simulator::CrowdSimInterface::ObjectPtr object_ptr);
};

//================================================================================
crowd_simulator::AgentPose3d Convert(const ignition::math::Pose3d& ignition_pose);

ignition::math::Pose3d Convert(const crowd_simulator::AgentPose3d& agent_pose);

} //namespace crowd_simulation_ign
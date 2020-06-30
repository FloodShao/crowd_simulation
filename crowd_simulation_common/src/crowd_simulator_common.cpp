#include <fstream>

#include <crowd_simulator_common.hpp>

namespace crowd_simulator {

//================================================================
void MengeHandle::SetSimTimeStep(float simTimeStep)
{
  this->_simTimeStep = simTimeStep;
}

float MengeHandle::GetSimTimeStep()
{
  return this->_simTimeStep;
}

size_t MengeHandle::GetAgentCount()
{
  if (this->_agentCount == 0)
  {
    this->_agentCount = this->_sim->getNumAgents();
  }
  return this->_agentCount;
}

void MengeHandle::SimStep()
{
  this->_sim->step();
}

AgentPtr MengeHandle::GetAgent(size_t id)
{
  return AgentPtr(this->_sim->getAgent(id));
}

std::string MengeHandle::_ResourceFilePath(const std::string& relativePath)
const
{

  std::string fullPath = this->_resourcePath + "/" + relativePath;
  std::cout << "Finding resource file: " << fullPath << std::endl;
  std::ifstream ifile(fullPath);
  if (!static_cast<bool>(ifile))
  {
    std::cerr << "File not found! " << fullPath << std::endl;
    assert(static_cast<bool>(ifile));
  }
  std::cout << "Found." << std::endl;
  return fullPath;
}

bool MengeHandle::_LoadSimulation()
{
  Menge::SimulatorDB simDB;
  Menge::PluginEngine::CorePluginEngine engine(&simDB);

  std::cout << "Start CrowdSimulator initializing [Menge]..." << std::endl;

  this->_sim = simDB.getDBEntry("orca")->getSimulator(
    this->_agentCount,
    this->_simTimeStep,
    0,
    std::numeric_limits<float>::infinity(),
    this->_behaviorFile,
    this->_sceneFile,
    "",
    "",
    false
  );

  if (this->_sim)
  {

    std::cout << std::endl << "Crowd Simulator initialized success [Menge]. " <<
      std::endl;
    return true;
  }

  return false;
}

//============================================

ModelTypeDatabase::Record* ModelTypeDatabase::Get(const std::string& typeName)
{
  if (this->_records.find(typeName) == this->_records.end())
  {
    std::cout << "The model type [ " << typeName << " ] is not defined in scene file!" << std::endl;
    return nullptr;
  }
  return this->_records[typeName];
}

size_t ModelTypeDatabase::Size()
{
  return this->_records.size();
}

//================================================================
bool CrowdSimInterface::SpawnObject(std::vector<std::string>& externalModels)
{
  //External models are loaded first in scene file
  size_t externalCount = externalModels.size();
  size_t totalAgentCount = this->_mengeHandle->GetAgentCount();

  //external model must be included in scene file
  assert(externalCount <= totalAgentCount);

  for (size_t i = 0; i < externalCount; ++i)
  {
    auto agentPtr = this->_mengeHandle->GetAgent(i);
    agentPtr->_external = true;
    this->AddObject(agentPtr, externalModels[i], "0", true);
  }

  for (size_t i = externalCount; i < totalAgentCount; ++i)
  {
    auto agentPtr = this->_mengeHandle->GetAgent(i);
    agentPtr->_external = false;

    std::string modelName = "agent" + std::to_string(i);

    // TODO: agent pointer didn't specify agentProfile name, in gazebo world, requires a name to configure model sdf
    // use size_t agentPtr->_class as the typename
    // std::cout << agentPtr->_typeName << std::endl;
    this->AddObject(agentPtr, modelName, agentPtr->_typeName, false);
  }

  return true;
}

void CrowdSimInterface::AddObject(AgentPtr agentPtr,
  const std::string& modelName,
  const std::string& typeName,
  bool isExternal = false)
{
  assert(agentPtr);

  // must provide a model name in gazebo if it's an external agent
  if (isExternal)
  {
    assert(!modelName.empty());
  }

  this->_objects.emplace_back(new Object{agentPtr, modelName, typeName,
      isExternal});
}


size_t CrowdSimInterface::GetNumObjects()
{
  return this->_objects.size();
}

CrowdSimInterface::ObjectPtr CrowdSimInterface::GetObjectById(size_t id)
{
  assert(id < this->_objects.size());
  return this->_objects[id];
}


void CrowdSimInterface::UpdateExternalAgent(size_t id,
  const ignition::math::Pose3d& modelPose)
{
  assert(id < this->GetNumObjects());

  auto agentPtr = this->_objects[id]->agentPtr;
  this->UpdateExternalAgent(agentPtr, modelPose);
}

void CrowdSimInterface::UpdateExternalAgent(const AgentPtr agentPtr,
  const ignition::math::Pose3d& modelPose)
{
  assert(agentPtr);
  agentPtr->_pos.setX(modelPose.Pos().X());
  agentPtr->_pos.setY(modelPose.Pos().Y());
}


void CrowdSimInterface::GetAgentPose(size_t id, double deltaSimTime,
  ignition::math::Pose3d& modelPose)
{

  assert(id < this->GetNumObjects());

  auto agentPtr = this->_objects[id]->agentPtr;

  this->GetAgentPose(agentPtr, deltaSimTime, modelPose);
}

void CrowdSimInterface::GetAgentPose(const AgentPtr agentPtr,
  double deltaSimTime,
  ignition::math::Pose3d& modelPose)
{
  //calculate future position in deltaSimTime.
  assert(agentPtr);
  double Px = static_cast<double>(agentPtr->_pos.x()) +
    static_cast<double>(agentPtr->_vel.x()) * deltaSimTime;
  double Py = static_cast<double>(agentPtr->_pos.y()) +
    static_cast<double>(agentPtr->_vel.y()) * deltaSimTime;

  // deltaDist = static_cast<double>(agentPtr->_vel.Length()) * deltaTime; //approximate distance travelled since last update

  modelPose.Pos().X(Px);
  modelPose.Pos().Y(Py);

  double xRot = static_cast<double>(agentPtr->_orient.x());
  double yRot = static_cast<double>(agentPtr->_orient.y());
  ignition::math::Quaterniond Ori({
      xRot, -yRot, 0,
      yRot, xRot, 0,
      0, 0, 1,
    });

  //TODO check whether correct, might be different from Kong Peng
  modelPose.Rot() = Ori;
}

void CrowdSimInterface::OneStepSim()
{
  this->_mengeHandle->SimStep();
}

} //namespace crowd_simulator



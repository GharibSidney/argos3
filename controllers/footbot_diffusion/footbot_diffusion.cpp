/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   bool bClear = true;
   size_t worstIdx = 0;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      if(tProxReads[i].Value > 0.0f) bClear = false;
      if(tProxReads[i].Value > tProxReads[worstIdx].Value) worstIdx = i;
   }
   // if all values equals, just go forward
   if(bClear) {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      return;
   }
   /* f(n) = g + h where h is angular distance FROM the worst sensor */
   std::vector<Real> cost(tProxReads.size());
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      Real g = tProxReads[i].Value;
      Real h = -Abs((tProxReads[i].Angle - tProxReads[worstIdx].Angle).SignedNormalize().GetValue());
      cost[i] = g + h;
   }

   size_t bestIdx = 0;
   for(size_t i = 1; i < tProxReads.size(); ++i) {
      if(cost[i] < cost[bestIdx]) bestIdx = i;
   }

   CRadians cAngle = tProxReads[bestIdx].Angle;
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle)) {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else if(cAngle.GetValue() > 0.0f) {
      m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
   }
   else {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
   }
}


/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

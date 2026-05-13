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
   m_cAlpha(2.0f),
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
   CVector2 cRepulsive;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      if(tProxReads[i].Value > 0.0f) {
         cRepulsive -= CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      }
   }
   
   CVector2 cAttractive(m_fWheelVelocity, CRadians::ZERO);
   CVector2 cNet = cAttractive + cRepulsive*(m_fWheelVelocity /2.5f);
   CRadians cAngle = cNet.Angle();
   std::cout << "Angle" << cNet.Angle() << std::endl;

   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle)) {
      /* Clear path: go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else if(cAngle.GetValue() > 0.0f) {
      /* Turn left */
      m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn right*/
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
   }
}


/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

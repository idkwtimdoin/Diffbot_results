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
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));       // never used
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   CVector2 cAccumulator;

   for(size_t i = 0; i < tProxReads.size(); i++) 
      if(tProxReads[i].Value > 0)
         // Sum up exponentially scaled sensor readings
         cAccumulator += CVector2(ExpoScale(tProxReads[i].Value), tProxReads[i].Angle);

   CRadians cAngle = cAccumulator.Angle();
   // If obstacle detected in the vicinity,
   if(cAccumulator.Length() > m_fDelta)
      // Turn according to angle
      if(cAngle > CRadians::ZERO)
         // Turn intensity is decided by the aggregate obstacle angle
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity * DefusionDelta(cAngle));
      else
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity * DefusionDelta(cAngle), m_fWheelVelocity);
   // If no obstacle detected, go straight
   else
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);

   // For output and error log purposes
   /****************************************/
   // Real closest = 0;
   // for(size_t i = 0; i < tProxReads.size(); i++) 
   //    if(closest < tProxReads[i].Value)   closest = tProxReads[i].Value;

   // if(closest == 1)        std::cerr<< "crash\n";
   // else if(closest > 0.3)  std::cout<< "closest encounter: " << closest << std::endl;
   /****************************************/
}

/****************************************/
/****************************************/

Real CFootBotDiffusion::DefusionDelta(CRadians angle){
   return -Cos(angle);
}

/****************************************/
/****************************************/

Real CFootBotDiffusion::ExpoScale(Real reading){
   return 2 * (exp(reading * reading) - 1);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")


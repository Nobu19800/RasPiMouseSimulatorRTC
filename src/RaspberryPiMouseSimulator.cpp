// -*- C++ -*-
/*!
 * @file  RaspberryPiMouseSimulator.cpp
 * @brief RaspberryPiMouse Simulator
 * @date $Date$
 *
 * @author Nobuhiko Miyamoto <n-miyamoto@aist.go.jp>
 *
 * LGPL
 *
 * $Id$
 */

#include "RaspberryPiMouseSimulator.h"

// Module specification
// <rtc-template block="module_spec">
static const char* raspberrypimousesimulator_spec[] =
  {
    "implementation_id", "RaspberryPiMouseSimulator",
    "type_name",         "RaspberryPiMouseSimulator",
    "description",       "RaspberryPiMouse Simulator",
    "version",           "1.0.0",
    "vendor",            "Miyamoto Nobuhiko",
    "category",          "Simulator",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.sampling_time", "-1",
    "conf.default.draw_time", "0.01",
    "conf.default.sensor_param", "1394,792,525,373,299,260,222,181,135,100,81,36,17,16",
	"conf.default.blocksConfigFile", "None",

    // Widget
    "conf.__widget__.sampling_time", "text",
    "conf.__widget__.draw_time", "text",
    "conf.__widget__.sensor_param", "text",
	"conf.__widget__.blocksConfigFile", "text",
    // Constraints

    "conf.__type__.sampling_time", "double",
    "conf.__type__.draw_time", "double",
    "conf.__type__.sensor_param", "string",
	"conf.__type__.blocksConfigFile", "string",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RaspberryPiMouseSimulator::RaspberryPiMouseSimulator(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_target_velocity_inIn("target_velocity_in", m_target_velocity_in),
    m_pose_updateIn("pose_update", m_pose_update),
    m_current_velocity_outOut("current_velocity_out", m_current_velocity_out),
    m_current_pose_outOut("current_pose_out", m_current_pose_out),
    m_ir_sensor_outOut("ir_sensor_out", m_ir_sensor_out),
    m_ir_sensor_metre_outOut("ir_sensor_metre_out", m_ir_sensor_metre_out)

    // </rtc-template>
{
	m_so = new RasPiMouseSimulatorObj();
	m_dt = NULL;
}

/*!
 * @brief destructor
 */
RaspberryPiMouseSimulator::~RaspberryPiMouseSimulator()
{
	delete m_so;
	if (m_dt)
	{
		delete m_dt;
	}
}



RTC::ReturnCode_t RaspberryPiMouseSimulator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("target_velocity_in", m_target_velocity_inIn);
  addInPort("pose_update", m_pose_updateIn);
  
  // Set OutPort buffer
  addOutPort("current_velocity_out", m_current_velocity_outOut);
  addOutPort("current_pose_out", m_current_pose_outOut);
  addOutPort("ir_sensor_out", m_ir_sensor_outOut);
  addOutPort("ir_sensor_metre_out", m_ir_sensor_metre_outOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("sampling_time", m_sampling_time, "-1");
  bindParameter("draw_time", m_draw_time, "0.01");
  bindParameter("sensor_param", m_sensor_param, "1394,792,525,373,299,260,222,181,135,100,81,36,17,16");
  bindParameter("blocksConfigFile", m_blocksConfigFile, "None");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RaspberryPiMouseSimulator::onActivated(RTC::UniqueId ec_id)
{
	m_so->destroyRobot();

	if (m_blocksConfigFile != "None")
	{
		m_so->loadBlocksData(m_blocksConfigFile);
	}
	m_so->makeRobot();

	if (m_sampling_time > 0)
	{
		m_so->setSamplingTime(m_sampling_time);
	}
	else
	{
		m_so->setSamplingTime(1.0 / this->getExecutionContext(0)->get_rate());
	}

	if (m_dt == NULL)
	{
		m_dt = new DrawThread_RasPiMouse(m_so, m_draw_time);

		m_dt->activate();
	}
	if (m_dt)
	{
		m_dt->fps = 1.0 / m_draw_time;
		m_dt->setRCPFlag();
	}

	m_so->RasPiMouse.setCurrentPosition(0, 0, 0);

	for (int i = 0; i < 4; i++)
	{
		m_so->RasPiMouse.ir_sensor[i].setSensorParam(m_sensor_param);
	}


  return RTC::RTC_OK;
}


RTC::ReturnCode_t RaspberryPiMouseSimulator::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RaspberryPiMouseSimulator::onExecute(RTC::UniqueId ec_id)
{
	if (m_target_velocity_inIn.isNew())
	{
		m_target_velocity_inIn.read();
		m_so->RasPiMouse.setTargetVelocity(m_target_velocity_in.data.vx, m_target_velocity_in.data.va);
	}
	if (m_pose_updateIn.isNew())
	{
		m_pose_updateIn.read();

		m_so->RasPiMouse.setCurrentPosition(m_pose_update.data.position.x, m_pose_update.data.position.y, m_pose_update.data.heading);
	}

	//m_so->RasPiMouse.setTargetVelocity(0.0, 3.14);
	//std::cout << m_target_velocity_in.data.va << std::endl;
	m_so->update();

	setTimestamp(m_current_pose_out);
	m_current_pose_out.data.position.x = m_so->RasPiMouse.current_px;
	m_current_pose_out.data.position.y = m_so->RasPiMouse.current_py;
	m_current_pose_out.data.heading = m_so->RasPiMouse.current_pa;
	m_current_pose_outOut.write();

	setTimestamp(m_current_velocity_out);
	m_current_velocity_out.data.vx = m_so->current_vx;
	m_current_velocity_out.data.vy = m_so->current_vy;
	m_current_velocity_out.data.va = m_so->current_va;
	m_current_velocity_outOut.write();

	m_ir_sensor_out.data.length(4);
	setTimestamp(m_ir_sensor_out);
	for (int i = 0; i < 4; i++)
	{
		m_ir_sensor_out.data[i] = (int)m_so->RasPiMouse.ir_sensor[i].getSensorData();
	}
	m_ir_sensor_outOut.write();

	m_ir_sensor_metre_out.data.length(4);
	setTimestamp(m_ir_sensor_metre_out);
	for (int i = 0; i < 4; i++)
	{
		m_ir_sensor_metre_out.data[i] = m_so->RasPiMouse.ir_sensor[i].data;
	}
	m_ir_sensor_metre_outOut.write();



  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RaspberryPiMouseSimulator::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void RaspberryPiMouseSimulatorInit(RTC::Manager* manager)
  {
    coil::Properties profile(raspberrypimousesimulator_spec);
    manager->registerFactory(profile,
                             RTC::Create<RaspberryPiMouseSimulator>,
                             RTC::Delete<RaspberryPiMouseSimulator>);
  }
  
};



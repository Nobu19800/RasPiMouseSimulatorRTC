// -*- C++ -*-
/*!
 * @file  RaspberryPiMouseSimulator.h
 * @brief RaspberryPiMouse Simulator
 * @date  $Date$
 *
 * @author Nobuhiko Miyamoto <n-miyamoto@aist.go.jp>
 *
 * LGPL
 *
 * $Id$
 */

#ifndef RASPBERRYPIMOUSESIMULATOR_H
#define RASPBERRYPIMOUSESIMULATOR_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;


#include "DrawThread_RasPiMouse.h"
#include "RasPiMouseSimulatorObj.h"

/*!
 * @class RaspberryPiMouseSimulator
 * @brief RaspberryPiMouse Simulator
 *
 */
class RaspberryPiMouseSimulator
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  RaspberryPiMouseSimulator(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~RaspberryPiMouseSimulator();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * シミュレーションの刻み幅
   * - Name: sampling_time sampling_time
   * - DefaultValue: -1
   * - Unit: s
   */
  double m_sampling_time;
  /*!
   * 描画の速度
   * - Name: draw_time draw_time
   * - DefaultValue: 0.01
   * - Unit: s
   */
  double m_draw_time;
  /*!
   * 距離センサのデータを生データに変換するパラメータ
   * 0.01、0.02、0.03、0.04、0.05、0.06、0.07、0.08、0.09、0.10、0
   * .15、0.20、0.25、0.30[m]に対応した値を設定
   * - Name: sensor_param sensor_param
   * - DefaultValue: 1394,792,525,373,299,260,222,181,135,100,81,36,17,16
   */
  std::string m_sensor_param;
  /*!
  * 障害物の配置設定ファイルの名前
  * - Name: blocksConfigFile blocksConfigFile
  * - DefaultValue: Non
  */
  std::string m_blocksConfigFile;
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedVelocity2D m_target_velocity_in;
  /*!
   * 目標速度
   * - Type: RTC::TimedVelocity2D
   * - Unit: m/s,rad/s
   */
  InPort<RTC::TimedVelocity2D> m_target_velocity_inIn;
  RTC::TimedPose2D m_pose_update;
  /*!
   * 現在位置の更新
   * - Type: RTC::TimedPose2D
   * - Unit: m,rad
   */
  InPort<RTC::TimedPose2D> m_pose_updateIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedVelocity2D m_current_velocity_out;
  /*!
   * 現在の速度
   * - Type: RTC::TimedVelocity2D
   * - Unit: m/s,rad/s
   */
  OutPort<RTC::TimedVelocity2D> m_current_velocity_outOut;
  RTC::TimedPose2D m_current_pose_out;
  /*!
   * 現在位置
   * - Type: RTC::TimedPose2D
   * - Unit: m,rad
   */
  OutPort<RTC::TimedPose2D> m_current_pose_outOut;
  RTC::TimedShortSeq m_ir_sensor_out;
  /*!
   * 距離センサの計測値(生データを再現)
   * - Type: RTC::TimedShortSeq
   * - Number: 4
   */
  OutPort<RTC::TimedShortSeq> m_ir_sensor_outOut;
  RTC::TimedDoubleSeq m_ir_sensor_metre_out;
  /*!
   * 距離センサの計測値
   * - Type: RTC::TimedDoubleSeq
   * - Unit: m
   */
  OutPort<RTC::TimedDoubleSeq> m_ir_sensor_metre_outOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
	 RasPiMouseSimulatorObj *m_so;
	 DrawThread_RasPiMouse *m_dt;
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void RaspberryPiMouseSimulatorInit(RTC::Manager* manager);
};

#endif // RASPBERRYPIMOUSESIMULATOR_H

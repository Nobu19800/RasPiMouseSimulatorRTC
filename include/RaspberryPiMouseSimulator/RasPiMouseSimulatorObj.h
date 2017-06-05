/*!
* @file  SimulatorObj.h
* @brief シミュレーション関連のクラス
*
*/

#ifndef RasPiMouseSIMULATOROBJ_H
#define RasPiMouseSIMULATOROBJ_H

//#define dDOUBLE

#include <coil/Mutex.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <vector>


#define _USE_MATH_DEFINES
#include <math.h>





#define DEFAULT_BODY_LENGTH 0.05
#define DEFAULT_BODY_WIDTH 0.08
#define DEFAULT_BODY_HEIGHT 0.05


#define DEFAULT_BODY_X 0
#define DEFAULT_BODY_Y 0
#define DEFAULT_BODY_Z (0.1)


#define DEFAULT_BODY_MASS 0.5

#define DEFAULT_WHEEL_RADIUS 0.02
#define DEFAULT_WHEEL_WIDTH 0.005

#define DEFAULT_WHEEL_X (DEFAULT_BODY_X)
#define DEFAULT_WHEEL_Y (DEFAULT_BODY_Y + DEFAULT_BODY_WIDTH / 2.0 + DEFAULT_WHEEL_WIDTH / 2.0)
#define DEFAULT_WHEEL_Z (DEFAULT_BODY_Z - 0.015)

#define DEFAULT_WHEEL_MASS 0.01


#define DEFAULT_TOPPLATE1_LENGTH DEFAULT_BODY_LENGTH
#define DEFAULT_TOPPLATE1_WIDTH DEFAULT_BODY_WIDTH
#define DEFAULT_TOPPLATE1_HEIGHT 0.005

#define DEFAULT_TOPPLATE1_X DEFAULT_BODY_X
#define DEFAULT_TOPPLATE1_Y DEFAULT_BODY_Y
#define DEFAULT_TOPPLATE1_Z (DEFAULT_BODY_Z + DEFAULT_BODY_HEIGHT/2.0 + DEFAULT_BOTTOMPLATE1_HEIGHT/2.0)


#define DEFAULT_TOPPLATE1_MASS 0.01



#define DEFAULT_TOPPLATE2_RADIUS DEFAULT_TOPPLATE1_WIDTH/2.0
#define DEFAULT_TOPPLATE2_HEIGHT DEFAULT_TOPPLATE1_HEIGHT

#define DEFAULT_TOPPLATE2_X (DEFAULT_TOPPLATE1_X + DEFAULT_TOPPLATE1_LENGTH/2.0)
#define DEFAULT_TOPPLATE2_Y DEFAULT_TOPPLATE1_Y
#define DEFAULT_TOPPLATE2_Z DEFAULT_TOPPLATE1_Z


#define DEFAULT_TOPPLATE2_MASS 0.01



#define DEFAULT_TOPPLATE3_RADIUS DEFAULT_TOPPLATE1_WIDTH/2.0
#define DEFAULT_TOPPLATE3_HEIGHT DEFAULT_TOPPLATE1_HEIGHT

#define DEFAULT_TOPPLATE3_X (DEFAULT_TOPPLATE1_X - DEFAULT_TOPPLATE1_LENGTH/2.0)
#define DEFAULT_TOPPLATE3_Y DEFAULT_TOPPLATE1_Y
#define DEFAULT_TOPPLATE3_Z DEFAULT_TOPPLATE1_Z


#define DEFAULT_TOPPLATE3_MASS 0.01





#define DEFAULT_BOTTOMPLATE1_RADIUS DEFAULT_BODY_WIDTH/2.0
#define DEFAULT_BOTTOMPLATE1_HEIGHT DEFAULT_TOPPLATE1_HEIGHT

#define DEFAULT_BOTTOMPLATE1_X (DEFAULT_TOPPLATE1_X + DEFAULT_BODY_LENGTH/2.0)
#define DEFAULT_BOTTOMPLATE1_Y DEFAULT_BODY_Y
#define DEFAULT_BOTTOMPLATE1_Z (DEFAULT_BODY_Z - DEFAULT_BODY_HEIGHT/2.0 - DEFAULT_BOTTOMPLATE1_HEIGHT/2.0)


#define DEFAULT_BOTTOMPLATE1_MASS 0.01



#define DEFAULT_BOTTOMPLATE2_RADIUS DEFAULT_BODY_WIDTH/2.0
#define DEFAULT_BOTTOMPLATE2_HEIGHT DEFAULT_TOPPLATE1_HEIGHT

#define DEFAULT_BOTTOMPLATE2_X (DEFAULT_TOPPLATE1_X - DEFAULT_BODY_LENGTH/2.0)
#define DEFAULT_BOTTOMPLATE2_Y DEFAULT_BODY_Y
#define DEFAULT_BOTTOMPLATE2_Z DEFAULT_BOTTOMPLATE1_Z

#define DEFAULT_BOTTOMPLATE2_MASS 0.01


#define DEFAULT_MIDDLEPLATE_MASS 0.01

#define DEFAULT_MIDDLEPLATE_RADIUS DEFAULT_BODY_WIDTH/2.0
#define DEFAULT_MIDDLEPLATE_HEIGHT DEFAULT_TOPPLATE1_HEIGHT

#define DEFAULT_MIDDLEPLATE_X (DEFAULT_BODY_X + DEFAULT_BODY_LENGTH/2.0)
#define DEFAULT_MIDDLEPLATE_Y DEFAULT_BODY_Y
#define DEFAULT_MIDDLEPLATE_Z DEFAULT_BODY_Z


#define DEFAULT_MIDDLEPLATE_MASS 0.01






#define DEFAULT_SUPPORTPLATE1_LENGTH 0.01
#define DEFAULT_SUPPORTPLATE1_WIDTH 0.01
#define DEFAULT_SUPPORTPLATE1_HEIGHT 0.005

#define DEFAULT_SUPPORTPLATE1_X (DEFAULT_BOTTOMPLATE1_X + 0.01)
#define DEFAULT_SUPPORTPLATE1_Y DEFAULT_BODY_Y
#define DEFAULT_SUPPORTPLATE1_Z (DEFAULT_BOTTOMPLATE1_Z -  DEFAULT_BOTTOMPLATE2_HEIGHT/2.0 - DEFAULT_SUPPORTPLATE1_HEIGHT/2.0)


#define DEFAULT_SUPPORTPLATE1_MASS 0.01



#define DEFAULT_SUPPORTPLATE2_LENGTH DEFAULT_SUPPORTPLATE1_LENGTH
#define DEFAULT_SUPPORTPLATE2_WIDTH DEFAULT_SUPPORTPLATE1_WIDTH
#define DEFAULT_SUPPORTPLATE2_HEIGHT DEFAULT_SUPPORTPLATE1_HEIGHT

#define DEFAULT_SUPPORTPLATE2_X (DEFAULT_BOTTOMPLATE2_X - 0.01)
#define DEFAULT_SUPPORTPLATE2_Y DEFAULT_BODY_Y
#define DEFAULT_SUPPORTPLATE2_Z DEFAULT_SUPPORTPLATE1_Z


#define DEFAULT_SUPPORTPLATE2_MASS 0.01


#define DEFAULT_RASPI_LENGTH 0.06
#define DEFAULT_RASPI_WIDTH 0.05
#define DEFAULT_RASPI_HEIGHT 0.005

#define DEFAULT_RASPI_X DEFAULT_BODY_X
#define DEFAULT_RASPI_Y DEFAULT_BODY_Y
#define DEFAULT_RASPI_Z (DEFAULT_TOPPLATE1_Z + 0.01)


#define DEFAULT_RASPI_MASS 0.01



#define DEFAULT_MAX_IRSENSOR_DISTANCE 100.0


#define DEFAULT_BLOCK_MASS 100

//センサの位置
//MIDDLE_PLATEからの距離

#define DEFAULT_IRSENSOR1_X 0.02
#define DEFAULT_IRSENSOR1_Y 0.03
#define DEFAULT_IRSENSOR1_Z DEFAULT_MIDDLEPLATE_HEIGHT
#define DEFAULT_IRSENSOR1_RADIUS M_PI/4


#define DEFAULT_IRSENSOR2_X 0.01
#define DEFAULT_IRSENSOR2_Y 0.04
#define DEFAULT_IRSENSOR2_Z DEFAULT_MIDDLEPLATE_HEIGHT
#define DEFAULT_IRSENSOR2_RADIUS 0


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif




enum EDirection { 
	Distance_10,
	Distance_20,
	Distance_30,
	Distance_40,
	Distance_50,
	Distance_60,
	Distance_70,
	Distance_80,
	Distance_90,
	Distance_100,
	Distance_150,
	Distance_200,
	Distance_250,
	Distance_300,
	Distance_NUM
};



/**
* @struct MyLink
*@brief ボディオブジェクト
*接続するジョイントも含む
*/
typedef struct {
  dBodyID  body;
  dGeomID  geom;
  dJointID joint;
  int  dir;
  float  red, green, blue;
  dReal    m,r,x,y,z,lx,ly,lz, the, dthe, ddthe, axisx, axisy, axisz, Tthe, Tdthe, Tddthe, tau, jx,jy,jz;
} MyLink;


/**
* @class IRSensorData
*@brief 距離センサの各種データ格納クラス
*/
class IRSensorData
{
public:
	IRSensorData();
	void setPos(double x, double y, double z);
	double calcDistance(double x, double y, double z);
	void resetDistance();
	double getSensorData();
	void setSensorParam(std::string d);
	std::vector<double> sensorParam;
	double data;
	double current_x, current_y, current_z;
};

/**
* @class RasPiMouse_Obj
*@brief RasPiMouseの各種データ格納クラス
*/
class RasPiMouseObj
{
public:
	RasPiMouseObj();
	void setTargetVelocity(double vx, double va);
	void setCurrentPosition(double px, double py, double pa);
	double target_vx, target_va;
	double current_px, current_py, current_pa;
	IRSensorData ir_sensor[4];
};
/**
* @class RasPiMouseSimulatorObj
*@brief シミュレーションの操作をするためのクラス
*/
class RasPiMouseSimulatorObj
{
public:
	/**
	*@brief コンストラクタ
	*/
	RasPiMouseSimulatorObj();
	/**
	*@brief デストラクタ
	*/
	~RasPiMouseSimulatorObj();
	coil::Mutex mu;
	MyLink centorUnit, wheelLeft, wheelRight, topPlate[3], middlePlate, bottomPlate[2], RaspPi, supportPlate[2];
	dGeomID IRSensor_ray[4];
	double current_vx, current_vy, current_va;
	std::vector<MyLink> blocks;
	bool plane_exist;
	MyLink plane;
	double st;
	double gravity;
	bool pause;
	dWorldID      world;       
	dSpaceID      space;       
	dGeomID       ground;       
	dJointGroupID contactgroup;
	RasPiMouseObj RasPiMouse;

	
	/**
	*@brief 距離センサを設定
	*/
	void setIRSensorRay();
	/**
	*@brief 各パラメータの初期化を行う
	*@param offset_z 高さを調整
	*/
	void makeParam(double offset_z=0);
	/**
	*@brief 直方体作成
	* @param body ボディオブジェクト
	*/
	void setBox(MyLink *body);
	/**
	*@brief 円柱作成
	* @param body ボディオブジェクト
	*/
	void setCylinder(MyLink *body);
	/**
	*@brief 球作成
	* @param body ボディオブジェクト
	*/
	void setSphere(MyLink *body);
	/**
	*@brief ヒンジジョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setHinge(MyLink *body1, MyLink *body2);
	/**
	*@brief スライダージョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setSlider(MyLink *body1, MyLink *body2);
	/**
	*@brief 固定ジョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setFixed(MyLink *body1, MyLink *body2);
	/**
	*@brief ボールジョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setBall(MyLink *body1, MyLink *body2);
	/**
	*@brief 全ボディ、接続する全ジョイント生成
	*/
	void makeRobot();
	/**
	*@brief ヒンジジョイント制御
	* @param body ボディオブジェクト
	* @param theta ヒンジジョイントの位置
	*/
	void controlHinge(MyLink *body, dReal theta);
	/**
	*@brief ヒンジジョイント制御
	* @param body ボディオブジェクト
	* @param theta ヒンジジョイントの位置
	* @param vel ヒンジジョイントの速度
	*/
	void controlHinge(MyLink *body, dReal theta, dReal vel);
	/**
	*@brief スライダージョイント制御
	* @param body ボディオブジェクト
	* @param length スライダージョイントの位置
	*/
	void controlSlider(MyLink *body, dReal length);
	/**
	*@brief 全ジョイント制御
	*/
	void control();
	/**
	*@brief 更新
	*/
	void update();
	/**
	*@brief 全ボディ、接続する全ジョイント消去
	*/
	void destroyRobot();

	/**
	*@brief 接触コールバック
	* @param o1 ジオメトリ1
	* @param o2 ジオメトリ2
	*/
	void m_nearCallback(dGeomID o1, dGeomID o2);
	/**
	*@brief 刻み幅設定
	* @param s サンプリング時間
	*/
	void setSamplingTime(double s);


	/**
	*@brief 地面生成
	*/
	void makePlane(double lx, double ly, double lz);

	/**
	*@brief 障害物生成
	*/
	void makeBlock(double x, double y, double z, double lx, double ly, double lz, double r);

	/**
	*@brief ファイルから障害物は位置読み込み
	*@return 読み込み成功(true)、失敗(false)
	*/
	bool loadBlocksData(std::string fname);


};



#endif

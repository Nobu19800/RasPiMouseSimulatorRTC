/*!
* @file  RasPiMouseSimulatorObj.cpp
* @brief シミュレーション関連のクラス
*
*/


#include "RasPiMouseSimulatorObj.h"
#include <fstream>
#include <coil/stringutil.h>

RasPiMouseSimulatorObj *obj_raspisim = NULL;


IRSensorData::IRSensorData():
data(0),
current_x(0),
current_y(0),
current_z(0)
{
}

void IRSensorData::setPos(double x, double y, double z)
{
	current_x = x;
	current_y = y;
	current_z = z;

	//std::cout << current_x << "\t" << current_y << "\t" << current_z << std::endl;

}


double IRSensorData::calcDistance(double x, double y, double z)
{
	double tmp = sqrt(pow(current_x - x,2) + pow(current_y - y,2) + pow(current_z - z,2));
	if (data > tmp)
	{
		data = tmp;
	}

	return data;
}

void IRSensorData::resetDistance()
{
	data = DEFAULT_MAX_IRSENSOR_DISTANCE;
}

void IRSensorData::setSensorParam(std::string d)
{
	sensorParam.clear();
	std::vector<std::string> dlist = coil::split(d,",");
	for (std::vector<std::string>::iterator itr = dlist.begin(); itr != dlist.end(); ++itr) {
		double v;
		coil::stringTo<double>(v,itr->c_str());
		sensorParam.push_back(v);
	}
}

double IRSensorData::getSensorData()
{
	if (sensorParam.size() < Distance_NUM)
	{
		return -1;
	}
	if (data <= 0.02)
	{
		double a = (sensorParam[Distance_20] - sensorParam[Distance_10])/0.01;
		double b = sensorParam[Distance_20] - a*0.02;
		return a*data + b;
	}
	else if (data > 0.02 && data <= 0.03)
	{
		double a = (sensorParam[Distance_30] - sensorParam[Distance_20]) / 0.01;
		double b = sensorParam[Distance_30] - a*0.03;
		return a*data + b;
	}
	else if (data > 0.03 && data <= 0.04)
	{
		double a = (sensorParam[Distance_40] - sensorParam[Distance_30]) / 0.01;
		double b = sensorParam[Distance_40] - a*0.04;
		return a*data + b;
	}
	else if (data > 0.04 && data <= 0.05)
	{
		double a = (sensorParam[Distance_50] - sensorParam[Distance_40]) / 0.01;
		double b = sensorParam[Distance_50] - a*0.05;
		return a*data + b;
	}
	else if (data > 0.05 && data <= 0.06)
	{
		double a = (sensorParam[Distance_60] - sensorParam[Distance_50]) / 0.01;
		double b = sensorParam[Distance_60] - a*0.06;
		return a*data + b;
	}
	else if (data > 0.06 && data <= 0.07)
	{
		double a = (sensorParam[Distance_70] - sensorParam[Distance_60]) / 0.01;
		double b = sensorParam[Distance_70] - a*0.07;
		return a*data + b;
	}
	else if (data > 0.07 && data <= 0.08)
	{
		double a = (sensorParam[Distance_80] - sensorParam[Distance_70]) / 0.01;
		double b = sensorParam[Distance_80] - a*0.08;
		return a*data + b;
	}
	else if (data > 0.08 && data <= 0.09)
	{
		double a = (sensorParam[Distance_90] - sensorParam[Distance_80]) / 0.01;
		double b = sensorParam[Distance_90] - a*0.09;
		return a*data + b;
	}
	else if (data > 0.09 && data <= 0.10)
	{
		double a = (sensorParam[Distance_100] - sensorParam[Distance_90]) / 0.01;
		double b = sensorParam[Distance_100] - a*0.10;
		return a*data + b;
	}
	else if (data > 0.10 && data <= 0.15)
	{
		double a = (sensorParam[Distance_150] - sensorParam[Distance_100]) / 0.05;
		double b = sensorParam[Distance_150] - a*0.15;
		return a*data + b;
	}
	else if (data > 0.15 && data <= 0.20)
	{
		double a = (sensorParam[Distance_200] - sensorParam[Distance_150]) / 0.05;
		double b = sensorParam[Distance_200] - a*0.20;
		return a*data + b;
	}
	else if (data > 0.20 && data <= 0.25)
	{
		double a = (sensorParam[Distance_250] - sensorParam[Distance_200]) / 0.05;
		double b = sensorParam[Distance_250] - a*0.25;
		return a*data + b;
	}
	else
	{
		double a = (sensorParam[Distance_300] - sensorParam[Distance_250]) / 0.05;
		double b = sensorParam[Distance_300] - a*0.30;
		return a*data + b;
	}
}

RasPiMouseObj::RasPiMouseObj() :
target_vx(0),
target_va(0),
current_px(0),
current_py(0),
current_pa(0)
{

}

void RasPiMouseObj::setTargetVelocity(double vx, double va)
{
	target_vx = vx;
	target_va = va;
}

void RasPiMouseObj::setCurrentPosition(double px, double py, double pa)
{
	current_px = px;
	current_py = py;
	current_pa = pa;
}




/**
*@brief シミュレーションの操作をするためのクラスのコンストラクタ
*/
RasPiMouseSimulatorObj::RasPiMouseSimulatorObj()
{
	st = 0.01;
	gravity = 9.8;
	pause = false;
	plane_exist = false;

	current_vx = 0;
	current_vy = 0;
	current_va = 0;
	

	dInitODE();
	world        = dWorldCreate();
	space        = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	ground       = dCreatePlane(space,0,0,1,0);

	dWorldSetGravity(world, 0, 0, -9.8);
	dWorldSetCFM(world, 1e-6);
	dWorldSetERP(world, 1.0);

	makeParam();
	makeRobot();
	//makePlane(100,100,10);

	obj_raspisim = this;
}

/**
*@brief シミュレーションの操作をするためのクラスのデストラクタ
*/
RasPiMouseSimulatorObj::~RasPiMouseSimulatorObj()
{
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
}

/**
*@brief 各パラメータの初期化を行う
*@param offset_z 高さを調整
*/
void RasPiMouseSimulatorObj::makeParam(double offset_z)
{
	centorUnit.m = DEFAULT_BODY_MASS;
	centorUnit.lx = DEFAULT_BODY_LENGTH;
	centorUnit.ly = DEFAULT_BODY_WIDTH;
	centorUnit.lz = DEFAULT_BODY_HEIGHT;
	centorUnit.x = DEFAULT_BODY_X;
	centorUnit.y = DEFAULT_BODY_Y;
	centorUnit.z = DEFAULT_BODY_Z + offset_z;
	centorUnit.red = 0.9;
	centorUnit.green = 0.9;
	centorUnit.blue = 0.9;

	wheelLeft.m = DEFAULT_WHEEL_MASS;
	wheelLeft.lx = DEFAULT_WHEEL_RADIUS;
	wheelLeft.lz = DEFAULT_WHEEL_WIDTH;
	wheelLeft.x = DEFAULT_WHEEL_X;
	wheelLeft.y = DEFAULT_WHEEL_Y;
	wheelLeft.z = DEFAULT_WHEEL_Z + offset_z;
	wheelLeft.jx = wheelLeft.x;
	wheelLeft.jy = wheelLeft.y;
	wheelLeft.jz = wheelLeft.z;
	wheelLeft.axisx = 0;
	wheelLeft.axisy = -1;
	wheelLeft.axisz = 0;
	wheelLeft.red = 0.1;
	wheelLeft.green = 0.1;
	wheelLeft.blue = 0.1;

	wheelRight.m = DEFAULT_WHEEL_MASS;
	wheelRight.lx = DEFAULT_WHEEL_RADIUS;
	wheelRight.lz = DEFAULT_WHEEL_WIDTH;
	wheelRight.x = DEFAULT_WHEEL_X;
	wheelRight.y = DEFAULT_BODY_Y - (DEFAULT_WHEEL_Y - DEFAULT_BODY_Y);
	wheelRight.z = DEFAULT_WHEEL_Z + offset_z;
	wheelRight.jx = wheelRight.x;
	wheelRight.jy = wheelRight.y;
	wheelRight.jz = wheelRight.z;
	wheelRight.axisx = 0;
	wheelRight.axisy = -1;
	wheelRight.axisz = 0;
	wheelRight.red = 0.1;
	wheelRight.green = 0.1;
	wheelRight.blue = 0.1;






	topPlate[0].m = DEFAULT_TOPPLATE1_MASS;
	topPlate[0].lx = DEFAULT_TOPPLATE1_LENGTH;
	topPlate[0].ly = DEFAULT_TOPPLATE1_WIDTH;
	topPlate[0].lz = DEFAULT_TOPPLATE1_HEIGHT;
	topPlate[0].x = DEFAULT_TOPPLATE1_X;
	topPlate[0].y = DEFAULT_TOPPLATE1_Y;
	topPlate[0].z = DEFAULT_TOPPLATE1_Z + offset_z;
	topPlate[0].red = 0.3;
	topPlate[0].green = 0.3;
	topPlate[0].blue = 0.3;

	topPlate[1].m = DEFAULT_TOPPLATE2_MASS;
	topPlate[1].lx = DEFAULT_TOPPLATE2_RADIUS;
	topPlate[1].lz = DEFAULT_TOPPLATE2_HEIGHT;
	topPlate[1].x = DEFAULT_TOPPLATE2_X;
	topPlate[1].y = DEFAULT_TOPPLATE2_Y;
	topPlate[1].z = DEFAULT_TOPPLATE2_Z + offset_z;
	topPlate[1].red = 0.3;
	topPlate[1].green = 0.3;
	topPlate[1].blue = 0.3;


	topPlate[2].m = DEFAULT_TOPPLATE3_MASS;
	topPlate[2].lx = DEFAULT_TOPPLATE3_RADIUS;
	topPlate[2].lz = DEFAULT_TOPPLATE3_HEIGHT;
	topPlate[2].x = DEFAULT_TOPPLATE3_X;
	topPlate[2].y = DEFAULT_TOPPLATE3_Y;
	topPlate[2].z = DEFAULT_TOPPLATE3_Z + offset_z;
	topPlate[2].red = 0.3;
	topPlate[2].green = 0.3;
	topPlate[2].blue = 0.3;


	middlePlate.m = DEFAULT_MIDDLEPLATE_MASS;
	middlePlate.lx = DEFAULT_MIDDLEPLATE_RADIUS;
	middlePlate.lz = DEFAULT_MIDDLEPLATE_HEIGHT;
	middlePlate.x = DEFAULT_MIDDLEPLATE_X;
	middlePlate.y = DEFAULT_MIDDLEPLATE_Y;
	middlePlate.z = DEFAULT_MIDDLEPLATE_Z + offset_z;
	middlePlate.red = 0.3;
	middlePlate.green = 0.3;
	middlePlate.blue = 0.3;

	bottomPlate[0].m = DEFAULT_BOTTOMPLATE1_MASS;
	bottomPlate[0].lx = DEFAULT_BOTTOMPLATE1_RADIUS;
	bottomPlate[0].lz = DEFAULT_BOTTOMPLATE1_HEIGHT;
	bottomPlate[0].x = DEFAULT_BOTTOMPLATE1_X;
	bottomPlate[0].y = DEFAULT_BOTTOMPLATE1_Y;
	bottomPlate[0].z = DEFAULT_BOTTOMPLATE1_Z + offset_z;
	bottomPlate[0].red = 0.9;
	bottomPlate[0].green = 0.9;
	bottomPlate[0].blue = 0.9;

	bottomPlate[1].m = DEFAULT_BOTTOMPLATE2_MASS;
	bottomPlate[1].lx = DEFAULT_BOTTOMPLATE2_RADIUS;
	bottomPlate[1].lz = DEFAULT_BOTTOMPLATE2_HEIGHT;
	bottomPlate[1].x = DEFAULT_BOTTOMPLATE2_X;
	bottomPlate[1].y = DEFAULT_BOTTOMPLATE2_Y;
	bottomPlate[1].z = DEFAULT_BOTTOMPLATE2_Z + offset_z;
	bottomPlate[1].red = 0.9;
	bottomPlate[1].green = 0.9;
	bottomPlate[1].blue = 0.9;


	RaspPi.m = DEFAULT_RASPI_MASS;
	RaspPi.lx = DEFAULT_RASPI_LENGTH;
	RaspPi.ly = DEFAULT_RASPI_WIDTH;
	RaspPi.lz = DEFAULT_RASPI_HEIGHT;
	RaspPi.x = DEFAULT_RASPI_X;
	RaspPi.y = DEFAULT_RASPI_Y;
	RaspPi.z = DEFAULT_RASPI_Z + offset_z;
	RaspPi.red = 0.1;
	RaspPi.green = 0.7;
	RaspPi.blue = 0.1;


	supportPlate[0].m = DEFAULT_SUPPORTPLATE1_MASS;
	supportPlate[0].lx = DEFAULT_SUPPORTPLATE1_LENGTH;
	supportPlate[0].ly = DEFAULT_SUPPORTPLATE1_WIDTH;
	supportPlate[0].lz = DEFAULT_SUPPORTPLATE1_HEIGHT;
	supportPlate[0].x = DEFAULT_SUPPORTPLATE1_X;
	supportPlate[0].y = DEFAULT_SUPPORTPLATE1_Y;
	supportPlate[0].z = DEFAULT_SUPPORTPLATE1_Z + offset_z;
	supportPlate[0].red = 0.0;
	supportPlate[0].green = 0.0;
	supportPlate[0].blue = 0.0;


	supportPlate[1].m = DEFAULT_SUPPORTPLATE2_MASS;
	supportPlate[1].lx = DEFAULT_SUPPORTPLATE2_LENGTH;
	supportPlate[1].ly = DEFAULT_SUPPORTPLATE2_WIDTH;
	supportPlate[1].lz = DEFAULT_SUPPORTPLATE2_HEIGHT;
	supportPlate[1].x = DEFAULT_SUPPORTPLATE2_X;
	supportPlate[1].y = DEFAULT_SUPPORTPLATE2_Y;
	supportPlate[1].z = DEFAULT_SUPPORTPLATE2_Z + offset_z;
	supportPlate[1].red = 0.0;
	supportPlate[1].green = 0.0;
	supportPlate[1].blue = 0.0;


	
}

/**
*@brief 直方体作成
* @param body ボディオブジェクト
*/
void RasPiMouseSimulatorObj::setBox(MyLink *body)
{
	dMass mass;
	body->body  = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass,body->m , body->lx, body->ly, body->lz);
	dBodySetMass(body->body,&mass);
	body->geom = dCreateBox(space,body->lx, body->ly, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

/**
*@brief 円柱作成
* @param body ボディオブジェクト
*/
void RasPiMouseSimulatorObj::setCylinder(MyLink *body)
{
	dMass mass;
	body->body  = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass,body->m , 2, body->lx,  body->lz);
	dBodySetMass(body->body,&mass);
	body->geom = dCreateCylinder(space,body->lx, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

/**
*@brief 球作成
* @param body ボディオブジェクト
*/
void RasPiMouseSimulatorObj::setSphere(MyLink *body)
{
	dMass mass;
	body->body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, body->m, body->lz);
	dBodySetMass(body->body, &mass);
	body->geom = dCreateSphere(space, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

/**
*@brief ヒンジジョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void RasPiMouseSimulatorObj::setHinge(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateHinge(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetHingeAnchor(body1->joint, body1->jx, body1->jy, body1->jz);
	dJointSetHingeAxis(body1->joint, body1->axisx, body1->axisy,body1->axisz);
}

/**
*@brief スライダージョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void RasPiMouseSimulatorObj::setSlider(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateSlider(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	
	dJointSetSliderAxis(body1->joint, body1->axisx, body1->axisy,body1->axisz);
}

/**
*@brief 固定ジョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void RasPiMouseSimulatorObj::setFixed(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateFixed(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetFixed(body1->joint);
}


/**
*@brief ボールジョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void RasPiMouseSimulatorObj::setBall(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateBall(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetBallAnchor(body1->joint, body1->jx, body1->jy, body1->jz);
	//dJointSetBallAnchor2(body1->joint, body1->jx, body1->jy, body1->jz);
}

/**
*@brief 全ボディ、接続する全ジョイント生成
*/
void RasPiMouseSimulatorObj::makeRobot()
{
	mu.lock();
	dMatrix3 R;
	setBox(&centorUnit);
	setCylinder(&wheelLeft);
	setCylinder(&wheelRight);
	setBox(&topPlate[0]);
	setCylinder(&topPlate[1]);
	setCylinder(&topPlate[2]);
	setCylinder(&middlePlate);
	setCylinder(&bottomPlate[0]);
	setCylinder(&bottomPlate[1]);
	setBox(&RaspPi);
	setBox(&supportPlate[0]);
	setBox(&supportPlate[1]);


	
	dRFromAxisAndAngle(R, 1, 0, 0, M_PI/2);
	dGeomSetRotation(wheelLeft.geom, R);
	
	dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
	dGeomSetRotation(wheelRight.geom, R);



	setHinge(&wheelLeft, &centorUnit);
	setHinge(&wheelRight, &centorUnit);
	setFixed(&topPlate[0], &centorUnit);
	setFixed(&topPlate[1], &topPlate[0]);
	setFixed(&topPlate[2], &topPlate[0]);
	setFixed(&middlePlate, &centorUnit);
	setFixed(&bottomPlate[0], &centorUnit);
	setFixed(&bottomPlate[1], &centorUnit);
	setFixed(&RaspPi, &topPlate[0]);
	setFixed(&supportPlate[0], &bottomPlate[0]);
	setFixed(&supportPlate[1], &bottomPlate[1]);

	for(int i=0;i < 4;i++)
	{
		IRSensor_ray[i] = dCreateRay(space, 200);
	}


	
	

	pause = true;

	mu.unlock();

	
}

/**
*@brief 接触コールバック
* @param o1 ジオメトリ1
* @param o2 ジオメトリ2
*/
void RasPiMouseSimulatorObj::m_nearCallback(dGeomID o1, dGeomID o2)
{
	//return;
	
  dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
  if (b1 && b2 && dAreConnected(b1,b2)) return;
  //if ((o1 != ground) && (o2 != ground)) return;
  //if ((o1 == plane.geom) || (o2 == plane.geom)) return;
 
   static const int N = 20;
	dContact contact[N];
	int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

	
	


	if (n > 0) {
		for (int i = 0; i < 4; i++)
		{
			if (o1 == IRSensor_ray[i] || o2 == IRSensor_ray[i])
			{
				int j = 0;
				for (std::vector<MyLink>::iterator itr = blocks.begin(); itr != blocks.end(); ++itr) {
					if (o1 == itr->geom || o2 == itr->geom)
					{
						//std::cout << i << "\t" << j << std::endl;
						//std::cout << contact[0].geom.pos[0] << "\t" << contact[0].geom.pos[1] << "\t" <<  contact[0].geom.pos[2] << std::endl;
						RasPiMouse.ir_sensor[i].calcDistance(contact[0].geom.pos[0], contact[0].geom.pos[1], contact[0].geom.pos[2]);
						return;
					}
					j++;
				}

				return;
			}
		}

		for (int i=0; i<n; i++) {
			contact[i].surface.mode = dContactApprox1|dContactSoftERP|dContactSoftCFM|dContactSlip1|dContactSlip2;
			//contact[i].surface.mode =  dContactSoftERP | dContactSoftCFM;


			if(o1 == supportPlate[0].geom || o2 == supportPlate[0].geom || o1 == supportPlate[1].geom || o2 == supportPlate[1].geom)
			{
				contact[i].surface.mu   = 0.0;
			}
			else
			{
				//contact[i].surface.mu   = dInfinity;
				contact[i].surface.mu = 200.0;
			}
			
			contact[i].surface.slip1 = 0.001;
			contact[i].surface.slip2 = 0.001;
			contact[i].surface.soft_erp = 0.3;
			contact[i].surface.soft_cfm = 1e-4;
			dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
			dJointAttach(c,b1,b2);
		}
	}
}

/**
*@brief 接触コールバック
* @param data データ
* @param o1 ジオメトリ1
* @param o2 ジオメトリ2
*/
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	
	if(obj_raspisim)
	{
		obj_raspisim->m_nearCallback(o1, o2);
	}
  
		
}

/**
*@brief ヒンジジョイント制御
* @param body ボディオブジェクト
* @param theta ヒンジジョイントの位置
*/
void RasPiMouseSimulatorObj::controlHinge(MyLink *body, dReal theta)
{
	dReal kp = 100;
	dReal tmp = dJointGetHingeAngle(body->joint);
	dReal diff = theta - tmp;
	dReal u = kp * diff;

	dJointSetHingeParam(body->joint,dParamVel, u);
	dJointSetHingeParam(body->joint,dParamFMax,20.);
}

/**
*@brief ヒンジジョイント制御
* @param body ボディオブジェクト
* @param theta ヒンジジョイントの位置
* @param vel ヒンジジョイントの速度
*/
void RasPiMouseSimulatorObj::controlHinge(MyLink *body, dReal theta, dReal vel)
{
	dReal kp = 100;
	dReal tmp = dJointGetHingeAngle(body->joint);
	dReal diff = theta - tmp;
	dReal u = kp * diff;
	if (abs(vel) < abs(u))
	{
		u = vel;
	}

	dJointSetHingeParam(body->joint, dParamVel, u);
	dJointSetHingeParam(body->joint, dParamFMax, 20.);
}

/**
*@brief スライダージョイント制御
* @param body ボディオブジェクト
* @param length スライダージョイントの位置
*/
void RasPiMouseSimulatorObj::controlSlider(MyLink *body, dReal length)
{
	dReal kp = 10;
	dReal tmp = dJointGetSliderPosition(body->joint);
	dReal diff = length - tmp;
	dReal u = kp * diff;

	dJointSetSliderParam (body->joint,dParamVel, u);
	dJointSetSliderParam (body->joint,dParamFMax,20.);
}

/**
*@brief 全ジョイント制御
*/
void RasPiMouseSimulatorObj::control()
{
	double vx = RasPiMouse.target_vx;
	double va = RasPiMouse.target_va;
	double wheel_distance = (wheelLeft.y - wheelRight.y) / 2.0;
	double wheel_radius = wheelLeft.lx;

	double right_motor_speed = (vx + va*wheel_distance) / wheel_radius;
	double left_motor_speed = (vx - va*wheel_distance) / wheel_radius;

	//std::cout << right_motor_speed << "\t" << left_motor_speed << std::endl;
	dJointSetHingeParam(wheelLeft.joint, dParamVel, left_motor_speed);
	dJointSetHingeParam(wheelLeft.joint, dParamFMax, 20.);

	dJointSetHingeParam(wheelRight.joint, dParamVel, right_motor_speed);
	dJointSetHingeParam(wheelRight.joint, dParamFMax, 20.);
	
	double left_wheel_speed = dJointGetHingeParam(wheelLeft.joint, dParamVel) * wheel_radius;
	double right_wheel_speed = dJointGetHingeParam(wheelRight.joint, dParamVel) * wheel_radius;
	double o = (right_wheel_speed - left_wheel_speed) / (2.0*wheel_distance);
	double v = (right_wheel_speed + left_wheel_speed) / 2.0;

	current_vx = v * cos(RasPiMouse.current_pa);
	current_vy = v * sin(RasPiMouse.current_pa);
	current_va = o;

	//std::cout << dJointGetHingeParam(wheelRight.joint, dParamVel) << "\t" << dJointGetHingeParam(wheelLeft.joint, dParamVel) << std::endl;

	//RasPiMouse.current_px += current_vx*st;
	//RasPiMouse.current_py += current_vy*st;
	//RasPiMouse.current_pa += current_va*st;

	dVector3 b;
	dBodyGetRelPointPos(centorUnit.body, DEFAULT_WHEEL_X, 0, 0, b);
	//const dReal *b = dBodyGetPosition(centorUnit.body);
	RasPiMouse.current_px = b[0];
	RasPiMouse.current_py = b[1];
	const dReal *r = dBodyGetRotation(centorUnit.body);
	RasPiMouse.current_pa = atan2(r[4],r[0]);
	//std::cout << b[0] << "\t" << b[1] << std::endl;
	//std::cout << RasPiMouse.current_px << "\t" << RasPiMouse.current_py << std::endl;
	/*if (RasPiMouse.current_pa > M_PI * 2)
	{
		std::cout << RasPiMouse.current_pa << std::endl;
		RasPiMouse.current_pa -= M_PI * 2;
	}*/
}

/**
*@brief 距離センサを設定
*/
void RasPiMouseSimulatorObj::setIRSensorRay()
{
	dVector3 pos0, pos1;


	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR2_X, DEFAULT_IRSENSOR2_Y, DEFAULT_IRSENSOR2_Z, pos0);
	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR2_X + DEFAULT_MAX_IRSENSOR_DISTANCE*cos(DEFAULT_IRSENSOR2_RADIUS), DEFAULT_IRSENSOR2_Y + DEFAULT_MAX_IRSENSOR_DISTANCE*sin(DEFAULT_IRSENSOR2_RADIUS), DEFAULT_IRSENSOR2_Z, pos1);
	RasPiMouse.ir_sensor[3].setPos(pos0[0], pos0[1], pos0[2]);
	//std::cout << pos0[0] << "\t" << pos0[1] << "\t" << pos0[2] << std::endl;
	
	dGeomRaySet(IRSensor_ray[3], pos0[0], pos0[1], pos0[2], pos1[0] - pos0[0], pos1[1] - pos0[1], pos1[2] - pos0[2]);
	RasPiMouse.ir_sensor[3].resetDistance();
	//std::cout << pos1[0] << "\t" << pos1[1] << "\t" << pos1[2] << std::endl;



	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR1_X, DEFAULT_IRSENSOR1_Y, DEFAULT_IRSENSOR1_Z, pos0);
	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR1_X + DEFAULT_MAX_IRSENSOR_DISTANCE*cos(DEFAULT_IRSENSOR1_RADIUS), DEFAULT_IRSENSOR1_Y + DEFAULT_MAX_IRSENSOR_DISTANCE*sin(DEFAULT_IRSENSOR1_RADIUS), DEFAULT_IRSENSOR1_Z, pos1);
	RasPiMouse.ir_sensor[2].setPos(pos0[0], pos0[1], pos0[2]);
	dGeomRaySet(IRSensor_ray[2], pos0[0], pos0[1], pos0[2], pos1[0] - pos0[0], pos1[1] - pos0[1], pos1[2] - pos0[2]);
	RasPiMouse.ir_sensor[2].resetDistance();
	

	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR1_X, -DEFAULT_IRSENSOR1_Y, DEFAULT_IRSENSOR1_Z, pos0);
	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR1_X + DEFAULT_MAX_IRSENSOR_DISTANCE*cos(-DEFAULT_IRSENSOR1_RADIUS), -DEFAULT_IRSENSOR1_Y + DEFAULT_MAX_IRSENSOR_DISTANCE*sin(-DEFAULT_IRSENSOR1_RADIUS), DEFAULT_IRSENSOR1_Z, pos1);
	RasPiMouse.ir_sensor[1].setPos(pos0[0], pos0[1], pos0[2]);
	dGeomRaySet(IRSensor_ray[1], pos0[0], pos0[1], pos0[2], pos1[0] - pos0[0], pos1[1] - pos0[1], pos1[2] - pos0[2]);
	RasPiMouse.ir_sensor[1].resetDistance();


	

	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR2_X, -DEFAULT_IRSENSOR2_Y, DEFAULT_IRSENSOR2_Z, pos0);
	dBodyGetRelPointPos(middlePlate.body, DEFAULT_IRSENSOR2_X + DEFAULT_MAX_IRSENSOR_DISTANCE*cos(-DEFAULT_IRSENSOR2_RADIUS), -DEFAULT_IRSENSOR2_Y + DEFAULT_MAX_IRSENSOR_DISTANCE*sin(-DEFAULT_IRSENSOR2_RADIUS), DEFAULT_IRSENSOR2_Z, pos1);
	RasPiMouse.ir_sensor[0].setPos(pos0[0], pos0[1], pos0[2]);
	dGeomRaySet(IRSensor_ray[0], pos0[0], pos0[1], pos0[2], pos1[0] - pos0[0], pos1[1] - pos0[1], pos1[2] - pos0[2]);
	RasPiMouse.ir_sensor[0].resetDistance();
}


/**
*@brief 更新
*/
void RasPiMouseSimulatorObj::update()
{
	if(pause)
	{
		mu.lock();
		control();

		setIRSensorRay();

		dSpaceCollide(space,0,&nearCallback);
		dWorldStep(world, st);
		dJointGroupEmpty(contactgroup);
		mu.unlock();

	}
}

/**
*@brief 全ボディ、接続する全ジョイント消去
*/
void RasPiMouseSimulatorObj::destroyRobot()
{
	mu.lock();
	pause = false;

	dJointDestroy(wheelLeft.joint);
	dJointDestroy(wheelRight.joint);

	dJointDestroy(topPlate[0].joint);
	dJointDestroy(topPlate[1].joint);
	dJointDestroy(topPlate[2].joint);

	dJointDestroy(middlePlate.joint);

	dJointDestroy(bottomPlate[0].joint);
	dJointDestroy(bottomPlate[1].joint);

	dJointDestroy(RaspPi.joint);


	dJointDestroy(supportPlate[0].joint);
	dJointDestroy(supportPlate[1].joint);

	dBodyDestroy(centorUnit.body);
	dBodyDestroy(wheelLeft.body);
	dBodyDestroy(wheelRight.body);


	dBodyDestroy(topPlate[0].body);
	dBodyDestroy(topPlate[1].body);
	dBodyDestroy(topPlate[2].body);

	dBodyDestroy(middlePlate.body);

	dBodyDestroy(bottomPlate[0].body);
	dBodyDestroy(bottomPlate[1].body);

	dBodyDestroy(RaspPi.body);


	dBodyDestroy(supportPlate[0].body);
	dBodyDestroy(supportPlate[1].body);



	dGeomDestroy(centorUnit.geom);
	dGeomDestroy(wheelLeft.geom);
	dGeomDestroy(wheelRight.geom);



	dGeomDestroy(topPlate[0].geom);
	dGeomDestroy(topPlate[1].geom);
	dGeomDestroy(topPlate[2].geom);

	dGeomDestroy(middlePlate.geom);

	dGeomDestroy(bottomPlate[0].geom);
	dGeomDestroy(bottomPlate[1].geom);

	dGeomDestroy(RaspPi.geom);

	dGeomDestroy(supportPlate[0].geom);
	dGeomDestroy(supportPlate[1].geom);

	for(int i=0;i < 4;i++)
	{
		dGeomDestroy(IRSensor_ray[i]);
	}
	

	

	if (plane_exist)
	{
		dJointDestroy(plane.joint);
		dBodyDestroy(plane.body);
		dGeomDestroy(plane.geom);
		plane_exist = false;
	}
	

	for (std::vector<MyLink>::iterator itr = blocks.begin(); itr != blocks.end(); ++itr) {
		dJointDestroy(itr->joint);
		dBodyDestroy(itr->body);
		dGeomDestroy(itr->geom);
	}
	blocks.clear();

	mu.unlock();
}

void RasPiMouseSimulatorObj::makePlane(double lx, double ly, double lz)
{
	mu.lock();
	plane.m = DEFAULT_BLOCK_MASS;
	plane.lx = lx;
	plane.ly = ly;
	plane.lz = lz;
	plane.x = 0;
	plane.y = 0;
	plane.z = lz / 2.0;
	plane.red = 0.;
	plane.green = 1.;
	plane.blue = 0.;

	setBox(&plane);

	plane.joint = dJointCreateFixed(world, 0);
	dJointAttach(plane.joint, plane.body, 0);
	dJointSetFixed(plane.joint);

	plane_exist = true;
	mu.unlock();
}

/**
*@brief 障害物生成
*/
void RasPiMouseSimulatorObj::makeBlock(double x, double y, double z, double lx, double ly, double lz, double r)
{
	mu.lock();
	MyLink block;
	block.m = DEFAULT_BLOCK_MASS;
	block.lx = lx;
	block.ly = ly;
	block.lz = lz;
	block.x = x;
	block.y = y;
	block.z = z;
	block.red = 0.;
	block.green = 1.;
	block.blue = 0.;

	setBox(&block);

	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 0, 1, r);
	dGeomSetRotation(block.geom, R);

	block.joint = dJointCreateFixed(world, 0);
	dJointAttach(block.joint, block.body, 0);
	dJointSetFixed(block.joint);

	blocks.push_back(block);
	mu.unlock();
}


/**
*@brief ファイルから障害物は位置読み込み
*@return 読み込み成功(true)、失敗(false)
*/
bool RasPiMouseSimulatorObj::loadBlocksData(std::string fname)
{
	
#ifdef WIN32
	coil::replaceString(fname, "/", "\\");
#else
	coil::replaceString(fname, "\\", "/");
#endif
	std::ifstream ifs(fname);
	if (ifs.fail())
	{
		return false;
	}

	std::string str;
	int count = 0;
	while (getline(ifs, str))
	{
		if (count > 0)
		{
			std::vector<std::string> vlist = coil::split(str,",");
			std::vector<double> rlist;
			for (std::vector<std::string>::iterator itr = vlist.begin(); itr != vlist.end(); ++itr) {
				std::string v = (*itr);
				coil::eraseBothEndsBlank(v);
				double val = 0;
				bool ret = coil::stringTo<double>(val, v.c_str());
				rlist.push_back(val);
			}
			if (rlist.size() >= 7)
			{
				makeBlock(rlist[0], rlist[1], rlist[2], rlist[3], rlist[4], rlist[5], rlist[6]);
			}
			
		}
		count += 1;
	}

	return true;
}


/**
*@brief 刻み幅設定
* @param s サンプリング時間
*/
void RasPiMouseSimulatorObj::setSamplingTime(double s)
{
	st = s;
}

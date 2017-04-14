/*!
* @file  SimulatorObj.h
* @brief �V�~�����[�V�����֘A�̃N���X
*
*/

#ifndef RasPiMouseSIMULATOROBJ_H
#define RasPiMouseSIMULATOROBJ_H

#define dDOUBLE

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

//�Z���T�̈ʒu
//MIDDLE_PLATE����̋���

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
*@brief �{�f�B�I�u�W�F�N�g
*�ڑ�����W���C���g���܂�
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
*@brief �����Z���T�̊e��f�[�^�i�[�N���X
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
*@brief RasPiMouse�̊e��f�[�^�i�[�N���X
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
*@brief �V�~�����[�V�����̑�������邽�߂̃N���X
*/
class RasPiMouseSimulatorObj
{
public:
	/**
	*@brief �R���X�g���N�^
	*/
	RasPiMouseSimulatorObj();
	/**
	*@brief �f�X�g���N�^
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
	*@brief �����Z���T��ݒ�
	*/
	void setIRSensorRay();
	/**
	*@brief �e�p�����[�^�̏��������s��
	*@param offset_z �����𒲐�
	*/
	void makeParam(double offset_z=0);
	/**
	*@brief �����̍쐬
	* @param body �{�f�B�I�u�W�F�N�g
	*/
	void setBox(MyLink *body);
	/**
	*@brief �~���쐬
	* @param body �{�f�B�I�u�W�F�N�g
	*/
	void setCylinder(MyLink *body);
	/**
	*@brief ���쐬
	* @param body �{�f�B�I�u�W�F�N�g
	*/
	void setSphere(MyLink *body);
	/**
	*@brief �q���W�W���C���g�쐬
	* @param body1 �{�f�B1
	* @param body2 �{�f�B2
	*/
	void setHinge(MyLink *body1, MyLink *body2);
	/**
	*@brief �X���C�_�[�W���C���g�쐬
	* @param body1 �{�f�B1
	* @param body2 �{�f�B2
	*/
	void setSlider(MyLink *body1, MyLink *body2);
	/**
	*@brief �Œ�W���C���g�쐬
	* @param body1 �{�f�B1
	* @param body2 �{�f�B2
	*/
	void setFixed(MyLink *body1, MyLink *body2);
	/**
	*@brief �{�[���W���C���g�쐬
	* @param body1 �{�f�B1
	* @param body2 �{�f�B2
	*/
	void setBall(MyLink *body1, MyLink *body2);
	/**
	*@brief �S�{�f�B�A�ڑ�����S�W���C���g����
	*/
	void makeRobot();
	/**
	*@brief �q���W�W���C���g����
	* @param body �{�f�B�I�u�W�F�N�g
	* @param theta �q���W�W���C���g�̈ʒu
	*/
	void controlHinge(MyLink *body, dReal theta);
	/**
	*@brief �q���W�W���C���g����
	* @param body �{�f�B�I�u�W�F�N�g
	* @param theta �q���W�W���C���g�̈ʒu
	* @param vel �q���W�W���C���g�̑��x
	*/
	void controlHinge(MyLink *body, dReal theta, dReal vel);
	/**
	*@brief �X���C�_�[�W���C���g����
	* @param body �{�f�B�I�u�W�F�N�g
	* @param length �X���C�_�[�W���C���g�̈ʒu
	*/
	void controlSlider(MyLink *body, dReal length);
	/**
	*@brief �S�W���C���g����
	*/
	void control();
	/**
	*@brief �X�V
	*/
	void update();
	/**
	*@brief �S�{�f�B�A�ڑ�����S�W���C���g����
	*/
	void destroyRobot();

	/**
	*@brief �ڐG�R�[���o�b�N
	* @param o1 �W�I���g��1
	* @param o2 �W�I���g��2
	*/
	void m_nearCallback(dGeomID o1, dGeomID o2);
	/**
	*@brief ���ݕ��ݒ�
	* @param s �T���v�����O����
	*/
	void setSamplingTime(double s);


	/**
	*@brief �n�ʐ���
	*/
	void makePlane(double lx, double ly, double lz);

	/**
	*@brief ��Q������
	*/
	void makeBlock(double x, double y, double z, double lx, double ly, double lz, double r);

	/**
	*@brief �t�@�C�������Q���͈ʒu�ǂݍ���
	*@return �ǂݍ��ݐ���(true)�A���s(false)
	*/
	bool loadBlocksData(std::string fname);


};



#endif
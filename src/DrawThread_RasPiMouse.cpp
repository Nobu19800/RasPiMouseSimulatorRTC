#include <fstream>
#include "DrawThread_RasPiMouse.h"
#include "searchFile.h"
#include <coil/stringutil.h>

//std::ofstream ofs( "test.txt" );

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#define dsDrawTriangle    dsDrawTriangleD
#endif

DrawThread_RasPiMouse *obj = NULL;


/**
*@brief シミュレーションの描画をするスレッドのコンストラクタ
*/
DrawThread_RasPiMouse::DrawThread_RasPiMouse(RasPiMouseSimulatorObj *so, double dt)
{
	m_so = so;
	

	setDrawStuff();

	fps = 1.0 / dt;

	obj = this;
	RCP_flag = false;
}

/**
*@brief シミュレーションループ
* @param pause 未使用
*/
void simLoop(int pause)
{
#ifdef WIN32
	Sleep(1000.0 / obj->fps);
#else
	struct timespec ts;
	ts.tv_sec = 1;
	ts.tv_nsec = 0;
	nanosleep(&ts, NULL);
#endif
	if(obj)
	{
		obj->drawRobot();
		obj->resetCameraPosition();
	}
}



/**
*@brief シミュレーション初期化
*/
void start()
{
  //float xyz[3] = {  .0f,  1.0f, 3.0f};  
  float xyz[3] = {  -0.5f,  -0.0f, 0.25f}; 
  float hpr[3] = {-180.0f, -10.0f, 0.0f};  
  //float hpr[3] = {0.0f, -90.0f, 90.0f}; 
  //float xyz[3] = {  5.0f,  -5.0f, 3.0f};
  //float hpr[3] = {180.0f, -10.0f, 0.0f};
  dsSetViewpoint(xyz,hpr);                
  dsSetSphereQuality(3);
  dsSetCapsuleQuality(6);


  
  
}

/**
*@brief スレッド実行関数
* @return
*/
int DrawThread_RasPiMouse::svc()
{
	int argc = 0;
	char *argv[] = {""};

	dsSimulationLoop(argc,argv,800,480,&fn);

	return 0;
}


/**
*@brief DrawStuff初期化
*/
void DrawThread_RasPiMouse::setDrawStuff()
{
	fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  
  static std::string drawstuff = search_file("drawstuff/textures", "PATH", ";");
  
  coil::replaceString(drawstuff, "\\", "/");
  
  
  if (drawstuff == "")
  {
	  fn.path_to_textures = "drawstuff/textures";
  }
  else
  {
	  fn.path_to_textures = drawstuff.c_str();
	  
  }

}

/**
*@brief 直方体描画
* @param body ボディオブジェクト
*/
void DrawThread_RasPiMouse::drawBox(MyLink *body)
{
	const double sides[3] = {body->lx, body->ly, body->lz};
	dsSetColorAlpha(body->red,body->green,body->blue,1.0);
	dsDrawBoxD(dBodyGetPosition(body->body),
						dBodyGetRotation(body->body),sides);
}

/**
*@brief 円柱描画
* @param body ボディオブジェクト
*/
void DrawThread_RasPiMouse::drawCylinder(MyLink *body)
{
	dsSetColorAlpha(body->red,body->green,body->blue,1.0);
	dsDrawCylinderD(dBodyGetPosition(body->body),
						dBodyGetRotation(body->body),body->lz,body->lx);
}


/**
*@brief 球描画
* @param body ボディオブジェクト
*/
void DrawThread_RasPiMouse::drawSphere(MyLink *body)
{
	dsSetColorAlpha(body->red, body->green, body->blue, 1.0);
	dsDrawSphereD(dBodyGetPosition(body->body),
		dBodyGetRotation(body->body), body->lz);
}

/**
*@brief 全ボディ描画
*/
void DrawThread_RasPiMouse::drawRobot()
{
	if(m_so->pause)
	{
		m_so->mu.lock();

		drawBox(&m_so->centorUnit);

	
		drawCylinder(&m_so->wheelLeft);
		drawCylinder(&m_so->wheelRight);


		drawBox(&m_so->topPlate[0]);
		drawCylinder(&m_so->topPlate[1]);
		drawCylinder(&m_so->topPlate[2]);

		drawCylinder(&m_so->middlePlate);

		drawCylinder(&m_so->bottomPlate[0]);
		drawCylinder(&m_so->bottomPlate[1]);

		drawBox(&m_so->RaspPi);


		drawCylinder(&m_so->supportPlate[0]);
		drawCylinder(&m_so->supportPlate[1]);


		dsSetColorAlpha(1.0, 0, 0, 1.0);
		
		for (int i = 0; i < 4; i++)
		{
			dVector3 stpos, dir, epos;
			dGeomRayGet(m_so->IRSensor_ray[i], stpos, dir);
			
			for (int j = 0; j < 4; j++)
			{
				epos[j] = stpos[j] + (dir[j] * m_so->RasPiMouse.ir_sensor[i].data);
			}
			dsDrawLine(stpos, epos);
		}
		/*
		dVector3 ray_pos0, ray_pos1;

		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR2_X, DEFAULT_IRSENSOR2_Y, DEFAULT_IRSENSOR2_Z, ray_pos0);
		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR2_X + m_so->RasPiMouse.ir_sensor[3].data * cos(DEFAULT_IRSENSOR2_RADIUS), DEFAULT_IRSENSOR2_Y + m_so->RasPiMouse.ir_sensor[3].data*sin(DEFAULT_IRSENSOR2_RADIUS), DEFAULT_IRSENSOR2_Z, ray_pos1);
		dsDrawLine(ray_pos0, ray_pos1);
		




		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR1_X, DEFAULT_IRSENSOR1_Y, DEFAULT_IRSENSOR1_Z, ray_pos0);
		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR1_X + m_so->RasPiMouse.ir_sensor[2].data*cos(DEFAULT_IRSENSOR1_RADIUS), DEFAULT_IRSENSOR1_Y + m_so->RasPiMouse.ir_sensor[2].data*sin(DEFAULT_IRSENSOR1_RADIUS), DEFAULT_IRSENSOR1_Z, ray_pos1);
		dsDrawLine(ray_pos0, ray_pos1);


		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR1_X, -DEFAULT_IRSENSOR1_Y, DEFAULT_IRSENSOR1_Z, ray_pos0);
		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR1_X + m_so->RasPiMouse.ir_sensor[1].data*cos(-DEFAULT_IRSENSOR1_RADIUS), -DEFAULT_IRSENSOR1_Y + m_so->RasPiMouse.ir_sensor[1].data*sin(-DEFAULT_IRSENSOR1_RADIUS), DEFAULT_IRSENSOR1_Z, ray_pos1);
		dsDrawLine(ray_pos0, ray_pos1);




		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR2_X, -DEFAULT_IRSENSOR2_Y, DEFAULT_IRSENSOR2_Z, ray_pos0);
		dBodyGetRelPointPos(m_so->middlePlate.body, DEFAULT_IRSENSOR2_X + m_so->RasPiMouse.ir_sensor[0].data*cos(-DEFAULT_IRSENSOR2_RADIUS), -DEFAULT_IRSENSOR2_Y + m_so->RasPiMouse.ir_sensor[0].data*sin(-DEFAULT_IRSENSOR2_RADIUS), DEFAULT_IRSENSOR2_Z, ray_pos1);
		dsDrawLine(ray_pos0, ray_pos1);
		*/
		//RasPiMouse.ir_sensor[0].data
		if (m_so->plane_exist)
		{
			drawBox(&m_so->plane);
		}

		for (std::vector<MyLink>::iterator itr = m_so->blocks.begin(); itr != m_so->blocks.end(); ++itr) {
			drawBox(&(*itr));
		}

		m_so->mu.unlock();
	}
}



/**
*@brief カメラ位置再設定
*/
void DrawThread_RasPiMouse::resetCameraPosition()
{
	m_so->mu.lock();
	if (RCP_flag)
	{
		const dReal *pos = dBodyGetPosition(m_so->centorUnit.body);
		float xyz[3] = { -0.3f + pos[0], 0.3f + pos[1], 0.2f + pos[2] };
		float hpr[3] = { -40.0f, -30.0f, 0.0f };
		dsSetViewpoint(xyz, hpr);
		dsSetSphereQuality(3);
		dsSetCapsuleQuality(6);


		RCP_flag = false;

	}
	m_so->mu.unlock();
}


/**
*@brief カメラ位置再設定フラグを立てる
*/
void DrawThread_RasPiMouse::setRCPFlag()
{
	m_so->mu.lock();
	RCP_flag = true;
	m_so->mu.unlock();
}
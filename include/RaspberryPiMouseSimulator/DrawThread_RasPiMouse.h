/*!
* @file  DrawThread.h
* @brief シミュレーション描画関連のクラス
*
*/

#ifndef DRAWTHREAD_H
#define DRAWTHREAD_H


#include <coil/Task.h>

#include <stdio.h>
#include <stdlib.h>
#include <drawstuff/drawstuff.h>
#include "RasPiMouseSimulatorObj.h"



/**
* @class DrawThread
*@brief シミュレーションの描画をするスレッド
*/
class DrawThread_RasPiMouse : public virtual coil::Task
{
public:
		/**
		*@brief コンストラクタ
		*/
		DrawThread_RasPiMouse(RasPiMouseSimulatorObj *so, double dt);
		
		/**
		*@brief スレッド実行関数
		* @return 
		*/
		virtual int svc();
		/**
		*@brief DrawStuff初期化
		*/
		void setDrawStuff();
		/**
		*@brief 直方体描画
		* @param body ボディオブジェクト
		*/
		void drawBox(MyLink *body);
		/**
		*@brief 円柱描画
		* @param body ボディオブジェクト
		*/
		void drawCylinder(MyLink *body);
		/**
		*@brief 球描画
		* @param body ボディオブジェクト
		*/
		void drawSphere(MyLink *body);
		/**
		*@brief 全ボディ描画
		*/
		void drawRobot();
		/**
		*@brief カメラ位置再設定
		*/
		void resetCameraPosition();
		/**
		*@brief カメラ位置再設定フラグを立てる
		*/
		void setRCPFlag();
		

		double fps;
		bool RCP_flag;

private:
	RasPiMouseSimulatorObj *m_so;
	dsFunctions   fn;
	

};

#endif
﻿#ifndef ACTIVECONTROL_H
#define ACTIVECONTROL_H
#include "control_card.h"
#include"boundarydetection.h"
#include<vector>
#include "matrix.h"

class ActiveControl {
public:
    ActiveControl();
    ~ActiveControl();
	void LoadParamFromFile();
    void StartMove();
    void StopMove();
	// 采集一次六维力的数据，计算出电机速度，然后指示电机以这个速度运动.这是一轮循环
	void Step();
	bool IsFire();
	// 获取机器人末端位置
	void CalculatePlaneXY(short Axis_X, short Axis_Y, double XY[2]);
	// 擦窗户游戏中获取抹布位置
	void CalculateRagXY(double XY[2]);
	void SetDamping(float FC=0.1);
	// 设置关节运动范围
	void SetSAAMax(double saa);
	void SetSFEMax(double sfe);

public:
	//输出肩肘部压力转化后的力矩数据输出到txt文件
	void MomentExport();
	//把力矩传感器测得的数据输出到txt文件
	void TorqueExport();
	boundaryDetection detect;
	

public:
	bool is_exit_thread_;
	bool is_moving_;
	double six_dimension_offset_[6];
	double shoulder_offset[4];
	double elbow_offset[4];
	double two_arm_offset[8];
	double cycle_time_in_second_;

private:
	void MoveInNewThread();
	void ExitMoveThread();
	void ActMove();
	//将原始值进行坐标变换
	void Raw2Trans(double RAWData[6], double DistData[6]);
	//将转换后的值进行滤波-二阶巴特沃斯低通滤波器
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	void Trans2Filter2(double TransData[4], double FiltedData[4]);
	void FiltedVolt2Vel(double FiltedData[6]);
	void FiltedVolt2Vel2(double ForceVector[4]);
	//将传感器的数据处理成两个二维矢量，由于矢量只在两个方向上有作用，故需输出4个数据。这里要先知道传感器的安装位置
	void SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4],double ForceVector[4]);


private:
	HANDLE move_thread_handle_;
	Matrix3d rotate_matrix_;
	//手柄坐标系下手柄坐标系原点到六维力坐标系原点的向量
	Vector3d force_position_;

	double shoulder_angle_max_;
	double elbow_angle_max_;

};

#endif // ACTIVECONTROL_H

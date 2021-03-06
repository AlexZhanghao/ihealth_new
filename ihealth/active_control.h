﻿#ifndef ACTIVECONTROL_H
#define ACTIVECONTROL_H
#include "control_card.h"
#include"boundarydetection.h"
#include<vector>
//#include"passive_control.h"
#include "matrix.h"

//class PassiveControl;
struct ActiveGravityCompensation {
	std::vector<double> mean_positions[2];
	std::vector<double> mean_sixdemsional_force[6];
};

class ActiveControl {
public:
    ActiveControl();
    ~ActiveControl();
	void LoadParamFromFile();
    void StartMove(int id);
    void StopMove();
	// 采集一次六维力的数据，计算出电机速度，然后指示电机以这个速度运动.这是一轮循环
	void Step();
	//六维力控制肩肘两个关节的线程
	void SixDimForceStep();
	//使用力矩传感器的循环
	void TorqueStep();
	//使用压力传感器的循环
	void PressureStep();
	bool IsFire();
	// 获取机器人末端位置
	void CalculatePlaneXY(short Axis_X, short Axis_Y, double XY[2]);
	// 擦窗户游戏中获取抹布位置
	void CalculateRagXY(double XY[2]);
	void SetDamping(float FC=0.1);
	// 设置关节运动范围
	void SetSAAMax(double saa);
	void SetSFEMax(double sfe);
	void SetArmSensitivity(double arm_senitivity);
	void SetShoulderSensitivity(double shoulder_senitivity);
	//将力矩由主动关节换算到所有关节
	void ActiveTorqueToAllTorque(double torque[2], double alltorque[5]);
	//获取六维力传感器的偏置
	void GetSixdemPolarization();

public:
	//输出肩肘部压力转化后的力矩数据输出到txt文件
	void MomentExport();
	//把力矩传感器测得的数据输出到txt文件
	void TorqueExport();
	boundaryDetection detect;
	ActiveGravityCompensation mean_force_and_position_;

public:
	bool is_exit_thread_;
	bool is_moving_;
	//压力传感器是否使能，注意这里并不只是单纯的将它关闭，而是切换到了纯六维力模式
	bool m_pressure_sensor_enable;
	double elbow_offset[2];
	double six_dimension_offset_[6]{0.0};
	double mass_psv_;
	double torque_offset[2];
	double cycle_time_in_second_;
	int is_left;
	std::string m_score;
	//PassiveControl *passive_control_ = nullptr;
private:
	void MoveInNewThread(int id);
	void ExitMoveThread();
	void ActMove();
	//将原始值进行坐标变换
	void Raw2Trans(double RAWData[6], double DistData[6]);
	//将转换后的值进行滤波-二阶巴特沃斯低通滤波器
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	void Trans2FilterForPressure(double TransData[2], double FiltedData[2]);
	//void FiltedVolt2Vel(double FiltedData[6]);
	void MomentCalculation(double ForceVector, double& vel);
	//只用六维力情况下的力矩计算
	void SixDimForceMomentCalculation(double ForceVector[6], double vel[2]);
	//将传感器的数据处理成两个二维矢量，由于矢量只在两个方向上有作用，故需输出4个数据。这里要先知道传感器的安装位置
	void SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4],double ForceVector[4]);


private:
	HANDLE move_thread_handle_;
	Matrix3d rotate_matrix_;
	//手柄坐标系下手柄坐标系原点到六维力坐标系原点的向量
	Vector3d force_position_;

	double shoulder_angle_max_;
	double elbow_angle_max_;
	static double six_dimforce[6];
	//static double six_dimension_offset_[6];
	double joint_angle[2];
	static double elbow_Sensitivity_;
	static double shoulder_Sensitivity_;
	
};

#endif // ACTIVECONTROL_H

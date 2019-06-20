#include "active_control.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <process.h>
#include <windows.h>

#include "Matrix.h"
#include "Log.h"
#include "data_acquisition.h"
#include "pupiltracker/utils.h"

using namespace Eigen;
using namespace std;

double Force_Fc = 0.3;
double Force_a =0.3;
double Force_b = 1;

vector<double> torque_data[2];
vector<double> moment_data[2];

double anglearm = 0;//手臂关节角
double angleshoul = 0;//肩部关节角
double Ud_Arm = 0;//力控模式算出手臂的命令速度
double Ud_Shoul = 0;//力控模式算出肩部的命令速度
const char *FCH = "Dev2/ai6";//握力采集通道

static const int kPlaneMaxX = 734;
static const int kPlaneMaxY = 601;

static const int kRagMaxX = 710;
static const int kRagMaxY = 596;

extern Vector3d AxisDirection[5] {
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
};
extern Vector3d AxisPosition[5] {
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
	Vector3d(0, 0, 0),
};

ActiveControl::ActiveControl() {
	move_thread_handle_ = 0;
	is_exit_thread_ = false;
	is_moving_ = false;
	LoadParamFromFile();
}

void ActiveControl::LoadParamFromFile() {
	pupiltracker::ConfigFile cfg;
	cfg.read("../../resource/params/active_control.cfg");
	string s;
	stringstream ss;
	string item;

	// rotate_matrix_
	s = cfg.get<string>("rotate_matrix");
	ss.clear();
	ss.str(s);
	vector<int> r;
	while (getline(ss, item, ',')) {
		r.push_back(stoi(item));
	}
	rotate_matrix_ <<
		r[0], r[1], r[2],
		r[3], r[4], r[5],
		r[6], r[7], r[8];

	// force_position_
	s = cfg.get<string>("force_position");
	ss.clear();
	ss.str(s);
	vector<double> f;
	while (getline(ss, item, ',')) {
		f.push_back(stod(item));
	}
	force_position_ << f[0], f[1], f[2];

	// param depend on left or right arm
	int is_left = cfg.get<int>("is_left");
	if (is_left) {
		AxisDirection[0] = Vector3d(1.0, 0, 0);
		AxisDirection[1] = Vector3d(0, 0, 1.0);
		AxisDirection[2] = Vector3d(0, -1.0, 0);
		AxisDirection[3] = Vector3d(0, -1.0, 0);
		AxisDirection[4] = Vector3d(-1.0, 0, 0);

		AxisPosition[0] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
		AxisPosition[1] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
		AxisPosition[2] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
		AxisPosition[3] = Vector3d(-LowerArmLength, 0, 0);
		AxisPosition[4] = Vector3d(-LowerArmLength, 0, 0);
	} else {
		AxisDirection[0] = Vector3d(-1, 0, 0);
		AxisDirection[1] = Vector3d(0, 0, -1);
		AxisDirection[2] = Vector3d(0, -1, 0);
		AxisDirection[3] = Vector3d(0, -1, 0);
		AxisDirection[4] = Vector3d(1, 0, 0);

		AxisPosition[0] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
		AxisPosition[1] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
		AxisPosition[2] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
		AxisPosition[3] = Vector3d(-LowerArmLength, 0, 0);
		AxisPosition[4] = Vector3d(-LowerArmLength, 0, 0);
	}

	// cycle_time_in_second
	cycle_time_in_second_ = cfg.get<double>("cycle_time_in_second");
	
	// shoulder and elbow range
	shoulder_angle_max_ = cfg.get<double>("shoulder_angle_max");
	elbow_angle_max_ = cfg.get<double>("elbow_angle_max");

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//cout << rotate_matrix_ << endl;
	//cout << force_position_ << endl;
	//cout << is_left << endl;
	//cout << AxisPosition << endl;
	//cout << AxisDirection << endl;

}

ActiveControl:: ~ActiveControl() {
	//DataAcquisition::GetInstance().StopTask();
}

unsigned int __stdcall ActiveMoveThread(PVOID pParam) {
	ActiveControl *active = (ActiveControl*)pParam;
	UINT start, end;
	start = GetTickCount();
	// 求六维力传感器的偏置
	//double sum[6]{ 0.0 };
	//double buf[6]{ 0.0 };
	//for (int i = 0;i < 10;++i) {
	//	DataAcquisition::GetInstance().AcquisiteSixDemensionData(buf);
	//	for (int j = 0;j < 6;++j) {
	//		sum[j] += buf[j];
	//	}
	//}
	//for (int i = 0;i < 6;++i) {
	//	active->six_dimension_offset_[i] = sum[i] / 10;
	//}

	////求压力传感器的偏置
	//double shoulder_sum[4]{ 0.0 };
	//double elbow_sum[4]{ 0.0 };
	//double shoulder_buf[4]{ 0.0 };
	//double elbow_buf[4]{ 0.0 };
	//for (int i = 0; i < 10; ++i) {
	//	DataAcquisition::GetInstance().AcquisiteShoulderTensionData(shoulder_buf);
	//	DataAcquisition::GetInstance().AcquisiteElbowTensionData(elbow_buf);
	//	for (int j = 1; j < 4; ++j) {
	//		shoulder_sum[i] += shoulder_buf[i];
	//		elbow_sum[i] += elbow_buf[i];
	//	}
	//}
	//for (int i = 0; i < 4; ++i) {
	//	active->shoulder_offset[i] = shoulder_sum[i] / 10;
	//	active->elbow_offset[i] = elbow_sum[i] / 10;
	//}

	//将肩肘部合在一起
	double two_arm_sum[8]{ 0.0 };
	double two_arm_buf[8]{ 0.0 };
	for (int i = 0; i < 10; ++i) {
		DataAcquisition::GetInstance().AcquisiteTensionData(two_arm_buf);
		for (int j = 0; j < 8; ++j) {
			two_arm_sum[i] += two_arm_buf[i];
		}
	}

	for (int i = 0; i < 8; ++i) {
		active->two_arm_offset[i] = two_arm_sum[i] / 10;
	}

	DataAcquisition::GetInstance().StopTask();
	DataAcquisition::GetInstance().StartTask();

	while (true) {
		if (active->is_exit_thread_) {
			break;
		}

		// 每隔一定时间进行一次循环，这个循环时间应该是可调的。
		while (true) {
			end = GetTickCount();
			if (end - start >= active->cycle_time_in_second_ * 1000) {
				start = end;
				break;
			} else {
				SwitchToThread();
			}
		}

		active->Step();


	}

	active->MomentExport();
	active->TorqueExport();
	//std::cout << "ActiveMoveThread Thread ended." << std::endl;
	return 0;
}
void ActiveControl::MoveInNewThread() {
	is_exit_thread_ = false;
	move_thread_handle_ = (HANDLE)_beginthreadex(NULL, 0, ActiveMoveThread, this, 0, NULL);
}
void ActiveControl::ExitMoveThread() {
	is_exit_thread_ = true;

	if (move_thread_handle_ != 0) {
		::WaitForSingleObject(move_thread_handle_, INFINITE);
		move_thread_handle_ = 0;
	}
}

void ActiveControl::StartMove() {
	ControlCard::GetInstance().SetMotor(ControlCard::MotorOn);
	ControlCard::GetInstance().SetClutch(ControlCard::ClutchOn);
	is_moving_ = true;
	MoveInNewThread();
}

void ActiveControl::StopMove() {
	//这里不放开离合的原因是为了防止中间位置松开离合导致手臂迅速下坠
	ControlCard::GetInstance().SetMotor(ControlCard::MotorOff);
	is_moving_ = false;
	ExitMoveThread();
}

void ActiveControl::Step() {
	double readings[6] = { 0 };
	double distData[6] = { 0 };
	double filtedData[6] = { 0 };
	double bias[6] = { 0 };
	double sub_bias[6] = { 0 };

	//压力传感器相关
	double shoulder_data[4] = { 0 };
	double elbow_data[4] = { 0 };
	double shoulder_suboffset[4] = { 0 };
	double elbow_suboffset[4] = { 0 };
	double shoulder_smooth[4] = { 0 };
	double elbow_smooth[4] = { 0 };
	double two_arm_data[8] = { 0 };
	double two_arm_suboffset[8] = { 0 };
	double force_vector[4] = { 0 };

	//DataAcquisition::GetInstance().AcquisiteSixDemensionData(readings);
	//DataAcquisition::GetInstance().AcquisiteShoulderTensionData(shoulder_data);
	//DataAcquisition::GetInstance().AcquisiteElbowTensionData(elbow_data);
	DataAcquisition::GetInstance().AcquisiteTensionData(two_arm_data);

	torque_data[0].push_back(detect.shoulder_torque);
	torque_data[1].push_back(detect.elbow_torque);

	//减偏置
	//for (int i = 0; i < 4; ++i) {
	//	shoulder_suboffset[i] = shoulder_data[i] - shoulder_offset[i];
	//	elbow_suboffset[i] = elbow_data[i] - elbow_offset[i];
	//}
	for (int i = 0; i < 8; ++i) {
		two_arm_suboffset[i] = two_arm_data[i] - two_arm_offset[i];
	}

	// 求减去偏置之后的六维力，这里对z轴的力和力矩做了一个反向
	//for (int i = 0; i < 6; ++i) {
	//	sub_bias[i] = readings[i] - six_dimension_offset_[i];
	//}
	//sub_bias[2] = -sub_bias[2];
	//sub_bias[5] = -sub_bias[5];

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("fx:%lf    fy:%lf    fz:%lf \n Mx:%lf    My:%lf    Mz:%lf \n", sub_bias[0], sub_bias[1], sub_bias[2], sub_bias[3], sub_bias[4], sub_bias[5]);

	//Raw2Trans(sub_bias, distData);
	//Trans2Filter(distData, filtedData);
	//FiltedVolt2Vel(filtedData);
	
	//因为滤波会用到之前的数据，所以在这里还是得把数据分开
	for (int i = 0; i < 4; ++i) {
		shoulder_suboffset[i] = two_arm_suboffset[i];
	}
	for (int j = 0; j < 4; ++j) {
		elbow_suboffset[j] = two_arm_suboffset[j + 4];
	}

	//先将传感器获取的数据滤波
	Trans2Filter2(shoulder_suboffset,shoulder_smooth);
	Trans2Filter2(elbow_suboffset, elbow_smooth);
	//Trans2Filter2(two_arm_suboffset, two_arm_smooth);
	
	//将传感器数据转成力矢量
	SensorDataToForceVector(shoulder_smooth, elbow_smooth, force_vector);

	FiltedVolt2Vel2(force_vector);

	//if (is_moving_) {
	//	 ActMove();
	//}

	//qDebug()<<"readings is "<<filtedData[0]<<" "<<filtedData[1]<<" "<<filtedData[2]<<" "<<filtedData[3]<<" "<<filtedData[4]<<" "<<filtedData[5];
}

void ActiveControl::Raw2Trans(double RAWData[6], double DistData[6]) {
	//这一段就是为了把力从六维力传感器上传到手柄上，这里的A就是总的一个转换矩阵。
	//具体的旋转矩阵我们要根据六维力的安装确定坐标系方向之后然后再确定。
	MatrixXd A(6, 6);
	A.setZero();
	VectorXd Value_Origi(6);
	VectorXd Value_Convers(6);
	Matrix3d ForcePositionHat;
	//这里就是这个p，我们可以想象，fx不会产生x方向的力矩，fy产生的看z坐标，fz产生的y坐标。
	//这里做的就是把力矩弄过去。这个相对坐标都是六维力坐标在手柄坐标系下的位置。
	//比如fx在y方向上有一个力臂，就会产生一个z方向上的力矩。这个力矩的方向和相对位置无关。
	//所以这个地方我们不用改这个ForcePositionHat，只用改force_position_这个相对位置就可以了
	ForcePositionHat <<
		0, -force_position_[2], force_position_[1],
		force_position_[2], 0, -force_position_[0],
		-force_position_[1], force_position_[0], 0;
	A.block(0, 0, 3, 3) = rotate_matrix_;
	A.block(0, 3, 3, 1) = ForcePositionHat * rotate_matrix_;
	A.block(3, 3, 3, 3) = rotate_matrix_;
	


	//之前是fxfyfzMxMyMz,现在变成MxMyMzfxfyfz
	for (int i = 0; i < 6; i++) {
		if (i<3) {
			Value_Origi(i) = RAWData[i + 3];
		}
		else {
			Value_Origi(i) = RAWData[i - 3];
		}
	}

	//这里计算后就是
	Value_Convers = A * Value_Origi;
	for (int m = 0; m<6; m++) {
		DistData[m] = Value_Convers(m);
	}
}

void ActiveControl::Trans2Filter(double TransData[6], double FiltedData[6]) {
	double Wc = 5;
	double Ts = 0.1;
	static int i = 0;
	static double Last_Buffer[6] = { 0 };
	static double Last2_Buffer[6] = { 0 };
	static double Force_Buffer[6] = { 0 };
	static double Last_FT[6] = { 0 };
	static double Last2_FT[6] = { 0 }; 
	for (int m = 0; m < 6; m++)
	{
		if (i == 0)
		{
			Last2_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else if (i == 1)
		{
			Last_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else
		{
			//二阶巴特沃斯低通滤波器
			Force_Buffer[m] = TransData[m];
			FiltedData[m] = (1 / (Wc*Wc + 2 * 1.414*Wc / Ts + 4 / (Ts*Ts)))*((Wc*Wc)*Force_Buffer[m]
				+ (2 * Wc*Wc)*Last_Buffer[m]
				+ (Wc*Wc)*Last2_Buffer[m]
				- (2 * Wc*Wc - 8 / (Ts*Ts))*Last_FT[m]
				- (Wc*Wc - 2 * 1.414*Wc / Ts + 4 / (Ts*Ts))*Last2_FT[m]);

			Last2_FT[m] = Last_FT[m];
			Last_FT[m] = FiltedData[m];
			Last2_Buffer[m] = Last_Buffer[m];
			Last_Buffer[m] = Force_Buffer[m];
		}
	}
	//printf("fx:%lf    fy:%lf    fz:%lf \n Mx:%lf    My:%lf    Mz:%lf \n", FiltedData[3], FiltedData[4], FiltedData[5], FiltedData[0], FiltedData[1], FiltedData[2]);
}

void ActiveControl::Trans2Filter2(double TransData[4], double FiltedData[4]) {
	double Wc = 5;
	double Ts = 0.1;
	static int i = 0;
	static double Last_Buffer[4] = { 0 };
	static double Last2_Buffer[4] = { 0 };
	static double Force_Buffer[4] = { 0 };
	static double Last_FT[4] = { 0 };
	static double Last2_FT[4] = { 0 };
	for (int m = 0; m < 4; m++)
	{
		if (i == 0)
		{
			Last2_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else if (i == 1)
		{
			Last_Buffer[m] = TransData[m];
			FiltedData[m] = 0;
			i++;
		}
		else
		{
			//二阶巴特沃斯低通滤波器
			Force_Buffer[m] = TransData[m];
			FiltedData[m] = (1 / (Wc*Wc + 2 * 1.414*Wc / Ts + 4 / (Ts*Ts)))*((Wc*Wc)*Force_Buffer[m]
				+ (2 * Wc*Wc)*Last_Buffer[m]
				+ (Wc*Wc)*Last2_Buffer[m]
				- (2 * Wc*Wc - 8 / (Ts*Ts))*Last_FT[m]
				- (Wc*Wc - 2 * 1.414*Wc / Ts + 4 / (Ts*Ts))*Last2_FT[m]);

			Last2_FT[m] = Last_FT[m];
			Last_FT[m] = FiltedData[m];
			Last2_Buffer[m] = Last_Buffer[m];
			Last_Buffer[m] = Force_Buffer[m];
		}
	}
}

void ActiveControl::SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]) {
	double shoulderdataX = shouldersensordata[0] - shouldersensordata[1];
	double shoulderdataY = shouldersensordata[2] - shouldersensordata[3];
	double elbowdataX = elbowsensordata[0] - elbowsensordata[1];
	double elbowdataY = elbowsensordata[2] - elbowsensordata[3];
	
	//合成的力矢量
	Vector2d shoulderforce;
	Vector2d elbowforce;
	shoulderforce << shoulderdataX, shoulderdataY;
	elbowforce << elbowdataX, elbowdataY;

	//将力分别旋转到坐标系3、5上面
	Matrix2d shoulderrotationmatrix;
	Matrix2d elbowrotationmatrix;
	shoulderrotationmatrix << cos(29.49* M_PI / 180), cos(60.51* M_PI / 180),
		cos(119.49* M_PI / 180), cos(29.49* M_PI / 180);
	elbowrotationmatrix << cos(59.87* M_PI / 180), cos(30.13* M_PI / 180),
		cos(30.13* M_PI / 180), cos(120.13* M_PI / 180);

	shoulderforce = shoulderrotationmatrix * shoulderforce;
	elbowforce = elbowrotationmatrix * elbowforce;

	ForceVector[0] = shoulderforce(0);
	ForceVector[1] = shoulderforce(1);
	ForceVector[2] = elbowforce(0);
	ForceVector[3] = elbowforce(1);
}

void ActiveControl::FiltedVolt2Vel(double FiltedData[6]) {
	MatrixXd Vel(2, 1);
	MatrixXd Pos(2, 1);
	MatrixXd A(6, 6);
	VectorXd Six_Sensor_Convert(6);
	double angle[2];
	ControlCard::GetInstance().GetEncoderData(angle);
	Pos(0, 0) = angle[0];
	Pos(1, 0) = angle[1];

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("elbow angle: %lf\n", angle[1]);
	//printf("shoulder angle: %lf\n", angle[0]);
	//printf("fx:%lf    fy:%lf    fz:%lf \n Mx:%lf    My:%lf    Mz:%lf \n", FiltedData[3], FiltedData[4], FiltedData[5], FiltedData[0], FiltedData[1], FiltedData[2]);
	
	for (int i = 0; i < 6; i++)
	{
		Six_Sensor_Convert(i) = FiltedData[i];
	}
	damping_control(Six_Sensor_Convert, Pos, Vel, Force_Fc, Force_a, Force_b);
	Ud_Shoul = Vel(0, 0);
	Ud_Arm = Vel(1, 0);

	if ((Ud_Arm > -0.5) && (Ud_Arm < 0.5))
	{
		Ud_Arm = 0;
	}
	if ((Ud_Shoul > -0.5) && (Ud_Shoul < 0.5))
	{
		Ud_Shoul = 0;
	}
	if (Ud_Arm > 5)
	{
		Ud_Arm = 5;
	}
	else if (Ud_Arm < -5)
	{
		Ud_Arm = -5;
	}
	if (Ud_Shoul > 5)
	{
		Ud_Shoul = 5;
	}
	else if (Ud_Shoul < -5)
	{
		Ud_Shoul = -5;
	}

	//printf("肩部速度: %lf\n", Ud_Shoul);
	//printf("肘部速度: %lf\n", Ud_Arm);
}

void ActiveControl::FiltedVolt2Vel2(double ForceVector[4]) {
	MatrixXd vel(2, 1);
	MatrixXd pos(2, 1);
	
	VectorXd shoulder_force_vector(3);
	VectorXd elbow_force_vector(3);
	VectorXd six_dimensional_force_simulation(6);

	double angle[2];
	double moment[5];
	
	ControlCard::GetInstance().GetEncoderData(angle);

	pos(0, 0) = angle[0];
	pos(1, 0) = angle[1];
	shoulder_force_vector(0) = ForceVector[0];
	shoulder_force_vector(1) = ForceVector[1];
	shoulder_force_vector(2) = 0;
	elbow_force_vector(0) = ForceVector[2];
	elbow_force_vector(1) = ForceVector[3];
	elbow_force_vector(2) = 0;

	MomentBalance(shoulder_force_vector, elbow_force_vector, angle, moment);

	moment_data[0].push_back(moment[0]);
	moment_data[1].push_back(moment[2]);
}

void ActiveControl::ActMove() {
	ControlCard::GetInstance().ProtectedVelocityMove(ControlCard::ShoulderAxisId, Ud_Shoul);
	ControlCard::GetInstance().ProtectedVelocityMove(ControlCard::ElbowAxisId, Ud_Arm);
}

bool ActiveControl::IsFire() {
	bool fire = false;

	double grip;
	//这里就是采集握力的数据
	DataAcquisition::GetInstance().AcquisiteGripData(&grip);
	if (grip > 0.3)
		fire = true;
	return fire;
}

void ActiveControl::CalculatePlaneXY(short rangeX, short rangeY, double XY[2]) {
	//MatrixXd Theta(5, 1);
	//MatrixXd T0h(4, 4);
	//VectorXd Pos(2);

	double angle[2] = { 0 };
	ControlCard::GetInstance().GetEncoderData(angle);

	int x = (angle[0] / shoulder_angle_max_) * kPlaneMaxX;
	int y = (angle[1] / elbow_angle_max_) * kPlaneMaxY;

	if (y < 0) {
		y = 0;
	}
	else if (y > 100) {
		y = 100;
	}

	if (x < 0) {
		x = 0;
	}
	else if (x > kPlaneMaxX) {
		x = kPlaneMaxX;
	}

	// XY[0] = kPlaneMaxX - x;
	XY[0] = x;
	XY[1] = kPlaneMaxY - y;

	/*Pos << angle[0], angle[1];
	fwd_geo_coup(Pos, Theta);
	fwd_geo_kineB(Theta, T0h);
	double x = -T0h(1, 3);
	double y = ShoulderLength + UpperArmLength + LowerArmLength - T0h(0, 3);

	x = std::max<double>(std::min<double>(x, 0.3), 0);
	y = std::max<double>(std::min<double>(y, 0.3), 0);
	XY[0] = (x / 0.3)*rangeX;
	XY[1] =(1 - 0.3 * y / 0.3)*rangeY;*/
}

void ActiveControl::CalculateRagXY(double XY[2]) {
	double angle[2] = { 0 };
	ControlCard::GetInstance().GetEncoderData(angle);

	// 根据比例得到抹布位置
	int x = (angle[0] / shoulder_angle_max_) * kRagMaxX;
	int y = (angle[1] / elbow_angle_max_) * kRagMaxY;

	if (y < 0) {
		y = 0;
	} else if (y > kRagMaxY) {
		y = kRagMaxY;
	}

	if (x < 0) {
		x = 0;
	} else if (x > kRagMaxX) {
		x = kRagMaxX;
	}

	XY[0] = x;
	XY[1] = kRagMaxY - y;
}

void ActiveControl::SetDamping(float FC)
{
	Force_Fc = FC;
}

void ActiveControl::SetSAAMax(double saa) {
	shoulder_angle_max_ = saa;
}

void ActiveControl::SetSFEMax(double sfe) {
	elbow_angle_max_ = sfe;
}

void ActiveControl::MomentExport() {
	ofstream dataFile1;
	dataFile1.open("moment.txt", ofstream::app);
	dataFile1 << "moment_sholuder" << "   " << "moment_elbow" << endl;
	for (int i = 0; i < moment_data[0].size(); ++i) {
		dataFile1 << moment_data[0][i] << "        " << moment_data[1][i] << endl;
	}
	dataFile1.close();
}

void ActiveControl::TorqueExport() {
	ofstream dataFile2;
	dataFile2.open("torque.txt", ofstream::app);
	dataFile2 << "torque_sholuder" << "   " << "torque_elbow" << endl;
	for (int i = 0; i < torque_data[0].size(); ++i) {
		dataFile2 << torque_data[0][i] << "        " << torque_data[1][i] << endl;
	}
	dataFile2.close();
}
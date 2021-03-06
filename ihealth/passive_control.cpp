﻿#include "passive_control.h"
#include <windows.h>
#include <process.h> 
#include<iostream>
#include<fstream>
#include "Matrix.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "data_acquisition.h"
//#include<ctime>
using namespace std;

#define TIMER_SLEEP   0.1
//#define HAVE_STRUCT_TIMESPEC
const double Comfort_Pos[2]={0,0};//开始运动的初始位置，人觉得舒服的位置
bool is_exit_thread_ = false;
int loop_counter_in_thread = 0;

//导出被动运动数据
// std::string pathname="..\\..\\resource\\ExportData\\";
// time_t t = time(0);
// char ch[64];
// //strftime(ch, sizeof(ch), "%Y-%m-%d %H-%M-%S", localtime(&t)); //年-月-日 时-分-秒
// std::string paitent_info="passive_";
// ofstream joint_value(pathname+paitent_info+"joint_"+".txt", ios::app | ios::out);
// ofstream torque_value(pathname+paitent_info+"torque_"+".txt", ios::app | ios::out);
// ofstream sixdim_force_value(pathname+paitent_info+"sixdim_force_"+".txt", ios::app | ios::out);
// ofstream sum_pressure_force_value(pathname+paitent_info+"sum_pressure_force_"+".txt", ios::app | ios::out);
// ofstream pull_force_value(pathname + paitent_info + "pull_force_" + ".txt", ios::app | ios::out);

PassiveControl::PassiveControl() {
    //初始化动作队列
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			hermite_time_interval_[i][j] = 0;
			//更新插值位置范围
			hermite_pos_interval_[i][j] = 0;
			//更新插值速度范围
			hermite_vel_interval_[i][j] = 0;
		}
	}
	is_busy_ = false;
	in_record_status_ = false;
	in_move_status_ = false;
	is_teach = false;
}

void PassiveControl::SetActiveControl(ActiveControl *p) {
	active_control_ = p;
}

PassiveControl::~PassiveControl() {
}

void PassiveControl::StoreCurrentRecord() {
	movement_set_.push_back(record_data_);
}

// 这里示教和被动运动都写在了这个函数里面,通过flag决定运行哪一部分
unsigned int __stdcall RecordOrMoveThread(PVOID pParam) {
	PassiveControl *passive = (PassiveControl*)pParam;
	UINT start, end;

	start = GetTickCount();
	std::string pathname="..\\..\\resource\\ExportData\\";
    time_t t = time(0);
    char ch[64];
    strftime(ch, sizeof(ch), "%Y-%m-%d %H-%M-%S", localtime(&t)); //年-月-日 时-分-秒
    std::string paitent_info="passive_";
	ofstream joint_value(pathname+paitent_info+"joint_"+ch+".txt", ios::app | ios::out);
    ofstream torque_value(pathname+paitent_info+"torque_"+ch+".txt", ios::app | ios::out);
    ofstream sixdim_force_value(pathname+paitent_info+"sixdim_force_"+ch+".txt", ios::app | ios::out);
	ofstream sum_pressure_force_value(pathname+paitent_info+"sum_pressure_force_"+ch+".txt", ios::app | ios::out);
	ofstream pull_force_value(pathname + paitent_info + "pull_force_" + ch +".txt", ios::app | ios::out);
	double angle[2]{0};
    double torque[2]{0};
    double six_dim_force[6]{0};
	joint_value << " shoulder horizontal flexion/extension  " << " shoulder adduction/abduction  " << " shoulder flexion/extension " << " elbow flexion/extension"
		<< " forearm pronation/supination " << endl;
   	torque_value << " shoulder(N.m)  "
                << "  elbow(N.m)  " << endl;
    sixdim_force_value<<" fx(N) "<<" fy(N) "<<" fz(N) "<<" tx(N.m) "<<" ty(N.m) "<<" tz(N.m) "<<endl;
    sum_pressure_force_value<<"F>0表示肘曲"<<"F<0表示肘伸"<<endl;
	pull_force_value << " shoulder_forward " << " shoulder_backward " << " elbow_forward " << " elbow_backward " << endl;

    // 求六维力传感器的偏置
	double sum[6]{ 0.0 };
	double buf[6]{ 0.0 };
	double six_dimension_offset_p[6]{0.0};
	for (int i = 0; i < 10; ++i) {
		DataAcquisition::GetInstance().AcquisiteSixDemensionData(buf);
		for (int j = 0; j < 6; ++j) {
			sum[j] += buf[j];
		}
	}
	for (int i = 0; i < 6; ++i) {
		six_dimension_offset_p[i] = sum[i] / 10;
	}

	while (TRUE) {
		//延时 TIMER_SLEEP s
		while (TRUE) {
			end = GetTickCount();
			if (end - start >= TIMER_SLEEP*1000) {
				start = end;
				break;
			} else {
				SwitchToThread();
			}
		}
	//写入被动运动数据
	double angle[2]{0};
    double torque[2]{0};
    double elbow_pressure[2]{0};
    double six_dim_force[6]{0};

	ControlCard::GetInstance().GetEncoderData(angle);
	DataAcquisition::GetInstance().AcquisiteTensionData(elbow_pressure);
	DataAcquisition::GetInstance().AcquisiteSixDemensionData(six_dim_force);
	/***********new pull sensor **********************/
	DataAcquisition::GetInstance().AcquisitePullAndTorqueData();
	double abs_shoulder_forward_pull = fabs(DataAcquisition::GetInstance().ShoulderForwardPull());
	double abs_shoulder_backward_pull = fabs(DataAcquisition::GetInstance().ShoulderBackwardPull());
	double abs_elbow_forward_pull = fabs(DataAcquisition::GetInstance().ElbowForwardPull());
	double abs_elbow_backward_pull = fabs(DataAcquisition::GetInstance().ElbowBackwardPull());
	torque[0] = DataAcquisition::GetInstance().ShoulderTorque();
	torque[1] = DataAcquisition::GetInstance().ElbowTorque();

	//将六维力的偏执减掉
	for(int i=0;i<6;++i){
		sixdim_force_value[i]-=six_dimension_offset_p[i];
	}

	joint_value << angle[0] << "   "<<angle[0]*0.88<<"    "<< angle[1]<<"    "<<angle[1]*1.3214<<"    "<<angle[1]*0.6607 << std::endl;
	torque_value << torque[0] << "          " << torque[1] << std::endl;
	sixdim_force_value << six_dim_force[0] << "          " << six_dim_force[1] << "          " << six_dim_force[2] << "          " << six_dim_force[3]
						<< "          " << six_dim_force[4] << "          " << six_dim_force[5] << std::endl;
	sum_pressure_force_value<< elbow_pressure[0] * 10- elbow_pressure[1] * 10<<endl;
	pull_force_value << abs_shoulder_forward_pull << "          " << abs_shoulder_backward_pull << "          " << abs_elbow_forward_pull << "          " << abs_elbow_backward_pull << endl;
		
		if (is_exit_thread_) {
			joint_value.close();
			torque_value.close();
			sixdim_force_value.close();
			sum_pressure_force_value.close();
			pull_force_value.close();
			break;
		}

		// 是否录制动作， 1s录制一次
		if (passive->in_record_status_ && (loop_counter_in_thread % 10 == 0)) {
			spdlog::info("{} ready into record thread", __LINE__);
			passive->RecordStep();
		}
    
		//是否开始运动
		if (passive->in_move_status_) {
			passive->MoveStep();
			passive->CollectionStep();
			//passive->SampleStep();
		}
		
		loop_counter_in_thread++;
	}

	if (passive->is_teach) {
		//passive->TeachPosData();
		passive->CruveSmoothing();
	}

	//检查六维力数组和位置数组是否有值，如果没有就将它初始化，如果有的话就clear后再初始化
	passive->LeadTemporaryToSum();

	//passive->InterpolationTraceExport();
	//passive->PracticalTraceExport();

	passive->is_busy_ = false;
	return 0;
}

void PassiveControl::ClearMovementSet()
{
	movement_set_.clear();
}

void PassiveControl::ClearSixdemAndAngle() {
	for (int i = 0; i < 2; ++i) {
		sum_data_.positions[i].clear();
	}
	for (int j = 0; j < 6; ++j) {
		sum_data_.sixdemsional_force[j].clear();
	}
}

void PassiveControl::StoreMovement(const PassiveData& movement) {
	movement_set_.push_back(movement);
}

bool PassiveControl::IsBusy() {
	return is_busy_;
}

void PassiveControl::BeginMove(int index) {
	if (index >= movement_set_.size() || index < 0)
		return;

	// 判断是否在运动中
	if(is_busy_)
		return;

	// 取出示教所需要的数据
	move_data_ = movement_set_.at(index);

	//检查六维力数组和位置数组是否有值，如果没有就将它初始化，如果有的话就clear后再初始化
	if (temporary_data_.positions[0].size() != 0 || temporary_data_.sixdemsional_force[0].size() != 0) {
		for (int i = 0; i < 2; ++i) {
			temporary_data_.positions[i].clear();
		}
		for (int j = 0; j < 6; ++j) {
			temporary_data_.sixdemsional_force[j].clear();
		}
	}

	//各项计数归零
	loop_counter_in_thread = 0;
	hermite_time_counter_ = 0;
	hermite_target_counter_ = 0;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			hermite_time_interval_[i][j] = 0;
			//更新插值位置范围
			hermite_pos_interval_[i][j] = 0;
			//更新插值速度范围
			hermite_vel_interval_[i][j] = 0;
		}
	}
	
	//打开电机，离合器
	ControlCard::GetInstance().SetMotor(ControlCard::MotorOn);
	//ControlCard::GetInstance().SetClutch(ControlCard::ClutchOn);

	//关闭示教采集功能
	in_record_status_ = false;
	//打开线程
	is_exit_thread_ = false;
	//开始运动
	in_move_status_ = true;

	HANDLE handle;
	handle = (HANDLE)_beginthreadex(NULL, 0, RecordOrMoveThread, this, 0, NULL);
}
void PassiveControl::StopMove() {
	//关闭电机
	ControlCard::GetInstance().SetMotor(ControlCard::MotorOff);
	//关闭离合器
	//ControlCard::GetInstance().SetClutch(ControlCard::ClutchOff);
    //关闭线程
	in_move_status_ = false;
	is_exit_thread_ = true;
	in_record_status_ = false;
	is_busy_ = false;
}

void PassiveControl::GetCurrentMove(PassiveData& move) {
	move = move_data_;
}

double PassiveControl::PHermite(double foretime[2],double forepos[2],double forevel[2],double t)
{
    double Houtput=0;
    double a[2]={0};
    double b[2]={0};
    a[0]=(1-2*(t-foretime[0])/(foretime[0]-foretime[1]))*pow((t-foretime[1])/(foretime[0]-foretime[1]),2);
    a[1]=(1-2*(t-foretime[1])/(foretime[1]-foretime[0]))*pow((t-foretime[0])/(foretime[1]-foretime[0]),2);
    b[0]=(t-foretime[0])*pow((t-foretime[1])/(foretime[0]-foretime[1]),2);
    b[1]=(t-foretime[1])*pow((t-foretime[0])/(foretime[1]-foretime[0]),2);
    Houtput=a[0]*forepos[0]+a[1]*forepos[1]+b[0]*forevel[0]+b[1]*forevel[1];
    return Houtput;
}
void PassiveControl::BeginRecord() {
	if (is_busy_) {
		return;
	}	

	active_control_->StartMove(999); //被动运动示教id 设置为定值999
	in_record_status_ = true;
	is_exit_thread_ = false;
	is_teach = true;

	// 清空record_data_
	for (int k = 0; k < 2; k++) {
		if (!record_data_.target_positions[k].empty()) {
			record_data_.target_positions[k].clear();
			}

		if (!record_data_.target_velocitys[k].empty()) {
			record_data_.target_velocitys->clear();
		}
	}

	HANDLE handle;
	handle = (HANDLE)_beginthreadex(NULL, 0, RecordOrMoveThread, this, 0, NULL);
}

void PassiveControl::StopRecord() {
	active_control_->StopMove();
	in_record_status_ = false;
	is_exit_thread_ = true;
	is_busy_ = false;
}

void PassiveControl::GetCurrentRecord(PassiveData& record) {
	record = record_data_;
}


void PassiveControl::RecordStep() {
	//取角度
	double joint_angle[2]{ 0 };
	double joint_vel[2]{ 0 };
	ControlCard::GetInstance().GetEncoderData(joint_angle);
	ControlCard::GetInstance().GetJointVelocity(joint_vel);
	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//spdlog::info("*****************************");
	//spdlog::info("Record ****** shoulder angle : {} , elbow angle :{}", joint_angle[0], joint_angle[1]);
	for (int i = 0; i < 2; i++) {
		record_data_.target_positions[i].push_back(joint_angle[i]);
		record_data_.target_velocitys[i].push_back(joint_vel[i]);
	}
	is_busy_ = true;
}

//这个函数是对目标点进行插值，然后根据插值的位置进行运动
void PassiveControl::MoveStep() {
	double time = loop_counter_in_thread * 0.1;
	I32 Axis[2] = { ControlCard::ShoulderAxisId, ControlCard::ElbowAxisId };

	//每过一秒就更新插值区间
	if (loop_counter_in_thread % 10 == 0) {
		if (hermite_target_counter_ + 1 < move_data_.target_velocitys[0].size()) {
			for (int i = 0; i < 2; i++) {
				//更新插值时间范围
				hermite_time_interval_[i][0] = time;
				hermite_time_interval_[i][1] = time + 1;
				//更新插值位置范围
				hermite_pos_interval_[i][0] = move_data_.target_positions[i].at(hermite_target_counter_);
				hermite_pos_interval_[i][1] = move_data_.target_positions[i].at(hermite_target_counter_ + 1);
				//更新插值速度范围
				hermite_vel_interval_[i][0] = move_data_.target_velocitys[i].at(hermite_target_counter_);
				hermite_vel_interval_[i][1] = move_data_.target_velocitys[i].at(hermite_target_counter_ + 1);
			}
			hermite_target_counter_++;
		} else {
			//运动完成，停止运动
			StopMove();
		}
	}

	//在插值区间内，相当于每100ms就运动到一个新的插值点。///在循环内计算插值点，这样效率是否好？
	for (int j = 0; j < 2; j++) {
		double pos = PHermite(hermite_time_interval_[j],
			hermite_pos_interval_[j],
			hermite_vel_interval_[j],
			time);
		sample_data_.Interpolation_Data[j].push_back(pos);
		//APS_absolute_move(Axis[j], pos / ControlCard::Unit_Convert, 15 / ControlCard::Unit_Convert);
		//2020-09-15 解决抖动问题
		APS_ptp_v(Axis[j], option, pos / ControlCard::Unit_Convert, 5 / ControlCard::Unit_Convert, NULL);
	}

	is_busy_ = true;
}

void PassiveControl::SampleStep() {
	double joint_angle[2]{ 0 };
	ControlCard::GetInstance().GetEncoderData(joint_angle);
	for (int i = 0; i < 2; i++) {
		sample_data_.target_positions[i].push_back(joint_angle[i]);
	}
}

void PassiveControl::CollectionStep() {
	double joint_angle[2]{ 0 };
	double sixdemional_force[6]{ 0 };
	ControlCard::GetInstance().GetEncoderData(joint_angle);
	DataAcquisition::GetInstance().AcquisiteSixDemensionData(sixdemional_force);
	for (int i = 0; i < 2; i++) {
		temporary_data_.positions[i].push_back(joint_angle[i]);
	}
	for (int j = 0; j < 6; ++j) {
		temporary_data_.sixdemsional_force[j].push_back(sixdemional_force[j]);
	}
}

void PassiveControl::LeadTemporaryToSum() {
	int n = temporary_data_.sixdemsional_force[0].size();

	if (sum_data_.positions[0].size()==0 || sum_data_.sixdemsional_force[0].size()==0) {
		vector<double> initialize(n);
		for (int i = 0; i < 2; ++i) {
			sum_data_.positions[i] = initialize;
		}
		for (int j = 0; j < 6; ++j) {
			sum_data_.sixdemsional_force[j] = initialize;
		}
	}

	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < 2; ++j) {
			sum_data_.positions[j][i] += temporary_data_.positions[j][i];
		}
		for (int k = 0; k < 6; ++k) {
			sum_data_.sixdemsional_force[k][i] += temporary_data_.sixdemsional_force[k][i];
		}
	}
}

void PassiveControl::GetMeanData(int total_times) {
	for (int i = 0; i < sum_data_.positions[0].size(); i++) {
		for (int j = 0; j < 2; ++j) {
			sum_data_.positions[j][i] = sum_data_.positions[j][i] / total_times;	
		}
		for (int k = 0; k < 6; ++k) {
			sum_data_.sixdemsional_force[k][i] = sum_data_.sixdemsional_force[k][i] / total_times;
		}
	}
}

void PassiveControl::SixdemToBaseCoordinate() {
	VectorXd sixdemsional_data(6);
	double mean_angle[2] = { 0 };
	for (int i = 0; i < sum_data_.sixdemsional_force[0].size(); ++i) {
		for (int j = 0; j < 6; ++j) {
			sixdemsional_data[j] = sum_data_.sixdemsional_force[j][i];
		}
		for (int k = 0; k < 2; k++) {
			mean_angle[k] = sum_data_.positions[k][i];
		}

		MSixdemToBaseCoordinate(sixdemsional_data, mean_angle, active_control_->is_left);

		for (int p = 0; p < 6; ++p) {
			active_control_->mean_force_and_position_.mean_sixdemsional_force[p].push_back(sixdemsional_data[p]);
		}
		for (int q = 0; q < 2; ++q) {
			active_control_->mean_force_and_position_.mean_positions[q].push_back(mean_angle[q]);
		}
	}
	GravityAndAngleExport();
}

void PassiveControl::SetHWND(HWND hWnd) {
	hWnd_ = hWnd;
}

void PassiveControl::CruveSmoothing() {
	for (int i = 0; i < 2; ++i) {
		max_pos[i] = GetMaxData(record_data_.target_positions[i]);
	}
	array_size = record_data_.target_positions[0].size();
	//if (array_size % 2 == 0) {
	//	array_size++;
	//}
	//curve_x = array_size - 1;

	// guarantee the size of array is even number 2020-07-30
	if (array_size % 2 == 1) {
		array_size++;
	}
	curve_x = array_size -1;
	DrawSincruve();
	is_teach = false;



}

double PassiveControl::GetMaxData(std::vector<double> &data) {
	double maxdata = data[0];
	for (int i = 1; i<data.size(); ++i) {
		if (abs(maxdata)<abs(data[i])) {
			maxdata = data[i];
		}
	}
	return maxdata;
}

void PassiveControl::DrawSincruve() {
	//double curve_pi = M_PI / curve_x;
	//for (int j = 0; j < 2; ++j) {
	//	for (int i = 0; i < array_size; ++i) {
	//		record_data_.target_positions[j][i] = sin(i * curve_pi)*max_pos[j];
	//		record_data_.target_velocitys[j][i] = cos(i * curve_pi)*max_pos[j] * curve_pi;
	//	}
	//	for (int i = 0; i < 2; ++i) {
	//		record_data_.target_positions[i][array_size - 1] = 0;
	//	}
	//}

	//draw curve by using cubic polynomial 2020-08-07
	double ts = 1;
	double tf = array_size * ts/2;
	Vector4d parameterA[2];
	Vector4d positionInfo[2];
	Matrix4d M;
	Vector4d timeInfo[2];
	Vector4d timeInfoDot[2];
	M << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 1, tf, tf*tf, tf*tf*tf,
		 0, 1, 2 * tf, 3 * tf*tf;

	for (int j = 0; j < 2; ++j) {
		
		for (int i = 0; i < (array_size / 2); ++i) {
			positionInfo[j] << 0, 0, max_pos[j], 0;
			timeInfo[j] << 1, i*ts, pow(i*ts, 2), pow(i*ts, 3);
			timeInfoDot[j] << 0, 1, 2 * i*ts, 3 * pow(i*ts, 2);
			parameterA[j] = M.inverse()*positionInfo[j];
			record_data_.target_positions[j][i] = parameterA[j].transpose()*timeInfo[j];
			record_data_.target_velocitys[j][i] = parameterA[j].transpose()*timeInfoDot[j];
		}
		for (int i = (array_size / 2); i < array_size; ++i) {
			positionInfo[j] << max_pos[j], 0, 0, 0;
			timeInfo[j] << 1, (i- (array_size / 2))*ts, pow((i - (array_size / 2))*ts, 2), pow((i - (array_size / 2))*ts, 3);
			timeInfoDot[j] << 0, 1, 2 * (i - (array_size / 2))*ts, 3 * pow((i - (array_size / 2))*ts, 2);
			parameterA[j] = M.inverse()*positionInfo[j];
			record_data_.target_positions[j][i] = parameterA[j].transpose()*timeInfo[j];
			record_data_.target_velocitys[j][i] = parameterA[j].transpose()*timeInfoDot[j];
			//if (record_data_.target_positions[j][i] = 0) {
			//	for (int k = i; k < array_size; ++k) {
			//		record_data_.target_positions[j][k] = 0;
			//		record_data_.target_velocitys[j][k] = 0;
			//	}
			//}

		}
		//for (int i = 0; i < array_size ; ++i) {
		//	if (record_data_.target_positions[j][i] < 0) {
		//		record_data_.target_positions[j][i] = 0;
		//		record_data_.target_velocitys[j][i] = 0;
		//	}
		//}
		record_data_.target_positions[j][array_size - 2] = 0;
		record_data_.target_positions[j][array_size - 1] = 0;
			
	}

}


void PassiveControl::TeachPosData() {
	ofstream dataFile1;
	dataFile1.open("teach_position_data.txt", ofstream::app);
	dataFile1 << "shoulder" << "   " << "elbow" << endl;
	for (int i = 0; i < record_data_.target_positions[0].size(); ++i) {
		dataFile1 << record_data_.target_positions[0][i] << "        " << record_data_.target_positions[1][i] << endl;
	}
	dataFile1.close();
}

void PassiveControl::InterpolationTraceExport() {
	ofstream dataFile2;
	dataFile2.open("interpolation_trace_data.txt", ofstream::app);
	dataFile2 << "shoulder" << "   " << "elbow" << endl;
	for (int i = 0; i < sample_data_.Interpolation_Data[0].size(); ++i) {
		dataFile2 << sample_data_.Interpolation_Data[0][i] << "        " << sample_data_.Interpolation_Data[1][i] << endl;
	}
	dataFile2.close();
}

void PassiveControl::PracticalTraceExport() {
	ofstream dataFile3;
	dataFile3.open("practice_trace_data.txt", ofstream::app);
	dataFile3 << "shoulder" << "   " << "elbow" << endl;
	for (int i = 0; i < sample_data_.target_positions[0].size(); ++i) {
		dataFile3 << sample_data_.target_positions[0][i] << "        " << sample_data_.target_positions[1][i] << endl;
	}
	dataFile3.close();
}

void PassiveControl::GravityAndAngleExport() {
	string path = "..\\..\\resource\\grivityandangle"; //指定文件目录
	vector<string> filesName;
	GetFileName(path, filesName);
	int k = filesName.size();
	string p = to_string(k);
	ofstream dataFile4;
	dataFile4.open("..\\..\\resource\\grivityandangle\\practice_trace_data_" + p + ".txt", ofstream::app);
	dataFile4 << "girvity1" << "          " << "girvity2" << "          " << "girvity3" << "          " << "girvity4" << "          " <<
		"girvity5" << "          " << "girvity6" << "          " << "shoulder_angle" << "          " << "elbow_angle" << endl;
	for (int i = 0; i < active_control_->mean_force_and_position_.mean_sixdemsional_force[0].size(); ++i) {
		dataFile4 << active_control_->mean_force_and_position_.mean_sixdemsional_force[0][i] << "        " << active_control_->mean_force_and_position_.mean_sixdemsional_force[1][i] << "        " <<
			active_control_->mean_force_and_position_.mean_sixdemsional_force[2][i] << "        " << active_control_->mean_force_and_position_.mean_sixdemsional_force[3][i] << "        " <<
			active_control_->mean_force_and_position_.mean_sixdemsional_force[4][i] << "        " << active_control_->mean_force_and_position_.mean_sixdemsional_force[5][i] << "        " <<
			active_control_->mean_force_and_position_.mean_positions[0][i] << "        " << active_control_->mean_force_and_position_.mean_positions[1][i] << endl;
	}
	dataFile4.close();
}

void PassiveControl::GetFileName(string path, vector<string>& filesName) {
	long   hFile = 0;                    //文件句柄 
	struct _finddata_t fileinfo;        //定义文件信息结构体
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) //使用函数_findfirst()打开文件并获取第一个文件名
	{
		do
		{
			if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)  //"."表示当前目录，".."表示父目录
				filesName.push_back(fileinfo.name);
		} while (_findnext(hFile, &fileinfo) == 0);      //使用函数_findnext()继续获取其他文件名
		_findclose(hFile);              //使用函数_findclose()关闭文件夹
	}
}
#pragma once
#include <NIDAQmx.h>
#include <Eigen/core>

class DataAcquisition {
public:
	static DataAcquisition &GetInstance();
	DataAcquisition(const DataAcquisition &) = delete;
	DataAcquisition(DataAcquisition &&) = delete;
	DataAcquisition &operator=(const DataAcquisition &) = delete;
	DataAcquisition &operator=(DataAcquisition &&) = delete;

	void AcquisiteTorqueData();
	void AcquisitePullSensorData();
	void AcquisiteSixDemensionData(double output_buf[6]);
	//�粿���������ݵĲɼ�����
	void AcquisiteShoulderTensionData(double shoulder_tension_output[4]);
	//�ⲿ���������ݵĲɼ�����
	void AcquisiteElbowTensionData(double elbow_tension_output[4]);
	//���ﳢ���°Ѽ�������ݲɼ�����һ�𣬸о��������ܿ�������
	void AcquisiteTensionData(double tension_output[8]);
	void AcquisiteGripData(double grip[1]);
	double ShoulderTorque();
	double ElbowTorque();
	double ShoulderForwardPull();
	double ShoulderBackwardPull();
	double ElbowForwardPull();
	double ElbowBackwardPull();

	bool StartTask();
	bool StopTask();

private:
	DataAcquisition();

private:
	TaskHandle m_task_handle;

	double shoulder_raw_torque_ = 0.0;
	double elbow_raw_torque_ = 0.0;

	double shoulder_raw_forward_pull_ = 0.0;
	double shoulder_raw_backward_pull_ = 0.0;
	double elbow_raw_forward_pull_ = 0.0;
	double elbow_raw_backward_pull_ = 0.0;

	static const char *kTorqueChannel;
	static const char *kPullSensorChannel;
	static const char *kSixDimensionForceChannel;
	static const char *kGripChannel;
	static const char *kShoulderTensionChannel;
	static const char *kElbowTensionChannel;
	static const double kRawToReal;

	static Eigen::Matrix<double, 6, 6>  kTransformMatrix;

};

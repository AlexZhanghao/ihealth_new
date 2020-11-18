#ifndef Matrix_H
#define Matrix_H
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>


using namespace Eigen;

//const double ShoulderWidth = 0.297;
//const double ShoulderLength = 0.435;
//const double UpperArmLength = 0.296;
//const double LowerArmLength = 0.384;
//const double guide = 0.07;
//const double shouler_installationsite_to_coordinate4 = 0.140;
//const double elbow_installationsite_to_coordinate5 = 0.150;
//const double d1 = 0.23;
//const double d2 = 0.4683;
//const double d3 = 0.307;
//const double d4 = 0.276 + guide;
//const double d5 = 0.174;
//const double r5 = 0.074;
//const double dy_2 = 0.087;
//const double dz_2 = 0.188;
const double sixdim_to_coordinate3_level = 0.133;
const double sixdim_to_coordinate3_vertical = 0.115;

const double InitAngle[3] = {
	0, 0, 0
};

 //Vector3d AxisDirection[3];
 //Vector3d AxisPosition[3];
 //Matrix3d RF13;
 //extern Matrix3d sixdim_rotation;


#ifdef LEFT_ARM

static Vector3d AxisDirection[3] = {
	Vector3d(-1.0,0,0),
	Vector3d(0,0,1.0),
	Vector3d(0,-1.0,0),
};
static Vector3d AxisPosition[3] = {
	Vector3d(0,ixdim_to_coordinate3_level,0),
	Vector3d(-sixdim_to_coordinate3_vertical,ShoulderWidth,0),
	Vector3d(-sixdim_to_coordinate3_vertical,0,0),

};
#else
static Vector3d AxisDirection[3] = {
	Vector3d(0,-1,0),
	Vector3d(1,0,0),
	Vector3d(0,0,1),

};
static Vector3d AxisPosition[3] = {
	Vector3d(0,0,-sixdim_to_coordinate3_level),
	Vector3d(0,-sixdim_to_coordinate3_vertical,-sixdim_to_coordinate3_level),
	Vector3d(0,-sixdim_to_coordinate3_vertical,0),

};
#endif


template<typename DerivedA, typename DerivedB,typename DerivedC>
void pinv(const MatrixBase<DerivedA>& A,const MatrixBase<DerivedB>&G, MatrixBase<DerivedC>& B)
{
	//这里就是在算这个投影矩阵p，最后的结果用B输出来，这里的A就是那个Γ，这里的G就是那个G
	MatrixXd A_temp(2,2);
	A_temp=A.transpose()*G*A;
	B=(A_temp.inverse())*A.transpose()*G;

}

template<typename DerivedA, typename DerivedB>
void pinv2(const MatrixBase<DerivedA>& A, MatrixBase<DerivedB>& B) {
	MatrixXd A_temp(2, 2);
	A_temp = A.transpose()*A;
	B = (A_temp.inverse())*A.transpose();
}


// screw axes, [Bi] =[[w],v; 0 0]  
template<typename DerivedA, typename DerivedB, typename DerivedC>
void CalculateSpinor(const MatrixBase<DerivedA>& axis, const MatrixBase<DerivedB>&pos, MatrixBase<DerivedC>& twist) {
	// [Bi] = ([wi] vi; 0 0)	
	Matrix3d axis_hat;
	twist.setZero();
	axis_hat <<
		0, -axis(2), axis(1),
		axis(2), 0, -axis(0),
		-axis(1), axis(0), 0;
	//这里算出来的w_q就是运动w × q
	Vector3d w_q = axis.cross(pos);
	twist.block(0, 0, 3, 3) = axis_hat;
	twist.block(0, 3, 3, 1) = -w_q;
}


// skew matrix, from cross to dot product
template<typename DerivedA, typename DerivedB>
void VectorToMatrix(const MatrixBase<DerivedA>& X, MatrixBase<DerivedB>& Y)
{

	MatrixXd y(3, 3);
	y.setZero();
	y(1, 2) = -X(0);
	y(0, 2) = X(1);
	y(0, 1) = -X(2);
	Y = y - y.transpose();
}

//calculate adjoint matrix, for example for T = (r(3,3),p(3,1);0(1,3) 1), we can get Ad_T = (r ,0;[p]*r, r )
template<typename DerivedA, typename DerivedB>
void CalculateAdjointMatrix(const MatrixBase<DerivedA>& X, MatrixBase<DerivedB>& A) {
	MatrixXd Y(3, 3);
	Vector3d h(3);
	A.setZero();
	A.block(0, 0, 3, 3) = X.block(0, 0, 3, 3);
	A.block(3, 3, 3, 3) = X.block(0, 0, 3, 3);
	h = X.block(0, 3, 3, 1);
	VectorToMatrix(h, Y);
	A.block(3, 0, 3, 3) = Y*X.block(0, 0, 3, 3);
}



//[Bi] = ([wi] vi; 0 0) to Bi = (wi,vi)
template<typename DerivedA, typename DerivedB>
void MatrixToVector(const MatrixBase<DerivedA>& X, MatrixBase<DerivedB>& b) {

	//[Bi] = ([wi] vi; 0 0) 转换成 Bi = (wi,vi)向量
	b(0) = -X(1, 2);
	b(1) = X(0, 2);
	b(2) = -X(0, 1);
	b.block(3, 0, 3, 1) = X.block(0, 3, 3, 1);
}

template<typename DerivedA, typename DerivedB>
void ActiveJointsAngleToAllJointsAngle(const MatrixBase<DerivedA>& U, MatrixBase<DerivedB>& theta) {
	MatrixXd meta(3, 2);
	VectorXd thetab(3);

	//meta就是传动矩阵η
	meta << 1, 0,
		0.88, 0,
		0, 1;
	

	thetab << InitAngle[0], InitAngle[1], InitAngle[2];
	//电机的角度就是初始角度加上转换后的角度
	theta = thetab + meta*U;
}



template<typename DerivedA, typename DerivedB>
void AdmittanceControl(const MatrixBase<DerivedA>& torque, MatrixBase<DerivedB>& vel)
{
	Vector2d ratio;
	Matrix2d m_ratio;
	Vector2d admittance;
	Matrix2d m_admittance;

	MatrixXd meta(3, 2);
	meta << 1, 0,
		0.88, 0,
		0, 1;

		//0, 1.3214,
		//0, 0.6607;

	MatrixXd projection(2, 3);

	pinv2(meta, projection);

	ratio << 0.9, 0.9;
	admittance << 2, 2;
	m_ratio = ratio.asDiagonal();
	m_admittance = admittance.asDiagonal();

	vel = m_admittance * (m_ratio*projection * torque);
}


// transformation matrix T(4,4), from body frame to fixed frame 
template<typename DerivedA, typename DerivedB>
void fwd_geo_kineB(const MatrixBase<DerivedA>& theta, MatrixBase<DerivedB>& T0h)
{
	Matrix4d exp_m[3];
	Matrix4d Bh[3];
	for (size_t i = 0; i < 3; i++) {
		CalculateSpinor(AxisDirection[i], AxisPosition[i], Bh[i]);
		exp_m[i] = Bh[i] * (2 * M_PI / 360)*theta(i);
	}
	Matrix4d T0h0;
	T0h0 <<
		1, 0, 0, 0,
		0, 1, 0, sixdim_to_coordinate3_vertical,
		0, 0, 1, sixdim_to_coordinate3_level,
		0, 0, 0, 1;
	// based on screw axes in body frame
	T0h = T0h0*(exp_m[0].exp())*(exp_m[1].exp())*(exp_m[2].exp());
}



//template<typename DerivedA>
//void MomentBalance(const MatrixBase<DerivedA>& sixDimensionalForce, double motorangle[2], double jointMoment[3],int is_left) {
//
//	MatrixXd theta(3,1);
//	MatrixXd motorangleMatrix(2, 1);
//	MatrixXd jacobian(6, 3);
//	MatrixXd jacobianNew(6, 3);
//	
//	for (int i = 0; i < 2; i++) {
//		motorangleMatrix(i) = motorangle[i];
//	}
//	// 3个关节的角度。下面这个函数就是把电机的角度转换为关节的角度。
//	ActiveJointsAngleToAllJointsAngle(motorangleMatrix, theta);
//
//	for (int i = 0; i < 3; i++) {
//		theta(i) = (M_PI / 180)*theta(i);
//	}
//
//	Matrix4d spinor_hat[3];
//	Matrix<double, 6, 1> spinor[3];
//	// calculate [B0] [B1] [B2], and B0 B1 B2 
//	for (int i = 0; i < 3; i++) {
//		//AxisDirection:3*1 AxisPosition:3*1 spinor_hat:4*4
//
//		CalculateSpinor(AxisDirection[i], AxisPosition[i], spinor_hat[i]);
//		MatrixToVector(spinor_hat[i], spinor[i]);
//	}
//
//	// -[B1]*theta1 -[B2]*theta2
//	Matrix4d m[2];
//	for (int i = 0; i < 2; i++) {
//		m[i] = -spinor_hat[i + 1] *theta(i + 1);
//	}
//	// Ad_exp(-[Bi]*thetai) 
//	Matrix4d exp_m[2];
//	exp_m[0] = m[0].exp();
//	exp_m[1] = m[1].exp();
//	Matrix<double, 6, 6> A[2];
//	CalculateAdjointMatrix(exp_m[0], A[0]);
//	CalculateAdjointMatrix(exp_m[1], A[1]);
//
//	// Ji = Ad_exp(-[Bi+1]*theta(i+1))..Ad_exp(-[Bn]*theta(n))*Bi
//	// J = [Jw; Jv]
//	jacobian.block(0, 0, 6, 1) = A[1] * A[0] * spinor[0];
//	jacobian.block(0, 1, 6, 1) = A[1] * spinor[1];
//	jacobian.block(0, 2, 6, 1) = spinor[2];
//	// J = [Jv;Jw]
//	//jacobianNew.block(0, 0, 3, 3) = jacobian.block(3, 0, 3, 3);
//	//jacobianNew.block(3, 0, 3, 3) = jacobian.block(0, 0, 3, 3);
//	//// calculate joint moment
//	//Vector3d jointMomentVector;
//	//jointMomentVector = jacobian * sixDimensionalForce;
//	//for (size_t i = 0; i < 3; i++) {
//	//	jointMoment[i] = jointMomentVector(i);
//	//}
//
//
//
//	// admittance control with projection matrix
//	// the moment of sixdimensional sensor is useless
//	MatrixXd jointMomentMatrix(3, 1);
//	MatrixXd velocityMatrix(2, 1);
//	MatrixXd sixDimensionalwithoutForce(6, 1);
//	sixDimensionalwithoutForce.setZero();
//	sixDimensionalwithoutForce.block(0, 0, 3, 1) = sixDimensionalForce.block(0, 0, 3, 1);
//	AdmittanceControl(jointMomentMatrix, velocityMatrix);
//	//为了命名一致，得到的是两个主动关节的力矩，并通过导纳系数转换成速度，
//	for (int i = 0; i < 2; i++) {
//		jointMoment[i] = velocityMatrix(i);
//	}
//
//	//AllocConsole();
//	//freopen("CONOUT$", "w", stdout);
//	//cout << "f1_3:" << f1_3 << "\n" << "n1_3:" << n1_3 << endl;
//	//cout << "f2_5:" << f2_5 << "\n" << "n2_5:" << n2_5 << endl;
//	//cout << "shoulder_force_moment:" << shoulder_force_moment << "\n" << "elbow_force_moment" << elbow_force_moment << endl;
//}



template<typename DerivedA>
void MomentBalance(const MatrixBase<DerivedA>& sixDimensionalForce, double motorangle[2], double jointMoment[3], int is_left) {

	MatrixXd theta(3, 1);
	MatrixXd motorangleMatrix(2, 1);
	Matrix4d transformationMatrix[4];
	Matrix4d DHparameter;
	MatrixXd jacobian(6, 3);
	Vector3d z[3];
	Vector3d p[3];
	for (int i = 0; i < 2; i++) {
		motorangleMatrix(i) = motorangle[i];
	}
	// transfer the motor angle to the joint angle
	ActiveJointsAngleToAllJointsAngle(motorangleMatrix, theta);

	for (int i = 0; i < 3; i++) {
		theta(i) = (M_PI / 180)*theta(i);
	}
	// definition of the modified DH parameters for the right arm
	DHparameter << 0, 0, 0, theta(0),
		0, M_PI / 2, 0, -(M_PI / 2) + theta(1),
		0, -M_PI / 2, 0, theta(2),
		sixdim_to_coordinate3_vertical, 0, sixdim_to_coordinate3_level, -M_PI / 2 +M_PI/6;

	//// definition of the modified DH parameters for the left arm
	//DHparameter << 0, 0, 0, theta(0),
	//	0, M_PI / 2, 0, (M_PI / 2) + theta(1),
	//	0, M_PI / 2, 0, theta(2),
	//	sixdim_to_coordinate3_vertical, M_PI, sixdim_to_coordinate3_level, M_PI / 2-M_PI/6;

	//calculate transformation matrix T_i-1,i
	for (int i = 0; i < 4; i++) {
		transformationMatrix[i] << cos(DHparameter(i, 3)), -sin(DHparameter(i, 3)), 0, DHparameter(i, 0),
			sin(DHparameter(i, 3))*cos(DHparameter(i, 1)), cos(DHparameter(i, 3))*cos(DHparameter(i, 1)), -sin(DHparameter(i, 1)), -DHparameter(i, 2)*sin(DHparameter(i, 1)),
			sin(DHparameter(i, 3))*sin(DHparameter(i, 1)), cos(DHparameter(i, 3))*sin(DHparameter(i, 1)), cos(DHparameter(i, 1)), DHparameter(i, 2)*cos(DHparameter(i, 1)),
			0, 0, 0, 1;
	}
	// calculate transformation matrix T_0,i
	for (int i = 1; i < 4; i++) {
		transformationMatrix[i] = transformationMatrix[i - 1] * transformationMatrix[i];
	}
	// find z_0,i and p_0,i
	for (int i = 0; i < 3; i++) {
		z[i] = transformationMatrix[i].block(0, 2, 3, 1);
		p[i] = transformationMatrix[i].block(0, 3, 3, 1);
	}
	//calculate Jacobian matrix in based frame, so the sixdimensional force must be in the based frame
	for (int i = 0; i < 3; i++) {
		jacobian.block(3, i, 3, 1) = z[i];
		jacobian.block(0, i, 3, 1) = z[i].cross(p[2] - p[i]);
	}


	// admittance control with projection matrix
	// the moment of sixdimensional sensor is useless
	MatrixXd jointMomentMatrix(3, 1);
	MatrixXd velocityMatrix(2, 1);
	MatrixXd sixDimensionalwithoutMoment(6, 1);
	sixDimensionalwithoutMoment.setZero();
	sixDimensionalwithoutMoment.block(0, 0, 3, 1) = sixDimensionalForce.block(0, 0, 3, 1);

	// upper arm's gravity compensation (also can be the mass of the shell on shoulder)
	MatrixXd gravityForce(6, 1);
	MatrixXd gravityAcceleration(6, 1);
	MatrixXd gravityForceinBody(6, 1);
	double armMass;
	gravityAcceleration.setZero();
	gravityForce.setZero();
	gravityForceinBody.setZero();

	//根据具体护套的质量0.205
	armMass = 0;
	//右手
	gravityAcceleration << 0, 0, -9.81, 0, 0, 0;
	////左手
	//gravityAcceleration << 0, 0, 9.81, 0, 0, 0;


	gravityForce.block(0, 0, 3, 1) = armMass * gravityAcceleration.block(0, 0, 3, 1);
	//gravityForceinBody.block(0, 0, 3, 1) = transformationMatrix[3].block(0, 0, 3, 3)*gravityForce.block(0, 0, 3, 1);


	//初始位置多减了一次护套质量，armMass代表护套质量
	MatrixXd shell(6, 1);
	//右手
	shell << armMass*9.81*sin(M_PI / 6), armMass*9.81*cos(M_PI / 6), 0, 0, 0, 0;
	////左手
	//shell << armMass * 9.81*sin(M_PI / 6), -armMass*9.81*cos(M_PI / 6), 0, 0, 0, 0;


	// transfer the sixdimensional force to based frame
	MatrixXd transferBodytoBase(6, 6);
	MatrixXd sixDimensionalForceinBase(6, 1);
	CalculateAdjointMatrix(transformationMatrix[3], transferBodytoBase);
	sixDimensionalForceinBase = transferBodytoBase * (sixDimensionalwithoutMoment + shell) - gravityForce;


	jointMomentMatrix = jacobian.transpose()* sixDimensionalForceinBase;
	AdmittanceControl(jointMomentMatrix, velocityMatrix);

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("f1: %lf     f2: %lf  f3:%lf   \n", sixDimensionalForceinBase(0), sixDimensionalForceinBase(1), sixDimensionalForceinBase(2));
	//printf("tau1: %lf     tau2: %lf  tau3:%lf  \n", jointMomentMatrix(0), jointMomentMatrix(1), jointMomentMatrix(2));


	// calculate joint moment

	for (int i = 0; i < 2; i++) {
		jointMoment[i] = velocityMatrix(i);
	}


}





template<typename DerivedA>
void MSixdemToBaseCoordinate(MatrixBase<DerivedA>& sixdemdata, double motorangle[2], int is_left) {

	MatrixXd theta(3, 1);
	MatrixXd motorangleMatrix(2, 1);

	for (int i = 0; i < 2; i++) {
		motorangleMatrix(i) = motorangle[i];
	}
	// 3个关节的角度。下面这个函数就是把电机的角度转换为关节的角度。
	ActiveJointsAngleToAllJointsAngle(motorangleMatrix, theta);

	for (int i = 0; i < 3; i++) {
		theta(i) = (M_PI / 180)*theta(i);
	}

	Matrix4d spinor_hat[3];
	Matrix<double, 6, 1> spinor[3];
	// calculate [B0] [B1] [B2], and B0 B1 B2 
	for (int i = 0; i < 3; i++) {
		//AxisDirection:3*1 AxisPosition:3*1 spinor_hat:4*4

		CalculateSpinor(AxisDirection[i], AxisPosition[i], spinor_hat[i]);
		MatrixToVector(spinor_hat[i], spinor[i]);
	}
	// [B1]*theta1 [B2]*theta2
	Matrix4d m[3];
	for (int i = 0; i < 3; i++) {
		m[i] = spinor_hat[i] * theta(i);
	}
	// exp([Bi]*theta(i))
	Matrix4d exp_m[3];
	exp_m[0] = m[0].exp();
	exp_m[1] = m[1].exp();
	exp_m[2] = m[2].exp();

	// transformation matrix in home configuration
	Matrix4d M0;
	M0 <<
		1, 0, 0, 0,
		0, 1, 0, sixdim_to_coordinate3_vertical,
		0, 0, 1, sixdim_to_coordinate3_level,
		0, 0, 0, 1;

	// transformation matrix, from body frame to based frame
	Matrix4d sixdim_transfer;
	sixdim_transfer = M0 * exp_m[0] * exp_m[1] * exp_m[2];

	// adjoint matrix 
	MatrixXd A1(6, 6);
	CalculateAdjointMatrix(sixdim_transfer, A1);

	sixdemdata = A1 * sixdemdata;
}


#endif
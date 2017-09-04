#include <cmath>
#include <iomanip>

#include <QtCore/QDebug>
#include <numeric>
//#include <Dependencies\64 bit\Include\Eigen\Dense>
//#include <Dependencies\64 bit\Include\Eigen\EigenValues>
#include <Dense>
#include <EigenValues>
#include "hMatrix.h"
#include "hPoint.h"
#include "Quaternion.h"
#include "dualQuaternion.h"


template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

	// initialize original index locations
	vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });

	return idx;
}


float SIGN(double x){ return (x >= 0.0f) ? +1.0f : -1.0f; }

hMatrix::hMatrix()
{
	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			m[i][j] = 0.0f;
}

hMatrix operator+( const hMatrix& arg1, const hMatrix& arg2 )
{
	hMatrix sum;

	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			sum.m[i][j] = arg1.m[i][j] + arg2.m[i][j];

	return sum;
}

hMatrix operator-( const hMatrix& arg1, const hMatrix& arg2 )
{
	hMatrix sum;

	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			sum.m[i][j] = arg1.m[i][j] - arg2.m[i][j];

	return sum;
}

hMatrix operator*( const hMatrix& arg1, const hMatrix& arg2 )
{
	hMatrix mul;

	mul.Clear();
			
	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			for( int k = 0; k < 4; k ++ )
				mul.m[i][j] += arg1.m[i][k] * arg2.m[k][j];

	return mul;
}

hMatrix operator*( const hMatrix& arg1, const double arg2 )
{
	hMatrix sum;

	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			sum.m[i][j] = arg1.m[i][j] * arg2;

	return sum;
}

hMatrix operator*( const double arg1, const hMatrix& arg2 )
{
	hMatrix sum;

	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			sum.m[i][j] = arg1 * arg2.m[i][j];

	return sum;
}

hMatrix operator/( const hMatrix& arg1, const double arg2 )
{
	hMatrix sum;

	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			sum.m[i][j] = arg1.m[i][j] / arg2;

	return sum;
}

ostream& operator<<(ostream& os, hMatrix& arg)
{
	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
			os<<fixed<<setprecision(4)<<arg.m[i][j]<<" ";
		os<<endl;
	}
	return os;
}

hMatrix& hMatrix::operator=( hMatrix arg1 )
{
	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			m[i][j] = arg1.m[i][j];

	return *this;
}

void hMatrix::Clear()
{
	for( int i = 0; i < 4; i ++ )
		for( int j = 0; j < 4; j ++ )
			m[i][j] = 0.0f;

	return;
}

hMatrix hMatrix::transpose( )
{
	int i, j;
	hMatrix temp;

	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			temp.m[i][j] = m[j][i];

	return temp;
		
}


hMatrix hMatrix::Inverse( )
{
	hMatrix arg1;
	int *is, *js, i, j, k, l, u, v;

	double d, p, a[16];

	for( i = 0; i < 4 ; i ++ )
		for( j = 0; j < 4; j ++ )
			a[i*4+j] = m[i][j];

	int n = 4;

	is = new int[n];
	js = new int[n];

	for( k = 0; k <= n-1; k ++ )
	{
		d = 0.0f;
		for( i = k; i <= n-1; i ++ )
			for( j = k; j <= n-1; j ++ )
			{
				l = i * n + j;
				p = fabs( a[l] );
				if( p > d )
				{
					d = p; 
					is[k] = i;
					js[k] = j;
				}
			}

		if( d + 1.0f == 1.0f )
		{
			delete is;
			delete js;
			arg1.Clear();
			return arg1;
		}

		if( is[k] != k )
			for( j = 0; j <= n -1; j ++ )
			{
				u = k * n + j;
				v = is[k] * n + j;
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}

		if( js[k] != k )
			for( i = 0; i <= n-1; i ++ )
			{
				u = i * n + k;
				v = i * n + js[k];
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}

		l = k * n + k;
		a[l] = 1.0f / a[l];

		for( j = 0; j <= n-1; j ++ )
			if( j != k )
			{
				u = k * n + j;
				a[u] = a[u] * a[l];
			}

		for( i = 0; i <= n-1; i ++ )
			if( i !=k )
				for( j = 0; j <= n-1; j ++ )
					if( j != k )
					{
						u = i * n + j;
						a[u] = a[u] - a[i*n+k] * a[k*n+j];
					}

		for( i = 0; i <= n-1; i ++ )
			if( i != k )
			{
				u = i * n + k;
				a[u] = -a[u] * a[l];
			}
	}

	for( k = n-1; k >= 0; k -- )
	{
		if( js[k] != k )
			for( j = 0; j <= n-1; j ++ )
			{
				u = k * n + j;
				v = js[k] * n + j;
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}

		if( is[k] != k )
			for( i = 0; i <= n-1; i ++ )
			{
				u = i * n + k;
				v = i * n + is[k];
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}
	}

	for( i = 0; i < 4; i ++ )
		for( j = 0; j < 4; j ++ )
			arg1.m[i][j] = a[i*4+j];
		
	delete[] is;
	delete[] js;

	return arg1;
}

hMatrix hMatrix::homogeneousMatrixForJoints(const QVector3D& joint,CAPTURETYPE type)
{
	hMatrix result_matrix;
	if (type == PLANAR)
	{
		//QVector3D tempHip = joint;
		result_matrix.m[0][0] = 1.0f; result_matrix.m[0][1] = 0.0f; result_matrix.m[0][2] = 0.0f; result_matrix.m[0][3] = joint.x();
		result_matrix.m[1][0] = 0.0f; result_matrix.m[1][1] = 1.0f; result_matrix.m[1][2] = 0.0f; result_matrix.m[1][3] = joint.y();
		result_matrix.m[2][0] = 0.0f; result_matrix.m[2][1] = 0.0f; result_matrix.m[2][2] = 1.0f; result_matrix.m[2][3] = 1.0f;
		result_matrix.m[3][0] = 0.0f; result_matrix.m[3][1] = 0.0f; result_matrix.m[3][2] = 0.0f; result_matrix.m[3][3] = 1.0f;
	}
	else if (type == SPATIAL)
	{
		result_matrix.m[0][0] = 1.0f; result_matrix.m[0][1] = 0.0f; result_matrix.m[0][2] = 0.0f; result_matrix.m[0][3] = joint.x();
		result_matrix.m[1][0] = 0.0f; result_matrix.m[1][1] = 1.0f; result_matrix.m[1][2] = 0.0f; result_matrix.m[1][3] = joint.y();
		result_matrix.m[2][0] = 0.0f; result_matrix.m[2][1] = 0.0f; result_matrix.m[2][2] = 1.0f; result_matrix.m[2][3] = joint.z();
		result_matrix.m[3][0] = 0.0f; result_matrix.m[3][1] = 0.0f; result_matrix.m[3][2] = 0.0f; result_matrix.m[3][3] = 1.0f;
	}
	return result_matrix;
}

hMatrix hMatrix::homogeneousMatrix(QVector3D hand, QVector3D handtip, QVector3D handthumb,CAPTURETYPE type)
{
	hMatrix result_matrix; 
	QVector3D result_vector;
	QVector3D unit_x(1.0f, 0.0f, 0.0f); QVector3D unit_y(0.0f, 1.0f, 0.0f); QVector3D unit_z(0.0f, 0.0f, 1.0f);
	if (type == 0)
	{
		QVector3D tempHand = hand;
		QVector3D tempHandTip = handtip;
		QVector3D tempHandThumb = handthumb;
		QVector3D normalizedXAxis = (tempHandThumb - tempHand).normalized();
		

		QVector3D normalizedYAxis = (tempHandTip - tempHand).normalized();

		QVector3D norm_vector;
		QVector3D normalizedZAxis = (norm_vector.crossProduct(normalizedXAxis, normalizedYAxis)).normalized();

		
		normalizedXAxis = (norm_vector.crossProduct(normalizedYAxis, normalizedZAxis)).normalized();


		result_matrix.m[0][0] = result_vector.dotProduct(unit_x, normalizedXAxis); result_matrix.m[0][1] = result_vector.dotProduct(unit_x, normalizedYAxis); result_matrix.m[0][2] = result_vector.dotProduct(unit_x, normalizedZAxis); result_matrix.m[0][3] = tempHand.x();
		result_matrix.m[1][0] = result_vector.dotProduct(unit_y, normalizedXAxis); result_matrix.m[1][1] = result_vector.dotProduct(unit_y, normalizedYAxis); result_matrix.m[1][2] = result_vector.dotProduct(unit_y, normalizedZAxis); result_matrix.m[1][3] = tempHand.y();
		result_matrix.m[2][0] = result_vector.dotProduct(unit_z, normalizedXAxis); result_matrix.m[2][1] = result_vector.dotProduct(unit_z, normalizedYAxis); result_matrix.m[2][2] = result_vector.dotProduct(unit_z, normalizedZAxis); result_matrix.m[2][3] = tempHand.z();
		result_matrix.m[3][0] = 0.0f; result_matrix.m[3][1] = 0.0f; result_matrix.m[3][2] = 0.0f; result_matrix.m[3][3] = 1.0f;
		
		qDebug() << "ChECKING HOMOGENEOUS MATRIX FOR TASK POSITION: SPATIAL";
		qDebug() << "\t" << result_matrix.m[0][0] << "\t" << result_matrix.m[0][1] << "\t" << result_matrix.m[0][2] << "\t" << result_matrix.m[0][3];
		qDebug() << "\t" << result_matrix.m[1][0] << "\t" << result_matrix.m[1][1] << "\t" << result_matrix.m[1][2] << "\t" << result_matrix.m[1][3];
		qDebug() << "\t" << result_matrix.m[2][0] << "\t" << result_matrix.m[2][1] << "\t" << result_matrix.m[2][2] << "\t" << result_matrix.m[2][3];
		qDebug() << "\t" << result_matrix.m[3][0] << "\t" << result_matrix.m[3][1] << "\t" << result_matrix.m[3][2] << "\t" << result_matrix.m[3][3];


	}
	else if (type == 1)
	{
		
		QVector3D tempHand = hand;
		QVector3D tempHandTip = handtip;
		tempHand.setZ(1.0f);
		tempHandTip.setZ(1.0f);
		QVector3D normalizedYAxis = (tempHandTip - tempHand).normalized();
	
		
		QVector3D normalizedXAxis = (getRotatedVector(normalizedYAxis, -90.0f, QVector3D(0, 0, 1))).normalized();

		normalizedXAxis.setZ(1.0f); normalizedYAxis.setZ(1.0f);


		result_matrix.m[0][0] = result_vector.dotProduct(unit_x, normalizedXAxis); result_matrix.m[0][1] = result_vector.dotProduct(unit_x, normalizedYAxis); result_matrix.m[0][2] = 0.0f; result_matrix.m[0][3] = tempHand.x();
		result_matrix.m[1][0] = result_vector.dotProduct(unit_y, normalizedXAxis); result_matrix.m[1][1] = result_vector.dotProduct(unit_y, normalizedYAxis); result_matrix.m[1][2] = 0.0f; result_matrix.m[1][3] = tempHand.y();
		result_matrix.m[2][0] = 0.0f; result_matrix.m[2][1] = 0.0f; result_matrix.m[2][2] = 1.0f; result_matrix.m[2][3] = 1.0f;
		result_matrix.m[3][0] = 0.0f; result_matrix.m[3][1] = 0.0f; result_matrix.m[3][2] = 0.0f; result_matrix.m[3][3] = 1.0f;

		qDebug() << "ChECKING HOMOGENEOUS MATRIX FOR TASK POSITION: PLANAR";
		qDebug() << "\t" << result_matrix.m[0][0] << "\t" << result_matrix.m[0][1] << "\t" << result_matrix.m[0][2] << "\t" << result_matrix.m[0][3];
		qDebug() << "\t" << result_matrix.m[1][0] << "\t" << result_matrix.m[1][1] << "\t" << result_matrix.m[1][2] << "\t" << result_matrix.m[1][3];
		qDebug() << "\t" << result_matrix.m[2][0] << "\t" << result_matrix.m[2][1] << "\t" << result_matrix.m[2][2] << "\t" << result_matrix.m[2][3];
		qDebug() << "\t" << result_matrix.m[3][0] << "\t" << result_matrix.m[3][1] << "\t" << result_matrix.m[3][2] << "\t" << result_matrix.m[3][3];
	}
	return result_matrix;
}


/*Quaternion hMatrix::rotationMatrixToQuaternion(const hMatrix& rot_matrix)
{
	
	Quaternion result;
	result.q[3] = (rot_matrix.m[0][0] + rot_matrix.m[1][1] + rot_matrix.m[2][2] + 1.0f) / 4.0f;       // previously (-rot_matrix.m[0][0] + rot_matrix.m[1][1] + rot_matrix.m[2][2] + 1.0f) / 4.0f;
	result.q[0] = (rot_matrix.m[0][0] - rot_matrix.m[1][1] - rot_matrix.m[2][2] + 1.0f) / 4.0f;        // previously (rot_matrix.m[0][0] - rot_matrix.m[1][1] - rot_matrix.m[2][2] + 1.0f) / 4.0f;
	result.q[1] = (-rot_matrix.m[0][0] + rot_matrix.m[1][1] - rot_matrix.m[2][2] + 1.0f) / 4.0f;        // previously (rot_matrix.m[0][0] + rot_matrix.m[1][1] - rot_matrix.m[2][2] + 1.0f) / 4.0f;
	result.q[2] = (-rot_matrix.m[0][0] - rot_matrix.m[1][1] + rot_matrix.m[2][2] + 1.0f) / 4.0f;       // previously (-rot_matrix.m[0][0] - rot_matrix.m[1][1] + rot_matrix.m[2][2] + 1.0f) / 4.0f;
	

	if (result.q[0] < 0.0f)result.q[0] = 0.0f;
	if (result.q[1] < 0.0f)result.q[1] = 0.0f;
	if (result.q[2] < 0.0f)result.q[2] = 0.0f;
	if (result.q[3] < 0.0f)result.q[3] = 0.0f;

	result.q[3] = sqrt(result.q[3]);
	result.q[0] = sqrt(result.q[0]);
	result.q[1] = sqrt(result.q[1]);
	result.q[2] = sqrt(result.q[2]); 

	if (result.q[3] >= result.q[0] && result.q[3] >= result.q[1] && result.q[3] >= result.q[2])
	{
		result.q[3] *= +1.0f;
		result.q[0] *= SIGN(rot_matrix.m[2][1] - rot_matrix.m[1][2]);
		result.q[1] *= SIGN(rot_matrix.m[0][2] - rot_matrix.m[2][0]);
		result.q[2] *= SIGN(rot_matrix.m[1][0] - rot_matrix.m[0][1]);	
	}
	else if (result.q[0] >= result.q[3] && result.q[0] >= result.q[1] && result.q[0] >= result.q[2])
	{
		result.q[3] *= SIGN(rot_matrix.m[2][1] - rot_matrix.m[1][2]);
		result.q[0] *= +1.0f;
		result.q[1] *= SIGN(rot_matrix.m[1][0] + rot_matrix.m[0][1]);
		result.q[2] *= SIGN(rot_matrix.m[0][2] + rot_matrix.m[2][0]);
	}
	else if (result.q[1] >= result.q[3] && result.q[1] >= result.q[0] && result.q[1] >= result.q[2])
	{
		result.q[3] *= SIGN(rot_matrix.m[0][2] - rot_matrix.m[2][0]);
		result.q[0] *= SIGN(rot_matrix.m[1][0] + rot_matrix.m[0][1]);
		result.q[1] *= +1.0f;
		result.q[2] *= SIGN(rot_matrix.m[2][1] + rot_matrix.m[1][2]);
	}
	else if (result.q[2] >= result.q[3] && result.q[2] >= result.q[0] && result.q[2] >= result.q[1])
	{
		result.q[3] *= SIGN(rot_matrix.m[1][0] - rot_matrix.m[0][1]);
		result.q[0] *= SIGN(rot_matrix.m[2][0] + rot_matrix.m[0][2]);
		result.q[1] *= SIGN(rot_matrix.m[2][1] + rot_matrix.m[1][2]);
		result.q[2] *= +1.0f;
	}
	else{
		cout << "CALCULATION ERROR" << endl;
	}

	double N = result.Modulus();

	result.q[3] /= N;
	result.q[0] /= N;
	result.q[1] /= N;
	result.q[2] /= N;

	return (result);
}*/

Quaternion hMatrix::rotationMatrixToQuaternionEigen(const hMatrix& rotMat)
{
	Quaternion resultantQuat;
	Eigen::MatrixXd tempMat(3, 3);
	tempMat(0, 0) = rotMat.m[0][0];
	tempMat(0, 1) = rotMat.m[0][1];
	tempMat(0, 2) = rotMat.m[0][2];
	tempMat(1, 0) = rotMat.m[1][0];
	tempMat(1, 1) = rotMat.m[1][1];
	tempMat(1, 2) = rotMat.m[1][2];
	tempMat(2, 0) = rotMat.m[2][0];
	tempMat(2, 1) = rotMat.m[2][1];
	tempMat(2, 2) = rotMat.m[2][2];

	Eigen::MatrixXd m;
	m.setIdentity(3, 3);

	Eigen::EigenSolver<Eigen::MatrixXd> es(tempMat - m);

	vector<double> eigenValues;

	for (auto i = 0; i < es.eigenvalues().size(); i++)
	{
		eigenValues.emplace_back(abs(es.eigenvalues()[i].real()));
	}
	vector<size_t> idxs;
	idxs = sort_indexes(eigenValues); 
	int smallest_idx = idxs.at(0);
	
	vector<double> Axis;
	for (auto i = 0; i < es.eigenvectors().col(smallest_idx).size(); i++)
	{
		Axis.emplace_back(es.eigenvectors().col(smallest_idx)[i].real());
	}

	double twocosTheta = tempMat(0, 0) + tempMat(1, 1) + tempMat(2, 2) - 1;
	vector<double> twosinThetav = { tempMat(2, 1) - tempMat(1, 2), tempMat(0, 2) - tempMat(2, 0), tempMat(1, 0) - tempMat(0, 1) };

	double twosinTheta = 0.00;
	for (auto i = 0; i < Axis.size(); i++)
	{
		twosinTheta += (Axis.at(i)*twosinThetav.at(i));
	}


	double Theta = atan2(twosinTheta, twocosTheta);

	double x, y, z, w;
	x = Axis.at(0)*sin(Theta / 2);
	y = Axis.at(1)*sin(Theta / 2);
	z = Axis.at(2)*sin(Theta / 2);
	w = cos(Theta / 2);

	resultantQuat.q[0] = Axis.at(0)*sin(Theta / 2);
	resultantQuat.q[1] = Axis.at(1)*sin(Theta / 2);
	resultantQuat.q[2] = Axis.at(2)*sin(Theta / 2);
	resultantQuat.q[3] = cos(Theta / 2);

	return resultantQuat;
}

DualQuaternion hMatrix::rotationMatrixToDualQuaternionEigen(const hMatrix& rotMat)
{
	
	Eigen::MatrixXd tempMat(3, 3);
	tempMat(0, 0) = rotMat.m[0][0];
	tempMat(0, 1) = rotMat.m[0][1];
	tempMat(0, 2) = rotMat.m[0][2];
	tempMat(1, 0) = rotMat.m[1][0];
	tempMat(1, 1) = rotMat.m[1][1];
	tempMat(1, 2) = rotMat.m[1][2];
	tempMat(2, 0) = rotMat.m[2][0];
	tempMat(2, 1) = rotMat.m[2][1];
	tempMat(2, 2) = rotMat.m[2][2];

	Eigen::MatrixXd m;
	m.setIdentity(3, 3);

	Eigen::EigenSolver<Eigen::MatrixXd> es(tempMat - m);

	vector<double> eigenValues;

	for (auto i = 0; i < es.eigenvalues().size(); i++)
	{
		eigenValues.emplace_back(abs(es.eigenvalues()[i].real()));
	}
	vector<size_t> idxs;
	idxs = sort_indexes(eigenValues);
	int smallest_idx = idxs.at(0);
	
	vector<double> Axis;
	for (auto i = 0; i < es.eigenvectors().col(smallest_idx).size(); i++)
	{
		Axis.emplace_back(es.eigenvectors().col(smallest_idx)[i].real());
	}

	double twocosTheta = tempMat(0, 0) + tempMat(1, 1) + tempMat(2, 2) - 1;
	vector<double> twosinThetav = { tempMat(2, 1) - tempMat(1, 2), tempMat(0, 2) - tempMat(2, 0), tempMat(1, 0) - tempMat(0, 1) };

	double twosinTheta = 0.00;
	for (auto i = 0; i < Axis.size(); i++)
	{
		twosinTheta += (Axis.at(i)*twosinThetav.at(i));
	}

	
	double Theta = atan2(twosinTheta, twocosTheta);
	

	double x, y, z, w;
	x = Axis.at(0)*sin(Theta / 2);
	y = Axis.at(1)*sin(Theta / 2);
	z = Axis.at(2)*sin(Theta / 2);
	w = cos(Theta / 2);

	Quaternion realPart;

	realPart.q[0] = Axis.at(0)*sin(Theta / 2);
	realPart.q[1] = Axis.at(1)*sin(Theta / 2);
	realPart.q[2] = Axis.at(2)*sin(Theta / 2);
	realPart.q[3] = cos(Theta / 2);

	
	double Z1 = ((rotMat.m[0][3] / 2)*cos(Theta / 2)) + ((rotMat.m[1][3] / 2)*sin(Theta / 2));
	double Z2 = ((rotMat.m[1][3] / 2)*cos(Theta / 2)) - ((rotMat.m[0][3] / 2)*sin(Theta / 2));

	Quaternion dualPart(Z1, Z2, 0, 0);

	DualQuaternion resultantDQuat(realPart, dualPart);
	

	return resultantDQuat;
}


DualQuaternion hMatrix::homogeneousMatrixToDualQuaternion(const hMatrix &rot_matrix,CAPTURETYPE type){
	
	DualQuaternion resultantDq;
	if (type == SPATIAL)
	{
		Quaternion resultantQuat;
		
		resultantQuat=rotationMatrixToQuaternionEigen(rot_matrix);
		
		Vector translation;
		translation.coord[0] = rot_matrix.m[0][3]; translation.coord[1] = rot_matrix.m[1][3];
		translation.coord[2] = rot_matrix.m[2][3]; translation.coord[3] = 0.0;
		DualQuaternion spatialDq(resultantQuat, translation);
		resultantDq= spatialDq;
		
	}
	else if (type == PLANAR)
	{
		DualQuaternion planarDq = rotationMatrixToDualQuaternionEigen(rot_matrix);
		resultantDq = planarDq;
	}
	
	return resultantDq;
}

DualQuaternion hMatrix::dualQuaternionForLimbs(const QVector3D& start, const QVector3D& end, CAPTURETYPE type)
{
	DualQuaternion resultantDq;
	if (type == SPATIAL)
	{
		QVector3D Limb = end - start;
		float limbLen = (end - start).length();
		QVector3D normalizedLimb = Limb.normalized();
		QVector3D originalLimb(0.0f, 0.0f, 1.0f);
		QVector3D temp;
		QVector3D rotAxis = (temp.crossProduct(originalLimb, normalizedLimb)).normalized();

		double theta = acos(temp.dotProduct(originalLimb, normalizedLimb));
		Quaternion rotQuat = QuaternionFromAxisAngle(rotAxis, theta);

		Vector translation;
		translation.coord[0] = start.x(); translation.coord[1] = start.y(); 
		translation.coord[2] = start.z(); translation.coord[3] = 0.00;

		DualQuaternion spatialDq(rotQuat, translation);
		resultantDq = spatialDq;
	}
	else if (type==PLANAR)
	{
		QVector3D star = start;
		QVector3D en = end;

		star.setZ(0.0f);
		en.setZ(0.0f);

		QVector3D Limb = en - star;
		float limbLen = (en - star).length();
		QVector3D normalizedLimb = Limb.normalized();
		QVector3D originalLimb(1.0f, 0.0f, 0.0f);
		QVector3D temp;
		QVector3D rotAxis = (temp.crossProduct(originalLimb, normalizedLimb)).normalized();


		double theta = acos(temp.dotProduct(originalLimb, normalizedLimb));
		Quaternion rotQuat = QuaternionFromAxisAngle(rotAxis, theta);

		Vector translation;
		translation.coord[0] = star.x(); translation.coord[1] = star.y();
		translation.coord[2] = star.z(); translation.coord[3] = 0.00;

		DualQuaternion planarDq(rotQuat, translation);

		resultantDq = planarDq;
	}

	return resultantDq;
}

hMatrix hMatrix::homogeneousMatrixForLimbs(const QVector3D& knee, const QVector3D& ankle, const QVector3D& newKnee,CAPTURETYPE type)
{
	hMatrix result_matrix;
	if (type == SPATIAL)
	{
		QVector3D Limb = ankle - knee;
		float limbLen = (ankle - knee).length();
		QVector3D normalizedLimb = Limb.normalized();
		QVector3D originalLimb(0.0f, 0.0f, 1.0f);
		QVector3D temp;

		QVector3D rotAxis = (temp.crossProduct(originalLimb, normalizedLimb)).normalized();


		double theta = acos(temp.dotProduct(originalLimb, normalizedLimb));
		//qDebug() << "Rotation Axis" << "\t" << rotAxis;
		//qDebug() << "Checking theta" << "\t" << theta;


		Quaternion quat = QuaternionFromAxisAngle(rotAxis, theta);
		//qDebug() << "Quaternion" << "\t" << quat.q[0] << "\t" << quat.q[1] << "\t" << quat.q[2] << "\t" << quat.q[3];
		Matrix quatToRotMat = (quat.quaternionToRotationMatrix());



		result_matrix.m[0][0] = quatToRotMat.m[0][0]; result_matrix.m[0][1] = quatToRotMat.m[0][1]; result_matrix.m[0][2] = quatToRotMat.m[0][2]; result_matrix.m[0][3] = newKnee.x();
		result_matrix.m[1][0] = quatToRotMat.m[1][0]; result_matrix.m[1][1] = quatToRotMat.m[1][1]; result_matrix.m[1][2] = quatToRotMat.m[1][2]; result_matrix.m[1][3] = newKnee.y();
		result_matrix.m[2][0] = quatToRotMat.m[2][0]; result_matrix.m[2][1] = quatToRotMat.m[2][1]; result_matrix.m[2][2] = quatToRotMat.m[2][2]; result_matrix.m[2][3] = newKnee.z();
		result_matrix.m[3][0] = 0.0f; result_matrix.m[3][1] = 0.0f; result_matrix.m[3][2] = 0.0f; result_matrix.m[3][3] = 1.0f;
	}
	else if (type == PLANAR)
	{
		QVector3D strt = knee;
		QVector3D en = ankle;

		strt.setZ(0.0f);
		en.setZ(0.0f);

		//QVector3D Limb = ankle - knee;
		QVector3D Limb = en - strt;
		//float limbLen = (ankle - knee).length();
		QVector3D normalizedLimb = Limb.normalized();
		QVector3D originalLimb(1.0f, 0.0f, 0.0f);
		QVector3D temp;

		QVector3D rotAxis = (temp.crossProduct(originalLimb, normalizedLimb)).normalized();


		double theta = acos(temp.dotProduct(originalLimb, normalizedLimb));
		//qDebug() << "Rotation Axis" << "\t" << rotAxis;
		//qDebug() << "Checking theta" << "\t" << theta;


		Quaternion quat = QuaternionFromAxisAngle(rotAxis, theta);
		//qDebug() << "Quaternion" << "\t" << quat.q[0] << "\t" << quat.q[1] << "\t" << quat.q[2] << "\t" << quat.q[3];
		Matrix quatToRotMat = (quat.quaternionToRotationMatrix());


		result_matrix.m[0][0] = quatToRotMat.m[0][0]; result_matrix.m[0][1] = quatToRotMat.m[0][1]; result_matrix.m[0][2] = 0.0f; result_matrix.m[0][3] = newKnee.x();
		result_matrix.m[1][0] = quatToRotMat.m[1][0]; result_matrix.m[1][1] = quatToRotMat.m[1][1]; result_matrix.m[1][2] = 0.0f; result_matrix.m[1][3] = newKnee.y();
		result_matrix.m[2][0] = 0.0f; result_matrix.m[2][1] = 0.0f; result_matrix.m[2][2] = 1.0f; result_matrix.m[2][3] = 0.0f;
		result_matrix.m[3][0] = 0.0f; result_matrix.m[3][1] = 0.0f; result_matrix.m[3][2] = 0.0f; result_matrix.m[3][3] = 1.0f;

		/*result_matrix.m[0][0] = quatToRotMat.m[0][0]; result_matrix.m[0][1] = quatToRotMat.m[0][1]; result_matrix.m[0][2] = quatToRotMat.m[0][2]; result_matrix.m[0][3] = newKnee.x();
		result_matrix.m[1][0] = quatToRotMat.m[1][0]; result_matrix.m[1][1] = quatToRotMat.m[1][1]; result_matrix.m[1][2] = quatToRotMat.m[1][2]; result_matrix.m[1][3] = newKnee.y();
		result_matrix.m[2][0] = quatToRotMat.m[2][0]; result_matrix.m[2][1] = quatToRotMat.m[2][1]; result_matrix.m[2][2] = quatToRotMat.m[2][2]; result_matrix.m[2][3] = newKnee.z();
		result_matrix.m[3][0] = 0.0f; result_matrix.m[3][1] = 0.0f; result_matrix.m[3][2] = 0.0f; result_matrix.m[3][3] = 1.0f;*/
	}
	return result_matrix;
}


Quaternion hMatrix::QuaternionFromAxisAngle(const QVector3D& axis,const double& angle)
{
	Quaternion result;
	
	result.q[0] = axis.x()*sin(angle / 2);
	result.q[1] = axis.y()*sin(angle / 2);
	result.q[2] = axis.z()*sin(angle / 2);
	result.q[3] = cos(angle / 2);

	return result;
}



QVector3D hMatrix::getRotatedVector(const QVector3D& toRotate,float angle,QVector3D axis)
{
	QVector3D ax = axis.normalized();
	float an = static_cast<float>(angle*DEG_TO_RAD);

	float sina = sin(an);
	float cosa = cos(an);
	float cosb = 1.0 - cosa;

	return QVector3D(toRotate.x()*(ax.x()*ax.x()*cosb + cosa)
		+ toRotate.y()*(ax.x()*ax.y()*cosb - ax.z()*sina)
		+ toRotate.z()*(ax.x()*ax.z()*cosb + ax.y()*sina),
		toRotate.x()*(ax.y()*ax.x()*cosb + ax.z()*sina)
		+ toRotate.y()*(ax.y()*ax.y()*cosb + cosa)
		+ toRotate.z()*(ax.y()*ax.z()*cosb - ax.y()*sina),
		toRotate.x()*(ax.z()*ax.x()*cosb - ax.y()*sina)
		+ toRotate.y()*(ax.z()*ax.y()*cosb + ax.x()*sina)
		+ toRotate.z()*(ax.z()*ax.z()*cosb + cosa));
}



Matrix::Matrix()
{
	row = 0;
	column = 0;

	m = 0;
}

Matrix::Matrix( int a, int b )
{
	row = a;
	column = b;

	m = new double*[a];
	for( int i = 0; i < a; i ++ )	m[i] = new double[b];
}

Matrix::Matrix( const Matrix& mm )
{
	row = mm.row;
	column = mm.column;

	m = new double*[row];
	for( int i = 0; i < row; i ++ )	m[i] = new double[column];

	for(int i = 0; i < row; i ++ )
		for( int j = 0; j < column; j ++ )
			m[i][j] = mm.m[i][j];
}

Matrix::~Matrix( )
{
	if( m != 0 )
	{
		for( int i = 0; i < row; i ++ )
			delete[] m[i];

		delete[] m;
	}
}

Matrix operator+( const Matrix& arg1, const Matrix& arg2 )
{
	Matrix sum(arg1.row, arg1.column);

	if( arg1.row != arg2.row || arg1.column != arg2.column )	return sum;

	for( int i = 0; i < arg1.row; i ++ )
		for( int j = 0; j < arg1.column; j ++ )
			sum.m[i][j] = arg1.m[i][j] + arg2.m[i][j];

	return sum;
}

Matrix operator-( const Matrix& arg1, const Matrix& arg2 )
{
	Matrix sum(arg1.row, arg1.column);

	if( arg1.row != arg2.row || arg1.column != arg2.column )	return sum;

	for( int i = 0; i < arg1.row; i ++ )
		for( int j = 0; j < arg1.column; j ++ )
			sum.m[i][j] = arg1.m[i][j] - arg2.m[i][j];

	return sum;
}

Matrix operator*( const Matrix& arg1, const Matrix& arg2 )
{
	Matrix mul(arg1.row, arg2.column);

	if( arg1.column != arg2.row )	return mul;

	mul.Clear();

	for( int i = 0; i < arg1.row; i ++ )
		for( int j = 0; j < arg2.column; j ++ )
			for( int k = 0; k < arg1.column; k ++ )		
				mul.m[i][j] += arg1.m[i][k] * arg2.m[k][j];

	return mul;
}

Matrix operator*( const Matrix& arg1, const double arg2 )
{
	Matrix sum(arg1.row, arg1.column);

	for( int i = 0; i < arg1.row; i ++ )
		for( int j = 0; j < arg1.column; j ++ )
			sum.m[i][j] = arg1.m[i][j] * arg2;

	return sum;
}

Matrix operator*( const double arg1, const Matrix& arg2 )
{
	Matrix sum( arg2.row, arg2.column );

	for( int i = 0; i < arg2.row; i ++ )
		for( int j = 0; j < arg2.column; j ++ )
			sum.m[i][j] = arg1 * arg2.m[i][j];

	return sum;
}

Matrix operator/( const Matrix& arg1, const double arg2 )
{
	Matrix sum(arg1.row, arg1.column);

	for( int i = 0; i < arg1.row; i ++ )
		for( int j = 0; j < arg1.column; j ++ )
			sum.m[i][j] = arg1.m[i][j] / arg2;

	return sum;
}

void Matrix::operator=( const Matrix& arg1 )
{
	DeleteSpace( );
	row = arg1.row;
	column = arg1.column;
	MallocSpace( );

	for( int i = 0; i < row; i ++ )
		for( int j = 0; j < column; j ++ )
			m[i][j] = arg1.m[i][j];

	return;
}

Matrix Matrix::Inverse( )
{
	Matrix inv(row, column);

	if( row != column )	return inv;

	int *is, *js, i, j, k, l, u, v, n;
	double d, p, *a;

	n = row;

	a = new double[n*n];

	for( i = 0; i < n ; i ++ )
		for( j = 0; j < n; j ++ )
			a[i*n+j] = m[i][j];

	is = new int[n];
	js = new int[n];

	for( k = 0; k <= n-1; k ++ )
	{
		d = 0.0f;
		for( i = k; i <= n-1; i ++ )
			for( j = k; j <= n-1; j ++ )
			{
				l = i * n + j;
				p = fabs( a[l] );
				if( p > d )
				{
					d = p; 
					is[k] = i;
					js[k] = j;
				}
			}

		if( d + 1.0f == 1.0f )
		{
			delete[] is;
			delete[] js;

			return inv;
		}

		if( is[k] != k )
			for( j = 0; j <= n -1; j ++ )
			{
				u = k * n + j;
				v = is[k] * n + j;
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}

		if( js[k] != k )
			for( i = 0; i <= n-1; i ++ )
			{
				u = i * n + k;
				v = i * n + js[k];
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}

		l = k * n + k;
		a[l] = 1.0f / a[l];

		for( j = 0; j <= n-1; j ++ )
			if( j != k )
			{
				u = k * n + j;
				a[u] = a[u] * a[l];
			}

		for( i = 0; i <= n-1; i ++ )
			if( i !=k )
				for( j = 0; j <= n-1; j ++ )
					if( j != k )
					{
						u = i * n + j;
						a[u] = a[u] - a[i*n+k] * a[k*n+j];
					}

		for( i = 0; i <= n-1; i ++ )
			if( i != k )
			{
				u = i * n + k;
				a[u] = -a[u] * a[l];
			}
	}

	for( k = n-1; k >= 0; k -- )
	{
		if( js[k] != k )
			for( j = 0; j <= n-1; j ++ )
			{
				u = k * n + j;
				v = js[k] * n + j;
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}

		if( is[k] != k )
			for( i = 0; i <= n-1; i ++ )
			{
				u = i * n + k;
				v = i * n + is[k];
				p = a[u];
				a[u] = a[v];
				a[v] = p;
			}
	}

	for( i = 0; i < n; i ++ )
		for( j = 0; j < n; j ++ )
			inv.m[i][j] = a[i*n+j];
		
	delete[] is;
	delete[] js;
	delete[] a;

	return inv;
}

Matrix Matrix::Transpose(  )
{	
	Matrix arg1(column, row);

	for( int i = 0; i < row; i ++ )
		for( int j = 0; j < column; j ++ )
			arg1.m[j][i] = m[i][j];

	return arg1;
}

void Matrix::Clear( )
{
	for( int i = 0; i < row; i ++ )
		for( int j = 0; j < column; j ++ )
			m[i][j] = 0.0;

	return;
}

bool Matrix::MallocSpace( )
{
	if( row <=0 || column <= 0 )
		return false;

	m = new double*[row];
	for( int i = 0; i < row; i ++ )
		m[i] = new double[column];

	return true;
}

void Matrix::DeleteSpace( )
{
	if( m!= 0 )
	{
		for( int i = 0; i < row; i ++ )
			delete[] m[i];
		delete[] m;

		row = 0;
		column = 0;

		m = 0;
	}

	return;
}

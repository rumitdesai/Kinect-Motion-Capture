// hmatrix.h defines all operations of a homogeneous matrix

#ifndef HMATRIX_H
#define HMATRIX_H

#include <iostream>
#include <QtGui/QVector3D>
#include <QtGui/QVector2D>

#ifndef PI
	#define PI   3.141592653589
#endif // !PI

#ifndef DEG_TO_RAD
	#define DEG_TO_RAD  (PI/180.0)
#endif // !DEG_TO_RAD



enum CAPTURETYPE{ SPATIAL, PLANAR};
using namespace std;

class hPoint;
class Point;
class Quaternion;
class DualQuaternion;
// 4*4 homogeneous matrix
class hMatrix
{
public:
	double m[4][4];

public:
	hMatrix();
	friend hMatrix operator+( const hMatrix&, const hMatrix& );
	friend hMatrix operator-( const hMatrix&, const hMatrix& );
	friend hMatrix operator*( const hMatrix&, const hMatrix& );
	friend hMatrix operator*( const hMatrix&, const double );
	friend hMatrix operator*( const double, const hMatrix& );
	friend hMatrix operator/( const hMatrix&, const double );
	friend hPoint operator*( const hMatrix&, const hPoint& ); 
	friend Point  operator*( const hMatrix&, const Point& );
	friend ostream &operator<<(ostream&, hMatrix&);
	hMatrix& operator=( hMatrix ); 


	
	
	Quaternion rotationMatrixToQuaternionEigen(const hMatrix&);
	DualQuaternion rotationMatrixToDualQuaternionEigen(const hMatrix&);
	DualQuaternion homogeneousMatrixToDualQuaternion(const hMatrix&, CAPTURETYPE);

	hMatrix homogeneousMatrix(QVector3D hand, QVector3D handtip, QVector3D handthumb,CAPTURETYPE type);
	QVector3D getRotatedVector(const QVector3D& toRotate,float angle,QVector3D axis);

	hMatrix homogeneousMatrixForJoints(const QVector3D&,CAPTURETYPE);
	
	DualQuaternion dualQuaternionForLimbs(const QVector3D& start, const QVector3D& end, CAPTURETYPE type);
	hMatrix homogeneousMatrixForLimbs(const QVector3D& hip, const QVector3D& knee, const QVector3D& newKnee,CAPTURETYPE type);
	Quaternion QuaternionFromAxisAngle(const QVector3D& axis,const double& angle);


	void   Clear();
	hMatrix Inverse( );		// return the inverse of a 4*4 matrix
	hMatrix transpose();

};

// any arbitrary matrix
class Matrix
{
public:
	int row;
	int column;
	double **m;

public:
	Matrix();
	Matrix( int, int );
	Matrix( const Matrix& );
	~Matrix( );
	friend Matrix operator+( const Matrix&, const Matrix& );
	friend Matrix operator-( const Matrix&, const Matrix& );
	friend Matrix operator*( const Matrix&, const Matrix& );
	friend Matrix operator*( const Matrix&, const double );
	friend Matrix operator*( const double, const Matrix& );
	friend Matrix operator/( const Matrix&, const double );
	void   operator=( const Matrix& );
	void   Clear();
	Matrix Inverse(  );
	Matrix Transpose( );

	bool   MallocSpace( );
	void   DeleteSpace( );
};


#endif
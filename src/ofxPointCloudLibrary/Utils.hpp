#pragma once

#include <glm/gtx/matrix_decompose.hpp>
#include <ofxPointCloudLibrary/Common.hpp>
#include <ofxPointCloudLibrary/Types.hpp>

#include "ofMain.h"

namespace ofxPointCloudLibrary {

// -------------------------
// oF <--> Eigen conversions
// -------------------------

// convert eigen 4x4 matrix -> glm 4x4 matrix
inline glm::mat4 toOf( const Eigen::Matrix4f& mat )
{
	// transpose
	glm::mat4 res;
	for ( int i = 0; i < 4; ++i )
		for ( int j = 0; j < 4; ++j )
			res[j][i] = mat( i, j );
	return res;
}

// convert glm 4x4 matrix -> eigen 4x4 matrix
inline Eigen::Matrix4f toPcl( const glm::mat4& mat )
{
	// transpose
	Eigen::Matrix4f res;
	for ( int i = 0; i < 4; ++i )
		for ( int j = 0; j < 4; ++j )
			res( j, i ) = mat[i][j];
	return res;
}

// convert eigen Quaternion -> glm quat
inline glm::quat toOf( const Eigen::Quaternionf& quat )
{
	return { quat.x(), quat.y(), quat.z(), quat.w() };
}

// convert glm quat -> eigen Quaternion
inline Eigen::Quaternionf toPcl( const glm::quat& quat )
{
	// Eigen's quaternion constructor is ordered w x y z
	return { quat.w, quat.x, quat.y, quat.z };
}

//
// custom matrix decompose, to fix bugs in GLM version
//
inline bool decomposeTransform( const glm::mat4& transformMatrix, glm::vec3& scaleOut, glm::quat& orientationOut, glm::vec3& translationOut, glm::vec3& skewOut, glm::vec4& perspectiveOut )
{

	// this function combines:
	// GLM 0.9.9.8: https://github.com/g-truc/glm/blob/0.9.9.8/glm/gtx/matrix_decompose.inl
	// + SO fix: https://stackoverflow.com/a/56587367/5195277

	using namespace glm;
	mat4 LocalMatrix( transformMatrix );

	// Normalize the matrix.
	if ( epsilonEqual( LocalMatrix[3][3], static_cast<float>( 0 ), epsilon<float>() ) )
		return false;

	for ( length_t i = 0; i < 4; ++i )
		for ( length_t j = 0; j < 4; ++j )
			LocalMatrix[i][j] /= LocalMatrix[3][3];

	// perspectiveMatrix is used to solve for perspective, but it also provides
	// an easy way to test for singularity of the upper 3x3 component.
	mat4 PerspectiveMatrix( LocalMatrix );

	for ( length_t i = 0; i < 3; i++ )
		PerspectiveMatrix[i][3] = static_cast<float>( 0 );
	PerspectiveMatrix[3][3] = static_cast<float>( 1 );

	/// TODO: Fixme!
	if ( epsilonEqual( determinant( PerspectiveMatrix ), static_cast<float>( 0 ), epsilon<float>() ) )
		return false;

	// First, isolate perspective.  This is the messiest.
	if (
	    epsilonNotEqual( LocalMatrix[0][3], static_cast<float>( 0 ), epsilon<float>() ) ||
	    epsilonNotEqual( LocalMatrix[1][3], static_cast<float>( 0 ), epsilon<float>() ) ||
	    epsilonNotEqual( LocalMatrix[2][3], static_cast<float>( 0 ), epsilon<float>() ) ) {
		// rightHandSide is the right hand side of the equation.
		vec4 RightHandSide;
		RightHandSide[0] = LocalMatrix[0][3];
		RightHandSide[1] = LocalMatrix[1][3];
		RightHandSide[2] = LocalMatrix[2][3];
		RightHandSide[3] = LocalMatrix[3][3];

		// Solve the equation by inverting PerspectiveMatrix and multiplying
		// rightHandSide by the inverse.  (This is the easiest way, not
		// necessarily the best.)
		mat4 InversePerspectiveMatrix           = glm::inverse( PerspectiveMatrix );           //   inverse(PerspectiveMatrix, inversePerspectiveMatrix);
		mat4 TransposedInversePerspectiveMatrix = glm::transpose( InversePerspectiveMatrix );  //   transposeMatrix4(inversePerspectiveMatrix, transposedInversePerspectiveMatrix);

		perspectiveOut = TransposedInversePerspectiveMatrix * RightHandSide;
		//  v4MulPointByMatrix(rightHandSide, transposedInversePerspectiveMatrix, perspectivePoint);

		// Clear the perspective partition
		LocalMatrix[0][3] = LocalMatrix[1][3] = LocalMatrix[2][3] = static_cast<float>( 0 );
		LocalMatrix[3][3]                                         = static_cast<float>( 1 );
	} else {
		// No perspective.
		perspectiveOut = vec4( 0, 0, 0, 1 );
	}

	// Next take care of translation (easy).
	translationOut = vec3( LocalMatrix[3] );
	LocalMatrix[3] = vec4( 0, 0, 0, LocalMatrix[3].w );

	vec3 Row[3], Pdum3;

	// Now get scale and shear.
	for ( length_t i = 0; i < 3; ++i )
		for ( length_t j = 0; j < 3; ++j )
			Row[i][j] = LocalMatrix[i][j];

	// Compute X scale factor and normalize first row.
	scaleOut.x = length( Row[0] );  // v3Length(Row[0]);

	Row[0] = detail::scale( Row[0], static_cast<float>( 1 ) );

	// Compute XY shear factor and make 2nd row orthogonal to 1st.
	skewOut.z = dot( Row[0], Row[1] );
	Row[1]    = detail::combine( Row[1], Row[0], static_cast<float>( 1 ), -skewOut.z );

	// Now, compute Y scale and normalize 2nd row.
	scaleOut.y = length( Row[1] );
	Row[1]     = detail::scale( Row[1], static_cast<float>( 1 ) );
	skewOut.z /= scaleOut.y;

	// Compute XZ and YZ shears, orthogonalize 3rd row.
	skewOut.y = glm::dot( Row[0], Row[2] );
	Row[2]    = detail::combine( Row[2], Row[0], static_cast<float>( 1 ), -skewOut.y );
	skewOut.x = glm::dot( Row[1], Row[2] );
	Row[2]    = detail::combine( Row[2], Row[1], static_cast<float>( 1 ), -skewOut.x );

	// Next, get Z scale and normalize 3rd row.
	scaleOut.z = length( Row[2] );
	Row[2]     = detail::scale( Row[2], static_cast<float>( 1 ) );
	skewOut.y /= scaleOut.z;
	skewOut.x /= scaleOut.z;

	// At this point, the matrix (in rows[]) is orthonormal.
	// Check for a coordinate system flip.  If the determinant
	// is -1, then negate the matrix and the scaling factors.
	Pdum3 = cross( Row[1], Row[2] );  // v3Cross(row[1], row[2], Pdum3);
	if ( dot( Row[0], Pdum3 ) < 0 ) {
		for ( length_t i = 0; i < 3; i++ ) {
			scaleOut[i] *= static_cast<float>( -1 );
			Row[i] *= static_cast<float>( -1 );
		}
	}

	// Now, get the rotations out, as described in the gem.

	// FIXME - Add the ability to return either quaternions (which are
	// easier to recompose with) or Euler angles (rx, ry, rz), which
	// are easier for authors to deal with. The latter will only be useful
	// when we fix https://bugs.webkit.org/show_bug.cgi?id=23799, so I
	// will leave the Euler angle code here for now.

	// ret.rotateY = asin(-Row[0][2]);
	// if (cos(ret.rotateY) != 0) {
	//     ret.rotateX = atan2(Row[1][2], Row[2][2]);
	//     ret.rotateZ = atan2(Row[0][1], Row[0][0]);
	// } else {
	//     ret.rotateX = atan2(-Row[2][0], Row[1][1]);
	//     ret.rotateZ = 0;
	// }

	int i, j, k = 0;
	float root, trace = Row[0].x + Row[1].y + Row[2].z;
	if ( trace > static_cast<float>( 0 ) ) {
		root             = sqrt( trace + static_cast<float>( 1.0 ) );
		orientationOut.w = static_cast<float>( 0.5 ) * root;
		root             = static_cast<float>( 0.5 ) / root;

		//orientationOut.x = root * ( Row[1].z - Row[2].y );
		//orientationOut.y = root * ( Row[2].x - Row[0].z );
		//orientationOut.z = root * ( Row[0].y - Row[1].x );
		// FIX - see SO post: https://stackoverflow.com/a/56587367/5195277
		orientationOut.x = root * ( Row[2].y - Row[1].z );
		orientationOut.y = root * ( Row[0].z - Row[2].x );
		orientationOut.z = root * ( Row[1].x - Row[0].y );

	}  // End if > 0
	else {
		static int Next[3] = { 1, 2, 0 };
		i                  = 0;
		if ( Row[1].y > Row[0].x ) i = 1;
		if ( Row[2].z > Row[i][i] ) i = 2;
		j = Next[i];
		k = Next[j];

		root = sqrt( Row[i][i] - Row[j][j] - Row[k][k] + static_cast<float>( 1.0 ) );

		orientationOut[i] = static_cast<float>( 0.5 ) * root;
		root              = static_cast<float>( 0.5 ) / root;
		orientationOut[j] = root * ( Row[i][j] + Row[j][i] );
		orientationOut[k] = root * ( Row[i][k] + Row[k][i] );
		orientationOut.w  = root * ( Row[j][k] - Row[k][j] );
	}  // End if <= 0

	return true;
}

}  // namespace ofxPointCloudLibrary

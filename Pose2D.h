/**
 * @file Pose2D.h
 * Contains class Pose2D
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#ifndef __Pose2D_h__
#define __Pose2D_h__

#include <vector>
#include "math/Vector2.h"
#include "math/Vector3.h"
#include "math/Range.h"
#include "math/Common.h"

namespace itandroids_lib {
namespace math {

/** representation for 2D Transformation and Position (Location + Orientation)*/
class Pose2D {
public:

	/** Rotation as an angle*/
	double rotation;

	/** translation as an vector2*/
	Vector2<double> translation;

	Pose2D(Vector3<double> vector) {
		translation.x = vector.x;
		translation.y = vector.y;
		rotation = vector.z;
	}

	/** noargs-constructor*/
	Pose2D() :
			rotation(0), translation(0, 0) {
	}

	/** constructor from rotation and translation
	 * \param rotation rotation (double)
	 * \param translation translation (Vector2)
	 */
	Pose2D(const double rotation, const Vector2<double>& translation) :
			rotation(rotation), translation(translation) {
	}

	/** constructor from rotation and translation
	 * \param rot rotation (double)
	 * \param x translation.x (double)
	 * \param y translation.y (double)
	 */
	Pose2D(const double rot, const double x, const double y) :
			rotation(rot), translation(x, y) {
	}

	/** constructor from rotation
	 * \param rotation rotation (double)
	 */
	Pose2D(const double rotation) :
			rotation(rotation), translation(0, 0) {
	}

	/** constructor from translation
	 * \param translation translation (Vector2)
	 */
	Pose2D(const Vector2<double>& translation) :
			rotation(0), translation(translation) {
	}

	/** constructor from translation
	 * \param translation translation (Vector2)
	 */
	Pose2D(const Vector2<int>& translation) :
			rotation(0), translation((double) translation.x,
					(double) translation.y) {
	}

	/** constructor from two translation values
	 * \param x translation x component
	 * \param y translation y component
	 */
	Pose2D(const double x, const double y) :
			rotation(0), translation(x, y) {
	}

	/** set using vector of doubles as x,y,rot */
	void setValues(std::vector<double> vals) {
		if (vals.size() != 3)
			return;
		translation.x = vals[0];
		translation.y = vals[1];
		rotation = vals[2];
	}

	/** get the Angle
	 * @return Angle the Angle which defines the rotation
	 */
	inline double getAngle() const {
		return rotation;
	}

	/** set rotation from Angle
	 * @return the new Pose2D
	 */
	inline Pose2D fromAngle(const double a) {
		rotation = a;
		return *this;
	}

	/** get the cos of the angle
	 * @return the cos of the angle
	 */
	inline double getCos() const {
		return cosf(rotation);
	}

	/** get the sin of the angle
	 * @return the sin of the angle
	 */
	inline double getSin() const {
		return sinf(rotation);
	}

	/** Assignment operator
	 *\param other The other Pose2D that is assigned to this one
	 *\return A reference to this object after the assignment.
	 */
	Pose2D& operator=(const Pose2D& other) {
		rotation = other.rotation;
		translation = other.translation;
		return *this;
	}

	/** Copy constructor
	 *\param other The other vector that is copied to this one
	 */
	Pose2D(const Pose2D& other) {
		*this = other;
	}

	/** Multiplication of a Vector2 with this Pose2D
	 *\param point The Vector2 that will be multiplicated with this Pose2D
	 *\return The resulting Vector2
	 */

	Vector2<double> operator*(const Vector2<double>& point) const {
		double s = sinf(rotation);
		double c = cosf(rotation);
		return (Vector2<double>(point.x * c - point.y * s,
				point.x * s + point.y * c) + translation);
	}

	Pose2D operator*(double scalar) {
		Pose2D res(*this);
		res.translation *= scalar;
		res.rotation *= scalar;
		return res;
	}

	/** Comparison of another pose with this one.
	 *\param other The other pose that will be compared to this one
	 *\return Whether the two poses are equal.
	 */
	bool operator==(const Pose2D& other) const {
		return ((translation == other.translation)
				&& (rotation == other.rotation));
	}

	/** Comparison of another pose with this one.
	 *\param other The other pose that will be compared to this one
	 *\return Whether the two poses are unequal.
	 */
	bool operator!=(const Pose2D& other) const {
		return !(*this == other);
	}

	/**Concatenation of this pose with another pose.
	 *\param other The other pose that will be concatenated to this one.
	 *\return A reference to this pose after concatenation.
	 */
	Pose2D& operator+=(const Pose2D& other) {
		translation = *this * other.translation;
		rotation += other.rotation;
		rotation = normalize(rotation);
		return *this;
	}

	/**A concatenation of this pose and another pose.
	 *\param other The other pose that will be concatenated to this one.
	 *\return The resulting pose.
	 */
	Pose2D operator+(const Pose2D& other) const {
		Pose2D result;
		result.translation = this->translation + other.translation;
		result.rotation = this->rotation + other.rotation;
		return result;
		//return Pose2D(*this) += other;
	}

	/**Subtracts a difference pose from this one to get the source pose. So if A+B=C is the addition/concatenation, this calculates C-B=A.
	 *\param diff The difference Pose2D that shall be subtracted.
	 *\return The resulting source pose. Adding diff to it again would give the this pose back.
	 */
	Pose2D minusDiff(const Pose2D& diff) const {
		double rot = rotation - diff.rotation;
		double s = sinf(rot);
		double c = cosf(rot);
		return Pose2D(rot,
				translation.x - c * diff.translation.x + s * diff.translation.y,
				translation.y - c * diff.translation.y - s * diff.translation.x);
	}

	/**Difference of this pose relative to another pose. So if A+B=C is the addition/concatenation, this calculates C-A=B.
	 *\param other The other pose that will be used as origin for the new pose.
	 *\return A reference to this pose after calculating the difference.
	 */
	Pose2D& operator-=(const Pose2D& other) {
		translation -= other.translation;
		Pose2D p(-other.rotation);
		return *this = p + *this;
	}

	/**Difference of this pose relative to another pose.
	 *\param other The other pose that will be used as origin for the new pose.
	 *\return The resulting pose.
	 */
	Pose2D operator-(const Pose2D& other) const {
		return Pose2D(*this) -= other;
	}

	/**Concatenation of this pose with another pose
	 *\param other The other pose that will be concatenated to this one.
	 *\return A reference to this pose after concatenation
	 */
	Pose2D& conc(const Pose2D& other) {
		return *this += other;
	}

	/**Translate this pose by a translation vector
	 *\param trans Vector to translate with
	 *\return A reference to this pose after translation
	 */
	Pose2D& translate(const Vector2<double>& trans) {
		translation = *this * trans;
		return *this;
	}

	/**Translate this pose by a translation vector
	 *\param x x component of vector to translate with
	 *\param y y component of vector to translate with
	 *\return A reference to this pose after translation
	 */
	Pose2D& translate(const double x, const double y) {
		translation = *this * Vector2<double>(x, y);
		return *this;
	}

	/**Rotate this pose by a rotation
	 *\param angle Angle to rotate.
	 *\return A reference to this pose after rotation
	 */
	Pose2D& rotate(const double angle) {
		rotation += angle;
		return *this;
	}

	/** Calculates the inverse transformation from the current pose
	 * @return The inverse transformation pose.
	 */
	Pose2D invert() const {
		const double& invRotation = -rotation;
		return Pose2D(invRotation,
				(Vector2<double>() - translation).rotate(invRotation));
	}

	/** Converts from a coordinate frame relative to the origin Pose2D passed in,
	 to the one that the origin Pose2D was defined in. */
	Pose2D relativeToGlobal(const Pose2D &old_origin) const {
		Pose2D retVal(*this);
		retVal.translation.rotate(old_origin.rotation);
		retVal.translation += old_origin.translation;
		retVal.rotation += old_origin.rotation;
		return retVal;
	}

	/** Converts from the coordinate frame the origin Pose2D is defined in,
	 to one relative to it. */
	Pose2D globalToRelative(const Pose2D &new_origin) const {
		Pose2D retVal(*this);
		retVal.translation -= new_origin.translation;
		retVal.translation.rotate(-new_origin.rotation);
		retVal.rotation -= new_origin.rotation;
		return retVal;
	}

	/**
	 * The function creates a random pose.
	 * @param x The range for x-values of the pose.
	 * @param y The range for y-values of the pose.
	 * @param angle The range for the rotation of the pose.
	 */
	static Pose2D random(const Range<double>& x, const Range<double>& y,
			const Range<double>& angle) { // angle should even work in wrap around case!
		return Pose2D(
				double(randomDouble() * (angle.max - angle.min) + angle.min),
				Vector2<double>(
						double(randomDouble() * (x.max - x.min) + x.min),
						double(randomDouble() * (y.max - y.min) + y.min)));
	}

	double abs(){
		return translation.abs();
	}
};

} /* namespace math */
} /* namespace itandroids_lib */

#endif // __Pose2D_h__

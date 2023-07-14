package frc.team670.mustanglib.utils.math;
// package frc.team670.mustanglib.utils.math;
// /**
//  *  A class that represents a vector
//  *  @author laksh
//  */
// public class Vector {
// 	public double x, y, z;
// 	private double velocity;
// 	private double curvature;
// 	private double distance;
// 	/**Intilizaes a zero vector */
// 	public Vector() {

// 	}

// 	// The `public Vector(double x, double y, double z)` constructor is initializing a new Vector object
// 	// with the given x, y, and z coordinates. It also sets the velocity, curvature, and distance
// 	// variables to 0.
// 	public Vector(double x, double y, double z) {
// 		this.x = x;
// 		this.y = y;
// 		this.z = z;
// 		velocity = 0;
// 		curvature = 0;
// 		distance = 0;
// 	}

// 	/** The `public Vector(double x, double y)` constructor is initializing a new Vector object with the
// 	 given x and y coordinates. It sets the z coordinate to 0 and the velocity, curvature, and distance
// 	 variables to 0. This constructor is useful when you only need a 2D vector and don't need to specify
// 	/ a z coordinate
// 	*/ 
// 	public Vector(double x, double y) {
// 		this.x = x;
// 		this.y = y;
// 		this.z = 0;
// 		velocity = 0;
// 		curvature = 0;
// 		distance = 0;
// 	}

// 	/**
// 	 * makes a copy of the passed in vector
// 	 * @param v the vector that will be deep copied
// 	 */
// 	public Vector(Vector v) {
// 		this.x = v.x;
// 		this.y = v.y;
// 		this.z = v.z;
// 		this.velocity = v.velocity;
// 		this.curvature = v.curvature;
// 		this.distance = v.distance;
// 	}

// 	/**
// 	 * The function takes two vectors as input and returns their sum, either by creating a new vector or
// 	 * modifying an existing one.
// 	 * 
// 	 * @param a The vector a is the first vector that we want to add.
// 	 * @param b The parameter "b" is a Vector object representing the second vector to be added.
// 	 * @param target The target parameter is a Vector object that represents the vector where the result
// 	 * of the addition operation will be stored. If the target vector is null, a new Vector object is
// 	 * created and assigned to the target parameter. Otherwise, the target vector is updated with the sum
// 	 * of vectors a and b. Finally,
// 	 * @return The method is returning the Vector object "target".
// 	 */
// 	public static Vector add(Vector a, Vector b, Vector target) {
// 		if (target == null) {
// 			target = new Vector(a.x + b.x, a.y + b.y, a.z + b.z);
// 		} else {
// 			target.set(a.x + b.x, a.y + b.y, a.z + b.z);
// 		}
// 		return target;
// 	}

// 	/**
// 	 * The function "add" takes two Vector objects as input and returns their sum.
// 	 * 
// 	 * @param a The first vector to be added.
// 	 * @param b The parameter "b" is a Vector object that represents the second vector to be added.
// 	 * @return The method is returning a Vector object that is the sum of the passed in vectors
// 	 */
// 	public static Vector add(Vector a, Vector b) {
// 		return add(a, b, null);
// 	}

// 	/**
// 	 * The function "sub" subtracts vector b from vector a and stores the result in vector target.
// 	 * 
// 	 * @param a The vector a is the first vector that we want to subtract from.
// 	 * @param b The parameter "b" is a Vector object representing the second vector in the subtraction
// 	 * operation.
// 	 * @param target The target parameter is a Vector object that will store the result of the subtraction
// 	 * operation. If the target parameter is null, a new Vector object will be created and assigned the
// 	 * result of the subtraction. If the target parameter is not null, its values will be updated with the
// 	 * result of the subtraction.
// 	 * @return The method is returning the Vector object "target" which is the difference between the two vectors.
// 	 */
// 	public static Vector sub(Vector a, Vector b, Vector target) {
// 		if (target == null) {
// 			target = new Vector(a.x - b.x, a.y - b.y, a.z - b.z);
// 		} else {
// 			target.set(a.x - b.x, a.y - b.y, a.z - b.z);
// 		}
// 		return target;
// 	}

// 	/**
// 	 * The function "sub" subtracts two vectors and returns the result.
// 	 * 
// 	 * @param a The first vector to subtract from.
// 	 * @param b The parameter "b" is a Vector object that represents the vector to be subtracted from
// 	 * vector "a".
// 	 * @return The method is returning a Vector object that is the difference between the two vectors.
// 	 */
// 	public static Vector sub(Vector a, Vector b) {
// 		return sub(a, b, null);
// 	}

// 	/**
// 	 * The function takes a vector and a scalar value, and multiplies each component of the vector by the
// 	 * scalar, returning the result.
// 	 * 
// 	 * @param a The vector to be multiplied by a scalar value.
// 	 * @param n The parameter "n" is a double value that represents the scalar by which each component of
// 	 * the vector "a" will be multiplied.
// 	 * @param target The target parameter is a Vector object that represents the result of the
// 	 * multiplication operation. If the target parameter is null, a new Vector object is created and
// 	 * assigned the result of the multiplication. If the target parameter is not null, its values are
// 	 * updated with the result of the multiplication. Finally, the target
// 	 * @return The method is returning a Vector object that is the multiplication of the vector "a" by the scalar n
// 	 */
// 	public static Vector mult(Vector a, double n, Vector target) {
// 		if (target == null) {
// 			target = new Vector(a.x * n, a.y * n, a.z * n);
// 		} else {
// 			target.set(a.x * n, a.y * n, a.z * n);
// 		}
// 		return target;
// 	}

// 	/**
// 	 * The function "mult" takes a vector and a number as input and returns the vector multiplied by the
// 	 * number.
// 	 * 
// 	 * @param a a is a Vector object that represents a vector in a mathematical sense. It could be a
// 	 * vector in 2D or 3D space, for example.
// 	 * @param n The parameter "n" is a double value that represents the scalar value by which each element
// 	 * of the vector "a" will be multiplied.
// 	 * @return The method is returning a Vector object that is the result of the multiplication of a and n
// 	 */
// 	public static Vector mult(Vector a, double n) {
// 		return mult(a, n, null);
// 	}

// 	/**
// 	 * The function takes two vectors as input and multiplies their corresponding components, storing the
// 	 * result in a third vector.
// 	 * 
// 	 * @param a The parameter "a" is a Vector object representing the first vector.
// 	 * @param b The parameter "b" is a Vector object that represents the second vector in the
// 	 * multiplication operation.
// 	 * @param target The target parameter is a Vector object that will store the result of the
// 	 * multiplication operation. If the target parameter is null, a new Vector object will be created and
// 	 * assigned the result of the multiplication. If the target parameter is not null, its values will be
// 	 * updated with the result of the multiplication.
// 	 * @return The method is returning the target vector that contains the multiplication of vector a and vector b
// 	 */
// 	public static Vector mult(Vector a, Vector b, Vector target) {
// 		if (target == null) {
// 			target = new Vector(a.x * b.x, a.y * b.y, a.z * b.z);
// 		} else {
// 			target.set(a.x * b.x, a.y * b.y, a.z * b.z);
// 		}
// 		return target;
// 	}

// 	/**
// 	 * The function "mult" takes two Vector objects as input and returns their product as a new Vector
// 	 * object.
// 	 * 
// 	 * @param a The first vector to be multiplied.
// 	 * @param b The parameter "b" is a Vector object.
// 	 * @return The method is returning a Vector object.
// 	 */
// 	public static Vector mult(Vector a, Vector b) {
// 		return mult(a, b, null);
// 	}

// 	/**
// 	 * The function div divides the components of vector a by the corresponding components of vector b and
// 	 * stores the result in vector target.
// 	 * 
// 	 * @param a A vector representing the numerator values.
// 	 * @param b The parameter "b" is a Vector object that represents the divisor.
// 	 * @param target The target parameter is a Vector object that will store the result of the division
// 	 * operation. If the target parameter is null, a new Vector object will be created to store the
// 	 * result. If the target parameter is not null, its values will be updated with the result of the
// 	 * division operation.
// 	 * @return The method is returning the Vector object "target".
// 	 */
// 	public static Vector div(Vector a, float n, Vector target) {
// 		if (target == null) {
// 			target = new Vector(a.x / n, a.y / n, a.z / n);
// 		} else {
// 			target.set(a.x / n, a.y / n, a.z / n);
// 		}
// 		return target;
// 	}

// 	/**
// 	 * The function div divides each element of a Vector by a given number.
// 	 * 
// 	 * @param a a is a Vector object that represents the vector to be divided.
// 	 * @param n The parameter "n" is a float value that represents the number by which each element in the
// 	 * vector "a" will be divided.
// 	 * @return The method is returning a Vector.
// 	 */
// 	public static Vector div(Vector a, float n) {
// 		return div(a, n, null);
// 	}

// 	/**
// 	 * The function div divides the components of vector a by the corresponding components of vector b and
// 	 * stores the result in vector target.
// 	 * 
// 	 * @param a A vector representing the numerator values.
// 	 * @param b The parameter "b" is a Vector object that represents the divisor.
// 	 * @param target The target parameter is a Vector object that will store the result of the division
// 	 * operation. If the target parameter is null, a new Vector object will be created to store the
// 	 * result. If the target parameter is not null, its values will be updated with the result of the
// 	 * division operation.
// 	 * @return The method is returning the Vector object "target".
// 	 */
// 	public static Vector div(Vector a, Vector b, Vector target) {
// 		if (target == null) {
// 			target = new Vector(a.x / b.x, a.y / b.y, a.z / b.z);
// 		} else {
// 			target.set(a.x / b.x, a.y / b.y, a.z / b.z);
// 		}
// 		return target;
// 	}

// 	/**
// 	 * The function calculates the Euclidean distance between two vectors in three-dimensional space.
// 	 * 
// 	 * @param a a is a Vector object representing the coordinates of point A in 3D space. It has
// 	 * properties x, y, and z representing the x, y, and z coordinates respectively.
// 	 * @param b The parameter "b" is a Vector object representing the second vector in the calculation of
// 	 * the distance.
// 	 * @return The method is returning the distance between two vectors as a double value.
// 	 */
// 	static public double dist(Vector a, Vector b) {
// 		double dx = a.x - a.x;
// 		double dy = a.y - b.y;
// 		double dz = a.z - b.z;
// 		return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2) + Math.pow(dz, 2));
// 	}

// 	static public double dot(Vector a, Vector b) {
// 		return a.x * b.x + a.y * b.y + a.z * b.z;
// 	}

// 	/**
// 	 * The function calculates the angle between two vectors using the dot product and magnitude.
// 	 * 
// 	 * @param a a is a Vector object representing the first vector.
// 	 * @param b The parameter "b" in the above code represents a vector.
// 	 * @return The method is returning the angle between two vectors in radians.
// 	 */
// 	static public double angleBetween(Vector a, Vector b) {
// 		double dot = dot(a, b);
// 		double amag = Math.sqrt(Math.pow(a.x, 2) + Math.pow(a.y, 2) + Math.pow(a.z, 2));
// 		double bmag = Math.sqrt(Math.pow(b.x, 2) + Math.pow(b.y, 2) + Math.pow(b.z, 2));
// 		return Math.acos(dot / (amag * bmag));
// 	}

// 	/**
// 	 * The function sets the values of three variables x, y, and z to the given input values.
// 	 * 
// 	 * @param x The x parameter is a double value that represents the x-coordinate of a point.
// 	 * @param y The parameter "y" is a double value that represents the y-coordinate of a point.
// 	 * @param z The parameter "z" is a double value that represents the z-coordinate of a point.
// 	 */
// 	public void set(double x, double y, double z) {
// 		this.x = x;
// 		this.y = y;
// 		this.z = z;
// 	}

// 	/**
// 	 * The function returns the velocity as a double.
// 	 * 
// 	 * @return The method is returning a double value, which is the velocity.
// 	 */
// 	public double getVelocity() {
// 		return velocity;
// 	}

// 	/**
// 	 * The function sets the velocity of an object.
// 	 * 
// 	 * @param velocity The parameter "velocity" is a double data type, which means it can hold decimal
// 	 * values. It is used to set the velocity of an object.
// 	 */
// 	public void setVelocity(double velocity) {
// 		this.velocity = velocity;
// 	}

// 	/**
// 	 * The function returns the curvature value.
// 	 * 
// 	 * @return The method is returning the value of the variable "curvature", which is of type double.
// 	 */
// 	public double getCurvature() {
// 		return curvature;
// 	}

// 	/**
// 	 * The function sets the curvature value of an object.
// 	 * 
// 	 * @param curvature The "curvature" parameter is a double value that represents the curvature of an
// 	 * object or a path. It is used to set the curvature of an object or a path to a specific value.
// 	 */
// 	public void setCurvature(double curvature) {
// 		this.curvature = curvature;
// 	}

// 	/**
// 	 * The function returns the value of the distance variable.
// 	 * 
// 	 * @return The method is returning a double value, which is the distance.
// 	 */
// 	public double getDistance() {
// 		return distance;
// 	}

// 	/**
// 	 * The function sets the value of the distance variable.
// 	 * 
// 	 * @param distance The distance parameter is a double data type, which means it can hold decimal
// 	 * values. It is used to set the value of the distance variable in the class.
// 	 */
// 	public void setDistance(double distance) {
// 		this.distance = distance;
// 	}

// 	/**
// 	* The function returns a new Vector object with the same x, y, and z values as the original Vector.
// 	* 
// 	* @return A new instance of the Vector class with the same values for x, y, and z.
// 	*/
// 	public Vector duplicate() {
// 		return new Vector(x, y, z);
// 	}

// 	/**
// 	 * The norm function calculates the magnitude of a 3D vector.
// 	 * 
// 	 * @return The norm of a vector, which is the square root of the sum of the squares of its components.
// 	 */
// 	public double norm() {
// 		return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
// 	}

// 	/**
// 	 * The add function adds the x, y, and z components of a given vector to the current vector.
// 	 * 
// 	 * @param v The parameter "v" is a Vector object that contains three components: x, y, and z.
// 	 */
// 	public void add(Vector v) {
// 		x += v.x;
// 		y += v.y;
// 		z += v.z;
// 	}

// 	/**
// 	 * The function adds the values of x, y, and z to the corresponding instance variables.
// 	 * 
// 	 * @param x The value to be added to the current value of x.
// 	 * @param y The parameter "y" is a double value that is being added to the current value of the
// 	 * instance variable "this.y".
// 	 * @param z The parameter "z" is a double value that is being passed into the method.
// 	 */
// 	public void add(double x, double y, double z) {
// 		this.x += x;
// 		this.y += y;
// 		this.z += z;
// 	}

// 	/**
// 	 * The function subtracts the components of a given vector from the components of the current vector.
// 	 * 
// 	 * @param a The parameter "a" is a Vector object.
// 	 */
// 	public void sub(Vector a) {
// 		x -= a.x;
// 		y -= a.y;
// 		z -= a.z;
// 	}

// 	/**
// 	 * The function subtracts the values of x, y, and z from the corresponding instance variables.
// 	 * 
// 	 * @param x The value to subtract from the current value of x.
// 	 * @param y The parameter "y" is a double value that is subtracted from the current value of the
// 	 * instance variable "this.y".
// 	 * @param z The parameter "z" is a double value that is subtracted from the current value of the
// 	 * variable "this.z".
// 	 */
// 	public void sub(double x, double y, double z) {
// 		this.x -= x;
// 		this.y -= y;
// 		this.z -= z;
// 	}

// 	/**
// 	 * The function multiplies the values of x, y, and z by a given number.
// 	 * 
// 	 * @param n The parameter "n" is a double data type, which means it can hold decimal values.
// 	 */
// 	public void mult(double n) {
// 		x *= n;
// 		y *= n;
// 		z *= n;
// 	}

	
// 	/**
// 	 * The function multiplies the x, y, and z components of a vector by the corresponding components of
// 	 * another vector.
// 	 * 
// 	 * @param a The parameter "a" is a Vector object.
// 	 */
// 	public void mult(Vector a) {
// 		x *= a.x;
// 		y *= a.y;
// 		z *= a.z;
// 	}

// 	/**
// 	 * The div function divides the values of x, y, and z by a given number n.
// 	 * 
// 	 * @param n The parameter "n" is a double value that is used to divide the variables x, y, and z.
// 	 */
// 	public void div(double n) {
// 		x /= n;
// 		y /= n;
// 		z /= n;
// 	}

// 	/**
// 	 * The div function divides the x, y, and z components of a vector by the corresponding components of
// 	 * another vector.
// 	 * 
// 	 * @param a The parameter "a" is a Vector object.
// 	 */
// 	public void div(Vector a) {
// 		x /= a.x;
// 		y /= a.y;
// 		z /= a.z;
// 	}

// 	/**
// 	 * The function div takes two Vector objects as input and returns their division.
// 	 * 
// 	 * @param a The first vector, denoted as "a".
// 	 * @param b The parameter "b" is a Vector object that is being divided by another Vector object "a".
// 	 * @return The method is returning a Vector.
// 	 */
// 	public Vector div(Vector a, Vector b) {
// 		return div(a, b, null);
// 	}

// 	/**
// 	 * The function calculates the distance between two vectors in three-dimensional space.
// 	 * 
// 	 * @param v The parameter "v" is a Vector object that represents another point in three-dimensional
// 	 * space.
// 	 * @return The method is returning the distance between the current vector and the vector passed as a
// 	 * parameter.
// 	 */
// 	public double dist(Vector v) {
// 		double dx = x - v.x;
// 		double dy = y - v.y;
// 		double dz = z - v.z;
// 		return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2) + Math.pow(dz, 2));
// 	}

// 	public double dot(Vector v) {
// 		return x * v.x + y * v.y + z * v.z;
// 	}

// 	/**
// 	 * The dot function calculates the dot product of three vectors.
// 	 * 
// 	 * @param x The x parameter represents the x-coordinate of a vector.
// 	 * @param y The parameter "y" represents the second coordinate value in a three-dimensional space.
// 	 * @param z The parameter "z" represents the third component of a vector.
// 	 * @return The method is returning the dot product of the current object's coordinates (this.x,
// 	 * this.y, this.z) and the given coordinates (x, y, z).
// 	 */
// 	public double dot(double x, double y, double z) {
// 		return this.x * x + this.y * y + this.z * z;
// 	}

// 	/**
// 	 * The normalize() function divides the vector by its norm if the norm is not equal to 0 or 1.
// 	 */
// 	public void normalize() {
// 		double m = norm();
// 		if (m != 0 && m != 1) {
// 			div(m);
// 		}
// 	}

// 	/**
// 	 * The function normalizes a given vector by dividing its components by its magnitude.
// 	 * 
// 	 * @param target The "target" parameter is a Vector object that represents the vector to be
// 	 * normalized.
// 	 * @return The method is returning a Vector object.
// 	 */
// 	public Vector normalize(Vector target) {
// 		if (target == null) {
// 			target = new Vector();
// 		}
// 		double m = norm();
// 		if (m > 0) {
// 			target.set(x / m, y / m, z / m);
// 		} else {
// 			target.set(x, y, z);
// 		}
// 		return target;
// 	}

// 	/**
// 	 * The function limits the magnitude of a vector to a specified maximum value.
// 	 * 
// 	 * @param max The "max" parameter is a float value that represents the maximum magnitude that the
// 	 * vector can have.
// 	 */
// 	public void limit(float max) {
// 		if (norm() > max) {
// 			normalize();
// 			mult(max);
// 		}
// 	}

// 	/**
// 	 * The toString() function returns a string representation of the object's x and y values.
// 	 * 
// 	 * @return The toString() method is returning a string representation of the object's state, which
// 	 * includes the values of the variables x and y.
// 	 */
// 	@Override
// 	public String toString() {
// 		return "x: " + x + ", y: " + y;
// 	}

// 	/**
// 	 * The function checks if two Vector objects are equal by comparing their x, y, z, curvature,
// 	 * velocity, and distance values.
// 	 * 
// 	 * @param object The "object" parameter is the object that we are comparing the current Vector object
// 	 * to. It is of type Object, which means it can be any type of object.
// 	 * @return if two Vector objects are equal by comparing their x, y, z, curvature,
// 	 * velocity, and distance values
// 	 */
// 	@Override
// 	public boolean equals(Object object) {
// 		if (!(object instanceof Vector))
// 			return false;
// 		Vector vector = (Vector) object;
// 		return vector.x == x && vector.y == y && vector.z == z && vector.curvature == curvature && vector.velocity == velocity && vector.distance == distance;
// 	}
// }

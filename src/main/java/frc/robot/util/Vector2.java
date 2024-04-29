package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Vector2 {
  private double x, y;

  public Vector2() {
    this(0, 0);
  }

  public Vector2(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public Vector2 plus(Vector2 other) {
    return new Vector2(x + other.x, y + other.y);
  }

  public Vector2 minus(Vector2 other) {
    return new Vector2(x - other.x, y - other.y);
  }

  public Vector2 times(double scalar) {
    return new Vector2(scalar * x, scalar * y);
  }

  public Vector2 unaryMinus() {
    return this.times(-1);
  }

  public Vector2 div(double scalar) {
    return this.times(1 / scalar);
  }

  public boolean closeTo(Vector2 other, double margin) {
    return minus(other).norm() < margin;
  }

  // Checks if two vectors are within 1e-6 (i.e. effectively equivalent)
  public boolean closeTo(Vector2 other) {
    return closeTo(other, 1e-6);
  }

  public String toString() {
    return "<" + x + ", " + y + ">";
  }

  public double dot(Vector2 other) {
    return x * other.x + y * other.y;
  }

  public void set(Vector2 other) {
    this.x = other.getX();
    this.y = other.getY();
  }

  public double norm() {
    return Math.hypot(x, y);
  }

  public double magnitude() {
    return norm();
  }

  public Vector2 normalize() {
    return this.div(this.norm());
  }

  // equivalent to converting the vector to polar form
  // and then negating the angle (multiplying it by -1).
  public Vector2 negateAngle() {
    return new Vector2(this.x, -this.y);
  }

  // equivalent to converting the vector to polar form
  // and then subtracting 90 degrees (pi/2 rad) from the angle.
  // aka just returning a vector that is perpendicular to this vector
  // in the clockwise direction, or rotating it 90 degrees clockwise.
  public Vector2 cwPerp() {
    return new Vector2(this.y, -this.x);
  }

  // fun rotate(rad: Double) = Vector2(cos(rad) * x - sin(rad) * y, sin(rad) * x +
  // cos(rad) * y)
  // fun rotateDeg(deg: Double) = rotate(deg * PI/180.0)

  public Translation2d toTranslation2d() {
    return new Translation2d(x, y);
  }
}

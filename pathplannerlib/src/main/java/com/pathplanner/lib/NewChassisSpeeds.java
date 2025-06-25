// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.pathplanner.lib;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.proto.ChassisSpeedsProto;
import edu.wpi.first.math.kinematics.struct.ChassisSpeedsStruct;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

/**
 * Represents the speed of a robot chassis. Although this class contains similar members compared to
 * a Twist2d, they do NOT represent the same thing. Whereas a Twist2d represents a change in pose
 * w.r.t to the robot frame of reference, a NewChassisSpeeds object represents a robot's velocity.
 *
 * <p>A strictly non-holonomic drivetrain, such as a differential drive, should never have a dy
 * component because it can never move sideways. Holonomic drivetrains such as swerve and mecanum
 * will often have all three components.
 */
public class NewChassisSpeeds implements ProtobufSerializable, StructSerializable {
  /** Velocity along the x-axis. (Fwd is +) */
  public double vx;

  /** Velocity along the y-axis. (Left is +) */
  public double vy;

  /** Represents the angular velocity of the robot frame. (CCW is +) */
  public double omega;

  /** NewChassisSpeeds protobuf for serialization. */
  public static final ChassisSpeedsProto proto = new ChassisSpeedsProto();

  /** NewChassisSpeeds struct for serialization. */
  public static final ChassisSpeedsStruct struct = new ChassisSpeedsStruct();

  /** Constructs a NewChassisSpeeds with zeros for dx, dy, and theta. */
  public NewChassisSpeeds() {}

  /**
   * Constructs a NewChassisSpeeds object.
   *
   * @param vx Forward velocity.
   * @param vy Sideways velocity.
   * @param omega Angular velocity.
   */
  public NewChassisSpeeds(
      double vx, double vy, double omega) {
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
  }

  /**
   * Constructs a NewChassisSpeeds object.
   *
   * @param vx Forward velocity.
   * @param vy Sideways velocity.
   * @param omega Angular velocity.
   */
  public NewChassisSpeeds(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
    this(vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond));
  }

  /**
   * Creates a Twist2d from NewChassisSpeeds.
   *
   * @param dtSeconds The duration of the timestep.
   * @return Twist2d.
   */
  public Twist2d toTwist2d(double dtSeconds) {
    return new Twist2d(
        vx * dtSeconds,
        vy * dtSeconds,
        omega * dtSeconds);
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param vx Forward velocity.
   * @param vy Sideways velocity.
   * @param omega Angular velocity.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   * @return Discretized NewChassisSpeeds.
   */
  public static ChassisSpeeds discretize(
      double vx,
      double vy,
      double omega,
      double dtSeconds) {
    // Construct the desired pose after a timestep, relative to the current pose. The desired pose
    // has decoupled translation and rotation.
    var desiredDeltaPose =
        new Pose2d(
            vx * dtSeconds,
            vy * dtSeconds,
            new Rotation2d(omega * dtSeconds));

    // Find the chassis translation/rotation deltas in the robot frame that move the robot from its
    // current pose to the desired pose
    var twist = Pose2d.kZero.log(desiredDeltaPose);

    // Turn the chassis translation/rotation deltas into average velocities
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param vx Forward velocity.
   * @param vy Sideways velocity.
   * @param omega Angular velocity.
   * @param dt The duration of the timestep the speeds should be applied for.
   * @return Discretized NewChassisSpeeds.
   */
  public static ChassisSpeeds discretize(
      LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Time dt) {
    return discretize(
        vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), dt.in(Seconds));
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param continuousSpeeds The continuous speeds.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   * @return Discretized NewChassisSpeeds.
   */
  public static ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dtSeconds) {
    return discretize(
        continuousSpeeds.vx,
        continuousSpeeds.vy,
        continuousSpeeds.omega,
        dtSeconds);
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative NewChassisSpeeds
   * object.
   *
   * @param vx The component of speed in the x direction relative to the field.
   *     Positive x is away from your alliance wall.
   * @param vy The component of speed in the y direction relative to the field.
   *     Positive y is to your left when standing behind the alliance wall.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return NewChassisSpeeds object representing the speeds in the robot's frame of reference.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      double vx,
      double vy,
      double omega,
      Rotation2d robotAngle) {
    // CW rotation into chassis frame
    var rotated =
        new Translation2d(vx, vy).rotateBy(robotAngle.unaryMinus());
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), omega);
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative NewChassisSpeeds
   * object.
   *
   * @param vx The component of speed in the x direction relative to the field. Positive x is away
   *     from your alliance wall.
   * @param vy The component of speed in the y direction relative to the field. Positive y is to
   *     your left when standing behind the alliance wall.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return NewChassisSpeeds object representing the speeds in the robot's frame of reference.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Rotation2d robotAngle) {
    return fromFieldRelativeSpeeds(
        vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), robotAngle);
  }

  /**
   * Converts a user provided field-relative NewChassisSpeeds object into a robot-relative
   * NewChassisSpeeds object.
   *
   * @param fieldRelativeSpeeds The NewChassisSpeeds object representing the speeds in the field frame
   *     of reference. Positive x is away from your alliance wall. Positive y is to your left when
   *     standing behind the alliance wall.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return NewChassisSpeeds object representing the speeds in the robot's frame of reference.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      ChassisSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
    return fromFieldRelativeSpeeds(
        fieldRelativeSpeeds.vx,
        fieldRelativeSpeeds.vy,
        fieldRelativeSpeeds.omega,
        robotAngle);
  }

  /**
   * Converts a user provided robot-relative set of speeds into a field-relative NewChassisSpeeds
   * object.
   *
   * @param vx The component of speed in the x direction relative to the robot.
   *     Positive x is towards the robot's front.
   * @param vy The component of speed in the y direction relative to the robot.
   *     Positive y is towards the robot's left.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return NewChassisSpeeds object representing the speeds in the field's frame of reference.
   */
  public static ChassisSpeeds fromRobotRelativeSpeeds(
      double vx,
      double vy,
      double omega,
      Rotation2d robotAngle) {
    // CCW rotation out of chassis frame
    var rotated = new Translation2d(vx, vy).rotateBy(robotAngle);
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), omega);
  }

  /**
   * Converts a user provided robot-relative set of speeds into a field-relative NewChassisSpeeds
   * object.
   *
   * @param vx The component of speed in the x direction relative to the robot. Positive x is
   *     towards the robot's front.
   * @param vy The component of speed in the y direction relative to the robot. Positive y is
   *     towards the robot's left.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return NewChassisSpeeds object representing the speeds in the field's frame of reference.
   */
  public static ChassisSpeeds fromRobotRelativeSpeeds(
      LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Rotation2d robotAngle) {
    return fromRobotRelativeSpeeds(
        vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), robotAngle);
  }

  /**
   * Converts a user provided robot-relative NewChassisSpeeds object into a field-relative
   * NewChassisSpeeds object.
   *
   * @param robotRelativeSpeeds The NewChassisSpeeds object representing the speeds in the robot frame
   *     of reference. Positive x is towards the robot's front. Positive y is towards the robot's
   *     left.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
   *     considered to be zero when it is facing directly away from your alliance station wall.
   *     Remember that this should be CCW positive.
   * @return NewChassisSpeeds object representing the speeds in the field's frame of reference.
   */
  public static ChassisSpeeds fromRobotRelativeSpeeds(
      ChassisSpeeds robotRelativeSpeeds, Rotation2d robotAngle) {
    return fromRobotRelativeSpeeds(
        robotRelativeSpeeds.vx,
        robotRelativeSpeeds.vy,
        robotRelativeSpeeds.omega,
        robotAngle);
  }

  /**
   * Adds two NewChassisSpeeds and returns the sum.
   *
   * <p>For example, NewChassisSpeeds{1.0, 0.5, 0.75} + NewChassisSpeeds{2.0, 1.5, 0.25} =
   * NewChassisSpeeds{3.0, 2.0, 1.0}
   *
   * @param other The NewChassisSpeeds to add.
   * @return The sum of the NewChassisSpeeds.
   */
  public ChassisSpeeds plus(NewChassisSpeeds other) {
    return new ChassisSpeeds(
        vx + other.vx,
        vy + other.vy,
        omega + other.omega);
  }

  /**
   * Subtracts the other NewChassisSpeeds from the current NewChassisSpeeds and returns the difference.
   *
   * <p>For example, NewChassisSpeeds{5.0, 4.0, 2.0} - NewChassisSpeeds{1.0, 2.0, 1.0} =
   * NewChassisSpeeds{4.0, 2.0, 1.0}
   *
   * @param other The NewChassisSpeeds to subtract.
   * @return The difference between the two NewChassisSpeeds.
   */
  public ChassisSpeeds minus(NewChassisSpeeds other) {
    return new ChassisSpeeds(
        vx - other.vx,
        vy - other.vy,
        omega - other.omega);
  }

  /**
   * Returns the inverse of the current NewChassisSpeeds. This is equivalent to negating all components
   * of the NewChassisSpeeds.
   *
   * @return The inverse of the current NewChassisSpeeds.
   */
  public ChassisSpeeds unaryMinus() {
    return new ChassisSpeeds(-vx, -vy, -omega);
  }

  /**
   * Multiplies the NewChassisSpeeds by a scalar and returns the new NewChassisSpeeds.
   *
   * <p>For example, NewChassisSpeeds{2.0, 2.5, 1.0} * 2 = NewChassisSpeeds{4.0, 5.0, 1.0}
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled NewChassisSpeeds.
   */
  public ChassisSpeeds times(double scalar) {
    return new ChassisSpeeds(
        vx * scalar, vy * scalar, omega * scalar);
  }

  /**
   * Divides the NewChassisSpeeds by a scalar and returns the new NewChassisSpeeds.
   *
   * <p>For example, NewChassisSpeeds{2.0, 2.5, 1.0} / 2 = NewChassisSpeeds{1.0, 1.25, 0.5}
   *
   * @param scalar The scalar to divide by.
   * @return The scaled NewChassisSpeeds.
   */
  public ChassisSpeeds div(double scalar) {
    return new ChassisSpeeds(
        vx / scalar, vy / scalar, omega / scalar);
  }

  @Override
  public final int hashCode() {
    return Objects.hash(vx, vy, omega);
  }

  @Override
  public boolean equals(Object o) {
    return o == this
        || o instanceof NewChassisSpeeds c
            && vx == c.vx
            && vy == c.vy
            && omega == c.omega;
  }

  @Override
  public String toString() {
    return String.format(
        "NewChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
        vx, vy, omega);
  }
}

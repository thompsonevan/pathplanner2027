package com.pathplanner.lib.controllers;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Interface used by path following controllers */
public interface PathFollowingController {
  /**
   * Calculates the next output of the path following controller
   *
   * @param currentPose The current robot pose
   * @param targetState The desired trajectory state
   * @return The next robot relative output of the path following controller
   */
  ChassisSpeeds calculateRobotRelativeSpeeds(
      Pose2d currentPose, PathPlannerTrajectoryState targetState);

  /**
   * Resets the controller based on the current state of the robot
   *
   * @param currentPose Current robot pose
   * @param currentSpeeds Current robot relative chassis speeds
   */
  void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds);

  /**
   * Is this controller for holonomic drivetrains? Used to handle some differences in functionality
   * in the path following command.
   *
   * @return True if this controller is for a holonomic drive train
   */
  boolean isHolonomic();
}

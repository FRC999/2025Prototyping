// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.SwerveChassis;

public class AutoSubsystem extends SubsystemBase {
  HashMap<String, PathPlannerPath> trajPaths;
  ArrayList<Translation2d> testWayPoints;

  /** Creates a new AutoSubsystem. */
  public AutoSubsystem() {
    testWayPoints = new ArrayList<>();
    testWayPoints.add(new Translation2d(1.0, 0.0));
    trajPaths = new HashMap<>();
    trajPaths.put("Test", generateDynamicTrajectory(new Pose2d(0, 0, new Rotation2d(0.0)), testWayPoints,
        new Pose2d(2, 0, new Rotation2d(0.0))));
  }

  /**
   * Generates a custom trajectory dynamically using PathPlanner.
   *
   * @param startPose         Starting position and heading of the robot.
   * @param interiorWaypoints List of intermediate waypoints.
   * @param endPose           Ending position and heading of the robot.
   * @return A dynamically generated PathPlannerTrajectory.
   */
  public PathPlannerPath generateDynamicTrajectory(Pose2d startPose, List<Translation2d> interiorWaypoints,
      Pose2d endPose) {
    List<PathPoint> pathPoints = new ArrayList<>();

    pathPoints.add(new PathPoint(
        startPose.getTranslation(), // Position
        new RotationTarget(0.0, startPose.getRotation()) // Rotation target with initial position of 0
    ));

    // Add intermediate waypoints
    double positionAlongPath = 1.0; // Incremental position along the path
    for (Translation2d waypoint : interiorWaypoints) {
      pathPoints.add(new PathPoint(
          waypoint, // Position
          new RotationTarget(positionAlongPath, Rotation2d.fromDegrees(0)) // Rotation target
      ));
      positionAlongPath += 1.0; // Increment position
    }

    // Add the ending point
    pathPoints.add(new PathPoint(
        endPose.getTranslation(), // Position
        new RotationTarget(positionAlongPath, endPose.getRotation()) // Rotation target
    ));

    // Generate the trajectory
    return PathPlannerPath.fromPathPoints(pathPoints,
        new PathConstraints(
            SwerveChassis.MaxSpeed, SwerveChassis.maxAcceleration,
            SwerveChassis.MaxAngularRate, SwerveChassis.maxAngularAcceleration),
        new GoalEndState(0, endPose.getRotation()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousTrajectoryRioCommand extends FollowPathCommand {
  /** Creates a new AutonomousTrajectoryRioCommand. */
  public AutonomousTrajectoryRioCommand(PathPlannerPath trajectoryPath, RobotConfig robotConfig) {
    super(
        trajectoryPath,
        RobotContainer.driveSubsystem::getPose,
        RobotContainer.driveSubsystem::getChassisSpeeds,
        RobotContainer.driveSubsystem::driveWithChassisSpeeds,
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(
                SwerveChassis.DRIVE_CHASSIS_KP,
                SwerveChassis.DRIVE_CHASSIS_KI,
                SwerveChassis.DRIVE_CHASSIS_KD), // Translation PID constants
            new PIDConstants(
                SwerveChassis.ANGLE_CHASSIS_KP,
                SwerveChassis.ANGLE_CHASSIS_KI,
                SwerveChassis.ANGLE_CHASSIS_KD) // Rotation PID constants
        ),
        robotConfig,
        () -> {
          return false;
        },
        RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

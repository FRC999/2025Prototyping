// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunMotor extends InstantCommand {
  double power;
  public RunMotor(double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.motorSubsystem);
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Test1");
    RobotContainer.motorSubsystem.runWithPower(power);
  }
}

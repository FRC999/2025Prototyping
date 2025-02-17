// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunMotor;
import frc.robot.commands.StopMotor;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final MotorSubsystem motorSubsystem = new MotorSubsystem();
  public static final SmartDashboardSubsystem smartDashboardsubsystem = new SmartDashboardSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static Joystick joystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    joystick = new Joystick(0);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    testingKraken();
  }

  public void testingKraken() {
    new JoystickButton(joystick, 1)
      .onTrue(new RunMotor(0.2))
      .onFalse(new StopMotor());
      
    new JoystickButton(joystick, 2)
      .onTrue(new RunMotor(-0.2))
      .onFalse(new StopMotor());
    
    new JoystickButton(joystick, 3)
      .onTrue(new InstantCommand( ()->motorSubsystem.goToIncrementalPositionVoltage(20), motorSubsystem))
      .onFalse(new InstantCommand(() -> motorSubsystem.stopMotor(), motorSubsystem));
    
    new JoystickButton(joystick, 4)
      .onTrue(new InstantCommand(() -> motorSubsystem.goToIncrementalPositionDutyCycle(20), motorSubsystem))
      .onFalse(new InstantCommand(() -> motorSubsystem.stopMotor(), motorSubsystem));

    new JoystickButton(joystick, 5)
      .onTrue(new InstantCommand(() -> motorSubsystem.goToIncrementalPositionMotionMagicDutyCycle(10), motorSubsystem))
      .onFalse(new InstantCommand(() -> motorSubsystem.stopMotor(), motorSubsystem));
    
    new JoystickButton(joystick, 6)
      .onTrue(new InstantCommand(() -> motorSubsystem.goToIncrementalPositionMotionMagicVoltage(20), motorSubsystem))
      .onFalse(new InstantCommand(() -> motorSubsystem.stopMotor(), motorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

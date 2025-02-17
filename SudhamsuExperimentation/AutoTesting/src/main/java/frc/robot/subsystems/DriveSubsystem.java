// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
import frc.robot.Constants.SwerveConstants.SwerveChassis.SwerveModuleConstantsEnum;
import frc.robot.Constants.SwerveConstants.TunerConstants;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveChassis.MaxSpeed * SwerveChassis.chassisLinearMoveDeadband).withRotationalDeadband(SwerveChassis.MaxAngularRate * SwerveChassis.chassisAngularMoveDeadband) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
      
  private double previousOmegaRotationCommand;

  Pigeon2 imu;

  public static InterpolatingDoubleTreeMap chassisAngularVelocityConversion = new InterpolatingDoubleTreeMap();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    super(
        TalonFX::new, TalonFX::new, CANcoder::new,
        TunerConstants.DrivetrainConstants, (SwerveModuleConstants[]) configureSwerveChassis());

    imu = this.getPigeon2();
    this.registerTelemetry(this::telemeterize);

    setChassisAngularVelocityConversion();
  }

  @SuppressWarnings("unchecked")
  public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] configureSwerveChassis() {
    return new SwerveModuleConstants[] {

        // Front Left
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD0.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD0.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD0.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD0.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD0.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD0.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD0.isCANCoderIverted()),

        // Front Right
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD1.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD1.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD1.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD1.getAngleOffset()),
            Meters.of(SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(-SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD1.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD1.isCANCoderIverted()),

        // Back Left
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD2.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD2.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD2.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD2.getAngleOffset()),
            Meters.of(-SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD2.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD2.isCANCoderIverted()),

        // Back Right
        TunerConstants.ConstantCreator.createModuleConstants(
            SwerveModuleConstantsEnum.MOD3.getAngleMotorID(),
            SwerveModuleConstantsEnum.MOD3.getDriveMotorID(),
            SwerveModuleConstantsEnum.MOD3.getCancoderID(),
            Rotations.of(SwerveModuleConstantsEnum.MOD3.getAngleOffset()),
            Meters.of(-SwerveChassis.WHEEL_BASE / 2.0),
            Meters.of(-SwerveChassis.TRACK_WIDTH / 2.0),
            SwerveModuleConstantsEnum.MOD3.isDriveMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isAngleMotorInverted(),
            SwerveModuleConstantsEnum.MOD3.isCANCoderIverted())
    };
  }

  public void telemeterize(SwerveDriveState state) {
    SmartDashboard.putString("P:", state.Pose.toString());
    // SmartDashboard.putString("M0A:", state.ModuleStates[0].toString());
    // SmartDashboard.putString("M1A:", state.ModuleStates[1].toString());

  }

  public static void setChassisAngularVelocityConversion() {
    chassisAngularVelocityConversion.put(0.43, 0.2 * Math.PI);
    chassisAngularVelocityConversion.put(0.60, 0.25 * Math.PI);
    chassisAngularVelocityConversion.put(0.78, 0.3 * Math.PI);
    chassisAngularVelocityConversion.put(0.96, 0.35 * Math.PI);
    chassisAngularVelocityConversion.put(1.17, 0.4 * Math.PI);
    chassisAngularVelocityConversion.put(1.39, 0.45 * Math.PI);
    chassisAngularVelocityConversion.put(1.57, 0.5 * Math.PI);
    chassisAngularVelocityConversion.put(1.75, 0.55 * Math.PI);
    chassisAngularVelocityConversion.put(1.92, 0.6 * Math.PI);
    chassisAngularVelocityConversion.put(2.10, 0.65 * Math.PI);
    chassisAngularVelocityConversion.put(2.26, 0.7 * Math.PI);
    chassisAngularVelocityConversion.put(2.43, 0.75 * Math.PI);
    chassisAngularVelocityConversion.put(2.60, 0.8 * Math.PI);
    chassisAngularVelocityConversion.put(3.28, 1.0 * Math.PI);
    chassisAngularVelocityConversion.put(3.94, 1.2 * Math.PI);
    chassisAngularVelocityConversion.put(4.61, 1.4 * Math.PI);
    chassisAngularVelocityConversion.put(4.94, 1.5 * Math.PI);
    chassisAngularVelocityConversion.put(5.27, 1.6 * Math.PI);
    chassisAngularVelocityConversion.put(5.96, 1.8 * Math.PI);
    chassisAngularVelocityConversion.put(6.63, 2.0 * Math.PI);
  }

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s) {
    //System.out.println("X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s/SwerveChassis.MaxAngularRate);
    SmartDashboard.putString("Manual Drive Command Velocities","X: " + xVelocity_m_per_s + " y: " + yVelocity_m_per_s + " o:" + omega_rad_per_s);
    this.setControl(
      drive.withVelocityX(xVelocity_m_per_s)
        .withVelocityY(yVelocity_m_per_s)
        .withRotationalRate(omega_rad_per_s)
    );
    previousOmegaRotationCommand = omega_rad_per_s / SwerveChassis.MaxAngularRate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramMetersPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static class TunerConstants {
      public static class PIDConstants {
        public static final double steerGainsKP = 100;
        public static final double steerGainsKI = 0;
        // public static final double steerGainsKD = 2.0;
        public static final double steerGainsKD = 0.5;
        // public static final double steerGainsKS = 0.2;
        public static final double steerGainsKS = 0.1;
        // public static final double steerGainsKV = 2.66;
        public static final double steerGainsKV = 1.59;
        public static final double steerGainsKA = 0;

        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(steerGainsKP).withKI(steerGainsKI).withKD(steerGainsKD) //  generated code has KD 0.5
            .withKS(steerGainsKS).withKV(steerGainsKV).withKA(steerGainsKA) //  generated code has KS 0.1, KV as
                                                                            // 1.59
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        public static final double driveGainsKP = 0.1;
        public static final double driveGainsKI = 0;
        public static final double driveGainsKD = 0;
        public static final double driveGainsKS = 0;
        public static final double driveGainsKV = 0.124;
        // public static final double driveGainsKA = 0;

        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(driveGainsKP).withKI(driveGainsKI).withKD(driveGainsKD)
            .withKS(driveGainsKS).withKV(driveGainsKV);
      }

      private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

      private static final DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
      private static final SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

      // The remote sensor feedback type to use for the steer motors;
      // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
      // RemoteCANcoder
      private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

      private static final Current slipCurrent = Amps.of(120.0);

      private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
      private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a
                  // relatively low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));

      private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

      // Theoretical free speed (m/s) at 12v applied output;
      // This needs to be tuned to your individual robot
      // public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(6.21);
      // the old value was 5.21
      public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(5.21); // the old value was 5.21

      // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
      // This may need to be tuned to your individual robot
      private static final double kCoupleRatio = 3; // the old value was 3.5714285714285716
      // private static final double kCoupleRatio = 3.5714285714285716; //the old value was 3.5714285714285716

      private static final double kDriveGearRatio = 5.142857142857142 * (0.13 / 5.22); // the old value was
                                                                                       // 6.122448979591837 * (1/2.09)
      // private static final double kDriveGearRatio = 6.122448979591837;
      private static final double kSteerGearRatio = 12.8; // the old value was 21.428571428571427
      private static final Distance wheelRadius = Inches.of(SwerveChassis.WHEEL_DIAMETER / 2.0); // the old value
                                                                                                 // was Inches.of(5.33 /
                                                                                                 // 5.71)

      private static final boolean kInvertLeftSide = false;
      private static final boolean kInvertRightSide = true;

      public static final CANBus kCANBus = new CANBus("canivore1", "./logs/example.hoot");

      // These are only used for simulation
      private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
      private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
      // Simulated voltage necessary to overcome friction
      private static final Voltage kSteerFrictionVoltage = Volts.of(0.2); // old value was 0.25
      private static final Voltage kDriveFrictionVoltage = Volts.of(0.2); // old value was a 0.25

      public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(IMUConstants.kPigeonId)
          .withPigeon2Configs(IMUConstants.pigeonConfigs);

      public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCoupleRatio)
          .withWheelRadius(wheelRadius)
          .withSteerMotorGains(PIDConstants.steerGains)
          .withDriveMotorGains(PIDConstants.driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSlipCurrent(slipCurrent)
          .withSpeedAt12Volts(speedAt12Volts)
          .withDriveMotorType(driveMotorType)
          .withSteerMotorType(steerMotorType)
          .withFeedbackSource(steerFeedbackType)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withEncoderInitialConfigs(cancoderInitialConfigs)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage);
    }

    public static class SwerveChassis {
      public static final double TRACK_WIDTH = Meters.convertFrom(18.00, Inches); // left to right
      public static final double WHEEL_BASE = Meters.convertFrom(18.00, Inches); // front to back
      public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
      public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

      public static final double MaxSpeed = TunerConstants.speedAt12Volts.magnitude(); // kSpeedAt12VoltsMps
                                                                                       // desired top speed
      public static final double maxAcceleration = 41.68; // this is Max linear acceleration units: m/s^2
      public static final double MaxAngularRate = 2.0 * Math.PI; // 3/4 of a rotation per second max angular
                                                                 // velocity
      public static final double maxAngularAcceleration = 37.6992; // this is max angular acceleration units:
                                                                   // rad/s^2
      public static final double robotMass = 56.7; // kg
      public static final double robotInertia = 60.0; // KG*M^2 - for rotation
      public static final double wheelCOF = 1.2; // coefficient of friction for the wheels; colsons on carpet is
                                                 // 1.0
      public static final double chassisLinearMoveDeadband = 0.02; // determined by calibration method
      public static final double chassisAngularMoveDeadband = 0.05; // determined by calibration method
      // Customize the following values to your prototype
      public static final double metersPerRotationFX = (5.589 / 89.11199955) * (5.589 / 5.716); // measure this number
                                                                                                // on the robot -
                                                                                                // remeasure on carpet
      // drive motor only
      public static final double degreePerRotationFX = (1.0 / 122.11575) * 2048; // Angle motor only
      // On our swerve prototype 1 angular rotation of
      // the wheel = 1 full rotation of the encoder

      /**
       * Drive Motor PID. Assumed to be the same for all drive motors
       * These PID constants are only used for auto trajectory driving, and not
       * teleop.
       * We found that changing them a bit will not have a substantial impact on the
       * trajectory with PathPlanner
       * even if a trajectory includes a holonomic component.
       */
      // public static final double DRIVE_CHASSIS_KP = 3.5;
      public static final double DRIVE_CHASSIS_KP = 0.1;
      public static final double DRIVE_CHASSIS_KI = 0.00;
      public static final double DRIVE_CHASSIS_KD = 0;

      /**
       * Angle Motor PID. Assumed to be the same for all angle motors
       * These PID constants are only used for auto trajectory driving, and not
       * teleop.
       * Changes to these constants will have a substantial impact on the precision of
       * your
       * trajectory if it includes holonomic rotation.
       * Make sure to test the values and adjust them as needed for your robot.
       */
      public static final double ANGLE_CHASSIS_KP = 0.1;
      public static final double ANGLE_CHASSIS_KI = 0.0;
      public static final double ANGLE_CHASSIS_KD = 0.0;

      public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * chassisLinearMoveDeadband)
          .withRotationalDeadband(MaxAngularRate * chassisAngularMoveDeadband) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

      /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
      public static final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
      /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
      public static final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

      public static enum SwerveModuleConstantsEnum {
        MOD0( // Front Left,
            3, // driveMotorID
            4, // angleMotorID
            31, // CanCoder Id
            // -0.296142578125, // angleOffset of cancoder to mark zero-position
            0.022582890625, // angleOffset of cancoder to mark zero-position
            false, // Inversion for drive motor
            false, // Inversion for angle motor
            false // inversion for CANcoder
        ),
        MOD1( // Front Right
            1, // driveMotorID
            2, // angleMotorID
            30, // CanCoder Id
            // 0.041015625, // angleOffset of cancoder to mark zero-position
            -0.3797604921875, // angleOffset of cancoder to mark zero-position
            true, // Inversion for drive motor
            false, // Inversion for angle motor
            false // inversion for CANcoder
        ),
        MOD2( // Back Left
            7, // driveMotorID
            8, // angleMotorID
            33, // CanCoder Id
            // -0.296142578125, // angleOffset of cancoder to mark zero-position
            0.421386796875, // angleOffset of cancoder to mark zero-position
            false, // Inversion for drive motor
            false, // Inversion for angle motor
            false // inversion for CANcoder
        ),
        MOD3( // Back Right
            5, // driveMotorID
            6, // angleMotorID
            32, // CanCoder Id
            // 0.326171875, // angleOffset of cancoder to mark zero-position
            // 0.0576171875, // angleOffset of cancoder to mark zero-position
            0.088256890625, // angleOffset of cancoder to mark zero-position
            true, // Inversion for drive motor
            false, // Inversion for angle motor
            false // inversion for CANcoder
        );

        private int driveMotorID;
        private int angleMotorID;
        private int cancoderID;
        private double angleOffset;
        private boolean driveMotorInverted;
        private boolean angleMotorInverted;
        private boolean cancoderInverted;

        SwerveModuleConstantsEnum(int d, int a, int c, double o,
            boolean di, boolean ai, boolean ci) {
          this.driveMotorID = d;
          this.angleMotorID = a;
          this.cancoderID = c;
          this.angleOffset = o;
          this.driveMotorInverted = di;
          this.angleMotorInverted = ai;
          this.cancoderInverted = ci;
        }

        public int getDriveMotorID() {
          return driveMotorID;
        }

        public int getAngleMotorID() {
          return angleMotorID;
        }

        public double getAngleOffset() {
          return angleOffset;
        }

        public boolean isDriveMotorInverted() {
          return driveMotorInverted;
        }

        public boolean isAngleMotorInverted() {
          return angleMotorInverted;
        }

        public boolean isCANCoderIverted() {
          return cancoderInverted;
        }

        public int getCancoderID() {
          return cancoderID;
        }

      } // End ENUM SwerveModuleConstants
    }
  }

  public static class IMUConstants {
    public static final int kPigeonId = 40; //adjust the id according to the robot (C2025 is 40)

		// Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
		private static final Pigeon2Configuration pigeonConfigs = null;
  }
}

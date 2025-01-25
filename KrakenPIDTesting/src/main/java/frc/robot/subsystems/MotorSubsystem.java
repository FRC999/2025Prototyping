// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.*;

public class MotorSubsystem extends SubsystemBase {
  private TalonFX motor;
  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    motor = new TalonFX(10);
  }

  public void goToIncrementalPositionDutyCycle(double position) {
    // robot init, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.01;
    motor.getConfigurator().apply(slot0Configs, 0.050);

    
    double finalPosition = getEncoder() + position;
    System.out.println("Start time: " + LocalTime.now());
    motor.setControl(new PositionDutyCycle(finalPosition));
    System.out.println("End time: " + LocalTime.now());
  }

  public void goToIncrementalPositionVoltage(double position) {
    double finalPosition = getEncoder() + position;
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.01;
    
    config.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));
    
    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    motor.setControl(positionVoltage.withPosition(finalPosition));
  }

  public void goToIncrementalPositionMotionMagic(double position) {
    double finalPosition = getEncoder() + position;
    MotionMagicDutyCycle motMagDutyCycle = new MotionMagicDutyCycle(0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    //config.Slot0.kS = 0.24; // add 0.24 V to overcome friction
    //config.Slot0.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    //PID on Position
    config.Slot0.kP = 0.64;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.MotionMagic.MotionMagicCruiseVelocity = 50;
    config.MotionMagic.MotionMagicAcceleration = 100;
    config.MotionMagic.MotionMagicJerk = 1000;

    motor.getConfigurator().apply(config, 0.050);

    motMagDutyCycle.Slot = 0;
    motor.setControl(motMagDutyCycle.withPosition(finalPosition));
  }

  public void runWithPower(double power) {
    System.out.println("Test2");
    motor.setControl(new DutyCycleOut(power));
    System.out.println("Test3");
  }

  public void stopMotor() {
    motor.setControl(new DutyCycleOut(0));
  }

  public double getEncoder() {
    return motor.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class CANrangeSubsystem extends SubsystemBase {
  CANrange cr = new CANrange(60);
  CANrangeConfiguration crConfiguration = new CANrangeConfiguration();
  StatusSignal distance = cr.getDistance();
  /** Creates a new CANRangeSubsystem. */
  public CANrangeSubsystem() {
    crConfiguration.withProximityParams(new ProximityParamsConfigs().withProximityThreshold(10.0));
    crConfiguration.withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.LongRangeUserFreq).withUpdateFrequency(5.0));
    cr.getConfigurator().apply(crConfiguration);
  } 

  @Override
  public void periodic() {
    SmartDashboard.putString("r: " , distance.refresh().toString());
    // This method will be called once per scheduler run
  }
}

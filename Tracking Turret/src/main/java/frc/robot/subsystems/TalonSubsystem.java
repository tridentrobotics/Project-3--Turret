// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TalonConstants;

/** Open-loop Talon SRX motor controller. */
public class TalonSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_motor = new WPI_TalonSRX(TalonConstants.kMotorCanId);

  public TalonSubsystem() {
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Brake);
  }

  /** Percent output, -1..1. */
  public void set(double output) {
    m_motor.set(ControlMode.PercentOutput, output);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Talon/Output", m_motor.get());
  }
}

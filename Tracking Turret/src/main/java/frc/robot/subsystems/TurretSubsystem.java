// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/** Open-loop turret: a motor on a Talon SRX rotating the camera about a vertical axis. */
public class TurretSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_motor = new WPI_TalonSRX(TurretConstants.kMotorCanId);

  public TurretSubsystem() {
    m_motor.configFactoryDefault();
    m_motor.setInverted(TurretConstants.kMotorInverted);
    m_motor.setNeutralMode(NeutralMode.Brake);
    // Default deadband (4%) would swallow the very slow search output.
    m_motor.configNeutralDeadband(0.001);
    m_motor.configSupplyCurrentLimit(
        new com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration(
            true, TurretConstants.kCurrentLimitAmps, TurretConstants.kCurrentLimitAmps, 0.0));
  }

  /** Positive output rotates the turret in the direction of increasing camera yaw (to the right). */
  public void set(double output) {
    double magnitude = Math.abs(output);
    double minMove = TurretConstants.kMinMoveOutput;
    if (magnitude > 0.0 && magnitude < minMove) {
      // Below the stiction threshold a steady output does nothing, so pulse at the minimum
      // moving output instead. Average speed scales with the duty cycle.
      double period = TurretConstants.kPulsePeriodSeconds;
      double duty = magnitude / minMove;
      boolean on = (Timer.getFPGATimestamp() % period) < duty * period;
      output = on ? Math.copySign(minMove, output) : 0.0;
    }
    m_motor.set(ControlMode.PercentOutput, output);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Output", m_motor.get());
  }
}

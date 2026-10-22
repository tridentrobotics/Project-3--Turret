// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/** Open-loop turret: a Bag motor on a SPARK MAX rotating the camera about a vertical axis. */
public class TurretSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(TurretConstants.kMotorCanId, MotorType.kBrushed);

  public TurretSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(TurretConstants.kMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.kCurrentLimitAmps);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Positive output rotates the turret in the direction of increasing camera yaw (to the right). */
  public void set(double output) {
    m_motor.set(output);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Output", m_motor.get());
  }
}

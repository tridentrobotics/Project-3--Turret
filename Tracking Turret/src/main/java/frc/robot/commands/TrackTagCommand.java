// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrackingConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.OptionalDouble;

/**
 * Rotates the turret to drive the AprilTag's yaw (as seen by the turret-mounted camera) to zero.
 * Because the camera rides on the turret, the yaw reading is directly the pointing error.
 */
public class TrackTagCommand extends Command {
  private final TurretSubsystem m_turret;
  private final VisionSubsystem m_vision;
  private final PIDController m_pid =
      new PIDController(TrackingConstants.kP, 0.0, TrackingConstants.kD);

  public TrackTagCommand(TurretSubsystem turret, VisionSubsystem vision) {
    m_turret = turret;
    m_vision = vision;
    // Only the turret is required; the vision subsystem is read-only.
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    m_pid.reset();
  }

  @Override
  public void execute() {
    OptionalDouble yaw = m_vision.getYawDegrees();
    if (yaw.isEmpty()) {
      m_pid.reset();
      m_turret.stop();
      return;
    }

    double error = yaw.getAsDouble();
    if (Math.abs(error) < TrackingConstants.kToleranceDegrees) {
      m_turret.stop();
      return;
    }

    // Measurement is the tag's yaw, setpoint is 0. Negate so a tag to the right (positive yaw)
    // gives a positive output, which turns the turret right.
    double output = -m_pid.calculate(error, 0.0);
    output = MathUtil.clamp(output, -TrackingConstants.kMaxOutput, TrackingConstants.kMaxOutput);
    if (Math.abs(output) < TrackingConstants.kMinOutput) {
      output = Math.copySign(TrackingConstants.kMinOutput, output);
    }
    m_turret.set(output);
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }
}

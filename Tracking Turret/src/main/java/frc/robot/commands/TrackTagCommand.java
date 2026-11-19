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

  /** True while centered and holding still, until the error exceeds the re-engage band. */
  private boolean m_holding = false;

  public TrackTagCommand(TurretSubsystem turret, VisionSubsystem vision) {
    m_turret = turret;
    m_vision = vision;
    // Only the turret is required; the vision subsystem is read-only.
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    m_pid.reset();
    m_holding = false;
  }

  @Override
  public void execute() {
    OptionalDouble yaw = m_vision.getYawDegrees();
    if (yaw.isEmpty()) {
      // No tag in view: sweep toward the side it was last seen leaving (left if never seen).
      m_pid.reset();
      m_holding = false;
      m_turret.set(m_vision.getSearchDirection() * TrackingConstants.kSearchOutput);
      return;
    }

    double error = yaw.getAsDouble();
    // Hysteresis: once centered, hold still until the error grows past the wider re-engage
    // band, so camera latency and noise near the center can't trigger back-and-forth hunting.
    double threshold =
        m_holding ? TrackingConstants.kReengageDegrees : TrackingConstants.kToleranceDegrees;
    if (Math.abs(error) < threshold) {
      m_holding = true;
      m_pid.reset();
      m_turret.stop();
      return;
    }
    m_holding = false;

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.OptionalDouble;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Reads AprilTag detections from a PhotonVision camera mounted on the turret. */
public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera = new PhotonCamera(VisionConstants.kCameraName);

  private double m_yawDegrees = 0.0;
  private double m_lastSeenSeconds = Double.NEGATIVE_INFINITY;

  /** -1 = tag was last off to the left, +1 = to the right. Defaults to left. */
  private double m_searchDirection = -1.0;

  @Override
  public void periodic() {
    for (PhotonPipelineResult result : m_camera.getAllUnreadResults()) {
      PhotonTrackedTarget target = selectTarget(result);
      if (target != null) {
        m_yawDegrees = target.getYaw();
        m_lastSeenSeconds = Timer.getFPGATimestamp();
        // Only remember a side when the tag was clearly off-center, so a tag that vanishes while
        // centered keeps the earlier direction instead of a noisy sign.
        if (Math.abs(m_yawDegrees) >= VisionConstants.kDirectionMinYawDegrees) {
          m_searchDirection = Math.signum(m_yawDegrees);
        }
      }
    }

    SmartDashboard.putBoolean("Vision/HasTarget", getYawDegrees().isPresent());
    SmartDashboard.putNumber("Vision/YawDegrees", m_yawDegrees);
  }

  private PhotonTrackedTarget selectTarget(PhotonPipelineResult result) {
    if (!result.hasTargets()) {
      return null;
    }
    if (VisionConstants.kTargetTagId < 0) {
      return result.getBestTarget();
    }
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == VisionConstants.kTargetTagId) {
        return target;
      }
    }
    return null;
  }

  /** Direction to search after losing the tag: +1 = right, -1 = left (the side it left from). */
  public double getSearchDirection() {
    return m_searchDirection;
  }

  /**
   * Yaw to the tracked tag in degrees, positive when the tag is to the right of the camera's
   * centerline. Empty if the tag hasn't been seen recently.
   */
  public OptionalDouble getYawDegrees() {
    boolean fresh =
        Timer.getFPGATimestamp() - m_lastSeenSeconds < VisionConstants.kTargetTimeoutSeconds;
    return fresh ? OptionalDouble.of(m_yawDegrees) : OptionalDouble.empty();
  }
}

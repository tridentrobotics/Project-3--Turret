// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TurretConstants {
    /** CAN ID of the SPARK MAX driving the Bag motor. */
    public static final int kMotorCanId = 1;

    /** Flip if a positive output turns the turret counter-clockwise (viewed from above). */
    public static final boolean kMotorInverted = false;

    /** Bag motors stall hard; keep the current limit low. */
    public static final int kCurrentLimitAmps = 20;
  }

  public static class TalonConstants {
    /** CAN ID of the Talon SRX. */
    public static final int kMotorCanId = 12;

    /** Set true to spin the Talon at {@link #kEnabledOutput} whenever the robot is enabled. */
    public static final boolean kSpinWhenEnabled = false;

    /** Percent output (-1..1) applied while the robot is enabled. */
    public static final double kEnabledOutput = 0.1;
  }

  public static class VisionConstants {
    /** Camera name exactly as shown in the PhotonVision UI. */
    public static final String kCameraName = "turret_cam";

    /** AprilTag ID to follow. Set to -1 to follow whichever tag PhotonVision picks as best. */
    public static final int kTargetTagId = -1;

    /** Seconds without a fresh detection before we consider the tag lost. */
    public static final double kTargetTimeoutSeconds = 0.25;
  }

  public static class TrackingConstants {
    /** Motor output (-1..1) per degree of yaw error. Tune this first. */
    public static final double kP = 0.02;

    /** Optional derivative gain to reduce overshoot. Leave 0 until kP is tuned. */
    public static final double kD = 0.0;

    /** Yaw error (degrees) inside which the turret holds still. */
    public static final double kToleranceDegrees = 1.0;

    /** Output clamp so the turret doesn't whip around. */
    public static final double kMaxOutput = 0.4;

    /** Minimum output to overcome friction/stiction while outside tolerance. */
    public static final double kMinOutput = 0.05;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TalonConstants;
import frc.robot.commands.TrackTagCommand;
import frc.robot.subsystems.TalonSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final TalonSubsystem m_talon = new TalonSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Spin the Talon SRX at a constant output for as long as the robot is enabled.
    new Trigger(DriverStation::isEnabled)
        .whileTrue(
            Commands.runEnd(
                () -> m_talon.set(TalonConstants.kEnabledOutput), m_talon::stop, m_talon));

    // Track the AprilTag whenever nothing else is using the turret.
    m_turret.setDefaultCommand(new TrackTagCommand(m_turret, m_vision));

    // Hold A to jog the turret with the left stick (also handy for checking motor direction:
    // pushing right should turn the turret toward increasing camera yaw).
    m_driverController
        .a()
        .whileTrue(
            Commands.runEnd(
                () -> m_turret.set(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1) * 0.4),
                m_turret::stop,
                m_turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Tracking runs as the turret's default command, so autonomous has nothing extra to do.
    return Commands.none();
  }
}

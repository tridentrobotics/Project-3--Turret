# Copilot Instructions

FRC robot project (team 4296, WPILib 2026, Java). A single-axis AprilTag-tracking turret: a brushed Bag motor on a SPARK MAX rotates a PhotonVision camera about a vertical axis and keeps a tag centered.

The Gradle/WPILib project lives in the `Tracking Turret/` subdirectory, not the repo root. Run Gradle commands from there.

## Build and deploy

- Build: `./gradlew build` (from `Tracking Turret/`)
- Simulate: `./gradlew simulateJava`
- Deploy: `./gradlew deploy` (requires connection to the roboRIO)
- Use the WPILib VS Code extension commands where possible (Build Robot Code, Deploy Robot Code).

## Architecture

Command-based framework (`WPILibNewCommands`). Code is under `Tracking Turret/src/main/java/frc/robot/`:

- `Robot.java` / `Main.java`: lifecycle and scheduler calls only. Keep logic out of them.
- `RobotContainer.java`: constructs subsystems, sets default commands, and maps controller buttons to commands.
- `subsystems/TurretSubsystem.java`: open-loop motor control (`set`, `stop`). Positive output turns toward increasing camera yaw (right).
- `subsystems/VisionSubsystem.java`: wraps PhotonVision camera results.
- `commands/TrackTagCommand.java`: turret's default command; P(D) control on yaw error with tolerance, min/max output clamps, and a target-lost timeout.
- `ExampleCommand`, `ExampleSubsystem`, `Autos`: template leftovers; don't build on them.

## Conventions

- All tunable numbers and hardware IDs go in `Constants.java`, grouped in nested classes (`TurretConstants`, `VisionConstants`, `TrackingConstants`, `OperatorConstants`). Constants use the `k` prefix (`kMotorCanId`) with a Javadoc comment giving units and tuning guidance. No functional code in `Constants`.
- Member fields use the `m_` prefix (`m_turret`, `m_motor`).
- Two-space indentation, Google-style Java formatting, and the WPILib license header on each file.
- Subsystems expose small imperative methods (or command factories); commands depend on subsystems via constructor injection and call `addRequirements`.
- Prefer `Commands.*` factories and inline commands for simple behavior; create a command class when there is state or non-trivial logic.
- Publish telemetry with `SmartDashboard` using `Subsystem/Value` keys (e.g. `Turret/Output`).
- Hardware config goes in the subsystem constructor using REVLib 2026 APIs (`SparkMaxConfig`, `ResetMode.kResetSafeParameters`, `PersistMode.kPersistParameters`). Keep a current limit set (brushed Bag motors stall hard).
- Don't block in `periodic()` or commands: no `Thread.sleep`, no long loops.
- Handle "no target" explicitly in vision-driven code and stop the motor when the tag is lost.

## Dependencies

Vendor deps are in `Tracking Turret/vendordeps/`: REVLib, WPILibNewCommands, PhotonLib. Don't hand-edit these JSON files; update them through the WPILib "Manage Vendor Libraries" tool.

## Hardware assumptions

- SPARK MAX at CAN ID 1 driving a brushed motor (`MotorType.kBrushed`).
- PhotonVision camera named `turret_cam` (must match the PhotonVision UI).
- Driver controller on port 0 (Xbox). Hold A to jog the turret with the left stick.

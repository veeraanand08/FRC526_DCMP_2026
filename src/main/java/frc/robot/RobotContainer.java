// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.util.RobotUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // subsystems
  private final Drive drive;
//  private final Vision vision;
//  private final Shooter shooter;
//  private final Feeder feeder;
//  private final Intake intake;

  // controllers
  private final CommandXboxController driverController =
          new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
          new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (currentMode) {
      case REAL:
        drive =
                new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
        break;
      case SIM:
        drive =
                new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
        break;
      default:
        // replay
        drive =
                new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
    }

    // Configure the trigger bindings
    configureBindings();

    configureAutoCommands();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    // Set the default auto (do nothing)
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
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
    RobotUtil.setDriverController(driverController);
    RobotUtil.setOperatorController(operatorController);

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX()));

    // Switch to X pattern
    driverController.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0°
    driverController.povLeft().onTrue(
            Commands.runOnce(
                    () -> drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                            drive)
                    .ignoringDisable(true));
  }

  private void configureAutoCommands() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
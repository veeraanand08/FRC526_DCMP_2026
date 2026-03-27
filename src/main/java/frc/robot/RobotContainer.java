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
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterFallback;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.RobotUtil;
import frc.robot.util.autoalign.AutoAlign;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
  private final Vision vision;
  private final Shooter shooter;
  private final Feeder feeder;
  private final Intake intake;

  private SwerveDriveSimulation driveSimulation;

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
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (pose) -> {});
        vision = new Vision(
                drive,
                new VisionIOPhotonVision(VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(VisionConstants.CAMERA_1_NAME, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(VisionConstants.CAMERA_2_NAME, VisionConstants.robotToCamera2),
                new VisionIOPhotonVision(VisionConstants.CAMERA_3_NAME, VisionConstants.robotToCamera3)
        );
        shooter = new Shooter(
                new ShooterIOTalonFX(),
                drive::getPose,
                drive::getChassisSpeeds);
        feeder = new Feeder(new FeederIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        break;
      case SIM:
        SimulatedArena.getInstance().resetFieldForAuto();
        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
                new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                        new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
        vision = new Vision(
                drive,
//                new VisionIOPhotonVisionSim(VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0, drive::getPose),
//                new VisionIOPhotonVisionSim(VisionConstants.CAMERA_1_NAME, VisionConstants.robotToCamera1, drive::getPose),
//                new VisionIOPhotonVisionSim(VisionConstants.CAMERA_2_NAME, VisionConstants.robotToCamera2, drive::getPose),
//                new VisionIOPhotonVisionSim(VisionConstants.CAMERA_3_NAME, VisionConstants.robotToCamera3, drive::getPose)
                new VisionIO() {}
        );
        shooter = new Shooter(
                new ShooterIOSim(),
                drive::getPose,
                drive::getChassisSpeeds);
        feeder = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim());
        break;
      default:
        // replay
        drive =
                new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
        vision = new Vision(
                drive,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {}
        );
        shooter = new Shooter(
                new ShooterIO() {},
                drive::getPose,
                drive::getChassisSpeeds);
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
    }

    // Configure the trigger bindings
    configureBindings();

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
    Command defaultDriveCommand =
            DriveCommands.joystickDrive(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX());
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() ->
            drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
            drive).ignoringDisable(true);
    Command autoAlign =
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> AutoAlign.getAngleToTarget(
                            AutoAlign.Target.AUTO,
                            drive.getPose().getTranslation(),
                            drive.getChassisSpeeds()
                    )
            ).finallyDo(AutoAlign::disable);
    Command holdIntake = intake.intakeCommand();
    Command agitate = intake.agitateCommand();
    Command dump = Commands.parallel(intake.reverseIntakeCommand(), feeder.reverseCommand());
    Command resetIntake = intake.resetIntakeCommand();
    Command shoot = new ShooterCommand(shooter, feeder);
    Command shootDefault = new ShooterFallback(shooter, feeder);

    drive.setDefaultCommand(defaultDriveCommand);

    if (DriverStation.isTest()) {
      // single controller for testing
      driverController.a().whileTrue(autoAlign);
      driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive)); // Switch to X pattern
      driverController.povLeft().onTrue(zeroGyro);

      driverController.leftBumper().whileTrue(holdIntake);
      driverController.rightBumper().whileTrue(shoot);
      driverController.rightTrigger(0.7).whileTrue(shootDefault);
      driverController.y().onTrue(agitate);
      driverController.b().whileTrue(dump);
      driverController.povUp().onTrue(resetIntake);
    }
    else {
      // driver controls
      driverController.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive)); // Switch to X pattern
      driverController.povLeft().onTrue(zeroGyro);
      driverController.a().whileTrue(autoAlign);

      // operator controls
      operatorController.leftBumper().whileTrue(holdIntake);
      operatorController.rightBumper().whileTrue(shoot);
      operatorController.rightTrigger(0.7).whileTrue(shootDefault);
      operatorController.a().onTrue(agitate);
      operatorController.b().whileTrue(dump);
      operatorController.povUp().onTrue(resetIntake);
    }
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
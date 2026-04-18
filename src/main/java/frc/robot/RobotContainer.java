// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterFallback;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.RobotUtil;
import frc.robot.util.autoalign.AutoAlign;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private IntakeIOSimMaple maplesimIntake;
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
    maplesimIntake = null;

    switch (currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {});
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_1_NAME, VisionConstants.robotToCamera1),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_2_NAME, VisionConstants.robotToCamera2),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_3_NAME, VisionConstants.robotToCamera3));
        shooter = new Shooter(new ShooterIOTalonFX(), drive::getPose, drive::getChassisSpeeds);
        feeder = new Feeder(new FeederIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        break;
      case SIM:
        SimulatedArena.getInstance().resetFieldForAuto();
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                //                                new VisionIOPhotonVisionSim(
                //                                    VisionConstants.CAMERA_0_NAME,
                // VisionConstants.robotToCamera0,
                //                 drive::getPose),
                //                                new VisionIOPhotonVisionSim(
                //                                    VisionConstants.CAMERA_1_NAME,
                // VisionConstants.robotToCamera1,
                //                 drive::getPose),
                //                                new VisionIOPhotonVisionSim(
                //                                    VisionConstants.CAMERA_2_NAME,
                // VisionConstants.robotToCamera2,
                //                 drive::getPose),
                //                                new VisionIOPhotonVisionSim(
                //                                    VisionConstants.CAMERA_3_NAME,
                // VisionConstants.robotToCamera3,
                //                 drive::getPose));
                new VisionIO() {});
        shooter = new Shooter(new ShooterIOSim(), drive::getPose, drive::getChassisSpeeds);
        feeder = new Feeder(new FeederIOSim());
        maplesimIntake = new IntakeIOSimMaple(driveSimulation);
        intake = new Intake(maplesimIntake);
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
        vision =
            new Vision(
                drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {}, drive::getPose, drive::getChassisSpeeds);
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
    }

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set the default auto (do nothing)
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

    DriverStation.silenceJoystickConnectionWarning(true);

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
    RobotUtil.setDriverController(driverController);
    RobotUtil.setOperatorController(operatorController);

    // Default command, normal field-relative drive
    Command defaultDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX());
    // Lock wheels to X pattern
    Command lockWheels = Commands.runOnce(drive::stopWithX, drive);
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() -> drive.zeroGyro(true), drive).ignoringDisable(true);
    Command autoAlign =
        DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    AutoAlign.getAngleToTarget(
                        AutoAlign.Target.AUTO,
                        drive.getPose().getTranslation(),
                        drive.getChassisSpeeds()))
            .finallyDo(AutoAlign::disable);
    Command holdIntake = intake.intakeCommand();
    Command agitate = intake.agitateCommand();
    Command dump = Commands.parallel(intake.reverseIntakeCommand(), feeder.reverseCommand());
    Command resetIntake = intake.resetIntakeCommand();
    Command shoot = new ShooterCommand(shooter, feeder);
    Command shootDefault = new ShooterFallback(shooter, feeder);

    drive.setDefaultCommand(defaultDriveCommand);

    if (Constants.currentMode == Constants.Mode.SIM) {
      CommandGenericHID keyboard = new CommandGenericHID(0);
      keyboard.button(1).whileTrue(holdIntake);

      Command keyboardShootRepeat =
          Commands.repeatingSequence(
              // The actual shot logic
              Commands.runOnce(
                  () -> {
                    maplesimIntake.shoot(driveSimulation, drive);
                  }),
              Commands.waitSeconds(0.1));

      keyboard.button(2).whileTrue(keyboardShootRepeat);
      keyboard.button(3).whileTrue(autoAlign);
      keyboard.button(4).onTrue(resetIntake);
    }

    if (DriverStation.isTest()) {
      // single controller for testing
      driverController.a().whileTrue(autoAlign);
      driverController.x().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);

      driverController.leftBumper().whileTrue(holdIntake);
      driverController.rightBumper().whileTrue(shoot);
      driverController.rightTrigger(0.7).whileTrue(shootDefault);
      driverController.y().onTrue(agitate);
      driverController.b().whileTrue(dump);
      driverController.povUp().onTrue(resetIntake);
    } else {
      // driver controls
      driverController.a().whileTrue(autoAlign);
      driverController.leftBumper().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);

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

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/RobotPosition",
    driveSimulation.getSimulatedDriveTrainPose());
    // Logger.recordOutput("FieldSimulation/RobotPosition", new Pose2d(0, 0, Rotation2d.kZero)); // to setup the model
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);

    double hopperDistAdded =
        Math.sin(Math.toRadians(intake.getPivotPosition())) * 0.365; // intake length
    hopperDistAdded = hopperDistAdded > 0 ? hopperDistAdded : 0;
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPositions",
        new Pose3d[] {
          new Pose3d(
              -0.27,
              0,
              0.21,
              new Rotation3d(0, -Math.toRadians(intake.getPivotPosition() - 17.5), 0)),
          new Pose3d(-.3 - hopperDistAdded, 0, 0, new Rotation3d(0, 0, 0))
          // new Pose3d[] {new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)) //to setup the model
        });
  }
}

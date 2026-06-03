// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.currentMode;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.ShooterFallback;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.SuperstructureSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.BetterAutoChooser;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RobotBumpSim;
import frc.robot.util.RobotUtil;
import frc.robot.util.sotm.ShootingTasks;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
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

  // controllers
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Simulated things
  private SwerveDriveSimulation driveSimulation;
  private SuperstructureSim superstructureSim;
  private RobotBumpSim robotBumpSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (currentMode) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_0_NAME, VisionConstants.CAMERA_0_OFFSET),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_1_NAME, VisionConstants.CAMERA_1_OFFSET),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_2_NAME, VisionConstants.CAMERA_2_OFFSET));
        shooter = new Shooter(drive::getPose, drive::getChassisSpeeds);
        feeder = new Feeder();
        intake = new Intake();
      }
      case SIM -> {
        Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
        arena.setEfficiencyMode(true); // set to true to limit # of balls
        SimulatedArena.overrideInstance(arena);
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
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_0_NAME,
                    VisionConstants.CAMERA_0_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_1_NAME,
                    VisionConstants.CAMERA_1_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_2_NAME,
                    VisionConstants.CAMERA_2_OFFSET,
                    driveSimulation::getSimulatedDriveTrainPose));
        shooter = new Shooter(drive::getPose, drive::getChassisSpeeds);
        feeder = new Feeder();
        intake = new Intake();
        superstructureSim =
            new SuperstructureSim(
                intake, driveSimulation, drive::getChassisSpeeds, shooter::getSetpointRPS);
        robotBumpSim = new RobotBumpSim(Drive.getModuleTranslations());
      }
      default -> {
        /* REPLAY */
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
        shooter = new Shooter(drive::getPose, drive::getChassisSpeeds);
        feeder = new Feeder();
        intake = new Intake();
        // for robot component poses
        superstructureSim =
            new SuperstructureSim(
                intake, driveSimulation, drive::getChassisSpeeds, shooter::getVelocityRPS);
      }
    }

    PhoenixUtil.startTelemetry();

    // Configure the trigger bindings
    configureBindings();

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser", BetterAutoChooser.buildAutoChooser());

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
    Command lockWheels = Commands.startEnd(drive::stopWithX, () -> {}, drive);
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() -> drive.zeroGyro(true), drive).ignoringDisable(true);
    Command autoAlign =
        DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> {
                  if (!shooter.isShooting()) shooter.computeShot();
                  return shooter.getShot().driveAngle();
                })
            .beforeStarting(() -> ShootingTasks.isAutoAlignRunning = true)
            .finallyDo(
                () -> {
                  ShootingTasks.clearTarget();
                  ShootingTasks.isAutoAlignRunning = false;
                });
    Command holdIntake = intake.intakeCommand();
    Command agitate = intake.agitateCommand(feeder);
    Command dump = Commands.parallel(intake.reverseIntakeCommand(), feeder.reverseCommand());
    Command resetIntake = intake.resetIntakeCommand();
    Command shoot = new ShooterCommand(shooter, feeder, intake);
    if (Constants.currentMode == Constants.Mode.SIM) {
      shoot = shoot.alongWith(superstructureSim.shootCommand());
    }
    Command shootDefault = new ShooterFallback(shooter, feeder);

    new EventTrigger("Intake").whileTrue(holdIntake);
    NamedCommands.registerCommand("Auto Align", autoAlign);
    NamedCommands.registerCommand("Shoot", shoot);

    drive.setDefaultCommand(defaultDriveCommand);

    if (Constants.currentMode == Constants.Mode.SIM) {
      CommandGenericHID keyboard = new CommandGenericHID(2);
      keyboard.button(1).whileTrue(holdIntake);
      keyboard.button(2).whileTrue(shoot);
      keyboard.button(3).whileTrue(autoAlign);
      keyboard.button(4).onTrue(resetIntake);
      keyboard.button(5).toggleOnTrue(agitate);
    }

    if (DriverStation.isTest()) {
      // single controller for testing
      driverController.a().whileTrue(autoAlign);
      driverController.x().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);

      driverController.leftBumper().whileTrue(holdIntake);
      driverController.rightBumper().whileTrue(shoot);
      driverController.rightTrigger(0.7).whileTrue(shootDefault);
      driverController.y().toggleOnTrue(agitate);
      driverController.b().whileTrue(dump);
      driverController.povUp().onTrue(resetIntake);
      driverController
          .povRight()
          .debounce(2, Debouncer.DebounceType.kRising)
          .whileTrue(intake.markIntakeLowered().ignoringDisable(true));
    } else {
      // driver controls
      driverController.a().whileTrue(autoAlign);
      driverController.leftBumper().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);

      // operator controls
      operatorController.leftBumper().whileTrue(holdIntake);
      operatorController.rightBumper().whileTrue(shoot);
      operatorController.rightTrigger(0.7).whileTrue(shootDefault);
      operatorController.a().toggleOnTrue(agitate);
      operatorController.b().whileTrue(dump);
      operatorController.povUp().onTrue(resetIntake);
      operatorController
          .povRight()
          .debounce(2, Debouncer.DebounceType.kRising)
          .whileTrue(intake.markIntakeLowered().ignoringDisable(true));
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
    if (Constants.currentMode == Constants.Mode.REAL) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");

    Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();

    ChassisSpeeds fieldRelativeSpeeds =
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();

    Pose3d simPose3d = robotBumpSim.update(simPose, fieldRelativeSpeeds, 5);

    if (robotBumpSim.isOnRamp()) {
      driveSimulation.setSimulationWorldPose(robotBumpSim.getSimWorldPose(simPose));
    }

    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/RobotPosition", simPose3d);
    // to set up the model
    Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
  }
}

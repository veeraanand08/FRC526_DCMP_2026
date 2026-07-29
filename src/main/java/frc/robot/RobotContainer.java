// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControlScheme;
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
import frc.robot.Constants.ControlScheme;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.SuperstructureSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.BetterAutoChooser;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RobotBumpSim;
import frc.robot.util.RobotUtil;
import frc.robot.util.io.GuitarHeroController;
import frc.robot.util.sotm.ShootingTasks;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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
  private ControlScheme controlScheme = ControlScheme.SAXON_SPARKS;
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  private GuitarHeroController guitarHeroController;

  // default drive commands
  private Command defaultDriveCommand;
  private Command guitarHeroDriveCommand;

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
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft, false),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight, false),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft, false),
                new ModuleIOTalonFXReal(TunerConstants.BackRight, false),
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
        LED.createInstance(shooter);
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

    LoggedDashboardChooser<ControlScheme> controlProfiles =
        new LoggedDashboardChooser<>("Control Profile");
    controlProfiles.addDefaultOption("526 (Ben & Iris)", ControlScheme.SAXON_SPARKS);
    controlProfiles.addOption("611 (Josie & Harun)", ControlScheme.SAXONS);
    controlProfiles.addOption("Guitar Hero Operator", ControlScheme.GUITAR_HERO_OP);
    controlProfiles.addOption("Guitar Hero Full Control", ControlScheme.GUITAR_HERO_FULL);
    controlProfiles.addOption("Testing", ControlScheme.TEST);
    controlProfiles.onChange(this::setControlScheme);

    // Set up commands for PathPlanner
    configureAutoCommands();

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser", BetterAutoChooser.buildAutoChooser());

    // Set up SysId routines
    //    autoChooser.addOption(
    //        "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    //    autoChooser.addOption(
    //        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    //    autoChooser.addOption(
    //        "Drive SysId (Quasistatic Forward)",
    //        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //    autoChooser.addOption(
    //        "Drive SysId (Quasistatic Reverse)",
    //        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //    autoChooser.addOption(
    //        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //    autoChooser.addOption(
    //        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set up custom autos (non-PathPlanner)
    autoChooser.addOption("Full System Check", Autos.systemCheck(drive, shooter, feeder, intake));
    //    autoChooser.addOption(
    //        "Dynamic Left Cycle", Autos.leftCycle(drive, vision, shooter, feeder, intake));
    //    autoChooser.addOption(
    //        "Dynamic Right Cycle", Autos.rightCycle(drive, vision, shooter, feeder, intake));

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

    // Lock wheels to X pattern
    Command lockWheels = Commands.startEnd(drive::stopWithX, () -> {}, drive);
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() -> drive.zeroGyro(true), drive).ignoringDisable(true);
    Command autoAlign =
        getAutoAlignCommand(() -> -driverController.getLeftY(), () -> -driverController.getLeftX());
    Command holdIntake = intake.intake();
    Command agitate = intake.agitate(feeder);
    Command dump = Commands.parallel(intake.reverse(), feeder.reverseCommand());
    Command resetIntake = intake.stow();
    Command shoot = shooter.shoot(feeder);
    if (Constants.currentMode == Constants.Mode.SIM) {
      shoot = shoot.alongWith(superstructureSim.shootCommand());
    }
    Command shootDefault = shooter.shootDefault(feeder);

    new Trigger(() -> ShootingTasks.isAutoAlignRunning && !shooter.isShooting())
        .whileTrue(shooter.spinUp());

    // Default command, normal field-relative drive
    useDefaultDrive();

    if (currentMode == Constants.Mode.SIM) {
      CommandGenericHID keyboard = new CommandGenericHID(3);
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
      /* driver controls */
      driverController.a().whileTrue(autoAlign);
      driverController.x().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);

      /* operator controls */
      operatorController.leftBumper().whileTrue(holdIntake);
      operatorController.povUp().onTrue(resetIntake);
      operatorController
          .povRight()
          .debounce(0.3, Debouncer.DebounceType.kRising)
          .whileTrue(intake.markIntakeLowered().ignoringDisable(true));
      // test mode (single controller)
      BooleanSupplier testMode = () -> controlScheme == ControlScheme.TEST;
      driverController.leftBumper().and(testMode).whileTrue(holdIntake);
      driverController.rightBumper().and(testMode).whileTrue(shoot);
      driverController.rightTrigger(0.7).and(testMode).whileTrue(shootDefault);
      driverController.y().and(testMode).toggleOnTrue(agitate);
      driverController.b().and(testMode).whileTrue(dump);
      driverController.povUp().and(testMode).onTrue(resetIntake);
      driverController
          .povRight()
          .and(testMode)
          .debounce(0.3, Debouncer.DebounceType.kRising)
          .whileTrue(intake.markIntakeLowered().ignoringDisable(true));
      // 526 profile
      BooleanSupplier saxonSparksProfile = () -> controlScheme == ControlScheme.SAXON_SPARKS;
      operatorController.rightBumper().and(saxonSparksProfile).whileTrue(shoot);
      operatorController.rightTrigger(0.7).and(saxonSparksProfile).whileTrue(shootDefault);
      operatorController.a().and(saxonSparksProfile).toggleOnTrue(agitate);
      operatorController.b().and(saxonSparksProfile).whileTrue(dump);
      // 611 profile
      BooleanSupplier saxonsProfile = () -> controlScheme == ControlScheme.SAXONS;
      operatorController.rightTrigger(0.7).and(saxonsProfile).whileTrue(shoot);
      operatorController.rightBumper().and(saxonsProfile).whileTrue(shootDefault);
      operatorController.a().and(saxonsProfile).whileTrue(dump);
      operatorController.b().and(saxonsProfile).toggleOnTrue(agitate);
    }
  }

  public void setControlScheme(ControlScheme newScheme) {
    switch (newScheme) {
      case SAXONS:
      case SAXON_SPARKS:
      case TEST:
        if (controlScheme == ControlScheme.GUITAR_HERO_FULL) {
          useDefaultDrive();
        }
        break;
      case GUITAR_HERO_OP:
        configureGuitarHeroController(false);
        if (controlScheme == ControlScheme.GUITAR_HERO_FULL) {
          useDefaultDrive();
        }
        break;
      case GUITAR_HERO_FULL:
        configureGuitarHeroController(true);
        break;
      default:
    }
    controlScheme = newScheme;
  }

  private void configureGuitarHeroController(boolean fullControl) {
    if (guitarHeroController == null) {
      // lazy instantiation
      guitarHeroController =
          new GuitarHeroController(ControllerConstants.GUITAR_HERO_CONTROLLER_PORT);

      // configure triggers only once
      Command autoAlign =
          getAutoAlignCommand(
              () -> -guitarHeroController.getJoystickY(),
              () -> -guitarHeroController.getJoystickX());
      Command lockWheels = Commands.startEnd(drive::stopWithX, () -> {}, drive);
      Command shoot = shooter.shoot(feeder);
      if (currentMode == Constants.Mode.SIM) {
        shoot = shoot.alongWith(superstructureSim.shootCommand());
      }

      // controls are only active during the correct mode
      BooleanSupplier guitarHeroControls = () -> controlScheme.isGuitarHero;
      BooleanSupplier guitarHeroDrive = () -> controlScheme == ControlScheme.GUITAR_HERO_FULL;
      guitarHeroController.green().and(guitarHeroControls).whileTrue(intake.intake());
      guitarHeroController.red().and(guitarHeroControls).whileTrue(shoot);
      guitarHeroController.yellow().and(guitarHeroDrive).whileTrue(autoAlign);
      guitarHeroController.blue().and(guitarHeroDrive).whileTrue(lockWheels);
    }

    if (fullControl) {
      useGuitarHeroDrive();
    }
  }

  private void configureAutoCommands() {
    Command autoAlign =
        DriveCommands.aimAtAngle(
                drive,
                () -> {
                  shooter.computeShot();
                  return shooter.getShot().driveAngle();
                })
            .beforeStarting(
                () -> {
                  ShootingTasks.isAutoAlignRunning = true;
                })
            .finallyDo(
                () -> {
                  ShootingTasks.clearTarget();
                  ShootingTasks.isAutoAlignRunning = false;
                });

    new EventTrigger("Intake").whileTrue(intake.intake());
    NamedCommands.registerCommand("Auto Align", autoAlign);
    NamedCommands.registerCommand("Shoot", shooter.shoot(feeder));
    NamedCommands.registerCommand("Agitate", intake.agitate(feeder));
  }

  private void useDefaultDrive() {
    if (defaultDriveCommand == null) {
      defaultDriveCommand =
          DriveCommands.joystickDrive(
              drive,
              () -> -driverController.getLeftY(),
              () -> -driverController.getLeftX(),
              () -> -driverController.getRightX());
    }
    drive.setDefaultCommand(defaultDriveCommand);
    Command currentDriveCommand = drive.getCurrentCommand();
    if (currentDriveCommand != null) currentDriveCommand.cancel();
  }

  private void useGuitarHeroDrive() {
    if (guitarHeroDriveCommand == null) {
      guitarHeroDriveCommand =
          DriveCommands.joystickDrive(
              drive,
              () -> -guitarHeroController.getJoystickY(),
              () -> -guitarHeroController.getJoystickX(),
              () -> -guitarHeroController.getStrumBarAxis());
    }
    drive.setDefaultCommand(guitarHeroDriveCommand);
    Command currentDriveCommand = drive.getCurrentCommand();
    if (currentDriveCommand != null) currentDriveCommand.cancel();
  }

  private Command getAutoAlignCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              if (!shooter.isShooting()) shooter.computeShot();
              return shooter.getShot().driveAngle();
            },
            () -> shooter.getShot().driveAngularVelocityRadPerSec())
        .beforeStarting(
            () -> {
              ShootingTasks.isAutoAlignRunning = true;
              drive.setSpeedLimiter(true);
            })
        .finallyDo(
            () -> {
              ShootingTasks.clearTarget();
              ShootingTasks.isAutoAlignRunning = false;
              drive.setSpeedLimiter(false);
            });
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

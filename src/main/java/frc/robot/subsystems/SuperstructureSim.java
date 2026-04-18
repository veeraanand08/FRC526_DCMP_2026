package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSim {
  private final IntakeSimulation intakeSimulation;
  private final Intake intake;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final Drive drive;

  public SuperstructureSim(
      Intake intake, SwerveDriveSimulation swerveDriveSimulation, Drive drive) {
    this.intake = intake;
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.drive = drive;
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            swerveDriveSimulation,
            // Width of the intake
            Meters.of(0.7),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.25),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 40 fuel
            40);
  }

  public void shoot() {
    if (intakeSimulation.obtainGamePieceFromIntake()) {
      launchFuel();
    }
  }

  public void launchFuel() {
    // if there is a note in the intake, it will be removed and return true; otherwise, returnsfalse
    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            // Specify the position of the chassis when the note is launched
            swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, 0),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
            // the projectile
            drive.getChassisSpeeds(),
            // The shooter facing direction is the same as the robot’s facing direction
            swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
            // Add the shooter’s rotation
            // + Translation2d.kZero,
            // Initial height of the flying note
            Meters.of(.45),
            // The launch speed is proportional to the RPM; assumed to be 8 meters/second at 6000
            // RPM
            MetersPerSecond.of(4000.0 / 6000 * 10),
            // The angle at which the fuel is launched
            Degrees.of(65));
    fuelOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
        // Callback for when the fuel will eventually hit the target (if configured)
        (pose3ds) ->
            Logger.recordOutput(
                "Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
        // Callback for when the fuel will eventually miss the target, or if no target is configured
        (pose3ds) ->
            Logger.recordOutput(
                "Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new)));

    fuelOnFly
        // Configure the note projectile to become a NoteOnField upon touching the ground
        .enableBecomesGamePieceOnFieldAfterTouchGround();

    // Add the projectile to the simulated arena
    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
  }

  public void updateSuperstructureSim() {
    if (intake.getRollerRPS() > 83.0) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }

    double hopperDistAdded =
        Math.sin(Math.toRadians(intake.getPivotPosition() - 17.5)) * 0.3; // intake length
    hopperDistAdded = hopperDistAdded > 0 ? hopperDistAdded : 0;
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPositions",
        new Pose3d[] {
          new Pose3d(
              -0.27,
              0,
              0.21,
              new Rotation3d(
                  0, -Math.toRadians(intake.getPivotPosition() - 17.5), 0)), // intake pivot
          new Pose3d(-.3 - hopperDistAdded, 0, 0, new Rotation3d(0, 0, 0)) // hopper walls
          // new Pose3d[] {new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)) //to setup the model
        });
  }
}

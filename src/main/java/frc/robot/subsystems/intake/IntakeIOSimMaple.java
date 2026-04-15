package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSimMaple extends IntakeIOSim {
  private final IntakeSimulation intakeSimulation;

  public IntakeIOSimMaple(AbstractDriveTrainSimulation driveTrain) {
    // Here, create the intake simulation with respect to the intake on your real robot
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            // Width of the intake
            Meters.of(0.7),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 1 note
            40);
  }

  public void shoot(SwerveDriveSimulation driveSimulation, Drive drive) {
    if (intakeSimulation.obtainGamePieceFromIntake()) {
      launchFuel(driveSimulation, drive);
    }
  }

  // public void launchFuel() {
  //     // if there is a note in the intake, it will be removed and return true; otherwise, returns
  // false
  //     if (intakeSimulation.obtainGamePieceFromIntake())
  //         ShooterIOSimMaple.launchNote(); // notify the simulated flywheels to launch a note
  // }

  public void launchFuel(SwerveDriveSimulation driveSimulation, Drive drive) {
    // if there is a note in the intake, it will be removed and return true; otherwise, returnsfalse
    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            // Specify the position of the chassis when the note is launched
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, 0),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
            // the projectile
            drive.getChassisSpeeds(),
            // The shooter facing direction is the same as the robot’s facing direction
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            // Add the shooter’s rotation
            // + Translation2d.kZero,
            // Initial height of the flying note
            Meters.of(.45),
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000
            // RPM
            MetersPerSecond.of(2000.0 / 6000 * 20),
            // The angle at which the note is launched
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

  /** Sets the motor's speed given an RPM input */
  public void setRollerRPS(double rps) {
    super.setRollerRPS(rps);
    if (rps == 0.0) {
      intakeSimulation.stopIntake();
    } else {
      intakeSimulation.startIntake();
    }
  }
}

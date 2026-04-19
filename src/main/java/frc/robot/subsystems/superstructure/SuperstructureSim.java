package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class SuperstructureSim extends SubsystemBase implements AutoCloseable {
  private final Intake intake;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;
  private final DoubleSupplier shooterRPS;

  private final IntakeSimulation intakeSimulation;

  private final Notifier shooterLoop = new Notifier(this::shoot);

  public SuperstructureSim(
      Intake intake,
      SwerveDriveSimulation swerveDriveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds,
      DoubleSupplier shooterRPS) {
    this.intake = intake;
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.chassisSpeeds = chassisSpeeds;
    this.shooterRPS = shooterRPS;
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
            IntakeConstants.INTAKE_CAPACITY);
  }

  public Command shootCommand() {
    return startEnd(() -> shooterLoop.startPeriodic(ShooterConstants.BPS), shooterLoop::stop);
  }

  public void shoot() {
    if (intakeSimulation.obtainGamePieceFromIntake()) {
      launchFuel();
    }
  }

  private void launchFuel() {
    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            // Specify the position of the chassis when fuel is launched
            swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s
            // reference frame)
            new Translation2d(0.2, -.2 + (Math.random() * (.4))), // y controls sides
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of
            // the projectile
            chassisSpeeds.get(),
            // The shooter facing direction is the same as the robot’s facing direction
            swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
            // Add the shooter’s rotation
            // + Translation2d.kZero,
            // Initial height of the flying fuel
            Meters.of(.45),
            // The launch speed is proportional to the RPM
            // radius (m) * angular velocity (rad/s) = 2.0m * (rps*2pi)
            MetersPerSecond.of(4 * shooterRPS.getAsDouble() * Math.PI),
            // The angle at which the fuel is launched
            Degrees.of(65));
    fuelOnFly
        // Configure the note projectile to become a NoteOnField upon touching the ground
        .enableBecomesGamePieceOnFieldAfterTouchGround();

    // Add the projectile to the simulated arena
    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
  }

  @Override
  public void simulationPeriodic() {
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
        new Pose3d(
            -0.27,
            0,
            0.21,
            new Rotation3d(
                0, -Math.toRadians(intake.getPivotPosition() - 17.5), 0)), // intake pivot
        new Pose3d(-.3 - hopperDistAdded, 0, 0, new Rotation3d(0, 0, 0)) // hopper walls
        );
  }

  @Override
  public void close() throws Exception {
    shooterLoop.close();
  }
}

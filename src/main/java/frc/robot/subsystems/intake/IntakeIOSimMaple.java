package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

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

  public boolean isFuelInsideIntake() {
    return intakeSimulation.getGamePiecesAmount()
        != 0; // True if there is a game piece in the intake
  }

  // public void launchFuel() {
  //     // if there is a note in the intake, it will be removed and return true; otherwise, returns
  // false
  //     if (intakeSimulation.obtainGamePieceFromIntake())
  //         ShooterIOSimMaple.launchNote(); // notify the simulated flywheels to launch a note
  // }

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

package frc.robot.util.io.motors.linear;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.subsystems.RobotStateHandler;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class LinearSystem extends Motor<LinearSystemIO, LinearSystemIOInputsAutoLogged> {
  private final Function<Distance, Angle> distanceToAngle;

  public LinearSystem(
      String name, LinearSystemIO io, double currentLimit, double drumRadiusMeters) {
    this(name, io, RobotStateHandler::isEnabled, currentLimit, drumRadiusMeters);
  }

  public LinearSystem(
      String name,
      LinearSystemIO io,
      BooleanSupplier brakeMode,
      double currentLimit,
      double drumRadiusMeters) {
    this(
        name,
        io,
        brakeMode,
        currentLimit,
        distance -> Radians.of(distance.in(Meters) / drumRadiusMeters));
  }

  public LinearSystem(
      String name,
      LinearSystemIO io,
      BooleanSupplier brakeMode,
      double currentLimit,
      Function<Distance, Angle> distanceToAngle) {
    super(name, io, new LinearSystemIOInputsAutoLogged(), brakeMode, currentLimit);
    io.configure(true, true);
    this.distanceToAngle = distanceToAngle;
    Logger.recordOutput(name + "/SetpointRad", 0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();
  }

  public void runClosedLoop(Angle angle) {
    if (stalled || tempCritical) return;

    io.setPosition(angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointRad", angle.in(Radians));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runClosedLoop(Distance position) {
    runClosedLoop(distanceToAngle.apply(position));
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Radians.of(inputs.positionRad);
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }
}

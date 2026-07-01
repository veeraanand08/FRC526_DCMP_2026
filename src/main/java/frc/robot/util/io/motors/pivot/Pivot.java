package frc.robot.util.io.motors.pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.subsystems.RobotStateHandler;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends Motor<PivotIO, PivotIOInputsAutoLogged> {
  public Pivot(String name, PivotIO io) {
    this(name, io, 120.0);
  }

  public Pivot(String name, PivotIO io, double currentLimit) {
    this(name, io, RobotStateHandler::isEnabled, currentLimit);
  }

  public Pivot(String name, PivotIO io, BooleanSupplier brakeMode, double currentLimit) {
    super(name, io, new PivotIOInputsAutoLogged(), brakeMode, currentLimit);
    if (io instanceof MotorIOTalonFX) {
      ((MotorIOTalonFX) io).withPosition();
    }
    Logger.recordOutput(name + "/SetpointDeg", 0.0);
  }

  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void runClosedLoop(Angle angle) {
    if (stalled || tempCritical) return;

    io.setPosition(angle);
    mode = MotorIO.MotorIOMode.POSITION_CONTROL;
    Logger.recordOutput(name + "/SetpointDeg", angle.in(Degrees));
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void resetPosition(Angle newPosition) {
    io.resetPosition(newPosition);
  }

  public Angle getPosition() {
    return Degrees.of(inputs.positionDeg);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }
}

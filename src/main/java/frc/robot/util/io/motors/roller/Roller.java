package frc.robot.util.io.motors.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.io.motors.Motor;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.subsystems.RobotStateHandler;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Roller extends Motor<RollerIO, RollerIOInputsAutoLogged> {
  public Roller(String name, RollerIO io) {
    this(name, io, 120.0);
  }

  public Roller(String name, RollerIO io, double currentLimit) {
    this(name, io, RobotStateHandler::isEnabled, currentLimit);
  }

  public Roller(String name, RollerIO io, BooleanSupplier brakeMode, double currentLimit) {
    super(name, io, new RollerIOInputsAutoLogged(), brakeMode, currentLimit);
    io.configure(false, true);
    Logger.recordOutput(name + "/SetpointRPS", 0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    super.periodic();
  }

  public void runClosedLoop(double rps) {
    if (stalled || tempCritical) return;

    io.setVelocity(rps);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointRPS", rps);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRPS);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }
}

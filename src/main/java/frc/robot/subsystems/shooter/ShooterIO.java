package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public boolean topLeftConnected;
    public boolean bottomLeftConnected;
    public boolean topRightConnected;
    public boolean bottomRightConnected;

    public double velocityRPS;
    public double appliedVolts;
    public double statorCurrentAmps;
    public double supplyCurrentAmps;

    public double topLeftTempCelsius;
    public double bottomLeftTempCelsius;
    public double topRightTempCelsius;
    public double bottomRightTempCelsius;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets the motor's speed given an RPS input */
  default void setRPS(double rps) {}

  default void stop() {}
}

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // pivot
    public boolean pivotConnected = false;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public double pivotPositionDeg = 0.0;
    public double pivotVelocityRPS = 0.0;

    // roller
    public boolean rollerConnected = false;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerVelocityRPS = 0.0;

    // encoder
    public boolean encoderConnected = false;
    public double encoderPositionDeg = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotOpenLoop(double output) {}

  /** Sets the intake pivot's angle given a degree input */
  public default void setPivotSetpoint(double deg) {} // Standard PID

  public default void setPivotProfiled(double deg) {} // Profiled PID

  /** Sets the motor's speed given an RPM input */
  public default void setRollerRPS(double rps) {}

  /** Stop the motor */
  public default void stopPivot() {}

  public default void stopRoller() {}
}

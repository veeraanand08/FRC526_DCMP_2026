package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        // spindexer
        public boolean spindexerConnected = false;
        public double spindexerAppliedVolts = 0.0;
        public double spindexerCurrentAmps = 0.0;
        public double spindexerVelocityRPS = 0.0;

        // kicker
        public boolean kickerConnected = false;
        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;
        public double kickerVelocityRPS = 0.0;
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    /** Sets the motor's speed given an RPM input */
    public default void setSpindexerRPS(double rps) {}

    public default void setKickerRPS(double rps) {}

    /** Stop the motor */
    public default void stopSpindexer() {}

    public default void stopKicker() {}
}
package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        // indexer
        public boolean indexerConnected = false;
        public double indexerAppliedVolts = 0.0;
        public double indexerCurrentAmps = 0.0;
        public double indexerVelocityRPS = 0.0;

        // kicker
        public boolean kickerConnected = false;
        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;
        public double kickerVelocityRPS = 0.0;
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    /** Sets the motor's speed given an RPM input */
    public default void setIndexerRPS(double rps) {}

    public default void setKickerRPS(double rps) {}

    /** Stop the motor */
    public default void stopIndexer() {}

    public default void stopKicker() {}
}
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // leader
        public boolean topLeftConnected = false;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;
        public double shooterVelocityRPS = 0.0;

        // followers
        public boolean bottomLeftConnected = false;
        public boolean topRightConnected = false;
        public boolean bottomRightConnected = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Sets the motor's speed given an RPS input */
    public default void setRPS(double rps) {}

    public default void stop() {}
}
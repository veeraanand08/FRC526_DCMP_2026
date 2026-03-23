package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final double SHOOTER_MOI = 0.001;
    public static final double SHOOTER_GEAR_RATIO = 1.0;

    public static final int SHOOTER_SUPPLY_LIMIT = 60;
    public static final int SHOOTER_STATOR_LIMIT = 80;
    public static final InvertedValue SHOOTER_LEFT_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue SHOOTER_RIGHT_INVERTED = InvertedValue.Clockwise_Positive;
    public static final double SHOOTER_P = 0.00004;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;


     public static final int HOOD_SUPPLY_LIMIT = 60;
    public static final int HOOD_STATOR_LIMIT = 80;
    public static final InvertedValue HOOD_INVERTED = InvertedValue.Clockwise_Positive;
    public static final double HOOD_P = 0.00004;
    public static final double HOOD_I = 0;
    public static final double HOOD_D = 0;






    public static final double SHOOTER_FF = 1.0 / 5676.0;
    public static final double NEGATIVE_RATE_LIMIT = 2000;
    // set speeds
    public static final LoggedNetworkNumber DEFAULT_RPM =
            new LoggedNetworkNumber("/Tuning/Shooter/RPM", 3000);
    public static final double REVERSED_RPM = 2500; // reversal if something is stuck

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE = new InterpolatingDoubleTreeMap();
    static {
      // Distance, Degrees
      DISTANCE_TO_ANGLE.put(0.0, 0.0);
    }
}

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.signals.InvertedValue;

public final class FeederConstants
{
    public static final int SPINDEXER_STATOR_LIMIT = 80;
    public static final int SPINDEXER_SUPPLY_LIMIT = 60;
    public static final int KICKER_STATOR_LIMIT = 80;
    public static final int KICKER_SUPPLY_LIMIT = 60;
    
    public static final InvertedValue SPINDEXER_INVERTED = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue KICKER_INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double SPINDEXER_MOI = 0.001;
    public static final double KICKER_MOI = 0.001;
    public static final double KICKER_GEAR_RATIO = 1.0; // adjust

    public static final double SPINDEXER_P = 0.0004;
    public static final double SPINDEXER_I = 0;
    public static final double SPINDEXER_D = 0;
    public static final double SPINDEXER_FF = 1.0 / 5676.0;

    public static final double KICKER_P = 0.0004;
    public static final double KICKER_I = 0;
    public static final double KICKER_D = 0;
    public static final double KICKER_FF = 1.0 / 5676.0;

    public static final double SPINDEXER_RPS = 5000 / 60.0;
    public static final double KICKER_RPS = 5000 / 60.0;
}
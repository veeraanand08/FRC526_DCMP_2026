package frc.robot.subsystems.intake;

public final class IntakeConstants
{
    public static final int ROLLER_MOTOR = 14;
    public static final int ROLLER_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOI = 0.001;
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final double ROLLER_STALL_VELOCITY = 100;
    public static final double ROLLER_RPM = 5000;
    public static final double ROLLER_RPM_REVERSED = -4500;
    public static final double ROLLER_RPM_SLOW = 4000;
    public static final double ROLLER_P = 0.0003;
    public static final double ROLLER_I = 0;
    public static final double ROLLER_D = 0;
    public static final double ROLLER_FF = 1.0 / 5676.0;

    public static final boolean ROLLER_REVERSED = true;
    public static final boolean PIVOT_REVERSED = false;
    
    public static final int PIVOT_MOTOR = 13;
    public static final int PIVOT_CURRENT_LIMIT = 40;
    public static final double PIVOT_ENCODER_OFFSET = 0.0; // adjust
    public static final double PIVOT_GEAR_RATIO = 75;
    public static final double PIVOT_CHAIN_GEAR_RATIO = 3;
    public static final double PIVOT_ROT_TO_DEG = 360 / PIVOT_GEAR_RATIO;
    public static final double PIVOT_ROT_TO_DEG_ABS = 360 / PIVOT_CHAIN_GEAR_RATIO;
    public static final double PIVOT_RPM_TO_DEG_PER_SEC = (360.0/60.0) / PIVOT_GEAR_RATIO;
    public static final double PIVOT_STALL_VELOCITY = 0.5;
    public static final double PIVOT_P = 0.01;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;
    public static final double PIVOT_FF_S = 0.15;
    public static final double PIVOT_FF_V = 1.0 / 5676.0;
    public static final double PIVOT_FF_COS = 0.25; // gravity feedforward
    public static final double PIVOT_FF_COS_RATIO = PIVOT_GEAR_RATIO / 360;

    // MAXMotion
    public static final double PIVOT_CRUISE_VELOCITY = 30; // degrees per sec
    public static final double PIVOT_MAX_ACCEL = 10; // degrees per sec^2 (i think)
    public static final double ALLOWED_PROFILE_ERROR = 1;
    
    // setpoints, in degrees
    public static final double PIVOT_RAISED_ANGLE = 0;
    public static final double PIVOT_ENGAGED_ANGLE = 130; // lowered
    public static final double PIVOT_AGITATION_UPPER_ANGLE  = 40; //where the upper bound starts
    public static final double PIVOT_AGITATION_UPPER_ANGLE_MIN = 25; // where the upper bound ends
    public static final double PIVOT_AGITATION_LOWER_ANGLE  = 105;
    public static final double PIVOT_UPPER_AGITATION_DECAY_TIME = 12.0; // time it takes to decay in seconds

    public static final double AGITATION_PERIOD = 4; // Period is in seconds
}
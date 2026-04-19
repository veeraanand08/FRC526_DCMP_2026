package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;

public final class IntakeConstants {
  public static final int ROLLER_STATOR_LIMIT = 60;
  public static final int ROLLER_SUPPLY_LIMIT = 40;
  public static final InvertedValue ROLLER_INVERTED = InvertedValue.Clockwise_Positive;
  public static final double ROLLER_MOI = 0.5;
  public static final double ROLLER_GEAR_RATIO = 1.0;
  public static final double ROLLER_KP = 0.1;
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0;
  public static final double ROLLER_KV = 0.12;

  public static final double ROLLER_RPS = 5000 / 60.0;
  public static final double ROLLER_RPS_REVERSED = -4500 / 60.0;
  public static final double ROLLER_RPS_SLOW = 4000 / 60.0;

  public static final int PIVOT_STATOR_LIMIT = 80;
  public static final int PIVOT_SUPPLY_LIMIT = 40;
  public static final InvertedValue PIVOT_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final double PIVOT_ENCODER_OFFSET = 0.0;
  public static final double PIVOT_GEAR_RATIO = 75;
  public static final double PIVOT_KP = 0.1;
  public static final double PIVOT_KI = 0;
  public static final double PIVOT_KD = 0;
  public static final double PIVOT_KS = 0.05;
  public static final double PIVOT_KV = 0.12;
  public static final double PIVOT_KA = 0;
  public static final double PIVOT_KG = 0.0;
  public static final double PIVOT_CRUISE_VELOCITY = 5.0;
  public static final double PIVOT_CRUISE_ACCELERATION = 1.0;

  // setpoints, in degrees
  public static final double PIVOT_RAISED_ANGLE = 0;
  public static final double PIVOT_ENGAGED_ANGLE = 130; // lowered
  public static final double PIVOT_AGITATION_UPPER_ANGLE = 40; // where the upper bound starts
  public static final double PIVOT_AGITATION_UPPER_ANGLE_MIN = 25; // where the upper bound ends
  public static final double PIVOT_AGITATION_LOWER_ANGLE = 105;
  public static final double PIVOT_UPPER_AGITATION_DECAY_TIME =
      12.0; // time it takes to decay in seconds

  public static final double AGITATION_PERIOD = 4; // Period is in seconds

  // sim
  public static final int INTAKE_CAPACITY = 100;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum ControlScheme {
    SAXONS(false),
    SAXON_SPARKS(false),
    TEST(false),
    GUITAR_HERO_OP(true),
    GUITAR_HERO_FULL(true);

    public final boolean isGuitarHero;

    ControlScheme(boolean isGuitarHero) {
      this.isGuitarHero = isGuitarHero;
    }
  }

  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int GUITAR_HERO_CONTROLLER_PORT = 2;
  }

  public static final class CANConstants {
    public static final CANBus SUPERSTRUCTURE_CAN_BUS = new CANBus("Superstructure");

    public static final int SHOOTER_TOP_LEFT = 14;
    public static final int SHOOTER_BOTTOM_LEFT = 15;
    public static final int SHOOTER_TOP_RIGHT = 16;
    public static final int SHOOTER_BOTTOM_RIGHT = 17;

    public static final int INTAKE_PIVOT = 18;
    public static final int PIVOT_ENCODER = 22;
    public static final int INTAKE_ROLLER = 19;

    public static final int KICKER = 20;
    public static final int INDEXER = 21;
  }

  public static final class FieldConstants {
    public static final Translation2d RED_HUB = new Translation2d(11.938, 4.0);
    public static final Translation2d BLUE_HUB = new Translation2d(4.597, 4.0);


    public static final Translation2d RED_LEFT_BUMP = new Translation2d(11.928, 2.408);
    public static final Translation2d RED_RIGHT_BUMP = new Translation2d(11.928, 5.598);
    public static final Translation2d BLUE_LEFT_BUMP = new Translation2d(4.617, 5.598);
    public static final Translation2d BLUE_RIGHT_BUMP = new Translation2d(4.617, 2.408);

    public static final Translation2d RED_PASS_OFFSET = new Translation2d(-2.0, 0);
    public static final Translation2d BLUE_PASS_OFFSET = new Translation2d(2.0, 0);

    public static final Translation2d RED_LEFT_PASS = RED_LEFT_BUMP.plus(RED_PASS_OFFSET);
    public static final Translation2d RED_RIGHT_PASS = RED_RIGHT_BUMP.plus(RED_PASS_OFFSET);
    public static final Translation2d BLUE_LEFT_PASS = BLUE_LEFT_BUMP.plus(BLUE_PASS_OFFSET);
    public static final Translation2d BLUE_RIGHT_PASS = BLUE_RIGHT_BUMP.plus(BLUE_PASS_OFFSET);



    public static final Translation2d RED_LEFT_TRENCH = new Translation2d(11.928, 0.586);
    public static final Translation2d RED_RIGHT_TRENCH = new Translation2d(11.928, 7.423);
    public static final Translation2d BLUE_LEFT_TRENCH = new Translation2d(4.617, 7.423);
    public static final Translation2d BLUE_RIGHT_TRENCH = new Translation2d(4.617, 0.586);
    // the translations of these need to be found:
    // points at which robot should drive to before looking for balls to intake
    public static final Translation2d RED_LEFT_MID = new Translation2d();
    public static final Translation2d RED_RIGHT_MID = new Translation2d();
    public static final Translation2d BLUE_LEFT_MID = FlippingUtil.flipFieldPosition(RED_LEFT_MID);
    public static final Translation2d BLUE_RIGHT_MID =
        FlippingUtil.flipFieldPosition(RED_RIGHT_MID);
    // points at which robot should go to score
    public static final Translation2d RED_LEFT_SCORING = new Translation2d();
    public static final Translation2d RED_RIGHT_SCORING = new Translation2d();
    public static final Translation2d BLUE_LEFT_SCORING =
        FlippingUtil.flipFieldPosition(RED_LEFT_SCORING);
    public static final Translation2d BLUE_RIGHT_SCORING =
        FlippingUtil.flipFieldPosition(RED_RIGHT_SCORING);

    public static final double RED_ALLIANCE_BOUNDARY = RED_LEFT_BUMP.getX();
    public static final double BLUE_ALLIANCE_BOUNDARY = BLUE_LEFT_BUMP.getX();

    public static final Distance FIELD_LENGTH = Meters.of(16.541);
    public static final Distance FIELD_WIDTH = Meters.of(8.069);
  }
}

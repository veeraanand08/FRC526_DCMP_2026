// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.RobotUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ShootingTasks {
  public enum Target {
    HUB,
    PASS,
    AUTO,
    NONE {
      @Override
      public String toString() {
        return "N/A";
      }
    }
  }

  @Getter private static Target currentTarget = Target.NONE;
  public static boolean isAutoAlignRunning = false;

  private static Target getTarget(Translation2d robotTranslation, boolean isRedAlliance) {
    double robotX = robotTranslation.getX();
    if (!isRedAlliance && robotX < FieldConstants.BLUE_ALLIANCE_BOUNDARY) {
      return Target.HUB;
    } else if (isRedAlliance && robotX > FieldConstants.RED_ALLIANCE_BOUNDARY) {
      return Target.HUB;
    }

    return Target.PASS;
  }

  public static Translation2d getTargetTranslation(Target target, Translation2d robotTranslation) {
    boolean isRedAlliance = RobotUtil.isRedAlliance();
    Translation2d targetTranslation;
    if (target == Target.AUTO) {
      target = getTarget(robotTranslation, isRedAlliance);
    }
    switch (target) {
      case NONE:
      case HUB:
        targetTranslation = isRedAlliance ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
        break;
      case PASS:
        Translation2d leftBump, rightBump;

        if (isRedAlliance) {
          leftBump = FieldConstants.RED_LEFT_BUMP;
          rightBump = FieldConstants.RED_RIGHT_BUMP;
        } else {
          leftBump = FieldConstants.BLUE_LEFT_BUMP;
          rightBump = FieldConstants.BLUE_RIGHT_BUMP;
        }
        double leftBumpDistance = robotTranslation.getDistance(leftBump);
        double rightBumpDistance = robotTranslation.getDistance(rightBump);

        if (leftBumpDistance < rightBumpDistance) {
          targetTranslation = leftBump;
        } else {
          targetTranslation = rightBump;
        }
        break;
      default:
        targetTranslation = robotTranslation;
    }
    currentTarget = target;
    Logger.recordOutput("AutoAlign/Target", currentTarget);
    Logger.recordOutput("AutoAlign/TargetTranslation", targetTranslation);
    return targetTranslation;
  }

  public static void clearTarget() {
    currentTarget = Target.NONE;
    Logger.recordOutput("AutoAlign/Target", currentTarget);
  }
}

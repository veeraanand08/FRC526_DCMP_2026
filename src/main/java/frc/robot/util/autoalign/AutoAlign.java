// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.autoalign;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.RobotUtil;

public class AutoAlign {
  public enum Target {
    HUB,
    BUMP,
    AUTO,
    NONE {
      @Override
      public String toString() {
        return "N/A";
      }
    }
  }

  // used in shooter subsystem to determine if bot is ready to shoot
  private static Target currentTarget = Target.NONE;
  private static Translation2d virtualTarget = Translation2d.kZero;

  public static Rotation2d getAngleToTarget(Target target,
                                            Translation2d robotTranslation, ChassisSpeeds robotSpeeds)
  {
    Translation2d targetTranslation = getTargetTranslation(target, robotTranslation);
    virtualTarget = getVirtualTarget(robotSpeeds, robotTranslation, targetTranslation);

    Translation2d difference = virtualTarget.minus(robotTranslation);
    return new Rotation2d(difference.getX(), difference.getY());
  }

  public static Target getTarget(Translation2d robotTranslation, boolean isRedAlliance){
    double robotX = robotTranslation.getX();
    if (!isRedAlliance && robotX < FieldConstants.BLUE_ALLIANCE_BOUNDARY){
        return Target.HUB;
    } else if (isRedAlliance && robotX > FieldConstants.RED_ALLIANCE_BOUNDARY){
      return Target.HUB;
    }

    return Target.BUMP;
  }

  public static Translation2d getTargetTranslation(Target target, Translation2d robotTranslation) {
    boolean isRedAlliance = RobotUtil.isRedAlliance();
    Translation2d targetTranslation;
    if (target == Target.AUTO) {
      target = getTarget(robotTranslation, isRedAlliance);
      currentTarget = target;
    }
    switch (target) {
      case NONE:
      case HUB:
        targetTranslation = isRedAlliance ? FieldConstants.RED_HUB : FieldConstants.BLUE_HUB;
        break;
      case BUMP:
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

        if (leftBumpDistance < rightBumpDistance){
            targetTranslation = leftBump;
        } else {
            targetTranslation = rightBump;
        }
        break;
      default:
        targetTranslation = robotTranslation;
    }
    return targetTranslation;
  }

  public static Translation2d getVirtualTarget(ChassisSpeeds robotSpeed, Translation2d robotTranslation, Translation2d targetTranslation) {
    Translation2d virtualTargetTranslation = targetTranslation;

    for (int i = 0; i < AutoAlignConstants.MAX_ITERATIONS; i++){
      double distanceToTarget = robotTranslation.getDistance(virtualTargetTranslation);
      double shotTime = AutoAlignConstants.DISTANCE_TO_TIME.get(distanceToTarget);

      double xTranslation = robotSpeed.vxMetersPerSecond * shotTime;
      double yTranslation = robotSpeed.vyMetersPerSecond * shotTime;

      virtualTargetTranslation = targetTranslation.minus(new Translation2d(xTranslation, yTranslation));
    }

    return virtualTargetTranslation;
  }

  public static Translation2d getSavedVirtualTarget() {
    return virtualTarget;
  }

  public static Target getCurrentTarget() {
    return currentTarget;
  }

  public static void disable() {
    currentTarget = Target.NONE;
  }

  public static boolean isActive() {
    return currentTarget != Target.NONE;
  }
}

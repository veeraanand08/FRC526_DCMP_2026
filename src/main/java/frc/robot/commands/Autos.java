package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.util.RobotUtil.isRedAlliance;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class Autos {
  // system check
  private static final double DRIVING_SPEED = 2.5;
  private static final double WAIT_SHORT = 3.0;
  private static final double WAIT_LONG = 5.0;

  // auto cycle
  private static final double INTAKE_TIME = 5.0;
  private static final double AGITATE_START = 3.0;

  public static Command systemCheck(Drive drive, Shooter shooter, Feeder feeder, Intake intake) {
    return Commands.sequence(
            Commands.runOnce(() -> Logger.recordOutput("Autos/SystemCheck/State", "Drive")),
            // forwards
            Commands.run(
                    () -> drive.runVelocity(new ChassisSpeeds(DRIVING_SPEED, 0, 0), true), drive)
                .withTimeout(WAIT_SHORT),
            // right
            Commands.run(
                    () -> drive.runVelocity(new ChassisSpeeds(0, -DRIVING_SPEED, 0), true), drive)
                .withTimeout(WAIT_SHORT),
            // spin clockwise
            Commands.run(
                    () -> drive.runVelocity(new ChassisSpeeds(0.0, 0, -DRIVING_SPEED), true), drive)
                .withTimeout(WAIT_SHORT),
            Commands.runOnce(drive::stop, drive),
            Commands.runOnce(() -> Logger.recordOutput("Autos/SystemCheck/State", "Intake")),
            intake.intake().withTimeout(WAIT_LONG),
            Commands.runOnce(() -> ShooterConstants.DEFAULT_RPM.set(1500)),
            Commands.runOnce(
                () -> Logger.recordOutput("Autos/SystemCheck/State", "AgitateAndShoot")),
            Commands.parallel(intake.agitate(feeder), shooter.shootDefault(feeder))
                .withTimeout(WAIT_LONG))
        .finallyDo(() -> Logger.recordOutput("Autos/SystemCheck/State", "Idle"));
  }

  public static Command leftCycle(
      Drive drive, Vision vision, Shooter shooter, Feeder feeder, Intake intake) {
    Pose2d preIntake =
        new Pose2d(isRedAlliance() ? RED_LEFT_MID : BLUE_LEFT_MID, Rotation2d.kCW_Pi_2);
    Pose2d scoring =
        new Pose2d(
            isRedAlliance() ? RED_LEFT_SCORING : BLUE_LEFT_SCORING, Rotation2d.fromDegrees(45.0));
    Command autoAlign = NamedCommands.getCommand("Auto Align");

    return Commands.repeatingSequence(
        intake.deploy(),
        drive.driveToPose(preIntake),
        drive
            .driveToPose(vision::getObjectPose)
            .withTimeout(INTAKE_TIME), // vision method not impl yet
        intake.runOnce(intake::stopRoller),
        drive.driveToPose(scoring));
  }
}

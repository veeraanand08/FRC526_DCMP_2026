package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private static final double DRIVING_SPEED = 2.5;
  private static final double WAIT_SHORT = 3.0;
  private static final double WAIT_LONG = 5.0;

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
            intake.intakeCommand().withTimeout(WAIT_LONG),
            Commands.runOnce(() -> ShooterConstants.DEFAULT_RPM.set(1500)),
            Commands.runOnce(
                () -> Logger.recordOutput("Autos/SystemCheck/State", "AgitateAndShoot")),
            Commands.parallel(intake.agitateCommand(feeder), shooter.shootDefault(feeder))
                .withTimeout(WAIT_LONG))
        .finallyDo(() -> Logger.recordOutput("Autos/SystemCheck/State", "Idle"));
  }
}

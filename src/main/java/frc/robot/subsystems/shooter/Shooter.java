package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotUtil;
import frc.robot.util.autoalign.AutoAlign;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  private double desiredRPS;
  private double distanceToTarget;

  public Shooter(ShooterIO io, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
    this.io = io;

    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;

    Logger.recordOutput("Shooter/Shooter Ready", false);
    Logger.recordOutput("Shooter/Desired Hood Angle", 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Translation2d robotTranslation = robotPose.get().getTranslation();
    Translation2d virtualTarget;
    boolean autoAlignActive = AutoAlign.isActive();

    if (autoAlignActive) virtualTarget = AutoAlign.getSavedVirtualTarget();
    else virtualTarget = AutoAlign.getVirtualTarget(robotVelocity.get(), robotTranslation,
                                                    AutoAlign.getTargetTranslation(AutoAlign.Target.HUB, robotTranslation));
    distanceToTarget = robotTranslation.getDistance(virtualTarget);

    Logger.recordOutput("Shooter/Shooter Ready", RobotUtil.shiftTimer.isHubActive() && autoAlignActive);
    Logger.recordOutput("Shooter/Distance to Target", distanceToTarget);
  }

  public void shoot() {
    shoot(ShooterConstants.DISTANCE_TO_RPS.get(distanceToTarget));
  }

  public void shoot(double rps) {
    desiredRPS = rps;
    io.setRPS(desiredRPS);
    Logger.recordOutput("Shooter/Desired RPS", desiredRPS);
  }

  public boolean hasSpunUp() {
    return inputs.shooterVelocityRPS > desiredRPS - 1.5;
  }

  public void stop() {
    io.stop();
    desiredRPS = 0.0;
    Logger.recordOutput("Shooter/Desired RPS", desiredRPS);
  }
}

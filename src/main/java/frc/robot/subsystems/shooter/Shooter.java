package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.led.LED;
import frc.robot.util.RobotUtil;
import frc.robot.util.ShiftTimer;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.motors.roller.RollerIOTalonFX;
import frc.robot.util.sotm.ShootingTasks;
import frc.robot.util.sotm.ShotCalculator;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final Roller roller;
  @Getter private double setpointRPS;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  private final Alert speedWarning =
      new Alert("Shooter", "Requested speed above limit, check units", Alert.AlertType.kWarning);

  @Getter private ShotCalculator.LaunchParameters shot = ShotCalculator.LaunchParameters.INVALID;
  @Getter private boolean isShooting;

  public Shooter(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new RollerIOTalonFX(
              CANConstants.SUPERSTRUCTURE_CAN_BUS,
              CANConstants.SHOOTER_TOP_LEFT,
              new int[] {
                CANConstants.SHOOTER_BOTTOM_LEFT,
                CANConstants.SHOOTER_TOP_RIGHT,
                CANConstants.SHOOTER_BOTTOM_RIGHT
              },
              ShooterConstants.SHOOTER_CONFIG,
              new MotorAlignmentValue[] {
                MotorAlignmentValue.Aligned,
                MotorAlignmentValue.Opposed,
                MotorAlignmentValue.Opposed
              });
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(4),
              new MotorIO.MechanismConstraints(
                  ShooterConstants.SHOOTER_GEAR_RATIO, ShooterConstants.SHOOTER_MOI, 0.2, 0, 0, 0),
              3.1,
              ShooterConstants.SHOOTER_KD,
              3);
          default -> new RollerIO() {};
        };

    roller = new Roller("Shooter", rollerIO);

    loadShooterData();

    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;

    Logger.recordOutput("Shooter/Ready", false);
    Logger.recordOutput("Shooter/LaunchParameters", shot);
    Logger.recordOutput("Shooter/DistanceToTarget", shot.solvedDistanceM());
  }

  @Override
  public void periodic() {
    roller.periodic();

    if (!ShootingTasks.isAutoAlignRunning) {
      Logger.recordOutput("Shooter/DistanceToTarget", 0.0);
    }
    Logger.recordOutput(
        "Shooter/Ready",
        ShootingTasks.isAutoAlignRunning
            && shot != ShotCalculator.LaunchParameters.INVALID
            && ShiftTimer.instance.isHubActive());
  }

  public void computeShot() {
    Pose2d currentPose = robotPose.get();
    ChassisSpeeds robotSpeeds = robotVelocity.get();
    shot =
        SHOT_CALC.calculate(
            new ShotCalculator.ShotInputs(
                currentPose,
                ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, currentPose.getRotation()),
                robotSpeeds,
                ShootingTasks.getTargetTranslation(
                    ShootingTasks.Target.AUTO, currentPose.getTranslation()),
                0.9 // vision confidence, 0 to 1
                ));
    Logger.recordOutput("Shooter/LaunchParameters", shot);
    Logger.recordOutput("Shooter/DistanceToTarget", shot.solvedDistanceM());
  }

  public void runVelocity(double rps) {
    if (rps > 85.0) {
      rps = 85.0;
      speedWarning.set(true);
    }
    roller.runClosedLoop(rps);
    setpointRPS = rps;
    LED.getInstance().shoot();
  }

  public void stop() {
    roller.stop();
    setpointRPS = 0.0;
    ShootingTasks.clearTarget();
    isShooting = false;
    LED.getInstance().stopShoot();
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }

  public boolean hasSpunUp() {
    return setpointRPS != 0 && roller.getVelocityRPS() > setpointRPS - 1.5;
  }

  public Command shoot(Feeder feeder) {
    return runEnd(
            () -> {
              isShooting = true;
              computeShot();
              runVelocity(shot.rpm() / 60.0);
            },
            this::stop)
        .alongWith(Commands.waitUntil(this::hasSpunUp).andThen(feeder.feed()))
        .alongWith(
            Commands.runEnd(
                () -> {
                  if (!ShiftTimer.instance.isHubActive()) {
                    RobotUtil.setOperatorRumble(0.0, 0.8);
                  }
                },
                () -> RobotUtil.setOperatorRumble(0.0, 0.0)));
  }

  public Command shootDefault(Feeder feeder) {
    return startEnd(() -> runVelocity(ShooterConstants.DEFAULT_RPM.get() / 60.0), this::stop)
        .alongWith(Commands.waitUntil(this::hasSpunUp).andThen(feeder.feed()));
  }
}

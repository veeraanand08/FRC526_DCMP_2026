package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.util.ShiftTimer;
import frc.robot.util.autoalign.AutoAlign;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.motors.roller.RollerIOTalonFX;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final Roller roller;
  private double setpointRPS;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  @AutoLogOutput private double distanceToTarget;

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
              ShooterConstants.SHOOTER_KP,
              ShooterConstants.SHOOTER_KD,
              3);
          default -> new RollerIO() {};
        };

    roller = new Roller("Shooter", rollerIO);

    this.robotPose = robotPose;
    this.robotVelocity = robotVelocity;

    Logger.recordOutput("Shooter/Ready", false);
  }

  @Override
  public void periodic() {
    roller.periodic();

    Translation2d robotTranslation = robotPose.get().getTranslation();
    Translation2d virtualTarget;
    boolean autoAlignActive = AutoAlign.isActive();

    if (autoAlignActive) virtualTarget = AutoAlign.getSavedVirtualTarget();
    else
      virtualTarget =
          AutoAlign.getVirtualTarget(
              robotVelocity.get(),
              robotTranslation,
              AutoAlign.getTargetTranslation(AutoAlign.Target.AUTO, robotTranslation));
    distanceToTarget = robotTranslation.getDistance(virtualTarget);

    Logger.recordOutput("Shooter/Ready", autoAlignActive && ShiftTimer.instance.isHubActive());
  }

  public void shoot() {
    shoot(ShooterConstants.DISTANCE_TO_RPS.get(distanceToTarget));
  }

  public void shoot(double rps) {
    roller.runClosedLoop(rps);
    setpointRPS = rps;
  }

  public void stop() {
    roller.stop();
    setpointRPS = 0.0;
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }

  @AutoLogOutput
  public boolean hasSpunUp() {
    return setpointRPS != 0 && roller.getVelocityRPS() > setpointRPS - 1.5;
  }

  public Command startFlywheel() {
    return startEnd(() -> shoot(ShooterConstants.DEFAULT_RPM.get() / 60.0), this::stop);
  }
}

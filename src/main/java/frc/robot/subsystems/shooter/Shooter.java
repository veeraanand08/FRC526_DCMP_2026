package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOT_CALC;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.motors.roller.RollerIOTalonFX;
import frc.robot.util.sotm.ProjectileSimulator;
import frc.robot.util.sotm.ShootingTasks;
import frc.robot.util.sotm.ShotCalculator;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final Roller roller;
  private double setpointRPS;

  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotVelocity;

  @AutoLogOutput private double distanceToTarget;
  @Getter private ShotCalculator.LaunchParameters shot = ShotCalculator.LaunchParameters.INVALID;
  public boolean isShooting;

  private static void generateLookupTable() {
    ProjectileSimulator.SimParameters params =
        new ProjectileSimulator.SimParameters(
            0.215, // ball mass kg
            0.1501, // ball diameter m
            0.47, // drag coeff (smooth sphere)
            0.2, // Magnus coeff
            1.225, // air density
            ShooterConstants
                .EXIT_HEIGHT, // exit height (m), floor to where the ball leaves the shooter
            ShooterConstants.WHEEL_DIAMETER, // flywheel diameter (m), measure with calipers
            1.83, // target height (m), from game manual
            ShooterConstants
                .SLIP_FACTOR, // slip factor (0=no grip, 1=perfect), tune this on the real robot
            ShooterConstants.LAUNCH_ANGLE, // launch angle from horizontal, measure from CAD
            0.001, // sim timestep
            1500, // min RPM
            5000, // max RPM
            25, // iterations
            5.0 // max sim time
            );

    ProjectileSimulator sim = new ProjectileSimulator(params, -1.0);
    ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();

    // print data
    System.out.println("Generated lookup table:");
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        System.out.println(
            "SHOT_CALC.loadLUTEntry("
                + entry.distanceM()
                + ", "
                + entry.rpm()
                + ", "
                + entry.tof()
                + ");");
      }
    }
    System.out.println("End of data");
  }

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
  }

  @Override
  public void periodic() {
    roller.periodic();
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
  }

  public void shoot() {
    shoot(shot.rpm() / 60.0);
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

  public boolean hasSpunUp() {
    return setpointRPS != 0 && roller.getVelocityRPS() > setpointRPS - 1.5;
  }

  public Command startFlywheel() {
    return startEnd(() -> shoot(ShooterConstants.DEFAULT_RPM.get() / 60.0), this::stop);
  }
}

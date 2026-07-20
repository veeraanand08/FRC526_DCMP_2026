package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.intake.IntakeConstants.SETPOINTS;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.led.LED;
import frc.robot.util.RobotUtil;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.motors.pivot.Pivot;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.pivot.PivotIOSim;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.subsystems.ExtendedSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends ExtendedSubsystem {
  public enum PivotState {
    RAISING,
    AGITATING_UPPER {
      @Override
      public String toString() {
        return "AGITATING";
      }
    },
    AGITATING_LOWER {
      @Override
      public String toString() {
        return "AGITATING";
      }
    },
    LOWERING
  }

  private PivotState pivotState = PivotState.RAISING;

  private final Roller roller;
  private final Pivot pivot;

  public Intake() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
                  Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
                  Constants.CANConstants.INTAKE_PIVOT,
                  IntakeConstants.PIVOT_CONFIG)
              .withControlRequest(new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true));
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  IntakeConstants.PIVOT_GEAR_RATIO,
                  SingleJointedArmSim.estimateMOI(0.5, 2),
                  0.5,
                  0,
                  IntakeConstants.MAX_ANGLE,
                  0),
              IntakeConstants.PIVOT_KP,
              IntakeConstants.PIVOT_KD,
              0);
          default -> new PivotIO() {};
        };
    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.INTAKE_ROLLER,
              IntakeConstants.ROLLER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.RotationalMechanismConstraints(
                  IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.ROLLER_MOI, 0.2, 0, 0, 0),
              IntakeConstants.ROLLER_KP * 20,
              IntakeConstants.ROLLER_KD,
              0);
          default -> new RollerIO() {};
        };

    pivot =
        new Pivot(
            "Intake/Pivot", pivotIO, IntakeConstants.PIVOT_CONFIG.CurrentLimits.StatorCurrentLimit);
    roller = new Roller("Intake/Roller", rollerIO);

    Logger.recordOutput("Intake/PivotState", pivotState.toString());
    Logger.recordOutput("Intake/Running", false);
  }

  @Override
  public void disable() {
    pivot.stop();
    roller.stop();
  }

  @Override
  public void enable() {
    pivot.stop();
    roller.stop();
  }

  @Override
  public void periodic() {
    pivot.periodic();
    roller.periodic();
  }

  public void setPivotState(PivotState newState) {
    pivot.runClosedLoop(SETPOINTS.get(newState));
    pivotState = newState;
    Logger.recordOutput("Intake/PivotState", pivotState.toString());
  }

  public void start() {
    if (pivotState != PivotState.LOWERING) {
      setPivotState(PivotState.LOWERING);
    }
    roller.runClosedLoop(IntakeConstants.ROLLER_RPS);
    LED.getInstance().intake();
    Logger.recordOutput("Intake/Running", true);
  }

  public void slowRoller() {
    roller.runClosedLoop(IntakeConstants.ROLLER_RPS_SLOW);
    Logger.recordOutput("Intake/Running", false);
  }

  public void reverseRoller() {
    roller.runClosedLoop(IntakeConstants.ROLLER_RPS_REVERSED);
    Logger.recordOutput("Intake/Running", false);
  }

  public void stopRoller() {
    roller.stop();
    LED.getInstance().stopIntake();
    Logger.recordOutput("Intake/Running", false);
  }

  public double getPivotPosition() {
    return pivot.getPositionDeg();
  }

  public double getRollerRPS() {
    return roller.getVelocityRPS();
  }

  /**
   * Enable the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to run the intake
   */
  public Command intake() {
    return startEnd(this::start, this::stopRoller);
  }

  /**
   * Toggle the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to run the intake
   */
  public Command deploy() {
    return runOnce(this::start);
  }

  /**
   * Reverse the intake roller while held
   *
   * @return a command to reverse the intake
   */
  public Command reverse() {
    return startEnd(this::reverseRoller, this::stopRoller);
  }

  /**
   * Agitate balls in hopper
   *
   * @return a command to agitate the intake
   */
  public Command agitate(Feeder feeder) {
    return new FunctionalCommand(
        () -> {
          if (!feeder.isEnabledForShooting()) {
            feeder.agitate();
          }
          slowRoller();
          setPivotState(PivotState.AGITATING_UPPER);
        },
        () -> {
          if (pivotState == PivotState.AGITATING_UPPER
              && Math.abs(
                      pivot.getPositionDeg()
                          - SETPOINTS.get(PivotState.AGITATING_UPPER).in(Degrees))
                  < 3.5) {
            setPivotState(PivotState.AGITATING_LOWER);
          } else if (pivotState == PivotState.AGITATING_LOWER
              && Math.abs(
                      pivot.getPositionDeg()
                          - SETPOINTS.get(PivotState.AGITATING_LOWER).in(Degrees))
                  < 3.5) {
            setPivotState(PivotState.AGITATING_UPPER);
          }
        },
        interrupted -> {
          if (!feeder.isEnabledForShooting()) {
            feeder.stop();
          }
          setPivotState(PivotState.LOWERING);
          stopRoller();
        },
        () -> false,
        this);
  }

  /**
   * Stop the roller motor and bring the intake back up
   *
   * @return a command to reset the intake back to starting position
   */
  public Command stow() {
    return runOnce(
        () -> {
          setPivotState(PivotState.RAISING);
          stopRoller();
        });
  }

  public Command markIntakeLowered() {
    RobotUtil.RumbleRequest rumble = new RobotUtil.RumbleRequest(0.75, -1);
    return startEnd(
        () -> {
          RobotUtil.requestDriverRumble(rumble);
          RobotUtil.requestOperatorRumble(rumble);
          pivot.resetPosition(Degrees.of(IntakeConstants.MAX_ANGLE));
        },
        () -> {
          RobotUtil.stopDriverRumble(rumble);
          RobotUtil.stopOperatorRumble(rumble);
        });
  }

  public Command markIntakeRaised() {
    return runOnce(() -> pivot.resetPosition(Rotations.zero()));
  }
}

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.RobotUtil;
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

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean rollerEnabled;

  private boolean motorSafetyEngaged;

  public Intake(IntakeIO io) {
    this.io = io;

    new Trigger(() -> inputs.pivotCurrentAmps >= IntakeConstants.PIVOT_STATOR_LIMIT)
            .debounce(0.3, Debouncer.DebounceType.kBoth)
            .onTrue(
                    runOnce(
                            () -> {
                              disable();
                              motorSafetyEngaged = true;
                              RobotUtil.setOperatorRumble(0.85, 0.85);
                            }))
            .onFalse(
                    runOnce(
                            () -> {
                              motorSafetyEngaged = false;
                              RobotUtil.setOperatorRumble(0, 0);
                            }));

    Logger.recordOutput("Intake/Pivot State", pivotState.toString());
    Logger.recordOutput("Intake/Intake Running", rollerEnabled);
  }

  @Override
  public void disable() {
    io.setPivotOpenLoop(0);
    io.stopRoller();
  }

  @Override
  public void enable() {
    io.stopPivot();
    motorSafetyEngaged = false;
  }

  @Override
  /* Periodically raises/lowers the pivot depending on its current state. Will not run if in lowered/lowering state. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setPivotState(PivotState newState) {
    switch (newState) {
      case RAISING:
        setPivotAngle(IntakeConstants.PIVOT_RAISED_ANGLE);
        break;
      case LOWERING:
        setPivotAngle(IntakeConstants.PIVOT_ENGAGED_ANGLE);
        break;
      case AGITATING_UPPER:
        setPivotAngle(IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE);
        break;
      case AGITATING_LOWER:
        setPivotAngle(IntakeConstants.PIVOT_AGITATION_LOWER_ANGLE);
        break;
      default:
        break;
    }
    pivotState = newState;
    Logger.recordOutput("Intake/Pivot State", pivotState.toString());
  }

  public void toggleRoller() {
    setRoller(!rollerEnabled);
  }

  public void setRoller(boolean enabled) {
    if (enabled) io.setRollerRPS(IntakeConstants.ROLLER_RPS);
    else io.stopRoller();
    rollerEnabled = enabled;
    Logger.recordOutput("Intake/Intake Running", rollerEnabled);
  }

  public void slowRoller() {
    io.setRollerRPS(IntakeConstants.ROLLER_RPS_SLOW);
    rollerEnabled = false;
    Logger.recordOutput("Intake/Intake Running", false);
  }

  public void setRollerReversed(boolean enabled) {
    if (enabled) io.setRollerRPS(IntakeConstants.ROLLER_RPS_REVERSED);
    else io.stopRoller();
    rollerEnabled = false;
    Logger.recordOutput("Intake/Intake Running", false);
  }

  /**
   * Set the pivot motor's setpoint to a given angle.
   *
   * @param deg Angle (in degrees) to rotate.
   */
  public void setPivotAngle(double deg) {
    if (motorSafetyEngaged) return;
    io.setPivotProfiled(deg);
    Logger.recordOutput("Intake/Pivot Setpoint", deg);
  }

  public double getPivotPosition() {
    return inputs.pivotPositionDeg;
  }

  public double getRollerRPS() {
    return inputs.rollerVelocityRPS;
  }

  public void stop() {
    setRoller(false);
    io.stopPivot();
  }

  /**
   * Enable the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to toggle the intake
   */
  public Command intakeCommand() {
    // Inline construction of command goes here.
    return startEnd(
        () -> {
          if (pivotState == PivotState.LOWERING) {
            setRoller(true);
          } else {
            setRoller(true);
            setPivotState(PivotState.LOWERING);
          }
        },
        () -> setRoller(false));
  }

  /**
   * Toggle the intake roller. If the intake is raised, it will lower and then start.
   *
   * @return a command to toggle the intake
   */
  public Command toggleIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          if (pivotState == PivotState.LOWERING) {
            toggleRoller();
          } else {
            setRoller(true);
            setPivotState(PivotState.LOWERING);
          }
        });
  }

  /**
   * Reverse the intake roller while held
   *
   * @return a command to reverse the intake
   */
  public Command reverseIntakeCommand() {
    // Inline construction of command goes here.
    return startEnd(() -> setRollerReversed(true), () -> setRollerReversed(false));
  }

  /**
   * Agitate balls in hopper
   *
   * @return a command to agitate the intake
   */
  public Command agitateCommand() {
    return startRun(
            () -> {
              setPivotState(PivotState.AGITATING_UPPER);
              slowRoller();
            },
            () -> {
              if (pivotState == PivotState.AGITATING_UPPER
                  && Math.abs(inputs.pivotPositionDeg - IntakeConstants.PIVOT_AGITATION_UPPER_ANGLE)
                      < 3.5) {
                setPivotState(PivotState.AGITATING_LOWER);
              } else if (pivotState == PivotState.AGITATING_LOWER
                  && Math.abs(inputs.pivotPositionDeg - IntakeConstants.PIVOT_AGITATION_LOWER_ANGLE)
                      < 3.5) {
                setPivotState(PivotState.AGITATING_UPPER);
              }
            })
        .finallyDo(() -> {
          setPivotState(PivotState.LOWERING);
          io.stopRoller();
        });
  }

  /**
   * Stop the roller motor and bring the intake back up
   *
   * @return a command to reset the intake back to starting position
   */
  public Command resetIntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setRoller(false);
          setPivotState(PivotState.RAISING);
        });
  }

  public Command markIntakeLowered() {
    return startEnd(
        () -> {
          RobotUtil.setDriverRumble(0.75, 0.75);
          RobotUtil.setOperatorRumble(0.75, 0.75);
          io.resetPivotPosition(Degrees.of(IntakeConstants.MAX_ANGLE));
        },
        () -> {
          RobotUtil.setDriverRumble(0, 0);
          RobotUtil.setOperatorRumble(0, 0);
        });
  }

  public Command markIntakeRaised() {
    return runOnce(() -> io.resetPivotPosition(Rotations.zero()));
  }

  @Override
  public void simulationPeriodic() {}
}

package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.Constants.canIDConstants;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff;
import frc.robot.Subsystems.Handoff.HandoffIO;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Handoff.Handoff.HandoffStates;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Intake.Intake.IntakeStates;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Shooter.Shooter.ShooterStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class Superstructure extends SubsystemBase {
  private Intake s_intake;
  private Handoff s_handoff;
  private Elevator s_elevator;
  private Shooter s_shooter;
  private SuperstructureStates systemState = SuperstructureStates.IDLE;

  private Timer intakeCurrentTriggerTimer = new Timer();
  private boolean intakeCurrentTriggerTimerStarted = false;
  private double stateStartTime = 0;

  LoggedTunableNumber handoffShooterVoltage = new LoggedTunableNumber("Superstructure/handoffShooterVoltage", 3);
  LoggedTunableNumber handoffIntakeVoltage = new LoggedTunableNumber("Superstructure/handoffIntkaeVoltage", 1);
  LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Superstructure/intakeVoltage", 3.6);
  LoggedTunableNumber outakeVoltage = new LoggedTunableNumber("Superstructure/outakeVoltage", -2);
  LoggedTunableNumber ampShooterVel = new LoggedTunableNumber("Superstructure/ampShooterVel", 3.2);
  LoggedTunableNumber shootMidVel = new LoggedTunableNumber("Superstructure/shootMIDvel", 20);
  LoggedTunableNumber shootRightVel = new LoggedTunableNumber("Superstructure/shootRIGHTvel", 10);
  LoggedTunableNumber shootLeftVel = new LoggedTunableNumber("Superstructure/shootLEFTvel",20); 
  LoggedTunableNumber midRatio = new LoggedTunableNumber( "Superstructure/MIDratio",0.7); 
  LoggedTunableNumber rightRatio = new LoggedTunableNumber( "Superstructure/RIGHTratio",2); 
  LoggedTunableNumber leftRatio = new LoggedTunableNumber("Superstructure/LEFTratio",0.5);
  LoggedTunableNumber climbUpHeight = new LoggedTunableNumber("Superstructure/climbUpHeight", 0.45);
  LoggedTunableNumber climbDownHeight = new LoggedTunableNumber("Superstructure/climbDownHeight", 0);


  public Superstructure(IntakeIO intake, HandoffIO handoff, ElevatorIO elevator, ShooterIO shooter) {
    this.s_intake = new Intake(intake);
    this.s_handoff = new Handoff(handoff);
    this.s_elevator = new Elevator(elevator);
    this.s_shooter = new Shooter(shooter);
  }

  public enum SuperstructureStates{
    IDLE,
    HOMING,
    INTAKE,
    AMP_SHOOTER,
    SHOOT_RIGHT,
    SHOOT_MID,
    SHOOT_LEFT,
    PREPARE_AMP_ELEVATOR,
    AMP_ELEVATOR,
    EXIT_AMP_ELEVATOR,
    CLIMB_UP,
    CLIMB_DOWN
  }
  
  @Override
  public void periodic(){
    s_intake.Loop();
    s_elevator.Loop();
    s_handoff.Loop();
    s_shooter.Loop();

    Logger.recordOutput("SuperstructureState", systemState);
    switch(systemState){
            case IDLE: 
                s_elevator.requestElevatorHeight(0, false);
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                break;
            case HOMING: 
                s_elevator.setState(ElevatorState.HOMING);
                
                if(s_elevator.getState() == ElevatorState.IDLE){
                    setState(SuperstructureStates.IDLE);
                }

                break;
            case INTAKE:
                s_elevator.requestElevatorHeight(0, false);
                s_shooter.requestVelocity(0, 0);
                s_intake.requestIntake(intakeVoltage.get());
                s_handoff.requestHandoff(handoffIntakeVoltage.get());

                if(s_intake.getStatorCurrent() > 40 && !intakeCurrentTriggerTimerStarted){
                    intakeCurrentTriggerTimer.reset();
                    intakeCurrentTriggerTimer.start();
                    intakeCurrentTriggerTimerStarted = true;
                }

                if(s_intake.getStatorCurrent() < 40){
                    intakeCurrentTriggerTimer.stop();
                    intakeCurrentTriggerTimerStarted = false;
                }

                if(s_intake.isPastCurrentThreshold() && RobotController.getFPGATime()/1.0E6 - stateStartTime > 0.25 && intakeCurrentTriggerTimer.get() > 0.05){
                    setState(SuperstructureStates.IDLE);
                }

                break;
            case AMP_SHOOTER:  
                s_elevator.requestElevatorHeight(0, false);
                s_shooter.requestVelocity(ampShooterVel.get(), 1);
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if(RobotController.getFPGATime()/1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.IDLE);
                }

                break;
            case SHOOT_RIGHT:
                s_elevator.requestElevatorHeight(0, false);
                s_shooter.requestVelocity(shootRightVel.get(), rightRatio.get());
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());  
                
                if(RobotController.getFPGATime()/1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SHOOT_MID:
                s_elevator.requestElevatorHeight(0, false);
                s_shooter.requestVelocity(20, 0.7);
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if(RobotController.getFPGATime()/1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SHOOT_LEFT:
                s_elevator.requestElevatorHeight(0, false);
                s_shooter.requestVelocity(shootLeftVel.get(), leftRatio.get());
                s_intake.requestHandoff(handoffShooterVoltage.get());
                s_handoff.requestHandoff(handoffShooterVoltage.get());

                if(RobotController.getFPGATime()/1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.IDLE);
                }
                break; 
            case PREPARE_AMP_ELEVATOR:
                s_elevator.requestElevatorHeight(0.45, false);
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);

                if(s_elevator.atElevatorSetpoint(0.45)){
                    setState(SuperstructureStates.AMP_ELEVATOR);
                }              
                break;
            case AMP_ELEVATOR:
                s_elevator.requestElevatorHeight(0.45, false);
                s_shooter.requestVelocity(0, 0);
                s_intake.requestOutake(outakeVoltage.get());
                s_handoff.requestHandoff(0);

                if(RobotController.getFPGATime()/1.0E6 - stateStartTime > 0.75){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case CLIMB_UP:
                s_elevator.requestElevatorHeight(climbUpHeight.get(), false);
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);  
                break;
            case CLIMB_DOWN:
                s_elevator.requestElevatorHeight(climbDownHeight.get(), true);
                s_shooter.requestVelocity(0, 0);
                s_intake.setState(IntakeStates.IDLE);
                s_handoff.setState(HandoffStates.IDLE);
                break;
            default:
                break;
        }
  }
  
  public void setState(SuperstructureStates nextState){
    this.systemState = nextState;
    stateStartTime = RobotController.getFPGATime()/1E6;
  } 
}

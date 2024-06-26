
package frc.robot.Subsystems.OTB_Intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class OTB_Intake{
    private final OTB_IntakeIO otbIntakeIO;
    private OTB_IntakeIOInputsAutoLogged inputs = new OTB_IntakeIOInputsAutoLogged();
    private OTB_IntakeStates state = OTB_IntakeStates.IDLE;
    private double angleSetpoint = 0;
    private double voltageSetpoint = 0;
    //private final SysIdRoutine pivotSysID;

    public enum OTB_IntakeStates{
        IDLE,
        INTAKE,
        HOMING,
        SETPOINT
    }

    public OTB_Intake(OTB_IntakeIO otbIntakeIO) {
        this.otbIntakeIO = otbIntakeIO;
        /*pivotSysID  = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(2), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> otbIntakeIO.requestPivotVoltage(volts.in(Volts)), null,
                    this));*/
      }
      /* 
    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                pivotSysID
                        .quasistatic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 110),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                pivotSysID
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 110),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> otbIntakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    } */
    public void Loop(){
        otbIntakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);

        switch(state){
            case IDLE:
                otbIntakeIO.requestPivotVoltage(0);
                otbIntakeIO.requestIntakeVoltage(0);
                break;
            case HOMING:
                otbIntakeIO.requestPivotVoltage(-1);
                otbIntakeIO.requestIntakeVoltage(0);
                break;
            case INTAKE:
                otbIntakeIO.requestSetpoint(angleSetpoint);
                otbIntakeIO.requestIntakeVoltage(voltageSetpoint);
                break;
            case SETPOINT:
                otbIntakeIO.requestSetpoint(angleSetpoint);
                otbIntakeIO.requestIntakeVoltage(0);
                break;
        }
    }

    public void requestIntake(double angleSetpointDeg, double voltage){
        this.angleSetpoint = angleSetpointDeg;
        this.voltageSetpoint = voltage;
        setState(OTB_IntakeStates.INTAKE);
    }

    public void requestSetpoint(double angleSetpointDeg){
        this.angleSetpoint = angleSetpointDeg;
        this.voltageSetpoint = 0;
        setState(OTB_IntakeStates.SETPOINT);
    }
    public void setState(OTB_IntakeStates nextState){
        this.state = nextState;
    }

    public OTB_IntakeStates getState(){
        return this.state;
    }
    
    public double getStatorCurrent(){
        return inputs.intakeCurrent;
    }

    public double getPivotStatorCurrent(){
        return inputs.pivotCurrent;
    }

    public void zeroPosition(){
        otbIntakeIO.zeroPosition();
    }
}

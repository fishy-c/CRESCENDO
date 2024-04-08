// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.Handoff.HandoffIO;
import frc.robot.Subsystems.Handoff.HandoffIOTalonFX;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.AmpDriveAssistCommand;
import frc.robot.Subsystems.Swerve.SpeakerDriveAssistCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.AutonomousSelector;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
  public static final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private AutonomousSelector selector;
  private final IntakeIO s_intake = new IntakeIOTalonFX();
  private final HandoffIO s_handoff = new HandoffIOTalonFX();
  private final ElevatorIO s_elevator = new ElevatorIOTalonFX();
  private final ShooterIO s_shooter = new ShooterIOTalonFX();
  private final LEDs led = new LEDs();
  private final Superstructure superstructure = new Superstructure(s_intake, s_handoff, s_elevator, s_shooter, led);
  private final Swerve s_swerve = new Swerve();
  public RobotContainer() {

    NamedCommands.registerCommand("Intake", new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)));
    NamedCommands.registerCommand("ShootMid", new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)));
    SuperstructureStates shootSideSource = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? SuperstructureStates.SPIN_UP_RIGHT : SuperstructureStates.SPIN_UP_RIGHT;
    NamedCommands.registerCommand("ShootSource", new InstantCommand(() -> superstructure.setState(shootSideSource)));
    SuperstructureStates shootSideAmp = DriverStation.getAlliance().equals(Alliance.Blue) ? SuperstructureStates.SPIN_UP_LEFT : SuperstructureStates.SPIN_UP_LEFT;
    NamedCommands.registerCommand("ShootAmp", new InstantCommand(() -> superstructure.setState(shootSideAmp)));
    NamedCommands.registerCommand("Wait0.3", new WaitCommand(0.3));
    NamedCommands.registerCommand("Wait1.25", new WaitCommand(1.25));

    configureAutonomousSelector();
    s_elevator.elevatorConfiguration();
    s_shooter.shooterConfiguration();
    s_swerve.zeroWheels();
    s_swerve.zeroGyro();
    s_swerve.setDefaultCommand(
            new TeleopSwerve(
                s_swerve, 
                () -> -controller.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -controller.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> -controller.getRawAxis(XboxController.Axis.kRightX.value)
              
            )
        );

    configureBindings();
    configureDefaultCommands();

  }

  private void configureBindings() {
    operator.a().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.IDLE)));

    operator.rightBumper().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)));

    operator.x().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_LEFT)));

    operator.y().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)));

    operator.b().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_RIGHT)));

    operator.leftBumper().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_AMP)));

    operator.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.OUTAKE)));

    operator.rightTrigger().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.PASS)));

    //controller.rightTrigger().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.CLIMB_UP)));

    //controller.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.CLIMB_DOWN)));

    //controller.a().onTrue(new InstantCommand(() -> superstructure.disablingElevator()));

    controller.start().onTrue(new InstantCommand(() -> superstructure.disablingElevator()));

    controller.b().onTrue(new InstantCommand(() -> s_swerve.zeroGyro()));

    controller.rightBumper().whileTrue(new AmpDriveAssistCommand(s_swerve, superstructure));

    controller.rightTrigger().whileTrue(new SpeakerDriveAssistCommand(s_swerve));

    controller.leftBumper().onTrue((new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_AMP_ELEVATOR))));

    
  }
  private void configureDefaultCommands() {
   
  } 

  public Command getAutonomousCommand() {
    return selector.get();
  }

  public void configureAutonomousSelector(){
    selector = new AutonomousSelector(s_swerve, superstructure);
  }
}


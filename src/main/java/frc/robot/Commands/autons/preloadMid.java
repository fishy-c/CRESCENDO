package frc.robot.Commands.autons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class preloadMid extends SequentialCommandGroup{
    
    public preloadMid(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID))
        );
    
    }
}

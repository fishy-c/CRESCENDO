package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.modes.TWOPieceMid;
import frc.robot.autons.modes.TWOPieceAMP;
import frc.robot.autons.modes.TWOPieceSOURCE; //E for Source
import frc.robot.autons.modes.EFarMID;
import frc.robot.autons.modes.EFarAMP;
import frc.robot.autons.modes.EFarMIDTHREE; //EFarAMP with TWOPieceMid
import frc.robot.autons.modes.THREEPieceAMP;
import frc.robot.autons.modes.THREEPieceMid;
import frc.robot.autons.modes.NAMEDTWOPieceMid;

public class AutonomousSelector {
    private SendableChooser<SequentialCommandGroup> autonomousSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        autonomousSelector.setDefaultOption(
            "testTWOPieceMid", new TWOPieceMid(swerve, superstructure));
       
        autonomousSelector.addOption("testTWOPieceAMP", new TWOPieceAMP(swerve, superstructure));
        
        autonomousSelector.addOption("testTWOPieceSOURCE", new TWOPieceSOURCE(swerve, superstructure));

        autonomousSelector.addOption("testEFarMID", new EFarMID(swerve, superstructure)); //THIS WILL RUN INTO NOTE!

        autonomousSelector.addOption("testEFarAMP", new EFarAMP(swerve, superstructure));

        autonomousSelector.addOption("testEFarMIDTHREE", new EFarMIDTHREE(swerve, superstructure));

        autonomousSelector.addOption("THREEPieceAMP", new THREEPieceAMP(swerve, superstructure));

        autonomousSelector.addOption("THREEPieceMid", new THREEPieceMid(swerve, superstructure));

        autonomousSelector.addOption("NAMEDTWOPieceMid", new NAMEDTWOPieceMid(swerve, superstructure));

        SmartDashboard.putData("Auto Choices", autonomousSelector);

    }
    
public SequentialCommandGroup get(){
    return autonomousSelector.getSelected();
}
}

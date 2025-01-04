package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class AutoController {
    private Swerve swerve;

    public AutoController(Swerve swerve){
        this.swerve = swerve;
    }

    public Command wait(double seconds) {
        return new WaitCommand(seconds);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber;

public class climberCmd extends Command{
    private final climber climber;
    private final XboxController xbox;

    private boolean autoShooter;

    private double climberPos;

    public climberCmd(climber climber, XboxController xbox) {
        this.climber = climber;
        addRequirements(climber);

        this.xbox = xbox;
        autoShooter = false;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (DriverStation.isTeleop()) {
          /*   if (xbox.getLeftBumperButtonPressed()) {
                climber.climberUp();
            } 

            if (xbox.getLeftBumperButtonReleased()) {
                climber.climberStop();
            }

            if (xbox.getRightBumperButtonPressed()) {
                climber.climberDown();
            }

            if (xbox.getRightBumperButtonReleased()) {
                climber.climberStop();
            }*/

        } 

    }
}

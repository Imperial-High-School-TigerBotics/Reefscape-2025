package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.ArmShootAndIntake;
import frc.robot.subsystems.Elevator;

public class AutoController {
    private Elevator elevator;
    private ArmRotation armRotation;
    private ArmShootAndIntake armShootAndIntake;

    public AutoController(Elevator elevator, ArmRotation armRotation,ArmShootAndIntake armShootAndIntake) {
        this.elevator = elevator;
        this.armRotation = armRotation;
        this.armShootAndIntake = armShootAndIntake;
    }

    public Command wait(double seconds){
        return new WaitCommand(seconds);
    }

    public Command scoreCoralL2(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setElevatorPosition(Constants.PresetElevatorAndArmConstants.elevatorScoreCoralL2Pos), elevator),
                new InstantCommand(() -> armRotation.setArmRotatorPosition(Constants.PresetElevatorAndArmConstants.armScoreCoralL2Pos), armRotation)
            ),
            new InstantCommand(() -> armShootAndIntake.CoralIntakeOut(), armShootAndIntake),
            new InstantCommand(() -> {
                armShootAndIntake.CoralIntakeStop();
                elevator.setElevatorPosition(Constants.ElevatorConstants.elevatorRestPos);
                armRotation.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
            }, 
                armShootAndIntake, elevator, armRotation
            )
        );
    }

    public Command scoreCoralL4(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setElevatorPosition(Constants.PresetElevatorAndArmConstants.elevatorScoreCoralL4Pos), elevator),
                new InstantCommand(() -> armRotation.setArmRotatorPosition(Constants.PresetElevatorAndArmConstants.armScoreCoralL4Pos), armRotation)
            ),
            new WaitCommand(1.5),
            new InstantCommand(() -> armShootAndIntake.CoralIntakeOut(), armShootAndIntake),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                armShootAndIntake.CoralIntakeStop();
                elevator.setElevatorPosition(Constants.ElevatorConstants.elevatorRestPos);
                armRotation.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
            }, 
                armShootAndIntake, elevator, armRotation
            )
        );
    }

    public Command coralIntakefromSource(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setElevatorPosition(Constants.PresetElevatorAndArmConstants.elevatorCoralIntakeFromSourcePos), elevator),
                new InstantCommand(() -> armRotation.setArmRotatorPosition(Constants.PresetElevatorAndArmConstants.armCoralIntakeFromSourcePos), armRotation)
            ),
            new WaitCommand(1.5),
            new InstantCommand(() -> armShootAndIntake.CoralIntakeIn(), armShootAndIntake),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                armShootAndIntake.CoralIntakeStop();
                elevator.setElevatorPosition(Constants.ElevatorConstants.elevatorRestPos);
                armRotation.setArmRotatorPosition(Constants.ArmConstants.ArmRestPos);
            }, 
                armShootAndIntake, elevator, armRotation
            )
        );
    }
}

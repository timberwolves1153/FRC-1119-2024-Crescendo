package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Limelight;

public class CollectNote extends Command {
    
    private Collector collector;
    private Limelight limelight;
    
    
    public CollectNote(Collector collector) {
        addRequirements(collector);

        this.collector = collector;
    }

    @Override
    public void initialize() {
        collector.collectorIntake();
    }

    @Override
    public void execute() {

    }


    @Override
    public void end(boolean interrupted) {
    //    collector.collectorStop();
    if (collector.hasNote()) {
            collector.collectorStop();
            // new SequentialCommandGroup(
            // new InstantCommand(() -> limelight.setLEDMode_ForceBlink("limelight"), limelight),
            // new WaitCommand(3),
            // new InstantCommand(() -> limelight.setLEDMode_ForceOff("limelight")));
          //  limelight.setLEDMode_ForceOff("limelight");

        }
        //new PositionNote(collector).withTimeout(.5);
        // new InstantCommand(() -> collector.collectorOuttake(), collector)
        //     .withTimeout(.5)
        //     .andThen(new InstantCommand(() -> collector.collectorStop(), collector));
    }

}

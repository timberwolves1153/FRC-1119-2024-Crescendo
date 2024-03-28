package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Collector;

public class CollectNote extends Command {
    
    private Collector collector;
    
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
    public boolean isFinished() {     
        return !collector.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
    //    collector.collectorStop();
        if (interrupted) {
            collector.collectorStop();
        }
        //new PositionNote(collector).withTimeout(.5);
        // new InstantCommand(() -> collector.collectorOuttake(), collector)
        //     .withTimeout(.5)
        //     .andThen(new InstantCommand(() -> collector.collectorStop(), collector));
    }

}

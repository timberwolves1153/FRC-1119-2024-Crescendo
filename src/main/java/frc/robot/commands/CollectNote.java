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

    // @Override
    // public void initialize() {
    //     collector.collectorIntake();
    // }

    @Override
    public void execute() {
        collector.collectorIntake();
    }

    // @Override
    // public void end(boolean interrupted) {
    //     collector.collectorStop();
    // }

    // @Override
    // public boolean isFinished() {
    //     return collector.hasNote();
    // }

}

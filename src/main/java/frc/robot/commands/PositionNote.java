package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Collector;

public class PositionNote extends Command {
    
    private Collector collector;
    
    public PositionNote(Collector collector) {
        addRequirements(collector);

        this.collector = collector;
    }

    @Override
    public void initialize() {
        collector.collectorOuttake();
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
        collector.collectorStop();
    }

}

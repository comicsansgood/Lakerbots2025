package frc.robot.Commands.CustomAutos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CustomTest extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  AutoBuilder autoBuilder;

    public CustomTest(CommandSwerveDrivetrain m_drivetrain, AutoBuilder autoBuilder) {
      this.m_drivetrain = m_drivetrain;
      this.autoBuilder = autoBuilder;
      addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() { 
      //m_drivetrain.auto
      //autoBuilder.followPath();
      //Commands
      
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
      return false;
    }
    
    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
package frc.robot.Commands.CustomAutos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveBlindForTime extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  LimelightSubsystem m_limelight;
  SwerveRequest.ApplyRobotSpeeds robotSpeeds;
  double calculatedForwardBackSpeed = 2;
  Translation2d calculatedTranslation;
  double speed = 1;
  double startingTime;
  double CurrentTime;
  double time;
  
  //Drives at a speed for a time
  public DriveBlindForTime(CommandSwerveDrivetrain m_drivetrain, LimelightSubsystem m_limelight, SwerveRequest.ApplyRobotSpeeds robotSpeeds, double time) {
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;
    this.robotSpeeds = robotSpeeds;
    this.time = time;
    addRequirements(m_drivetrain, m_limelight);
  }

    @Override
    public void initialize() { 
      startingTime = Timer.getTimestamp();
    }

    @Override
    public void execute() {

      CurrentTime = Timer.getTimestamp();
      
      calculatedTranslation = new Translation2d(
        0,
        calculatedForwardBackSpeed
        );
    
       m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(calculatedTranslation.getY(),speed*calculatedTranslation.getX(), 0)));
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0,0,0)));
    }

    @Override
    public boolean isFinished() {
      return CurrentTime - startingTime >= time;
    }
    
    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
    
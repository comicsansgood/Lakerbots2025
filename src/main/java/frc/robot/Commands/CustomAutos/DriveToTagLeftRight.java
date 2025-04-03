package frc.robot.Commands.CustomAutos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveToTagLeftRight extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  LimelightSubsystem m_limelight;
  SwerveRequest.ApplyRobotSpeeds robotSpeeds;
  Double[] desiredTagTranslation;
  Double[] values;
  double distance;
  Translation2d calculatedTranslation;
  double tolerence = 2;
  double speed = 1; //TODO:tune
  
  PIDController m_xPid = new PIDController(0.2, 0, 0);//different than other comma

  




    public DriveToTagLeftRight(CommandSwerveDrivetrain m_drivetrain, LimelightSubsystem m_limelight, SwerveRequest.ApplyRobotSpeeds robotSpeeds, Double[] desiredTagTranslation) {
      this.m_drivetrain = m_drivetrain;
      this.m_limelight = m_limelight;

      this.robotSpeeds = robotSpeeds;
      this.desiredTagTranslation = desiredTagTranslation;
      addRequirements(m_drivetrain, m_limelight);
    }

    @Override
    public void initialize() { 
      m_xPid.setTolerance(tolerence);
      m_xPid.setSetpoint(desiredTagTranslation[0]);

    }

    @Override
    public void execute() {
      


      values = m_limelight.getValues();
      System.out.println(values[0] +", " +values[2]);

      calculatedTranslation = new Translation2d(
        m_xPid.calculate(values[0]),
        0
        );
      //System.out.println("calculated x speed: " + calculatedTranslation.getX() + "   calculated y speed: " + calculatedTranslation.getY());
    
       m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(calculatedTranslation.getY(),speed*calculatedTranslation.getX(), 0)));
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0,0,0)));
    }

    @Override
    public boolean isFinished() {
      return m_xPid.atSetpoint();
    }
    
    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
    
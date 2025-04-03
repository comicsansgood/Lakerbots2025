package frc.robot.Commands.CustomAutos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveBack extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  SwerveRequest.ApplyRobotSpeeds robotSpeeds;
  double distance;
  double startingDistance;
  double calculatedForwardBackSpeed = 0.5;
  Translation2d calculatedTranslation;
  double tolerence = 0.1;//TODO:tune
  double speed = 1; //TODO:tune
  

  




    public DriveBack(CommandSwerveDrivetrain m_drivetrain, SwerveRequest.ApplyRobotSpeeds robotSpeeds, double distance) {
      this.m_drivetrain = m_drivetrain;
      this.robotSpeeds = robotSpeeds;
      this.distance = distance;
      addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() { 

      startingDistance = m_drivetrain.getPose().getX();
    }

    @Override
    public void execute() {
      if(Math.abs(m_drivetrain.getPose().getX() - startingDistance) < distance){
        calculatedForwardBackSpeed = -2;
      }
      else{
        calculatedForwardBackSpeed = 0;
      }


      

      calculatedTranslation = new Translation2d(
        0,
        calculatedForwardBackSpeed
        );
      System.out.println("diff" + (m_drivetrain.getPose().getX() - startingDistance) + ", des" + distance);
      //System.out.println("calculated x speed: " + calculatedTranslation.getX() + "   calculated y speed: " + calculatedTranslation.getY());
    
       m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(calculatedTranslation.getY(),calculatedTranslation.getX(), 0)));
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0,0,0)));
    }

    @Override
    public boolean isFinished() {
      return calculatedForwardBackSpeed ==0;//m_xPid.atSetpoint();
    }
    
    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
    
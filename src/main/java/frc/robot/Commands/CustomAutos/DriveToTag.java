package frc.robot.Commands.CustomAutos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

//Alligns to tag using the left right pos of tag on the camera feed and the area of the camera feed taken up by the tag

public class DriveToTag extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  LimelightSubsystem m_limelight;
  SwerveRequest.ApplyRobotSpeeds robotSpeeds;
  Double[] desiredTagTranslation;
  Double[] values;
  Translation2d calculatedTranslation;
  double tolerence = 0.1;
  double speed = 1;
  
  //PID loops
  PIDController m_xPid = new PIDController(0.1, 0, 0);
  PIDController m_aPid = new PIDController(0.19, 0.0325, 0);


  public DriveToTag(CommandSwerveDrivetrain m_drivetrain, LimelightSubsystem m_limelight, SwerveRequest.ApplyRobotSpeeds robotSpeeds, Double[] desiredTagTranslation) {
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;
    this.robotSpeeds = robotSpeeds;
    this.desiredTagTranslation = desiredTagTranslation;
    addRequirements(m_drivetrain, m_limelight);
  }

  @Override
  public void initialize() { 
    m_xPid.setTolerance(tolerence);
    m_aPid.setTolerance(tolerence);

    m_xPid.setSetpoint(desiredTagTranslation[0]);
    m_aPid.setSetpoint(desiredTagTranslation[1]);
  }

  @Override
  public void execute() {
    //pull camera feed values
    values = m_limelight.getValues();
    System.out.println(values[0] +", " +values[2]);

    //calculate drive speed based on PID
    calculatedTranslation = new Translation2d(
      m_xPid.calculate(values[0]),
      m_aPid.calculate(values[2])
      );
    
      m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(speed*calculatedTranslation.getY(),speed*calculatedTranslation.getX(), 0)));
  }

    @Override
    public void end(boolean interrupted) {
      m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0,0,0)));
    }

    @Override
    public boolean isFinished() {
      return m_xPid.atSetpoint() && m_aPid.atSetpoint();
    }
    
    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
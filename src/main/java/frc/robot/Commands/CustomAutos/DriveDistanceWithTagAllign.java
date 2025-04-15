package frc.robot.Commands.CustomAutos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

//Alligns with the tag left right while also driving a blind distance forward back

public class DriveDistanceWithTagAllign extends Command {

  CommandSwerveDrivetrain m_drivetrain;
  LimelightSubsystem m_limelight;
  SwerveRequest.ApplyRobotSpeeds robotSpeeds;
  Double[] desiredTagTranslation;
  Double[] values;
  double distance;
  double startingDistance;
  double calculatedForwardBackSpeed = 0.5;
  Translation2d calculatedTranslation;
  double tolerence = 0.1;
  double speed = 1;
  
  //only one pid controller
  PIDController m_xPid = new PIDController(0.1, 0, 0);

  public DriveDistanceWithTagAllign(CommandSwerveDrivetrain m_drivetrain, LimelightSubsystem m_limelight, SwerveRequest.ApplyRobotSpeeds robotSpeeds, Double[] desiredTagTranslation, double distance) {
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;
    this.robotSpeeds = robotSpeeds;
    this.desiredTagTranslation = desiredTagTranslation;
    this.distance = distance;
    addRequirements(m_drivetrain, m_limelight);
  }

    @Override
    public void initialize() { 
      m_xPid.setTolerance(tolerence);
      //set setpoint
      m_xPid.setSetpoint(desiredTagTranslation[0]);

      startingDistance = m_drivetrain.getPose().getX();
    }

    @Override
    public void execute() {
      //if the difference between the current x pos 
      //and the starting x pos is less than the input distance, keep driving at a speed of 2
      if(Math.abs(m_drivetrain.getPose().getX() - startingDistance) < distance){
        calculatedForwardBackSpeed = 2;
      }
      else{
        calculatedForwardBackSpeed = 0;
      }

      //pull values from camera feed
      values = m_limelight.getValues();
      System.out.println(values[0] +", " +values[2]);

      //speed to drive at is a component vector of the PID L-R calculation and the calculatedForwardBackSpeed described above
      calculatedTranslation = new Translation2d(
        m_xPid.calculate(values[0]),
        calculatedForwardBackSpeed
        );

      System.out.println("diff" + (m_drivetrain.getPose().getX() - startingDistance) + ", des" + distance);
    
       m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(calculatedTranslation.getY(),speed*calculatedTranslation.getX(), 0)));
    }

    @Override
    public void end(boolean interrupted) {
      m_drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0,0,0)));
    }

    @Override
    public boolean isFinished() {
      //Finish the command when it has finished its forward back target
      return calculatedForwardBackSpeed == 0;
    }
    
    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
    
package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToTarget extends Command {

    CommandSwerveDrivetrain m_Drivetrain;
    SwerveRequest.ApplyRobotSpeeds robotSpeeds;
    Pose2d currentPose;
    Pose2d desiredPose;
    Translation2d calculatedTranslation;
    double calcuatedRotation;
    double speed;
    double minSpeed = -5;
    double maxSpeed = 5;
 
    public DriveToTarget(CommandSwerveDrivetrain m_Drivetrain, SwerveRequest.ApplyRobotSpeeds robotSpeeds, Pose2d currentPose, Pose2d desiredPose, double speed) {

        this.m_Drivetrain = m_Drivetrain;
        this.robotSpeeds = robotSpeeds;
        this.currentPose = currentPose;
        this.desiredPose = desiredPose;
        this.speed = speed;
        addRequirements(m_Drivetrain);

    }

    @Override
    public void initialize() {
    m_Drivetrain.setTolerence(.1);

    //PID setpoint
    m_Drivetrain.setSetpoint(desiredPose.getX(), desiredPose.getY());
    System.out.println(desiredPose.getX());
    System.out.println(desiredPose.getY());      
    }

    @Override
    public void execute() {
      currentPose = m_Drivetrain.getPose();

      //calculate translation from PID signal
      calculatedTranslation = new Translation2d(
        MathUtil.clamp(speed*m_Drivetrain.getSpeeds(currentPose.getX(),currentPose.getY())[0]   , minSpeed, maxSpeed), 
        MathUtil.clamp(speed*m_Drivetrain.getSpeeds(currentPose.getX(),currentPose.getY())[1], minSpeed, maxSpeed)
      );
      System.out.println("calcuated x: " + calculatedTranslation.getX()+"     calculated y: " + calculatedTranslation.getY());
      
    
      m_Drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(calculatedTranslation.getX(), calculatedTranslation.getY(), 0)));
    }

    @Override
    public void end(boolean interrupted) {
      m_Drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public boolean isFinished() {
      return m_Drivetrain.isAtDesired();
    }
    
    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
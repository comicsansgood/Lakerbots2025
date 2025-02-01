

package frc.robot.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

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
    m_Drivetrain.setSetpoint(desiredPose.getX(), desiredPose.getY());
    System.out.println(desiredPose.getX());
    System.out.println(desiredPose.getY());


      

      //calcuatedRotation = desiredPose.getRotation().getDegrees()-currentPose.getRotation().getDegrees();
      
    }

    @Override
    public void execute() {
      currentPose = m_Drivetrain.getPose();
      //System.out.println("executing driveTarget");
      //System.out.println("x:"+currentPose.getX());
      //System.out.println("y"+currentPose.getY());
      calculatedTranslation = new Translation2d(
        MathUtil.clamp(speed*m_Drivetrain.getSpeeds(currentPose.getX(),currentPose.getY())[0]   , minSpeed, maxSpeed), 
        MathUtil.clamp(speed*m_Drivetrain.getSpeeds(currentPose.getX(),currentPose.getY())[1], minSpeed, maxSpeed)
      );
      System.out.println("calcuated x: " + calculatedTranslation.getX()+"     calculated y: " + calculatedTranslation.getY());
      
    
      m_Drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(calculatedTranslation.getX(), calculatedTranslation.getY(), 0)));



      //drive at set vector -1 to 1 (x,y)
      //if(desired x - starting x   is positive): vx = speed
      //else: vx = -speed
      //
      //if(desired y - starting y   is positive): vy = speed
      //else: vy = -speed
      //drive at vx ,vy
    }

    @Override
    public void end(boolean interrupted) {
      m_Drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public boolean isFinished() {
      return m_Drivetrain.isAtDesired();
      /*//wrong
      if(Math.abs(m_Drivetrain.getPose().getX()-desiredPose.getX()) < 2 && Math.abs(m_Drivetrain.getPose().getY()-desiredPose.getY()) < 2 && Math.abs(m_Drivetrain.getPose().getRotation().getDegrees()-desiredPose.getRotation().getDegrees()) < 2){
        return true;
      }
      return false; 
      */
      //if current x - starting x within a threshold of desired x and
      //if current y - starting y within a threshold of desired y:
      //  return true
      //else return false
    }
    

    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}
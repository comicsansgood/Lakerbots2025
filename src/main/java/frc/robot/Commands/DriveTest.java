

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


public class DriveTest extends Command {

    CommandSwerveDrivetrain m_Drivetrain;
    SwerveRequest.FieldCentric drive;
    SwerveRequest.ApplyRobotSpeeds robotSpeeds;
    Pose2d currentPose;
    Pose2d desiredPose;
    Translation2d calculatedTranslation;
    double calcuatedRotation;
    double speed;
    double minSpeed = -1000;
    double maxSpeed = 1000;//TODO:clamp values
 

    public DriveTest(CommandSwerveDrivetrain m_Drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.ApplyRobotSpeeds robotSpeeds, double speed) {

        this.m_Drivetrain = m_Drivetrain;
        this.drive = drive;
        this.robotSpeeds = robotSpeeds;
        this.speed = speed;
        addRequirements(m_Drivetrain);

    }

    @Override
    public void initialize() {
      
    }

    @Override
    public void execute() {
      
    //m_Drivetrain.setControl(drive.withVelocityX(0.2).withVelocityY(0.4));
    m_Drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(5, 0, 0)));

/* 
      m_Drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(0.5) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
            );
*/
    }

    @Override
    public void end(boolean interrupted) {
      Commands.runOnce(() ->{
      
        m_Drivetrain.applyRequest(() ->
                  drive.withVelocityX(0) // Drive forward with negative Y (forward)
                      .withVelocityY(0) // Drive left with negative X (left)
                      .withRotationalRate(0) // Drive counterclockwise with negative X (left)
              );}, m_Drivetrain
        );
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
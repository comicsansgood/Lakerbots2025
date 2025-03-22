package frc.robot.Commands.CustomAutos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoCommand {

    static CommandSwerveDrivetrain drivetrain;
    static LimelightSubsystem limelight;
    static SwerveRequest.ApplyRobotSpeeds robotSpeeds;

    static List<Waypoint> waypoints;
    
    //PathPlannerPath firstPath = PathPlannerPath.fromPathFile(Filesystem.getDeployDirectory().toPath().resolve("app"));
    static PathPlannerPath firstPath;

    //constraints of path
    static PathConstraints constraints = new PathConstraints(3, 3, 3*Math.PI, 4*Math.PI);//TODO: constraint values double check i think these are ok?

    public static Command autoTest(
            CommandSwerveDrivetrain p_drivetrain, 
            LimelightSubsystem p_limelight, 
            SwerveRequest.ApplyRobotSpeeds p_robotSpeeds
            )
        {
        drivetrain = p_drivetrain;
        limelight = p_limelight;
        robotSpeeds = p_robotSpeeds;

        try {
            firstPath = PathPlannerPath.fromPathFile("app.path"); //broken so i put a band aid on it
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }



        return Commands.sequence(
            AutoBuilder.followPath(firstPath),// approach
            new DriveToTag(drivetrain, limelight, robotSpeeds, Constants.TagConstants.tagTranslation),//tag
            AutoBuilder.followPath(generatePath())
            
            
        );
    }


    public static PathPlannerPath generatePath(){

        waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0)),
            new Pose2d(5,6, Rotation2d.fromDegrees(0))//rotation represents direction of travel not holonomic rotation
        );
        return new PathPlannerPath(waypoints, constraints, new IdealStartingState(0.0, Rotation2d.fromDegrees(180)), new GoalEndState(0.0, Rotation2d.fromDegrees(180)));
    }
    
}

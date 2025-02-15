// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.DriveTest;
import frc.robot.Commands.DriveToTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber2;
import frc.robot.subsystems.FlapHook;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    public static Manipulator manipulator = new Manipulator();
    public static Climber2 climber = new Climber2();
    public static FlapHook flapHook = new FlapHook();
    public static Elevator elevator = new Elevator();
    public static LimelightSubsystem limelight = new LimelightSubsystem();
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("drive to 0,0", new DriveToTarget(drivetrain, robotSpeeds,drivetrain.getState().Pose,
               new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                MaxSpeed
                ));
        SmartDashboard.putData("drive to 1,1", new DriveToTarget(drivetrain, robotSpeeds,drivetrain.getState().Pose,
            new Pose2d(new Translation2d(1,1), new Rotation2d(0)),
            MaxSpeed
            ));

        SmartDashboard.putData("testing drive in command", new DriveTest(drivetrain, drive, robotSpeeds, MaxSpeed));

        SmartDashboard.putData("spin until detect", manipulator.spinUntilDetected());



        //so broken lol
        ///SmartDashboard.putData("climber down", climber.climber2Spin(0.2));
        ///SmartDashboard.putData("climber 0", climber.climber2Spin(0));
        //SmartDashboard.putData("climber up", climber.climber2Spin(-0.2));

        SmartDashboard.putData("manipulator wrist up", manipulator.manipulatorWristSpin(-0.05));
        SmartDashboard.putData("manipulator wrist 0", manipulator.manipulatorWristSpin(0.0));
        SmartDashboard.putData("manipulator wrist down", manipulator.manipulatorWristSpin(0.05));

        SmartDashboard.putData("manipulator forward", manipulator.manipulatorSpin(0.2));
        SmartDashboard.putData("manipulator 0", manipulator.manipulatorSpin(0));
        SmartDashboard.putData("manipulator backwards", manipulator.manipulatorSpin(-0.2));
      
        SmartDashboard.putData("flaphook forward", flapHook.flapHookSpin(-0.05));
        SmartDashboard.putData("flaphook 0", flapHook.flapHookSpin(0));
        SmartDashboard.putData("flaphook backwards", flapHook.flapHookSpin(0.05));

        SmartDashboard.putData("climber222 down", climber.climber2Spin(-0.1));
        SmartDashboard.putData("climber222 0", climber.climber2Spin(0));
        SmartDashboard.putData("climber222 up", climber.climber2Spin(0.1));

        SmartDashboard.putData("flaphook open", flapHook.hookGoToPosition(Constants.FlapHookConstants.hookflapOpen));
        SmartDashboard.putData("flaphook prepare", flapHook.hookGoToPosition(Constants.FlapHookConstants.hookPrepare));
        SmartDashboard.putData("flaphook closed", flapHook.hookGoToPosition(Constants.FlapHookConstants.hookLatch));

        SmartDashboard.putData("climber position up", climber.climberGoToPosition(Constants.ClimberConstants.climberHome));
        SmartDashboard.putData("climber position down", climber.climberGoToPosition(Constants.ClimberConstants.climberDown));

        SmartDashboard.putData("manipulator algea", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgeaCollect));
        SmartDashboard.putData("manipulator home", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome));
        SmartDashboard.putData("manipulator travel", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel));
        SmartDashboard.putData("manipulator L1", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorCoralL1));
        SmartDashboard.putData("manipulator L4", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorCoralL4));

       
        SmartDashboard.putData("elevator home", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome));
        SmartDashboard.putData("elevator process", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorProcess));
        SmartDashboard.putData("elevator coral 1", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL1));
        SmartDashboard.putData("elevator coral 2", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL2));
        SmartDashboard.putData("elevator coral 3", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL3));
        SmartDashboard.putData("elevator coral 4", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL4));
        SmartDashboard.putData("elevator alegae2", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL2));
        SmartDashboard.putData("elevator alegae3", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL3));








        SmartDashboard.putBoolean("isLazerConnected", Constants.isLazerConnected);

        SmartDashboard.putNumber("Elevator Current", elevator.elevatorGetCurrent());

        /*SmartDashboard.putData("testingDrivecmd", drivetrain.applyRequest(() ->
        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));*/

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
       /*  joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        //joystick.x().onTrue(manipulator.spinUntilDetected());
        //joystick.a().onTrue(climber.climberDown(0.3));    // one speed will be negative, one positive 
        //joystick.y().onTrue(climber.climberUp(-0.3));
       // joystick.x().onTrue(climber.climberStop(0));

        joystick2.a().onTrue(flapHook.hookGoOut(0.3));      // one speed will be negative, one positive
        joystick2.y().onTrue(flapHook.hookGoIn(0.3));
        joystick2.x().onTrue(flapHook.hookStop(0));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}

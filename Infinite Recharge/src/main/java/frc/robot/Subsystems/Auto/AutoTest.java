package frc.robot.Subsystems.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Path;
import frc.robot.Lib.AutoFramework;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Hardware;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class AutoTest extends AutoFramework {
    private Drivetrain myDrive;
    //if myDrive fails, switch to robotDrive and make this.myDrive = robotDrive;
    //private final Drivetrain robotDrive = new Drivetrain(Hardware.rightMasterMotor, Hardware.rightSlaveMotor, Hardware.leftMasterMotor, Hardware.leftSlaveMotor, 
    //Hardware.MXP, Hardware.shifter);
    private int counter;

    public AutoTest(Drivetrain myDrive) {
        this.myDrive = myDrive;
    }

    @Override
    public void outputToSmartDashboard() {
        // TODO Auto-generated method stub
        
    }
    
    
    @Override
    public void update() {
        getAuto();
    }
    public Trajectory getTrajectory() {
    String trajectoryJSON = "paths/InitiationLine.wpilib.json";
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        return trajectory;
    }   
    catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        return null;
    }
    }
    
    public Command getAuto() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltsSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        getTrajectory(),
        myDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltsSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        myDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0), 
        // RamseteCommand passes volts to the callback
        myDrive::tankDriveVolts,
        myDrive
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> myDrive.tankDriveVolts(0, 0));
  }
}

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Util.doubleSolenoid;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase implements SubsystemFramework {
    private CANSparkMax rightMasterMotor;
    private CANSparkMax rightSlaveMotor;
    private CANSparkMax leftMasterMotor;
    private CANSparkMax leftSlaveMotor;
    private DifferentialDrive myRobot;
    private AHRS MXP;
    private doubleSolenoid shifter;
    private CANEncoder rightCanEncoder;
    private CANEncoder leftCanEncoder;
    private CANPIDController rightPIDController;
    private CANPIDController leftPIDController;
    //private double currentSetpoint;
    public DrivetrainStates state = DrivetrainStates.LowGear;
    public DrivetrainStates prevState;
    public final DifferentialDriveOdometry odometry;
    
    public Drivetrain(CANSparkMax rightMasterMotor, CANSparkMax rightSlaveMotor, CANSparkMax leftMasterMotor, CANSparkMax leftSlaveMotor,
    AHRS MXP, doubleSolenoid shifter){
        // Motor Controllers
        this.rightMasterMotor = rightMasterMotor;
        this.rightSlaveMotor = rightSlaveMotor;
        this.leftMasterMotor = leftMasterMotor;
        this.leftSlaveMotor = leftSlaveMotor;

        // Gyro
        this.MXP = MXP;

        // Shifter
        this.shifter = shifter;
        
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        // Setup Drivetrain
        myRobot = new DifferentialDrive(rightMasterMotor, leftMasterMotor);
    } 
    public enum DrivetrainStates {
        HighGear, LowGear;
    }
    public static boolean HighGear() {
        return(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_HIGHGEAR));
    }
    public static boolean LowGear() {
        return(Hardware.driverPad.getRawButton(Constants.DRIVETRAIN_LOWGEAR));
    }
    @Override
    public void update() {
        DrivetrainStates newState = state;
        resetMXP();
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftCanEncoder.getPosition(), rightCanEncoder.getPosition());
        switch(state) {
            case LowGear:
                // Shift into LowGear
                shifter.close();
                
                // Joystick Control
                myRobot.arcadeDrive(Hardware.driverPad.getRawAxis(1) * -1, Hardware.driverPad.getRawAxis(4) * -1, false);
                //myRobot.tankDrive(Hardware.driverPad.getRawAxis(4) * -1, Hardware.driverPad.getRawAxis(4) * -1, false);
                
                // Switch to HighGear when asked
                if (HighGear()) {
                    newState = DrivetrainStates.HighGear;
                }
                
                break;
            case HighGear:
                // Shift into HighGear
                shifter.open();

                // Joystick Control
                myRobot.arcadeDrive(Hardware.driverPad.getRawAxis(1) * -1, Hardware.driverPad.getRawAxis(4) * -1, false);
                //myRobot.tankDrive(Hardware.driverPad.getRawAxis(4) * -1, Hardware.driverPad.getRawAxis(4) * -1, false);

                // Switch to LowGear when asked
                if(LowGear()) {
                    newState = DrivetrainStates.LowGear;
                }
                
                break;
            default:
                newState = state;
                break;
            }
            if (newState != state) {
                state = newState;
            }
        outputToSmartDashboard();
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Encoder Position", rightCanEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Velocity", leftCanEncoder.getVelocity());
    }
    @Override
    public void setupSensors() {
        // Setup Gyro
        MXP.zeroYaw();
        
        // Setup Master Encoders
        rightMasterMotor.restoreFactoryDefaults();
        rightSlaveMotor.restoreFactoryDefaults();
        leftMasterMotor.restoreFactoryDefaults();
        leftSlaveMotor.restoreFactoryDefaults();
        
        rightCanEncoder = rightMasterMotor.getEncoder();
        leftCanEncoder = leftMasterMotor.getEncoder();
        
        rightPIDController = rightMasterMotor.getPIDController();
        leftPIDController = leftMasterMotor.getPIDController();
        
        rightPIDController.setP(Constants.Drivetrain_Kp);
        rightPIDController.setI(Constants.Drivetrain_Ki);
        rightPIDController.setD(Constants.Drivetrain_Kd);
        rightPIDController.setOutputRange(-1, 1);
        
        leftPIDController.setP(Constants.Drivetrain_Kp);
        leftPIDController.setI(Constants.Drivetrain_Ki);
        leftPIDController.setD(Constants.Drivetrain_Kd);
        leftPIDController.setOutputRange(-1, 1);
        
        // Setup Master Slave Relationship
        rightSlaveMotor.follow(rightMasterMotor);
        leftSlaveMotor.follow(leftMasterMotor);

        setScaleDistance();
    }
    public double getAngle() {
        return MXP.getAngle();
    }
    public void resetMXP() {
        if(MXP.getAngle() > 360 || MXP.getAngle() < -360) {
            MXP.zeroYaw();
        }
    }
    public void setDrivetrain(double distance) {
        double rotations = ((distance / Constants.Drivetrain_Wheel_Circumference) * Constants.Drivetrain_Gear_Ratio);
        rightPIDController.setReference(rotations, ControlType.kPosition);
        leftPIDController.setReference(rotations, ControlType.kPosition);
    }
    //public double getPosition() {
    //    return (rightCanEncoder.getPosition() * Constants.Drivetrain_Wheel_Circumference)/Constants.Drivetrain_Gear_Ratio;
    //}
    public void setScaleDistance() {
        // Accurately Accounts for Gearing and Wheel getDistance()
        leftCanEncoder.setPositionConversionFactor(Constants.Drivetrain_Wheel_Circumference * Constants.Drivetrain_Gear_Ratio);
        rightCanEncoder.setPositionConversionFactor(Constants.Drivetrain_Wheel_Circumference * Constants.Drivetrain_Gear_Ratio);
        // Velocity getRate()
        leftCanEncoder.setVelocityConversionFactor(Constants.Drivetrain_Wheel_Circumference * Constants.Drivetrain_Gear_Ratio / 60.0);
        rightCanEncoder.setVelocityConversionFactor(Constants.Drivetrain_Wheel_Circumference * Constants.Drivetrain_Gear_Ratio / 60.0);
    }
    public double getHeading() {
        return Math.IEEEremainder(getAngle(), 360);
    }
    public double getTurnRate() {
        // In degrees per second
        return MXP.getRate();
    }
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    public double getAverageEncoderDistance() {
        return ((leftCanEncoder.getPosition() + rightCanEncoder.getPosition()) / 2);
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftCanEncoder.getVelocity(), rightCanEncoder.getVelocity());
    }
    public void setMaxOutput(double maxOutput) {
        myRobot.setMaxOutput(maxOutput);
    }
    public void arcadeDrive(double forward, double rotation) {
        myRobot.arcadeDrive(forward, rotation);
    }
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        rightMasterMotor.setVoltage(-rightVolts);
        leftMasterMotor.setVoltage(leftVolts);
        myRobot.feed();
    }

}
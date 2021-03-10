package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Subsystems.CannonPolycord.CannonpolycordStates;
import frc.robot.Subsystems.Limelight.LimelightStates;
import frc.robot.Subsystems.CannonPolycord;
import frc.robot.Subsystems.Limelight;

public class CannonShooter implements SubsystemFramework {
    private WPI_TalonSRX masterShooter;
    private WPI_VictorSPX slaveShooter;
    private Limelight limelight = new Limelight();

    public static CannonShooterStates state = CannonShooterStates.Off;
    public CannonShooter(WPI_TalonSRX masterShooter, WPI_VictorSPX slaveShooter) {
        this.masterShooter = masterShooter;
        this.slaveShooter = slaveShooter;
    }
    public CannonShooter() {
        
    }
    
    public enum CannonShooterStates {
        Off, Shoot
    }    
    
    public boolean Shoot() {
        return(Hardware.driverPad.getRawAxis(Constants.SHOOT) > 0.1);
    }
    public void update() {
        CannonShooterStates newState = state;
        
        switch(state) {
            case Off:
                masterShooter.set(0);
                slaveShooter.set(0);
                SmartDashboard.putString("Shooter", "Off");
                limelight.state = LimelightStates.LEDon;
                //CannonPolycord.state = CannonpolycordStates.Off;
                if(Shoot()) {
                    newState = CannonShooterStates.Shoot;
                }
                break;
            case Shoot:
                //limelight.state = LimelightStates.LEDon;
                //masterShooter.set(ControlMode.Velocity, limelight.findVelocity());
                //slaveShooter.set(ControlMode.Velocity, limelight.findVelocity() - (limelight.findVelocity() * 0.20));
                masterShooter.set(1);
                slaveShooter.set(-1);
                SmartDashboard.putString("Shooter", "Shoot");
                //CannonPolycord.state = CannonpolycordStates.Shooting;
                if(!Shoot()) {
                    newState = CannonShooterStates.Off;
                }
                break;
            default:
                newState = CannonShooterStates.Off;
                break;
        }
        if (newState != state) {
            state = newState;
        }
        outputToSmartDashboard();
    }

    public void setupSensors() {
        masterShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        
        //slaveShooter.follow(masterShooter);

        slaveShooter.setInverted(false);
        masterShooter.setInverted(false);
        //slaveShooter.follow(masterShooter); // Created
        
        masterShooter.configNominalOutputForward(0, 10);
        masterShooter.configNominalOutputReverse(0, 10);
        masterShooter.configPeakOutputForward(1, 10);
        masterShooter.configPeakOutputReverse(-1, 10);
        
        //masterShooter.config_kF(0, 0.22, 10); 
        //masterShooter.config_kP(0, 0.0938, 10);
	    //masterShooter.config_kI(0, 0.1, 10);
        //masterShooter.config_kD(0, 0.0433, 10);
        //slaveShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        //slaveShooter.config_kF(0, 0.22, 10); //1023* duty cyc/getvelocity
        //slaveShooter.config_kP(0, 0.0795, 10);
        //slaveShooter.config_kD(0, 0.0359, 10);
        
        
        // Min/Max kf -1023 to 1023 
        // Create Velocity PID
    }

    public void outputToSmartDashboard() {
        //SmartDashboard.putNumber("ty", limelight.getTy());
        //SmartDashboard.putNumber("tx", limelight.getTx());
        SmartDashboard.putNumber("Velocity", masterShooter.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Velocity", slaveShooter.getSelectedSensorVelocity());
    }
}
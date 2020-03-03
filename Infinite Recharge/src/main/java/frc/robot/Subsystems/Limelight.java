package frc.robot.Subsystems;

import frc.robot.Hardware;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Lib.SubsystemFramework;



public class Limelight implements SubsystemFramework{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    NetworkTableEntry ta = table.getEntry("ta");

    private boolean LimelightHasValidTarget;
    private double LimelightDriveCommand;
    private double LimelightSteerCommand;
    private double camtran[];


    public Limelight(){
    }

    @Override
    public void update(){

        Update_Limelight_Tracking();

        //double steer = driverPad.getX();
        //double drive = driverPad.getY();
        boolean auto = Hardware.driverPad.getRawButton(8);

        //steer *= 1;
        //drive *= 1;
        
        if (auto)
            {
                if (LimelightHasValidTarget)
                {
                    //if (LimelightSteerCommand > 0.25) {
                        //myRobot.arcadeDrive(0, LimelightSteerCommand);
                    //}
                    //if (LimelightDriveCommand > 1) {
                    //    myRobot.arcadeDrive(1, 0);
                    //}
                }
            }
        outputToSmartDashboard();
    }



    @Override
    public void outputToSmartDashboard(){

    }

    @Override 
    public void setupSensors(){
    
    }


    public void Update_Limelight_Tracking() 
    {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = -0.05;                    // how hard to turn toward the target
        final double DRIVE_K = 0.1;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 0.0;        // Area of the target when the robot reaches the wall
	    //final double MAX_DRIVE = 2.5;                   // Simple speed limit so we don't drive too fast
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        this.camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
        //double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
	    //distance = CalculateTargetDistance();
	
        if (tv < 1.0)
        {
            LimelightHasValidTarget = false;
            LimelightDriveCommand = 0.0;
            LimelightSteerCommand = 0.0;
            return;
        }
        LimelightHasValidTarget = true;
        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA + ty) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        //if (drive_cmd > MAX_DRIVE)
        //{
        //    drive_cmd = MAX_DRIVE;
        //}
            LimelightDriveCommand = drive_cmd;
    
        if (ty <= 2 && ty >= -2){
            LimelightDriveCommand = 0;
        }
	    if (tx <= 2 && tx >= -2){
            LimelightSteerCommand = 0;
		}
	}
    public double getLimelightDriveCommand() {
        return this.LimelightDriveCommand;
    }
    public double getLimelightSteerCommand() {
        return this.LimelightSteerCommand;
    }
    public boolean getLimelightHasValidTarget() {
        return this.LimelightHasValidTarget;
    }
    public double getLimelightCamtran(int x) {
        return this.camtran[x];
    }
    public int getTy() {
        return (int)this.ty;
    }
}
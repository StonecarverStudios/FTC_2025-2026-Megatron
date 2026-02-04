package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class MegatronAutoFarRed extends LinearOpMode {
    
       //================HARDWARE-====================//

        //Drive Motors            //Shooter              //Feeder
        DcMotorEx m1, m2, m3, m4, leftThrow, rightThrow, topFeeder, bottomFeeder;
    
        //Shooter cup
        Servo trigger;
    
        //Sensors
        DistanceSensor topSensor, bottomSensor, cupSensor;
        IMU imu;
        
        //Vision
        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
          
      
      private void drive(double py, double px, double pa) {
     
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = px + py + pa; //fl
        double p2 = -px + py + pa; //bl
        double p3 = -px + py - pa; //fr
        double p4 = px + py - pa; //br
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }
    
    private void stopDrive(){
        drive(0,0,0);
        sleep(400);
    }
    
    private double getHeadingDeg() {
        // Returns the current robot heading in degrees
        // Assumes imu is already initialized
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    
    //AUTO SHOOT STATE MACHINE
    enum ShootState {
        IDLE,
        SPIN_UP,
        FEEDING,
        FIRING,
        WAIT_CLEAR,
        DONE,
        ABORT
    }
    
    ShootState shootState = ShootState.IDLE;
    
    int ballsToShoot = 3; // assuming 3 balls in close auto
    int ballsShot = 0;
    
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime cupDebounceTimer = new ElapsedTime();
    ElapsedTime runTime = new ElapsedTime();
    
    
    boolean cupLastDetect = true; // used for edge detection

    
    @Override
    public void runOpMode() {
        //====================HARDWARE MAP=========================//
        
        // Drive Motors
        m1 = hardwareMap.get(DcMotorEx.class, "bl"); //back left
        m2 = hardwareMap.get(DcMotorEx.class, "br"); //back right
        m3 = hardwareMap.get(DcMotorEx.class, "fr"); //front right
        m4 = hardwareMap.get(DcMotorEx.class, "fl"); //front left

        //Launcher motors
        leftThrow = hardwareMap.get(DcMotorEx.class, "leftThrow");
        rightThrow = hardwareMap.get(DcMotorEx.class, "rightThrow");

        //Feeder motor
        topFeeder = hardwareMap.get(DcMotorEx.class, "topFeeder");
        bottomFeeder = hardwareMap.get(DcMotorEx.class, "bottomFeeder");

        //Trigger Servo
        trigger = hardwareMap.get(Servo.class, "trigger");

        //Sensors
        topSensor = hardwareMap.get(DistanceSensor.class, "topSensor");
        bottomSensor = hardwareMap.get(DistanceSensor.class, "bottomSensor");
        cupSensor = hardwareMap.get(DistanceSensor.class, "cupSensor");

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters( 
            new RevHubOrientationOnRobot(
                //Adjust orientation if hub is mounted differently.
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        //====================HARDWARE MAP=========================//

        //Drive Motor Directions
        m2.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        //Shooter Direction
        leftThrow.setDirection(DcMotor.Direction.REVERSE);

        //Zero Power Behaviors
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomFeeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftThrow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightThrow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Motor Modes
        //Reset the encoders on Init, and set them to the correct mode 
        leftThrow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightThrow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftThrow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightThrow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //init triger
        trigger.setPosition(0.6);
        
        //init WebCam
        aprilTagWebcam.init(hardwareMap, telemetry);
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        boolean goAhead = false;
        
        double waitAutoTimer = 0;
        
        waitForStart();
        runTime.reset();
        
        
       //====================AUTO STARTS=========================//
        aprilTagWebcam.update();
        shootState = ShootState.SPIN_UP;
        shootTimer.reset();
        
        while (opModeIsActive() && shootState != shootState.DONE) {
        
            boolean cupSensorDetect = cupSensor.getDistance(DistanceUnit.CM) < 20;
        
                switch (shootState) {

                case IDLE:
                    // Do nothing
                    break;

                case SPIN_UP:
                    double targetVelocity = 1500.0;
                    leftThrow.setVelocity(targetVelocity);
                    rightThrow.setVelocity(targetVelocity);

    
                    if (shootTimer.seconds() > 1.5) {
                        shootState = ShootState.FEEDING;
                    }
                    break;
            
                case FEEDING:
                    
                    if (!cupSensorDetect) {
                        topFeeder.setPower(-1.0);
                        bottomFeeder.setPower(1.0);
                    } else {
                        topFeeder.setPower(0);
                        bottomFeeder.setPower(0);
                        shootState = ShootState.FIRING;
                        shootTimer.reset();
                    }
                    break;

            
                case FIRING:
                    trigger.setPosition(0.2); // fire
                
                    if (shootTimer.seconds() > 0.8) {
                        trigger.setPosition(0.6);
                        shootState = ShootState.WAIT_CLEAR;
                        shootTimer.reset();
                        cupLastDetect = true;
                        cupDebounceTimer.reset(); // start debounce timer
                        ballsShot++;
                    }
                    break;

            
                case WAIT_CLEAR:
    
                    topFeeder.setPower(0);
                    bottomFeeder.setPower(0);
                
                    if (shootTimer.seconds() > 0.2) {
                        if (cupLastDetect && !cupSensorDetect) {
                            if (ballsShot >= ballsToShoot) {
                                shootState = ShootState.DONE;
                            } else {
                                shootState = ShootState.FEEDING;
                            }
                            cupLastDetect = cupSensorDetect;
                        }
                        
                    }
                    break;



                case DONE:
                    leftThrow.setVelocity(0);
                    rightThrow.setVelocity(0);
                    topFeeder.setPower(0);
                    bottomFeeder.setPower(0);
            
                    shootState = ShootState.IDLE;
                    break;
                    
                case ABORT:
                    leftThrow.setVelocity(0);
                    rightThrow.setVelocity(0);
                    topFeeder.setPower(0);
                    bottomFeeder.setPower(0);
                    trigger.setPosition(0.6);
                    
                    ballsToShoot = 0;
                    shootState = ShootState.IDLE;
                    break;
            }
            
             //STATES
            telemetry.addLine("=== Shooter STATES ===");
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("Balls To Shoot", ballsToShoot);
            telemetry.addData("Balls Shot", ballsShot);
            telemetry.addData("cup sensor", cupSensorDetect);
            
            telemetry.addLine();
            telemetry.update();
            
            
            
        }
        
         //Drive off of the line RED
        drive(0,-1.5,0);
        sleep(280);
        drive(-1.5,0,0);
        sleep(420);
        stopDrive();
        
    }
}

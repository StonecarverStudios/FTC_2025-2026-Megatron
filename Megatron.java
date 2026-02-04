package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name="Megatron")
public class Megatron extends LinearOpMode {

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

    
    //=================STATES======================//
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
    
    int ballsToShoot = 0;
    int ballsShot = 0;
    
    ElapsedTime shootTimer = new ElapsedTime();
    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime cupDebounceTimer = new ElapsedTime();

    
    boolean aLastPressed = false;
    boolean cupLastDetect = true;


    //====================DRIVE CONTROL=========================//
    double lastTurnCmd = 0; // for slew limiting
    
    //====================IMU INIT VARIABLES=========================//
    double getHeadingDeg() {
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            return ypr.getYaw(AngleUnit.DEGREES);
        }

    @Override
    public void runOpMode(){

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

        //====================USER SETTINGS=========================//
        //Default shooting to far
        boolean farRange = false;
        
        //Power to the throwing motors
        double closeVelocity = 1400.0;
        double farVelocity = 1500.0;
        
        //====================OTHER SETTINGS / INIT VARIABLES=========================//
    
        //Sensor Variables
        boolean topSensorDetect = true;
        boolean bottomSensorDetect = true;
        boolean cupSensorDetect = true;
    
        //isLogged for fine power adjuster
        boolean yLastPressed = false;
        boolean xLastPressed = false;
        boolean dpadLeftLastPressed = false;
        boolean isLogged = false;

        boolean stealthMode = false;
    
        //IMU atuo turn
        double filteredHeadingRate = 0;
        boolean firingStarted = false;

    

        //====================READY TO START=========================//
        telemetry.addData("Press Start When Ready","");
        telemetry.addData("Far Range Mode Activated","");
        telemetry.update();
        
        waitForStart();
        runTime.reset();

        //====================MAIN LOOP=========================//
        while (opModeIsActive()) {

            //Update Vision
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20); //Blue Goal
            AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24); //Red Goal
            
            //Auto Aim Calculation 
            double BEARING_OFFSET = farRange ? 7.0 : 0.0;
            if (id24 != null) BEARING_OFFSET = 4.0;  // red target
            else if (id20 != null) BEARING_OFFSET = 2.0; // blue target

            //Auto Aim Logic
            double autoTurn = 0;  
            if (gamepad1.y) {
                AprilTagDetection tag = null;
            
                if (id24 != null) tag = id24;
                else if (id20 != null) tag = id20;
            
                if (tag != null) {
                    double bearing = tag.ftcPose.bearing + BEARING_OFFSET;
            
                    double currentHeading = getHeadingDeg();
                    double targetHeading = currentHeading + bearing;
            
                    double headingError = AngleUnit.normalizeDegrees(
                        targetHeading - currentHeading
                    );
            
                    double KP_TURN = 0.015;
                    double MAX_TURN = 0.4;
                    double DEAD_BAND = 1.0;
            
                    if (Math.abs(headingError) > DEAD_BAND) {
                        autoTurn = headingError * KP_TURN;
                        autoTurn = Math.max(-MAX_TURN, Math.min(MAX_TURN, autoTurn));
                    }
                }
            }
            
            // Manual override4
            //Manual override 
            double manualTurn = gamepad1.left_trigger - gamepad1.right_trigger; 
            if (Math.abs(manualTurn) > 0.05) autoTurn = 0;
            
            
            double turnCmd;

            // --- Manual turn ---
            if (Math.abs(manualTurn) > 0.05) {
                turnCmd = manualTurn;
            }
            
            // --- Auto aim ---
            else if (gamepad1.y && Math.abs(autoTurn) > 0) {
                turnCmd = autoTurn;
            }
            
            // --- Yaw damping ---
            else {
                double KD = 0.001;
                turnCmd = -filteredHeadingRate * KD;
            
                if (Math.abs(filteredHeadingRate) < 2.0) {
                    turnCmd = 0;
                }
            }




            //====================DRIVE CONTROL=========================//
            double px = gamepad1.left_stick_x; //The power of the left stick on the x axis / Left to Right, -1.00 to 1.00
            double py = -gamepad1.left_stick_y; //The power of the left stick on the y axis / Down to Up, -1.00 to 1.00
            double pa = turnCmd;
        
            //p1 correlates to m1, p2 correlates to m2, etc.
            
            //Mecanum wheel formulas
            double p1 = -px + py - pa; //bl
            double p2 = px + py + pa; //br
            double p3 = -px + py + pa; //fr
            double p4 = px + py - pa; //fl

            // Normalize motor powers
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            
            // Apply Power
            m1.setPower(p1);
            m2.setPower(p2); 
            m3.setPower(p3);
            m4.setPower(p4);
            
            //====================Stealth mode=========================//
            
            if(gamepad1.guide && isLogged == false){
                if(stealthMode == false) stealthMode = true;
                else if(stealthMode == true) stealthMode = false;
                isLogged = true;
                
            }else if (gamepad1.guide != true){
                isLogged = false;
            }
            
            if(stealthMode == true){
                m1.setPower(p1 * 0.1);
                m2.setPower(p2 * 0.1);
                m3.setPower(p3 * 0.1);
                m4.setPower(p4 * 0.1);
            }else {
                m1.setPower(p1 * 0.7);
                m2.setPower(p2 * 0.7);
                m3.setPower(p3 * 0.7);
                m4.setPower(p4 * 0.7);
            }

            //====================Sensor Updates=========================//
            topSensorDetect = topSensor.getDistance(DistanceUnit.CM) < 20;
            bottomSensorDetect = bottomSensor.getDistance(DistanceUnit.CM) < 20;
            cupSensorDetect = cupSensor.getDistance(DistanceUnit.CM) < 20;
            
            
            //====================Shooter Velocity Controls=========================// 
            
             //Range Toggle
            if (gamepad2.left_bumper){
                farRange = false; //close range mode
                
            }else if(gamepad2.right_bumper){
                farRange = true;  //far range mode
            }
            
            //Velocity fine tune adjuster
            boolean yPressed = gamepad2.y;
            boolean xPressed = gamepad2.x;
            
            if (yPressed && !yLastPressed) {
                if (farRange) {
                    farVelocity = Math.min(6000, farVelocity + 50);
                } else {
                    closeVelocity = Math.min(6000, closeVelocity + 50);
                }
            }
            
            if (xPressed && !xLastPressed) {
                if (farRange) {
                    farVelocity = Math.max(500, farVelocity - 50);
                } else {
                    closeVelocity = Math.max(500, closeVelocity - 50);
                }
            }
        
            yLastPressed = yPressed;
            xLastPressed = xPressed;


            //====================Shooter Control=========================//
            //ABORT LOGIC
            boolean dpadLeftPressed = gamepad2.dpad_left;

            if (dpadLeftPressed && !dpadLeftLastPressed) {
                shootState = ShootState.ABORT; // trigger abort immediately
            }
            
            dpadLeftLastPressed = dpadLeftPressed;

            int detectedBalls = 0;
            
            if (cupSensorDetect) detectedBalls++;
            if (topSensorDetect) detectedBalls++;
            if (bottomSensorDetect) detectedBalls++;

            
            boolean aPressed = gamepad2.a;

            if (aPressed && !aLastPressed && shootState == ShootState.IDLE) {
                ballsToShoot = detectedBalls;
                ballsShot = 0;
            
                if (ballsToShoot > 0) {
                    shootState = ShootState.SPIN_UP;
                    shootTimer.reset();
                }
               
            }

            aLastPressed = aPressed;

            switch (shootState) {

                case IDLE:
                    // Do nothing
                    break;
                
                //Spin up shooter state -> spin up shooter motors until correct velocity reached
                    double targetVelocity = farRange ? farVelocity : closeVelocity;
                    leftThrow.setVelocity(targetVelocity);
                    rightThrow.setVelocity(targetVelocity);

    
                    if (shootTimer.seconds() > 2.0) {
                        shootState = ShootState.FEEDING;
                    }
                    break;
                
                //Feeder state -> Keep feeding the balls forward until there is a ball in the cup
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

                //Firing state -> fire the ball in the cup
                case FIRING:
                    trigger.setPosition(0.2); // fire
                
                    if (shootTimer.seconds() > 0.8) {
                        trigger.setPosition(0.6);
                        shootState = ShootState.WAIT_CLEAR;
                        shootTimer.reset();
                        cupDebounceTimer.reset(); // start debounce timer
                        cupLastDetect = true;
                        ballsShot++;
                    }
                    break;

                //Wait clear state -> if there are still balls to shoot, go back to feeding state, if not go to Done state.
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

                //Done state -> turn off everything and set back to idle to await next loop
                case DONE:
                    leftThrow.setVelocity(0);
                    rightThrow.setVelocity(0);
                    topFeeder.setPower(0);
                    bottomFeeder.setPower(0);
            
                    shootState = ShootState.IDLE;
                    break;
                
                //Abort case ->Abort process no matter current state, similar to DONE and sets back to IDLE
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

        //=========================FEEDER===============================
           //Feeder Semi Automatic, one-joystick system when auto shooter is not activated
            if (shootState == ShootState.IDLE) {
    
                // Default motor power OFF
                double feederPower = gamepad2.right_stick_y;
                topFeeder.setPower(0);
                bottomFeeder.setPower(0);
            
            
                // Case 1: Cup empty → feed upward
                if (!cupSensorDetect) {
                    topFeeder.setPower(feederPower);
                    bottomFeeder.setPower(-feederPower);
                }
            
                // Case 2: Cup full, top empty → move bottom one up
                else if (cupSensorDetect && !topSensorDetect) {
                    topFeeder.setPower(feederPower);
                    bottomFeeder.setPower(-feederPower);
                }
            
                // Case 3: Cup and top full, bottom empty → fill bottom only
                else if (cupSensorDetect && topSensorDetect && !bottomSensorDetect) {
                    bottomFeeder.setPower(-feederPower);
                }
            
                // Case 4: Everything full → stop
                else {
                    topFeeder.setPower(0);
                    bottomFeeder.setPower(0);
                }
        }
    
            //=========================TELEMETRY===============================
            
            //ID 20 Telemetry from WebCam
            //aprilTagWebcam.displayDetectionTelemetry(id20);
            
            //Display Positions of the motors
            telemetry.addData("Motor Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
                    
            //Shooter
            telemetry.addLine("===Shooter MODE===");
            if(farRange == true){
                telemetry.addData("Far Range Mode Activated", "");
            }else{
                 telemetry.addData("Close Range Mode Activated", "");
            }
            //Display far and close power values
            telemetry.addData("Close Motor Velocity", "%.2f", closeVelocity);
            telemetry.addData("Far Motor Velocity", "%.2f", farVelocity);
            //STATES
            telemetry.addLine("=== Shooter STATES ===");
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("Balls To Shoot", ballsToShoot);
            telemetry.addData("Balls Shot", ballsShot);
            telemetry.addData("Cup Last Detect", cupLastDetect);
            telemetry.addLine();
            
            //Testing sensors for Feeder
            telemetry.addLine("=== Sensors ===");
            telemetry.addData("Cup", cupSensorDetect);
            telemetry.addData("Top", topSensorDetect);
            telemetry.addData("Bottom", bottomSensorDetect);
            telemetry.addData("Cup (cm)", "%.1f", cupSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Top (cm)", "%.1f", topSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Bottom (cm)", "%.1f", bottomSensor.getDistance(DistanceUnit.CM));
            telemetry.addLine();
            
            telemetry.addLine("==WebCam Vision===");
            telemetry.addData("Tag Found", id20 != null || id24 != null);
            if (id20 != null) telemetry.addData("Bearing", "%.2f", id20.ftcPose.bearing);
            if(id24 !=null) telemetry.addData("Bearing", "%.2f", id24.ftcPose.bearing);
            telemetry.addLine();
            telemetry.addData("autoTurn", "%.2f", autoTurn);
            telemetry.update();
        }
        //Stops all motors
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}

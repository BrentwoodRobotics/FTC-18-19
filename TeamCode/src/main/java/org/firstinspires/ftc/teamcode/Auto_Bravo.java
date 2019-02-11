/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Bravo", group="Autonomous")
@Disabled
public class Auto_Bravo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rearLift = null;
    private Servo dropperServo = null;
    private GoldAlignDetector detector;

    // Clears and resets encoder values
    public void clearDriveEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void dropClaimBeacon(){
        // Configurable values
        double DROPPER_CLOSED_POSITION = 0.9;
        double DROPPER_OPENED_POSITION = 0.2;

        dropperServo.setPosition(DROPPER_OPENED_POSITION);
        sleep(500);
        dropperServo.setPosition(DROPPER_CLOSED_POSITION);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Establish hardware map using configured device names
        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        rearLift = hardwareMap.get(DcMotor.class, "rearLift");
        dropperServo = hardwareMap.get(Servo.class, "dropperServo");

        // Set DC motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLift.setDirection(DcMotor.Direction.FORWARD);

        // Set DC motor zero-power behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Clear encoder data
        rearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 50; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -40; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double xPosition = detector.getXPosition();
        telemetry.addData("XPos", xPosition);
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            // Lowers the robot from the Lander
            rearLift.setTargetPosition(7500);
            rearLift.setPower(1);
            sleep(3500);
            rearLift.setPower(0);

            // Shimmy forward
            leftDrive.setTargetPosition(-200);
            rightDrive.setTargetPosition(800);
            leftDrive.setPower(-0.25);
            rightDrive.setPower(0.25);
            sleep(2000);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            clearDriveEncoders();

            // Retract the shaft
            rearLift.setTargetPosition(10);
            rearLift.setPower(-1);
            sleep(1000);

            // Aim towards target

            leftDrive.setTargetPosition(800);
            rightDrive.setTargetPosition(-200);
            leftDrive.setPower(0.25);
            rightDrive.setPower(-0.25);
            sleep(2000);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            clearDriveEncoders();

            if (xPosition > 500){
                // Push gold block
                leftDrive.setTargetPosition(900);
                rightDrive.setTargetPosition(100);
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                sleep(2000);
                clearDriveEncoders();
                leftDrive.setTargetPosition(1800);
                rightDrive.setTargetPosition(1800);
                sleep(2000);
                rightDrive.setTargetPosition(2200);
                sleep(2000);
                clearDriveEncoders();
                leftDrive.setTargetPosition(1700);
                rightDrive.setTargetPosition(1700);
                sleep(2000);
                rightDrive.setTargetPosition(2000);
                sleep(1000);
                dropClaimBeacon();
                leftDrive.setTargetPosition(1000);
                sleep(1000);
                leftDrive.setPower(-0.5);
                rightDrive.setPower(-0.5);
                sleep(2000);
                clearDriveEncoders();
                leftDrive.setTargetPosition(-6500);
                rightDrive.setTargetPosition(-6500);

            } else if (xPosition > 250) {
                // Drive to claim area
                leftDrive.setTargetPosition(4000);
                rightDrive.setTargetPosition(4000);
                rightDrive.setPower(0.5);
                leftDrive.setPower(0.5);
                sleep(4000);
                dropClaimBeacon();

                // Turn towards crater
                leftDrive.setTargetPosition(6600);
                sleep(3000);
                clearDriveEncoders();

                // Drive to crater
                leftDrive.setTargetPosition(7000);
                rightDrive.setTargetPosition(7000);

            } else {
                // Turn towards block
                leftDrive.setTargetPosition(100);
                rightDrive.setTargetPosition(700);
                rightDrive.setPower(0.5);
                leftDrive.setPower(0.5);
                sleep(2000);
                clearDriveEncoders();

                // Turn towards claim area
                leftDrive.setTargetPosition(1500);
                rightDrive.setTargetPosition(1500);
                sleep(2000);
                leftDrive.setTargetPosition(2500);
                sleep(3000);
                clearDriveEncoders();

                // Drive towards claim area
                leftDrive.setTargetPosition(1700);
                rightDrive.setTargetPosition(1700);
                sleep(2000);
                dropClaimBeacon();
                sleep(2000);

                // Turn towards crater
                leftDrive.setTargetPosition(3200);
                sleep(3000);
                clearDriveEncoders();
                leftDrive.setTargetPosition(1100);
                rightDrive.setTargetPosition(1100);
                sleep(1500);
                leftDrive.setTargetPosition(2000);
                sleep(2000);
                clearDriveEncoders();

                // Drive to crater
                rightDrive.setPower(0.75);
                leftDrive.setPower(0.75);
                leftDrive.setTargetPosition(8500);
                rightDrive.setTargetPosition(8500);
            }

            sleep(20000);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift Position", rearLift.getCurrentPosition());
            telemetry.update();
        }

    }
}

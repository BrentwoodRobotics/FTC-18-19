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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

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

@Autonomous(name="Drop Without Claiming", group="Autonomous")
// @Disabled
public class DropWithoutClaiming extends LinearOpMode {

    // Declare OpMode members.
    ExpansionHubEx expansionHub;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rearLift = null;
    private DcMotor armRotation = null;
    private Servo armExtension = null;
    private GoldAlignDetector detector;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public void clearDriveEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(200);
    }

    public double getCurrentHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = angles.firstAngle;
        return currentHeading;
    }

    public void driveWithoutEncoders() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivePower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void setDriveTarget(int target) {
        leftDrive.setTargetPosition(target);
        rightDrive.setTargetPosition(target);
    }

    public void navigateToHeading(double currentHeading, double targetHeading) {
        driveWithoutEncoders();
        // Turn right -- opModeIsActive() hopefully prevents crashes when stopping in while loops
        while (currentHeading > targetHeading + 1 && opModeIsActive()) {
            leftDrive.setPower(0.2);
            rightDrive.setPower(-0.2);
            currentHeading = getCurrentHeading();
        }
        // Turn left
        while (currentHeading < targetHeading - 1 && opModeIsActive()) {
            leftDrive.setPower(-0.2);
            rightDrive.setPower(0.2);
            currentHeading = getCurrentHeading();
        }
        setDrivePower(0);
        clearDriveEncoders();
    }

    public void dropTeamMarker() {
        // Rotate arm out
        armRotation.setPower(-0.5);
        armRotation.setTargetPosition(-900);
        sleep(1000);
        armRotation.setPower(0);

        // Rotate arm up
        armRotation.setPower(0.5);
        armRotation.setTargetPosition(-450);
        sleep(500);

        // Rotate arm in
        armRotation.setPower(0.25);
        armRotation.setTargetPosition(-10);
        sleep(500);
        armRotation.setPower(0);
    }

    @Override
    public void runOpMode() {
        RevExtensions2.init();
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Establish hardware map using configured device names
        leftDrive  = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        rearLift = hardwareMap.get(DcMotor.class, "rearLift");
        armRotation = hardwareMap.get(DcMotor.class, "armRotation");
        armExtension = hardwareMap.get(Servo.class, "armExtension");
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        // Set the Expansion Hub's LED to green
        expansionHub.setLedColor(0, 255, 0);

        // Enable phone charging
        expansionHub.setPhoneChargeEnabled(true);

        // Set DC motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLift.setDirection(DcMotor.Direction.FORWARD);
        armRotation.setDirection(DcMotor.Direction.REVERSE);

        // Set DC motor zero-power behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Clear encoder data
        rearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Get heading while on lander
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        telemetry.addData("Start Heading: ", startHeading);

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
        telemetry.addData("Block Position", detector.getXPosition());
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        waitForStart();
        double xPosition = detector.getXPosition();
        telemetry.addData("Block Position", xPosition);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        runtime.reset();

        // Runs when driver presses PLAY
        if (opModeIsActive()) {
            // Turns off DogeCV
            detector.disable();

            // Lowers the robot from the Lander
            rearLift.setTargetPosition(7500);
            rearLift.setPower(1);
            sleep(5500);

            // Get off lander hook
            leftDrive.setTargetPosition(-100);
            rightDrive.setTargetPosition(1000);
            leftDrive.setPower(1);
            rightDrive.setPower(1);
            sleep(1200);
            clearDriveEncoders();

            // Retract the shaft
            rearLift.setTargetPosition(10);
            rearLift.setPower(-1);

            if (xPosition > 500){   // RIGHT
                // Turn towards block
                navigateToHeading(getCurrentHeading(), -30);

                // Drive towards Sample Area and extend the Arm
                telemetry.addData("Arm", "Extending");
                telemetry.addData("Target", "Gold Block (left)");
                telemetry.addData("Heading", getCurrentHeading());
                telemetry.update();
                setDrivePower(0.5);
                setDriveTarget(2500);
                armExtension.setPosition(0);
                sleep(3000);
                armExtension.setPosition(0.5);
                setDrivePower(0);
                clearDriveEncoders();
            } else if (xPosition > 250) {   // CENTER
                // Position towards block
                navigateToHeading(getCurrentHeading(), 0);

                // Drive towards Sample Area and extend the Arm
                telemetry.addData("Arm", "Extending");
                telemetry.addData("Target", "Gold Block (center)");
                telemetry.addData("Heading", getCurrentHeading());
                telemetry.update();
                setDrivePower(0.5);
                setDriveTarget(4000);
                armExtension.setPosition(0);
                sleep(3000);
                telemetry.addData("Arm", "Extended");
                telemetry.addData("Target", "Depot");
                telemetry.addData("Heading", getCurrentHeading());
                telemetry.update();
                armExtension.setPosition(0.5);
                sleep(2000);
                setDrivePower(0);
                clearDriveEncoders();
            } else {        // LEFT (or undetermined)
                // Turn towards block
                navigateToHeading(getCurrentHeading(), 30);

                // Drive towards Sample Area and extend the Arm
                telemetry.addData("Arm", "Extending");
                telemetry.addData("Target", "Gold Block (left)");
                telemetry.addData("Heading", getCurrentHeading());
                telemetry.update();
                setDrivePower(0.5);
                setDriveTarget(2500);
                armExtension.setPosition(0);
                sleep(3000);
                armExtension.setPosition(0.5);
                setDrivePower(0);
                clearDriveEncoders();
            }

            // Sleep for safety
            sleep(20000);
        }
    }
}

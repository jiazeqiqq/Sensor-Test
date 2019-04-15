package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "EncoderTest", group = "9191")
public class EncoderTest extends LinearOpMode {
    private DcMotor wheelLeft;
    private DcMotor wheelRight;
    private DcMotor sweepLift;
    private DcMotor sweepLinear;
    private DcMotor sweepEx;
    private CRServo sweepLeft;
    private CRServo sweepRight;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    private ModernRoboticsI2cColorSensor colorSensor;

    public void runDistance(double power, int ticks) {
        wheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder
        wheelLeft.setTargetPosition(ticks); //tell the motor how many ticks to go
        wheelLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //set the mode of motor
        wheelLeft.setPower(power); // tell the motor how fast to go
        wheelRight.setTargetPosition(ticks);
        wheelRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRight.setPower(power);
        while (wheelRight.isBusy() && wheelLeft.isBusy()) {
            //wait until the motor get into position
        }
        wheelLeft.setPower(0);
        wheelRight.setPower(0);//reset the power

    }

    public void turnLeftDegrees (int degrees) {
        int ticks = (degrees/90) * 560;
        wheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder
        wheelLeft.setTargetPosition(-ticks); //tell the motor how many ticks to go
        wheelLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //set the mode of motor
        wheelLeft.setPower(0.2); // tell the motor how fast to go
        wheelRight.setTargetPosition(ticks);
        wheelRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRight.setPower(0.2);
        while (wheelRight.isBusy() && wheelLeft.isBusy()) {
            //wait until the motor get into position
        }
        wheelLeft.setPower(0);
        wheelRight.setPower(0);//reset the power
    }

    public void turnRightDegrees (int degrees) {

        int ticks = (degrees/90) * 560;

        wheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoder
        wheelLeft.setTargetPosition(ticks); //tell the motor how many ticks to go
        wheelLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //set the mode of motor
        wheelLeft.setPower(0.2); // tell the motor how fast to go
        wheelRight.setTargetPosition(-ticks);
        wheelRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelRight.setPower(0.2);
        while (wheelRight.isBusy() && wheelLeft.isBusy()) {
            //wait until the motor get into position
        }
        wheelLeft.setPower(0);
        wheelRight.setPower(0);//reset the power
    }





    @Override
    public void runOpMode() throws InterruptedException {
        wheelRight = hardwareMap.dcMotor.get("wheelRight");
        wheelLeft = hardwareMap.dcMotor.get("wheelLeft");
        sweepLift = hardwareMap.dcMotor.get("sweepLift");
        sweepLinear = hardwareMap.dcMotor.get("sweepLinear");
        sweepLeft = hardwareMap.crservo.get("sweepLeft");
        sweepRight = hardwareMap.crservo.get("sweepRight");
        sweepEx = hardwareMap.dcMotor.get("sweepEx");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "sensor_color");
        wheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        //go forward until find an object 1.5cm away
        wheelLeft.setPower(0.2);
        wheelRight.setPower(0.2);
        while (rangeSensor.getDistance(DistanceUnit.CM) > 1.5) {

        }
        wheelRight.setPower(0);
        wheelLeft.setPower(0);
        sleep(500);


        colorSensor.enableLed(true);// open color sensor LED light
        if (colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) > 6
                && colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) < 10) {
                runDistance(0.3,560);// if color sensor detects yellow color, push it
        }else{
            //do something
            runDistance(0.3,-560);
            sleep(100);
            turnLeftDegrees(90);


        }


    }
}

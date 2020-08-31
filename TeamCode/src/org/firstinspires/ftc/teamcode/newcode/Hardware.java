package org.firstinspires.ftc.teamcode.newcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technototes.library.hardware.motor.EncodedMotor;
import com.technototes.library.hardware.motor.Motor;
import com.technototes.library.hardware.sensor.IMU;
import com.technototes.library.hardware.sensor.RangeSensor;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.structure.HardwareBase;


public class Hardware extends HardwareBase {

    public HardwareMap hardwareMap;


    //upper assembly
    public Motor<CRServo> slide;
    public Servo turn;
    public Servo claw;

    //capstone pusher
    public Motor<CRServo> cap;

    //rear flipper
    public Servo blockFlipper;

    //color sensor
    public ColorSensor sensorColorBottom;

    //range sensors
    public RangeSensor sensorRangeFront;
    public RangeSensor sensorRangeRear;
    public RangeSensor sensorRangeLeft;
    public RangeSensor sensorRangeRight;

    //lift
    public EncodedMotor<DcMotor> lLiftMotor;
    public EncodedMotor<DcMotor> rLiftMotor;

    //drivebase
    public Motor<DcMotor> flMotor;
    public Motor<DcMotor> frMotor;
    public Motor<DcMotor> rlMotor;
    public Motor<DcMotor> rrMotor;

    public IMU imu;

    public Hardware(HardwareMap map) {
        hardwareMap = map;

        slide = new Motor<CRServo>("slide");
        turn = new Servo("grabTurn");
        claw = new Servo("claw").setRange(0, 0.7);

        cap = new Motor<CRServo>("cap");

        blockFlipper = new Servo("blockFlipper");

        sensorColorBottom = hardwareMap.get(ColorSensor.class, "sensorColorBottom");

        sensorRangeFront = new RangeSensor("sensorRangeFront");
        sensorRangeRear = new RangeSensor("sensorRangeRear");
        sensorRangeLeft = new RangeSensor("sensorRangeLeft");
        sensorRangeRight = new RangeSensor("sensorRangeRight");

        lLiftMotor = new EncodedMotor<DcMotor>("motorLiftLeft").setInverted(false);
        rLiftMotor = new EncodedMotor<DcMotor>("motorLiftRight").setInverted(true);

        flMotor = new Motor<DcMotor>("motorFrontLeft");
        frMotor = new Motor<DcMotor>("motorFrontRight");
        rlMotor = new Motor<DcMotor>("motorRearLeft");
        rrMotor = new Motor<DcMotor>("motorRearRight");

        imu = new IMU("imu1");


    }


}

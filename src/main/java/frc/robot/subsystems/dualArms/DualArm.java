package frc.robot.subsystems.dualArms;

import static frc.robot.subsystems.dualArms.DualArmConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;

public class DualArm extends SubsystemBase<DualArm.Command> {
    public enum Command {
        DISABLED,
        IDLE,
        RAISE,
        LOWER,
        INAKE,
        EXPEL
    }

    public enum Travel {
        PHASE_1,
        PHASE_2
    }

    private final DualArm2d dualArm2d = new DualArm2d("DualArm2d", new Color8Bit(255, 0, 0));

    private final Motor leftMotor;
    private final Motor rightMotor;

    private final Motor rollerLeft;
    private final Motor rollerRight;

    private double leftAngle_deg;
    private double rightAngle_deg;

    private double leftTarget_deg;
    private double rightTarget_deg;

    private static DualArm instance;

    private DualArm() {
        super("DualArm");

        MotorConfig leftConfig = new MotorConfig(LEFT_MOTOR_ID)
            .withCanbus(CANBUS_LEFT)
            .withInverted(INVERTED_LEFT)
            .withBrake(BRAKE_LEFT)
            .withFoc(FOC_LEFT)
            .withMeasurementUnit(UNIT_LEFT)
            .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_LEFT)
            .withPIDGains(kPLeft, kILeft, kDLeft, GRAVITY_LEFT)
            .withMotionMagic(CRUISE_VELOCITY_LEFT_degps, ACCELERATION_LEFT_degps2, JERK_LEFT_degp3)
            .withSim(DC_MOTOR_LEFT, GEARING_LEFT, SIM_MOI_LEFT_kgm2);

        this.leftMotor = new Motor("leftMotor", leftConfig);

        MotorConfig rightConfig = new MotorConfig(RIGHT_MOTOR_ID)
            .withCanbus(CANBUS_RIGHT)
            .withInverted(INVERTED_RIGHT)
            .withBrake(BRAKE_RIGHT)
            .withFoc(FOC_RIGHT)
            .withMeasurementUnit(UNIT_RIGHT)
            .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_RIGHT)
            .withPIDGains(kPRight, kIRight, kDRight, GRAVITY_RIGHT)
            .withMotionMagic(CRUISE_VELOCITY_RIGHT_degps, ACCELERATION_RIGHT_degps2, JERK_RIGHT_degp3)
            .withSim(DC_MOTOR_RIGHT, GEARING_RIGHT, SIM_MOI_RIGHT_kgm2);

        this.rightMotor = new Motor("rightMotor", rightConfig);

        MotorConfig leftRollerConfig = new MotorConfig(LEFT_ROLLER_MOTOR_ID)
            .withCanbus(CANBUS_LEFT)
            .withInverted(INVERTED_ROLLER_LEFT)
            .withBrake(BRAKE_ROLLER_LEFT)
            .withFoc(FOC_ROLLER_LEFT)
            .withMeasurementUnit(UNIT_ROLLER_LEFT)
            .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_ROLLER_LEFT)
            .withSim(DC_MOTOR_ROLLER_LEFT, GEARING_ROLLER_LEFT, SIM_MOI_ROLLER_LEFT_kgm2);

        this.rollerLeft = new Motor("rollerLeft", leftRollerConfig);

        MotorConfig rightRollerConfig = new MotorConfig(RIGHT_ROLLER_MOTOR_ID)
            .withCanbus(CANBUS_RIGHT)
            .withInverted(INVERTED_ROLLER_RIGHT)
            .withBrake(BRAKE_ROLLER_RIGHT)
            .withFoc(FOC_ROLLER_RIGHT)
            .withMeasurementUnit(UNIT_ROLLER_RIGHT)
            .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_ROLLER_RIGHT)
            .withSim(DC_MOTOR_ROLLER_RIGHT, GEARING_ROLLER_RIGHT, SIM_MOI_ROLLER_RIGHT_kgm2);

        this.rollerRight = new Motor("rollerRight", rightRollerConfig);

        setCommand(Command.IDLE);
    }

    public static DualArm getInstance() {
        if (instance == null) {
            instance = new DualArm();
        }

        return instance;
    }

    @Override
    protected void inputPeriodic() {
        leftMotor.readInputs();
        rightMotor.readInputs();
        rollerLeft.readInputs();
        rollerRight.readInputs();
    }

    @Override
    protected void handle() {
        switch (getCommand()) {
            case DISABLED:
			case IDLE:
                leftMotor.stop();
                rightMotor.stop();
                rollerLeft.stop();
                rollerRight.stop();
                break;
            case RAISE:
                if (firstLoop()) {
                    leftTarget_deg = RAISE_PHASE_2_THRESHOLD_deg;
                    rightTarget_deg = rightMotor.getPosition();
                    setSubstate(Travel.PHASE_1);
                }

                switch ((Travel) getSubstate()) {
                    case PHASE_1:
                        leftMotor.setMotionMagic(leftTarget_deg);
                        rightMotor.setMotionMagic(rightTarget_deg);
                        if (atTarget(leftMotor, leftTarget_deg, RAISE_TOLERANCE_deg)) {
                            setSubstate(Travel.PHASE_2);
                        }
                        break;
                    case PHASE_2:
                        rightTarget_deg = RAISE_FINAL_THRESHOLD_deg;
                        leftMotor.setMotionMagic(leftTarget_deg);
                        rightMotor.setMotionMagic(rightTarget_deg);
                        if (atTarget(leftMotor, leftTarget_deg, RAISE_TOLERANCE_deg)
                                && atTarget(rightMotor, rightTarget_deg, RAISE_TOLERANCE_deg)) {
                            setCommand(Command.IDLE);
                        }
                        break;
                    default:
                        throw new IllegalArgumentException("DualArm: Unknown substate: " + getSubstate());
                }
                break;
            case LOWER:
                if (firstLoop()) {
                    rightTarget_deg = LOWER_PHASE_2_THRESHOLD_deg;
                    leftTarget_deg = leftMotor.getPosition();
                    setSubstate(Travel.PHASE_1);
                }

                switch ((Travel) getSubstate()) {
                    case PHASE_1:
                        rightMotor.setMotionMagic(rightTarget_deg);
                        leftMotor.setMotionMagic(leftTarget_deg);
                        if (atTarget(rightMotor, rightTarget_deg, LOWER_TOLERANCE_deg)) {
                            setSubstate(Travel.PHASE_2);
                        }
                        break;
                    case PHASE_2:
                        leftTarget_deg = LOWER_FINAL_THRESHOLD_deg;
                        rightMotor.setMotionMagic(rightTarget_deg);
                        leftMotor.setMotionMagic(leftTarget_deg);
                        if (atTarget(rightMotor, rightTarget_deg, LOWER_TOLERANCE_deg)
                                && atTarget(leftMotor, leftTarget_deg, LOWER_TOLERANCE_deg)) {
                            setCommand(Command.IDLE);
                        }
                        break;
                    default:
                        throw new IllegalArgumentException("DualArm: Unknown substate: " + getSubstate());
                }
                break;
            case INAKE:
                rollerLeft.setVoltage(INTAKE_VOLTS);
                rollerRight.setVoltage(INTAKE_VOLTS);
                break;
            case EXPEL:
                rollerLeft.setVoltage(EXPEL_VOLTS);
                rollerRight.setVoltage(EXPEL_VOLTS);
                break;
            default:
                throw new IllegalArgumentException("DualArm: handling invalid state: " + getCommand());
        }
    }

    private boolean atTarget(Motor motor, double target_deg, double tolerance_deg) {
        return Math.abs(motor.getPosition() - target_deg) <= tolerance_deg;
    }

    @Override
    protected void outputPeriodic() {
        leftAngle_deg = leftMotor.getPosition();
        rightAngle_deg = rightMotor.getPosition();

        dualArm2d.setLeftArmAngle(leftAngle_deg);
        dualArm2d.setRightArmAngle(rightAngle_deg);
        dualArm2d.setLeftRollerAngle(rollerLeft.getPosition());
        dualArm2d.setRightRollerAngle(rollerRight.getPosition());
        dualArm2d.periodic();

        Logger.recordOutput("DualArm/LeftAngle_deg", leftAngle_deg);
        Logger.recordOutput("DualArm/RightAngle_deg", rightAngle_deg);
    }

    public void disable() {
        setCommand(Command.DISABLED);
    }

    public void idle() {
        setCommand(Command.IDLE);
    }

    public void raise() {
        setCommand(Command.RAISE);
    }

    public void lower() {
        setCommand(Command.LOWER);
    }

    public void intake() {
        setCommand(Command.INAKE);
    }

    public void expel() {
        setCommand(Command.EXPEL);
    }
}

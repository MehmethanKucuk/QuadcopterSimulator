using UnityEngine;

// PID-controlled drone controller supporting Manual / AutoHover modes and crash state
public class DroneController : MonoBehaviour
{
    public enum ControlMode { Manual, AutoHoverPID }

    [Header("General")]
    public ControlMode controlMode = ControlMode.AutoHoverPID;

    [Header("Physics (Matches with DronePhysics)")]
    public float mass = 1.0f;
    public float gravity = 9.81f;

    [Header("Engine Power")]
    [Tooltip("Base engine power (Newtons).")]
    public float baseMaxThrustNewton = 30f;

    [Tooltip("Engine power multiplier. Higher value = stronger engine, faster drone, faster propellers.")]
    public float maxThrustFactor = 3f;

    [Header("Initial Hover")]
    public float hoverHeight = 2.0f;

    [Header("Target Angle Limits (degrees)")]
    public float maxPitchAngle = 20f;    // W/S
    public float maxRollAngle  = 20f;    // A/D

    [Header("Attitude Control Gains (PID)")]
    public float pitchKp = 4f;
    public float pitchKi = 0.0f;
    public float pitchKd = 1.5f;

    public float rollKp  = 4f;
    public float rollKi  = 0.0f;
    public float rollKd  = 1.5f;

    public float yawKp   = 2f;
    public float yawKi   = 0.0f;
    public float yawKd   = 0.8f;

    [Header("Altitude Control Gains (PID)")]
    public float altKp = 5f;
    public float altKi = 0.0f;
    public float altKd = 3f;
    public float altitudeStep = 2f;

    [Header("Automatic PID Hover Settings (stabilizer)")]
    public Vector3 autoTargetPosition = new Vector3(0, 2, 0);
    public float positionKp = 1.0f;
    public float integralLimit = 10f;

    [Header("State")]
    public bool isCrashed = false;
    [HideInInspector] public float lastThrust = 0f;          // Used by propeller visuals
    [HideInInspector] public bool propVisualEnabled = false; // Props stop on play, activate on motion

    float desiredPitch;
    float desiredRoll;
    float desiredYaw;
    float desiredAltitude;

    float pitchInt;
    float rollInt;
    float yawInt;
    float altInt;

    float baseYaw;
    DronePhysics dronePhysics;

    void Start()
    {
        dronePhysics = GetComponent<DronePhysics>();
        var s = dronePhysics.state;

        desiredPitch    = s.eulerAngles.x;
        desiredRoll     = s.eulerAngles.z;
        desiredYaw      = s.eulerAngles.y;
        desiredAltitude = s.position.y;

        autoTargetPosition = s.position + Vector3.up * hoverHeight;
        desiredAltitude = autoTargetPosition.y;

        baseYaw = s.eulerAngles.y;

        // Stop propellers at start
        propVisualEnabled = false;
        lastThrust = 0f;
    }

    void Update()
    {
        if (!isCrashed && Input.GetKeyDown(KeyCode.Tab))
            ToggleMode();
    }

    public ControlInput GetControlInput()
    {
        mass = dronePhysics.mass;
        gravity = dronePhysics.gravity;

        var s = dronePhysics.state;
        float dt = Time.deltaTime;

        ControlInput u = new ControlInput();

        if (isCrashed)
        {
            lastThrust = 0f;
            u.totalThrust = 0f;
            u.moments = Vector3.zero;
            propVisualEnabled = false;
            return u;
        }

        // Propellers: stopped on play, enabled on input
        bool hasManualInput =
            Mathf.Abs(Input.GetAxis("Vertical")) > 0.01f ||
            Mathf.Abs(Input.GetAxis("Horizontal")) > 0.01f ||
            Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.E) ||
            Input.GetKey(KeyCode.Space) || Input.GetKey(KeyCode.LeftControl);

        bool hasAutoMotion = false;
        if (controlMode == ControlMode.AutoHoverPID)
        {
            Vector3 posErr = autoTargetPosition - s.position;
            hasAutoMotion = posErr.magnitude > 0.2f;
        }

        if (!propVisualEnabled && (hasManualInput || hasAutoMotion))
            propVisualEnabled = true;

        // target commands
        if (controlMode == ControlMode.Manual)
        {
            float pitchCmd = Input.GetAxis("Vertical");
            desiredPitch = maxPitchAngle * pitchCmd;

            float rollCmd = Input.GetAxis("Horizontal");
            desiredRoll = -maxRollAngle * rollCmd;

            float yawStep = 60f;
            if (Input.GetKey(KeyCode.Q)) desiredYaw -= yawStep * dt;
            if (Input.GetKey(KeyCode.E)) desiredYaw += yawStep * dt;

            if (Input.GetKey(KeyCode.Space))       desiredAltitude += altitudeStep * dt;
            if (Input.GetKey(KeyCode.LeftControl)) desiredAltitude -= altitudeStep * dt;
        }
        else
        {
            Vector3 posError = autoTargetPosition - s.position;

            desiredRoll  = Mathf.Clamp(positionKp * posError.x, -maxRollAngle,  maxRollAngle);
            desiredPitch = Mathf.Clamp(-positionKp * posError.z, -maxPitchAngle, maxPitchAngle);
            desiredAltitude = autoTargetPosition.y;
            desiredYaw = baseYaw;
        }

        //  Attitude PID -> moments
        float currentPitch = NormalizeAngle(s.eulerAngles.x);
        float currentRoll  = NormalizeAngle(s.eulerAngles.z);
        float currentYaw   = NormalizeAngle(s.eulerAngles.y);
        float targetYaw    = NormalizeAngle(desiredYaw);

        float pitchError   = desiredPitch - currentPitch;
        float pitchRateErr = 0f - s.angularVelocity.x;
        pitchInt += pitchError * dt;
        pitchInt = Mathf.Clamp(pitchInt, -integralLimit, integralLimit);
        float pitchMoment  = pitchKp * pitchError + pitchKi * pitchInt + pitchKd * pitchRateErr;

        float rollError    = desiredRoll - currentRoll;
        float rollRateErr  = 0f - s.angularVelocity.z;
        rollInt += rollError * dt;
        rollInt = Mathf.Clamp(rollInt, -integralLimit, integralLimit);
        float rollMoment   = rollKp * rollError + rollKi * rollInt + rollKd * rollRateErr;

        float yawError     = NormalizeAngle(targetYaw - currentYaw);
        float yawRateErr   = 0f - s.angularVelocity.y;
        yawInt += yawError * dt;
        yawInt = Mathf.Clamp(yawInt, -integralLimit, integralLimit);
        float yawMoment    = yawKp * yawError + yawKi * yawInt + yawKd * yawRateErr;

        //  Altitude PID -> thrust
        float altError  = desiredAltitude - s.position.y;
        float altVelErr = 0f - s.velocity.y;
        altInt += altError * dt;
        altInt = Mathf.Clamp(altInt, -integralLimit, integralLimit);

        float thrust = mass * gravity
                     + altKp * altError
                     + altKi * altInt
                     + altKd * altVelErr;

        float maxThrust = Mathf.Max(0.01f, baseMaxThrustNewton * Mathf.Max(0.01f, maxThrustFactor));
        thrust = Mathf.Clamp(thrust, 0f, maxThrust);

        lastThrust = thrust;

        u.totalThrust = thrust;
        u.moments = new Vector3(pitchMoment, yawMoment, rollMoment);

        return u;
    }

    public void ResetAfterCrash(DroneState s)
    {
        isCrashed = false;
        propVisualEnabled = false;
        controlMode = ControlMode.Manual;

        desiredPitch    = s.eulerAngles.x;
        desiredRoll     = s.eulerAngles.z;
        desiredYaw      = s.eulerAngles.y;
        desiredAltitude = s.position.y;

        autoTargetPosition = s.position + Vector3.up * hoverHeight;
        baseYaw = s.eulerAngles.y;

        pitchInt = rollInt = yawInt = altInt = 0f;
        lastThrust = 0f;
    }

    void ToggleMode()
    {
        controlMode = (controlMode == ControlMode.Manual)
            ? ControlMode.AutoHoverPID
            : ControlMode.Manual;

        pitchInt = rollInt = yawInt = altInt = 0f;
    }

    float NormalizeAngle(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        if (angle < -180f) angle += 360f;
        return angle;
    }
}

using UnityEngine;

// This script must be attached to the Drone object
public class DronePhysics : MonoBehaviour
{
    [Header("Physical Parameters")]
    public float mass = 1.0f;       // m=p/v 
    public float gravity = 9.81f;   // m/s^2

    [Header("Damping")]
    public float linearDamping = 1.5f;   // linear velocity damping
    public float angularDamping = 4f;    // angular velocity damping

    [Header("Angle Limits")]
    public float maxTiltAngle = 40f;     // maximum pitch/roll angle (deg)

    [Header("Initial State")]
    public bool useTransformAsInitial = true; // true: use scene transform as initial state
    public Vector3 initialPosition = new Vector3(0, 1, 0);
    public Vector3 initialEulerAngles = Vector3.zero;

    [HideInInspector]
    public DroneState state;

    void Awake()
    {
        state = new DroneState();

        if (useTransformAsInitial)
        {
            state.position = transform.position;
            state.eulerAngles = transform.eulerAngles;
        }
        else
        {
            state.position = initialPosition;
            state.eulerAngles = initialEulerAngles;
        }

        state.velocity = Vector3.zero;
        state.angularVelocity = Vector3.zero;

        transform.position = state.position;
        transform.rotation = Quaternion.Euler(state.eulerAngles);
    }

    // Computes state derivatives
    public DroneState ComputeDerivatives(DroneState s, ControlInput u)
    {
        DroneState ds = new DroneState();

        // Position derivative = velocity
        ds.position = s.velocity;

        // Linear acceleration
        // Thrust is applied along body Y-axis
        Quaternion rot = Quaternion.Euler(s.eulerAngles);
        Vector3 thrustBody = new Vector3(0, u.totalThrust, 0);
        Vector3 thrustWorld = rot * thrustBody;

        Vector3 gravityAcc = new Vector3(0, -gravity, 0);

        // Simple aerodynamic drag (force-like)
        Vector3 dampingForce = -linearDamping * s.velocity;

        Vector3 accel = (thrustWorld + dampingForce) / mass + gravityAcc;
        ds.velocity = accel;

        // Euler angle derivative = angular velocity
        ds.eulerAngles = s.angularVelocity;

        // Angular acceleration
        Vector3 w = s.angularVelocity;
        Vector3 m = u.moments;

        Vector3 wDot = new Vector3(
            (m.x - angularDamping * w.x),
            (m.y - angularDamping * w.y),
            (m.z - angularDamping * w.z)
        );

        ds.angularVelocity = wDot;

        return ds;
    }

    // Euler integration step
    public DroneState EulerStep(DroneState s, ControlInput u, float dt)
    {
        DroneState ds = ComputeDerivatives(s, u);

        DroneState newState = new DroneState();
        newState.position        = s.position        + ds.position * dt;
        newState.velocity        = s.velocity        + ds.velocity * dt;
        newState.eulerAngles     = s.eulerAngles     + ds.eulerAngles * dt;
        newState.angularVelocity = s.angularVelocity + ds.angularVelocity * dt;

        ClampAngles(ref newState);
        return newState;
    }

    // Runge-Kutta 4th order integration step
    public DroneState RungeKutta4Step(DroneState s, ControlInput u, float dt)
    {
        DroneState k1 = ComputeDerivatives(s, u);
        DroneState s2 = AddStates(s, k1, dt * 0.5f);

        DroneState k2 = ComputeDerivatives(s2, u);
        DroneState s3 = AddStates(s, k2, dt * 0.5f);

        DroneState k3 = ComputeDerivatives(s3, u);
        DroneState s4 = AddStates(s, k3, dt);

        DroneState k4 = ComputeDerivatives(s4, u);

        DroneState result = new DroneState();
        result.position = s.position + (dt / 6.0f) * (k1.position + 2f * k2.position + 2f * k3.position + k4.position);
        result.velocity = s.velocity + (dt / 6.0f) * (k1.velocity + 2f * k2.velocity + 2f * k3.velocity + k4.velocity);
        result.eulerAngles = s.eulerAngles + (dt / 6.0f) * (k1.eulerAngles + 2f * k2.eulerAngles + 2f * k3.eulerAngles + k4.eulerAngles);
        result.angularVelocity = s.angularVelocity + (dt / 6.0f) * (k1.angularVelocity + 2f * k2.angularVelocity + 2f * k3.angularVelocity + k4.angularVelocity);

        ClampAngles(ref result);
        return result;
    }

    private DroneState AddStates(DroneState s, DroneState k, float scale)
    {
        DroneState r = new DroneState();
        r.position        = s.position        + k.position * scale;
        r.velocity        = s.velocity        + k.velocity * scale;
        r.eulerAngles     = s.eulerAngles     + k.eulerAngles * scale;
        r.angularVelocity = s.angularVelocity + k.angularVelocity * scale;
        return r;
    }

    // Keeps pitch and roll within limits (prevents flipping)
    private void ClampAngles(ref DroneState s)
    {
        Vector3 e = s.eulerAngles;

        float pitch = Mathf.Clamp(NormalizeAngle(e.x), -maxTiltAngle, maxTiltAngle);
        float roll  = Mathf.Clamp(NormalizeAngle(e.z), -maxTiltAngle, maxTiltAngle);

        // Yaw is free
        float yaw = e.y;

        s.eulerAngles = new Vector3(pitch, yaw, roll);

        // Limit extreme angular velocities
        s.angularVelocity = Vector3.ClampMagnitude(s.angularVelocity, 200f);
    }

    private float NormalizeAngle(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        if (angle < -180f) angle += 360f;
        return angle;
    }
}

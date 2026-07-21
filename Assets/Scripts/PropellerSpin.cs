using UnityEngine;


// Manual control for drone propellers (such as rotation speed and direction).
public class PropellerSpin : MonoBehaviour
{
    public enum SpinAxis { X, Y, Z }

    [Header("Spin")]
    public SpinAxis axis = SpinAxis.Z;   
    public float maxSpinSpeed = 3500f;  
    public float spinAcceleration = 10f;
    public bool invertDirection = false;

    [Header("Safety Clamp")]
    public float maxRPMClamp = 12000f;   

    float currentSpeed = 0f;
    float targetSpeed = 0f;

    DroneController controller;
    DronePhysics physicsRef;
    bool crashed = false;

    void Start()
    {
        controller = GetComponentInParent<DroneController>();
        physicsRef = GetComponentInParent<DronePhysics>();
    }

    void Update()
    {
        if (controller == null || physicsRef == null) return;

        if (!controller.isCrashed && !crashed && controller.propVisualEnabled)
        {
            float mg = Mathf.Max(0.01f, physicsRef.mass * physicsRef.gravity);

            // hover: lastThrust / (m*g)  -> hover ~1
            float load = Mathf.Clamp(controller.lastThrust / mg, 0f, 3f);

            
            float motorMul = Mathf.Max(0.01f, controller.maxThrustFactor);

            targetSpeed = maxSpinSpeed * load * motorMul;
            targetSpeed = Mathf.Min(targetSpeed, maxRPMClamp);
        }
        else
        {
            targetSpeed = 0f;
        }

        currentSpeed = Mathf.Lerp(currentSpeed, targetSpeed, Time.deltaTime * spinAcceleration);

        float dir = invertDirection ? -1f : 1f;

        Vector3 spinVector = axis switch
        {
            SpinAxis.X => Vector3.right,
            SpinAxis.Y => Vector3.up,
            _ => Vector3.forward
        };

        transform.Rotate(spinVector, dir * currentSpeed * Time.deltaTime, Space.Self);
    }

    public void SetCrashed(bool c) => crashed = c;
}

using System.Collections;
using UnityEngine;

public class DroneCrashHandler : MonoBehaviour
{
    public DronePhysics dronePhysics;
    public DroneController droneController;
    public SimulationManager simManager;   // Simulation Manager on the scene (either connect or find automatically)

    [Header("Crash Detection (Cast)")]
    public LayerMask collisionMask = ~0;
    public float crashSpeedThreshold = 1.0f;
    public float castRadius = 0.8f;
    public float castSkin = 0.05f;

    [Header("Natural Drop")]
    public float motorCutTime = 0.30f;
    public float fallDistance = 0.15f;        
    public float minGroundClearance = 0.05f;  
    public float groundRayHeight = 1.0f;
    public float groundRayLength = 10.0f;


    //  0 = instantly , prevents another crash shortly after a reset
    [Header("Reset")]
    public float resetDelay = 0.0f;          
    public float graceAfterReset = 0.25f;    

    Vector3 spawnPosition;
    Vector3 spawnEuler;

    bool crashed = false;
    float graceTimer = 0f;

    void Start()
    {
        if (dronePhysics == null) dronePhysics = GetComponent<DronePhysics>();
        if (droneController == null) droneController = GetComponent<DroneController>();
        if (simManager == null) simManager = FindFirstObjectByType<SimulationManager>();

        // Spawn = starting point on the scene (the drone's starting point in the city)
        spawnPosition = transform.position;
        spawnEuler = transform.eulerAngles;

        graceTimer = graceAfterReset;
    }

    void Update()
    {
        if (graceTimer > 0f) graceTimer -= Time.deltaTime;
    }

    void FixedUpdate()
    {
        if (crashed) return;
        if (graceTimer > 0f) return;

        // In the controller crash lock, restart.
        if (droneController != null && droneController.isCrashed) return;

        Vector3 v = dronePhysics.state.velocity;
        float speed = v.magnitude;
        if (speed < 0.01f) return;

        Vector3 dir = v / speed;
        float dist = speed * Time.fixedDeltaTime + castSkin;

        if (Physics.SphereCast(transform.position, castRadius, dir, out RaycastHit hit, dist, collisionMask, QueryTriggerInteraction.Ignore))
        {
            if (!hit.transform.IsChildOf(transform) && speed >= crashSpeedThreshold)
            {
                StartCoroutine(CrashCutFallAndReset(hit.collider.name));
            }
        }
    }

    IEnumerator CrashCutFallAndReset(string hitName)
    {
        crashed = true;

        // Stopping the simulation (to avoid confusion between hover, thrust, etc.)
        if (simManager != null) simManager.StopSimulation();

        // Engine shut-off (controller lock)
        if (droneController != null) droneController.isCrashed = true;

        // Stop the propellers.
        foreach (var p in GetComponentsInChildren<PropellerSpin>(true))
            p.SetCrashed(true);

        // Controlled descent
        float t = 0f;
        float fallen = 0f;

        while (t < motorCutTime && fallen < fallDistance)
        {
            t += Time.fixedDeltaTime;

            float stepDown = Mathf.Min(
                (fallDistance - fallen),
                (fallDistance / Mathf.Max(0.0001f, motorCutTime)) * Time.fixedDeltaTime
            );

            DroneState s = dronePhysics.state;

            // Ground raycast: the height of the ground at the location.
            float groundY = float.NegativeInfinity;
            Vector3 rayStart = new Vector3(s.position.x, s.position.y + groundRayHeight, s.position.z);

            if (Physics.Raycast(rayStart, Vector3.down, out RaycastHit gHit, groundRayLength, collisionMask, QueryTriggerInteraction.Ignore))
                groundY = gHit.point.y;

            float targetY = s.position.y - stepDown;

            if (!float.IsNegativeInfinity(groundY))
            {
                float minY = groundY + minGroundClearance;
                if (targetY < minY) targetY = minY;
            }

            s.position = new Vector3(s.position.x, targetY, s.position.z);
            s.velocity = Vector3.zero;
            s.angularVelocity = Vector3.zero;
            dronePhysics.state = s;

            fallen += stepDown;
            yield return new WaitForFixedUpdate();
        }

        if (resetDelay > 0f)
            yield return new WaitForSeconds(resetDelay);

        // Reset state + transform
        DroneState rs = dronePhysics.state;
        rs.position = spawnPosition;
        rs.velocity = Vector3.zero;
        rs.eulerAngles = spawnEuler;
        rs.angularVelocity = Vector3.zero;
        dronePhysics.state = rs;

        transform.position = spawnPosition;
        transform.rotation = Quaternion.Euler(spawnEuler);

        // Turn the control back on.
        if (droneController != null)
        {
            droneController.isCrashed = false; 
            droneController.propVisualEnabled = false; // Let the fans start again from where they stopped.
            droneController.ResetAfterCrash(dronePhysics.state);
        }

        foreach (var p in GetComponentsInChildren<PropellerSpin>(true))
            p.SetCrashed(false);

        // Restart simulation
        if (simManager != null) simManager.StartSimulation();

        graceTimer = graceAfterReset;
        crashed = false;
    }
}

using UnityEngine;

public class GroundClamp : MonoBehaviour
{
    [Header("Refs")]
    public DronePhysics dronePhysics;

    [Header("Ground Detection")]
    public LayerMask groundMask = ~0;          // everything in groundmask
    public float castRadius = 0.45f;           // castradius for drone
    public float castDistance = 2.0f;          // castdistance for drone
    public float clearance = 0.05f;            // how far from ground
    public bool useBoxColliderRadius = true;   

    private BoxCollider boxCol;

    void Awake()
    {
        if (dronePhysics == null) dronePhysics = GetComponent<DronePhysics>();
        boxCol = GetComponent<BoxCollider>();
    }

    // SimulationManager will call this after each simulation step.
    public void ClampState(ref DroneState s)
    {
        float r = castRadius;

        if (useBoxColliderRadius && boxCol != null)
        {
            // BoxColler width approximately radius
            Vector3 ext = boxCol.bounds.extents;
            r = Mathf.Max(ext.x, ext.z);
        }

        // (If the drone is buried in the ground, it can rescue it)
        Vector3 origin = s.position + Vector3.up * 0.25f;

        if (Physics.SphereCast(origin, r, Vector3.down, out RaycastHit hit, castDistance, groundMask, QueryTriggerInteraction.Ignore))
        {
            // hit point
            float targetY = hit.point.y + clearance;

            // If the drone is below the ground or touches the ground while descending -> lift it up and stop the fall.
            if (s.position.y < targetY || s.velocity.y < 0f)
            {
                s.position = new Vector3(s.position.x, Mathf.Max(s.position.y, targetY), s.position.z);

                
                if (s.velocity.y < 0f)
                    s.velocity = new Vector3(s.velocity.x, 0f, s.velocity.z);
            }
        }
    }
}

using UnityEngine;


//The console shows how many times the drone hit peripherals at each step. Just for the control about the DroneCrashHandler.
public class PhysicsProbe : MonoBehaviour
{
    public float radius = 1.5f;
    public LayerMask mask = ~0;

    void FixedUpdate()
    {
        var hits = Physics.OverlapSphere(transform.position, radius, mask, QueryTriggerInteraction.Ignore);

        int count = 0;
        string first = "";

        foreach (var h in hits)
        {
            if (h == null) continue;
            if (h.transform.IsChildOf(transform)) continue;
            count++;
            if (first == "") first = h.name;
        }

        Debug.Log($"[ProbeSphere] hits={count} first={first}");
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.DrawWireSphere(transform.position, radius);
    }
}

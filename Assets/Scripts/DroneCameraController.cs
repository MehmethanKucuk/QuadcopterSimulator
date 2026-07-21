using UnityEngine;

public class DroneCameraController : MonoBehaviour
{
    public enum ViewMode { ThirdPerson, FirstPerson }

    [Header("Target to be tracked (Drone)")]
    public Transform target;

    [Header("Third Person Settings")]
    public Vector3 thirdPersonOffset = new Vector3(0f, 2f, -5f);   // Position: behind and above the drone
    public float thirdPersonFov = 60f;

    [Header("First Person Settings")]
    public Vector3 firstPersonOffset = new Vector3(0f, 0.1f, 0.2f); // right in front of (at the tip of) the nose
    public float firstPersonFov = 75f;

    [Header("General Settings")]
    public float followSmooth = 5f;   // position softening
    public float rotateSmooth = 10f;  // rotation softening
    public ViewMode mode = ViewMode.ThirdPerson;

    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
        if (target == null)
        {
            Debug.LogWarning("DroneCameraController: target not Assigned!");
        }
    }

    void LateUpdate()
    {
        if (target == null) return;

        // Switching modes (FPS or TPS) is done using the "C" key on the keyboard.
        if (Input.GetKeyDown(KeyCode.C))
        {
            ToggleMode();
        }

        // Which offset?
        Vector3 offset = (mode == ViewMode.ThirdPerson)
            ? thirdPersonOffset
            : firstPersonOffset;

        
        Vector3 desiredPos = target.TransformPoint(offset);
        transform.position = Vector3.Lerp(
            transform.position,
            desiredPos,
            followSmooth * Time.deltaTime
        );

        // Where is gonna look
        Vector3 lookTarget;
        if (mode == ViewMode.ThirdPerson)
        {
            // aim a little way ahead of the drone.
            lookTarget = target.position + target.forward * 10f;
        }
        else
        {
            // FPV:
            lookTarget = target.position + target.forward * 20f;
        }

        Quaternion desiredRot = Quaternion.LookRotation(lookTarget - transform.position, Vector3.up);
        transform.rotation = Quaternion.Slerp(
            transform.rotation,
            desiredRot,
            rotateSmooth * Time.deltaTime
        );
    }

    private void ToggleMode()
    {
        mode = (mode == ViewMode.ThirdPerson) ? ViewMode.FirstPerson : ViewMode.ThirdPerson;
        if (cam != null)
        {
            cam.fieldOfView = (mode == ViewMode.ThirdPerson) ? thirdPersonFov : firstPersonFov;
        }
        Debug.Log("Camera mode: " + mode);
    }
}

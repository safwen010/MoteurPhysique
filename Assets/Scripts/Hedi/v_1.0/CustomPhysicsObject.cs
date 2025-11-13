using UnityEngine;

[RequireComponent(typeof(CustomRigidbody3D))]
[RequireComponent(typeof(CustomCollider3D))]
public class CustomPhysicsObject : MonoBehaviour
{
    void Start()
    {
        var rb = GetComponent<CustomRigidbody3D>();
        var col = GetComponent<CustomCollider3D>();
        CustomPhysicsManager.Instance.Register(rb, col);
    }
}

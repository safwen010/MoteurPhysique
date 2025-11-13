using UnityEngine;
using System.Collections.Generic;

public class PhysicsDemo : MonoBehaviour
{
    SimplePhysics physics;
    List<GameObject> cubes;

    void Start()
    {
        physics = new SimplePhysics();
        cubes = new List<GameObject>();

        for (int i = 0; i < 2; i++)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.transform.position = new Vector3(i * 2, 5, 0);

            var rb = new CustomRigidbody(new Vec3(i * 2, 5, 0));
            var col = new BoxCollider3D(rb.position, new Vec3(1, 1, 1));

            physics.Add(rb, col);
            cubes.Add(go);
        }
    }

    void Update()
    {
        physics.Step(Time.deltaTime);

        for (int i = 0; i < cubes.Count; i++)
        {
            cubes[i].transform.position = new Vector3(
                physics.bodies[i].position.x,
                physics.bodies[i].position.y,
                physics.bodies[i].position.z
            );
        }
    }
}

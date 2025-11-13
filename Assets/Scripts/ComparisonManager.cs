// ComparisonManager.cs
using UnityEngine;

public class ComparisonManager : MonoBehaviour
{
    [Header("Alpha Values for Comparison")]
    public float[] alphaValues = { 0.0f, 0.5f, 1.0f, 1.5f };

    [Header ("Scene References")]
    public BrittleFractureSimulator[] simulations;

    void Start()
    {
        // Set up each simulation with different alpha values
        for (int i = 0; i < Mathf.Min(alphaValues.Length, simulations.Length); i++)
        {
            if (simulations[i] != null)
            {
                simulations[i].alpha = alphaValues[i];
                simulations[i].gameObject.name = $"Simulation_Alpha_{alphaValues[i]}";
            }
        }
    }

    void Update()
    {
        // Synchronize simulation start
        if (Input.GetKeyDown(KeyCode.Space))
        {
            foreach (BrittleFractureSimulator sim in simulations)
            {
                if (sim != null)
                {
                    // Use reflection to start simulation since it's private
                    sim.SendMessage("StartSimulation");
                }
            }
        }
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 400, 300));
        GUILayout.Label("BRITTLE FRACTURE COMPARISON");
        GUILayout.Label("Press SPACE to start all simulations simultaneously");
        GUILayout.Label("");

        for (int i = 0; i < simulations.Length; i++)
        {
            if (simulations[i] != null)
            {
                GUILayout.Label($"Scene {i + 1}: Alpha = {simulations[i].alpha:F1}");
            }
        }

        GUILayout.EndArea();
    }
}
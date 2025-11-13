using UnityEngine;

namespace CustomPhysics.v_1_1.Utils
{
    public static class GizmosDrawer
    {
        public static void DrawArrow(Vector3 from, Vector3 to, float arrowHeadLength = 0.25f, float arrowHeadAngle = 20.0f)
        {
            Gizmos.DrawLine(from, to);
            
            Vector3 direction = to - from;
            if (direction.magnitude < 0.001f) return;
            
            Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * Vector3.forward;
            Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * Vector3.forward;
            
            Gizmos.DrawLine(to, to + right * arrowHeadLength);
            Gizmos.DrawLine(to, to + left * arrowHeadLength);
        }
        
        public static void DrawBounds(Bounds bounds, Color color)
        {
            Gizmos.color = color;
            Gizmos.DrawWireCube(bounds.center, bounds.size);
        }
        
        public static void DrawCoordinateSystem(Vector3 position, Quaternion rotation, float scale = 1.0f)
        {
            // Axe X (rouge)
            Gizmos.color = Color.red;
            Gizmos.DrawRay(position, rotation * Vector3.right * scale);
            
            // Axe Y (vert)
            Gizmos.color = Color.green;
            Gizmos.DrawRay(position, rotation * Vector3.up * scale);
            
            // Axe Z (bleu)
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(position, rotation * Vector3.forward * scale);
        }
    }
}
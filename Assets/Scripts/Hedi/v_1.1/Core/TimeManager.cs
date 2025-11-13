using UnityEngine;

namespace CustomPhysics.v_1_1.Core
{
    public class TimeManager : MonoBehaviour
    {
        [Range(0.1f, 2.0f)]
        public float globalTimeScale = 1.0f;
        
        public static float FixedDeltaTime => Time.fixedDeltaTime * Instance.globalTimeScale;
        
        public static TimeManager Instance { get; private set; }
        
        void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
                DontDestroyOnLoad(gameObject);
            }
        }
    }
}
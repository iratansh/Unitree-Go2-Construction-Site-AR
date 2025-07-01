using UnityEngine;
using UnityEngine.XR;

public class ARConstructionProjector : MonoBehaviour
{
    [Header("AR Projection Settings")]
    public bool projectConstructionSite = true;
    public bool showVirtualObstacles = true;
    public bool showPathVisualization = true;
    public bool trackPhysicalGO2 = true;
    
    [Header("Construction Environment")]
    public GameObject[] constructionObjects;
    public GameObject[] obstacles;
    public GameObject[] machinery;
    public GameObject[] buildingElements;
    
    [Header("Path Visualization")]
    public LineRenderer pathRenderer;
    public GameObject waypointPrefab;
    public Material hologramMaterial;
    
    [Header("Physical GO2 Tracking")]
    public Transform physicalGO2Tracker;
    public float trackingUpdateRate = 30f; // Hz
    public bool usePhysicalGO2Position = true;
    
    [Header("AR Display Settings")]
    public float hologramOpacity = 0.7f;
    public Color constructionColor = Color.cyan;
    public Color obstacleColor = Color.red;
    public Color pathColor = Color.green;
    
    private Camera arCamera;
    private Material[] originalMaterials;
    private bool isProjecting = false;
    private Vector3 worldOrigin = Vector3.zero;
    private float lastTrackingUpdate = 0f;
    
    void Start()
    {
        SetupARProjection();
        SetupHologramMaterials();
        SetupPhysicalTracking();
        
        if (projectConstructionSite)
        {
            StartProjection();
        }
    }
    
    void SetupARProjection()
    {
        Debug.Log("Setting up AR Construction Site Projection...");
        
        // Find AR camera
        arCamera = Camera.main;
        if (arCamera == null)
        {
            arCamera = FindObjectOfType<Camera>();
        }
        
        // Configure camera for AR hologram projection
        if (arCamera != null)
        {
            arCamera.clearFlags = CameraClearFlags.SolidColor;
            arCamera.backgroundColor = Color.clear; // Transparent background for passthrough
            
            Debug.Log("AR Camera configured for hologram projection");
        }
        
        // Find all construction objects if not manually assigned
        if (constructionObjects == null || constructionObjects.Length == 0)
        {
            FindConstructionObjects();
        }
    }
    
    void FindConstructionObjects()
    {
        // Auto-discover construction site objects
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        System.Collections.Generic.List<GameObject> construction = new System.Collections.Generic.List<GameObject>();
        System.Collections.Generic.List<GameObject> obstacleList = new System.Collections.Generic.List<GameObject>();
        
        foreach (GameObject obj in allObjects)
        {
            string name = obj.name.ToLower();
            
            // Categorize objects based on common construction site names
            if (name.Contains("building") || name.Contains("wall") || name.Contains("floor") || 
                name.Contains("roof") || name.Contains("foundation") || name.Contains("structure"))
            {
                construction.Add(obj);
            }
            else if (name.Contains("obstacle") || name.Contains("debris") || name.Contains("barrier") ||
                     name.Contains("cone") || name.Contains("warning") || name.Contains("hazard"))
            {
                obstacleList.Add(obj);
            }
            else if (name.Contains("crane") || name.Contains("excavator") || name.Contains("bulldozer") ||
                     name.Contains("machinery") || name.Contains("equipment"))
            {
                if (machinery == null || machinery.Length == 0)
                {
                    machinery = new GameObject[] { obj };
                }
            }
        }
        
        constructionObjects = construction.ToArray();
        obstacles = obstacleList.ToArray();
        
        Debug.Log($"Auto-discovered: {constructionObjects.Length} construction objects, {obstacles.Length} obstacles");
    }
    
    void SetupHologramMaterials()
    {
        Debug.Log("Setting up hologram materials...");
        
        // Create hologram material if not assigned
        if (hologramMaterial == null)
        {
            hologramMaterial = new Material(Shader.Find("Standard"));
            hologramMaterial.SetFloat("_Mode", 3); // Transparent mode
            hologramMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            hologramMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            hologramMaterial.SetInt("_ZWrite", 0);
            hologramMaterial.DisableKeyword("_ALPHATEST_ON");
            hologramMaterial.EnableKeyword("_ALPHABLEND_ON");
            hologramMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            hologramMaterial.renderQueue = 3000;
        }
        
        // Store original materials for restoration
        StoreOriginalMaterials();
    }
    
    void StoreOriginalMaterials()
    {
        // Store original materials so we can toggle back
        System.Collections.Generic.List<Material> materials = new System.Collections.Generic.List<Material>();
        
        Renderer[] allRenderers = FindObjectsOfType<Renderer>();
        foreach (Renderer renderer in allRenderers)
        {
            materials.AddRange(renderer.materials);
        }
        
        originalMaterials = materials.ToArray();
    }
    
    void SetupPhysicalTracking()
    {
        Debug.Log("Setting up physical GO2 tracking...");
        
        // Create a tracker for the physical GO2 if not assigned
        if (physicalGO2Tracker == null)
        {
            GameObject tracker = new GameObject("Physical GO2 Tracker");
            physicalGO2Tracker = tracker.transform;
            
            // Add a visual indicator for the physical GO2 position
            GameObject indicator = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            indicator.name = "GO2 Position Indicator";
            indicator.transform.parent = physicalGO2Tracker;
            indicator.transform.localPosition = Vector3.zero;
            indicator.transform.localScale = Vector3.one * 0.2f;
            
            // Make it a bright holographic color
            Renderer renderer = indicator.GetComponent<Renderer>();
            if (renderer != null)
            {
                Material indicatorMat = new Material(hologramMaterial);
                indicatorMat.color = new Color(1f, 0f, 1f, 0.8f); // Bright magenta
                renderer.material = indicatorMat;
            }
            
            // Remove collider
            Destroy(indicator.GetComponent<Collider>());
        }
        
        Debug.Log("Physical GO2 tracking setup complete");
    }
    
    public void StartProjection()
    {
        if (isProjecting) return;
        
        Debug.Log("Starting AR construction site projection...");
        isProjecting = true;
        
        // Apply hologram materials to construction objects
        ApplyHologramMaterials(constructionObjects, constructionColor);
        ApplyHologramMaterials(obstacles, obstacleColor);
        ApplyHologramMaterials(machinery, Color.yellow);
        ApplyHologramMaterials(buildingElements, Color.white);
        
        // Setup path visualization
        if (showPathVisualization)
        {
            SetupPathVisualization();
        }
        
        Debug.Log("AR projection started - construction site is now holographic");
    }
    
    public void StopProjection()
    {
        if (!isProjecting) return;
        
        Debug.Log("Stopping AR projection...");
        isProjecting = false;
        
        // Restore original materials
        RestoreOriginalMaterials();
        
        Debug.Log("AR projection stopped - returned to normal rendering");
    }
    
    void ApplyHologramMaterials(GameObject[] objects, Color hologramColor)
    {
        if (objects == null) return;
        
        foreach (GameObject obj in objects)
        {
            if (obj == null) continue;
            
            Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();
            foreach (Renderer renderer in renderers)
            {
                Material[] newMaterials = new Material[renderer.materials.Length];
                for (int i = 0; i < newMaterials.Length; i++)
                {
                    newMaterials[i] = new Material(hologramMaterial);
                    newMaterials[i].color = new Color(hologramColor.r, hologramColor.g, hologramColor.b, hologramOpacity);
                    
                    // Add holographic effects
                    newMaterials[i].SetFloat("_Metallic", 0.8f);
                    newMaterials[i].SetFloat("_Smoothness", 0.9f);
                    newMaterials[i].EnableKeyword("_EMISSION");
                    newMaterials[i].SetColor("_EmissionColor", hologramColor * 0.3f);
                }
                renderer.materials = newMaterials;
            }
        }
    }
    
    void RestoreOriginalMaterials()
    {
        // This is a simplified restoration - in a full implementation,
        // you'd want to store and restore materials per object
        Renderer[] allRenderers = FindObjectsOfType<Renderer>();
        foreach (Renderer renderer in allRenderers)
        {
            // Reset to default material as a fallback
            Material defaultMat = new Material(Shader.Find("Standard"));
            Material[] defaultMaterials = new Material[renderer.materials.Length];
            for (int i = 0; i < defaultMaterials.Length; i++)
            {
                defaultMaterials[i] = defaultMat;
            }
            renderer.materials = defaultMaterials;
        }
    }
    
    void SetupPathVisualization()
    {
        if (pathRenderer == null)
        {
            GameObject pathGO = new GameObject("AR Path Visualizer");
            pathRenderer = pathGO.AddComponent<LineRenderer>();
        }
        
        // Configure path renderer for AR
        pathRenderer.material = new Material(hologramMaterial);
        pathRenderer.startColor = new Color(pathColor.r, pathColor.g, pathColor.b, hologramOpacity);
        pathRenderer.endColor = new Color(pathColor.r, pathColor.g, pathColor.b, hologramOpacity);
        pathRenderer.startWidth = 0.1f;
        pathRenderer.endWidth = 0.1f;
        pathRenderer.useWorldSpace = true;
        
        // Add emission for holographic effect
        pathRenderer.material.EnableKeyword("_EMISSION");
        pathRenderer.material.SetColor("_EmissionColor", pathColor * 0.5f);
        
        Debug.Log("Path visualization setup for AR projection");
    }
    
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.H))
        {
            ToggleProjection();
        }
        
        if (trackPhysicalGO2 && Time.time - lastTrackingUpdate > 1f / trackingUpdateRate)
        {
            UpdatePhysicalGO2Tracking();
            lastTrackingUpdate = Time.time;
        }
        
        UpdateHologramEffects();
    }
    
    void ToggleProjection()
    {
        if (isProjecting)
        {
            StopProjection();
        }
        else
        {
            StartProjection();
        }
    }
    
    void UpdatePhysicalGO2Tracking()
    {
        // This would integrate with your physical GO2's position data
        // For now, it's a placeholder that could be updated via network data
        
        if (usePhysicalGO2Position && physicalGO2Tracker != null)
        {
            // TODO: Get actual position from physical GO2 robot
            // This would come from your GO2's sensor data or tracking system
            
            // Placeholder: simulate movement
            Vector3 currentPos = physicalGO2Tracker.position;
            // In real implementation, this would be the actual GO2's position
            // physicalGO2Tracker.position = GetPhysicalGO2Position();
        }
    }
    
    void UpdateHologramEffects()
    {
        if (!isProjecting) return;
        
        // Add subtle animation to holograms
        float time = Time.time;
        float pulseEffect = 0.8f + Mathf.Sin(time * 2f) * 0.2f;
        
        // Update emission intensity for holographic effect
        if (hologramMaterial != null)
        {
            Color emission = hologramMaterial.GetColor("_EmissionColor");
            hologramMaterial.SetColor("_EmissionColor", emission * pulseEffect);
        }
    }
    
    public void SetWorldOrigin(Vector3 origin)
    {
        worldOrigin = origin;
        Debug.Log($"AR world origin set to: {origin}");
    }
    
    public void UpdatePathVisualization(Vector3[] pathPoints)
    {
        if (pathRenderer == null || !showPathVisualization) return;
        
        pathRenderer.positionCount = pathPoints.Length;
        pathRenderer.SetPositions(pathPoints);
        
        Debug.Log($"Updated AR path visualization with {pathPoints.Length} points");
    }
    
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("AR Construction Projection:");
        GUILayout.Label($"Projecting: {isProjecting}");
        GUILayout.Label($"Construction Objects: {(constructionObjects != null ? constructionObjects.Length : 0)}");
        GUILayout.Label($"Obstacles: {(obstacles != null ? obstacles.Length : 0)}");
        GUILayout.Label($"Physical GO2 Tracking: {trackPhysicalGO2}");
        GUILayout.Label("");
        GUILayout.Label("Controls:");
        GUILayout.Label("H - Toggle Hologram Projection");
        
        if (GUILayout.Button("Start Projection"))
        {
            StartProjection();
        }
        
        if (GUILayout.Button("Stop Projection"))
        {
            StopProjection();
        }
        
        GUILayout.EndArea();
    }
} 
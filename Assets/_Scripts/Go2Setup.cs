using UnityEngine;

[ExecuteInEditMode]
public class Go2Setup : MonoBehaviour
{
    [Header("Quick Setup")]
    public bool setupGO2 = false;
    public GameObject go2Prefab;
    
    void Update()
    {
        if (setupGO2)
        {
            setupGO2 = false;
            SetupGO2InScene();
        }
    }
    
    void SetupGO2InScene()
    {
        // Find or create GO2 Controller
        GameObject controller = GameObject.Find("GO2 Controller");
        if (controller == null)
        {
            controller = new GameObject("GO2 Controller");
        }
        
        // Add Go2PathController component
        Go2PathController pathController = controller.GetComponent<Go2PathController>();
        if (pathController == null)
        {
            pathController = controller.AddComponent<Go2PathController>();
        }
        
        // Find or create GO2 model
        GameObject go2Model = GameObject.Find("GO2");
        if (go2Model == null && go2Prefab != null)
        {
            go2Model = Instantiate(go2Prefab);
            go2Model.name = "GO2";
        }
        else if (go2Model == null)
        {
            // Create a placeholder if no prefab is provided
            go2Model = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            go2Model.name = "GO2";
            go2Model.transform.localScale = new Vector3(0.5f, 1f, 0.5f);
        }
        
        // Position GO2 at a reasonable location
        go2Model.transform.position = new Vector3(0, 0, 0);
        
        // Configure the path controller
        pathController.go2Model = go2Model;
        pathController.pathLength = 10f;
        pathController.defaultSpeed = 0.5f;
        pathController.turnSpeed = 90f;
        pathController.standaloneMode = true; // Enable standalone mode by default
        pathController.enableNetworking = false; // Disable networking for testing
        
        // Create start and end points
        GameObject startPoint = GameObject.Find("Start Point");
        if (startPoint == null)
        {
            startPoint = new GameObject("Start Point");
            startPoint.transform.position = Vector3.zero;
        }
        
        GameObject endPoint = GameObject.Find("End Point");
        if (endPoint == null)
        {
            endPoint = new GameObject("End Point");
            endPoint.transform.position = new Vector3(0, 0, 10);
        }
        
        pathController.startPoint = startPoint.transform;
        pathController.endPoint = endPoint.transform;
        
        // Create path visualizer
        LineRenderer lineRenderer = controller.GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = controller.AddComponent<LineRenderer>();
        }
        
        // Configure line renderer
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.positionCount = 2;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.green;
        lineRenderer.endColor = Color.red;
        
        pathController.pathVisualizer = lineRenderer;
        
        // Add visual markers
        AddMarker(startPoint, Color.green, "START");
        AddMarker(endPoint, Color.red, "END");
        
        Debug.Log("GO2 setup complete! Use these controls in Play mode:");
        Debug.Log("- Space: Start movement");
        Debug.Log("- S: Stop movement");
        Debug.Log("- R: Reset position");
        Debug.Log("- 1: Forward mode");
        Debug.Log("- 2: Rightward mode");
        Debug.Log("- Left/Right Arrow: Adjust gaze angle");
    }
    
    void AddMarker(GameObject target, Color color, string label)
    {
        // Add a visible marker
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = label + " Marker";
        marker.transform.parent = target.transform;
        marker.transform.localPosition = Vector3.zero;
        marker.transform.localScale = Vector3.one * 0.3f;
        
        // Set color
        Renderer renderer = marker.GetComponent<Renderer>();
        renderer.material = new Material(Shader.Find("Standard"));
        renderer.material.color = color;
        
        // Remove collider
        Collider collider = marker.GetComponent<Collider>();
        if (collider != null)
        {
            DestroyImmediate(collider);
        }
    }
} 
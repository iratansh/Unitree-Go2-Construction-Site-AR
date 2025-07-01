using UnityEngine;

public class Go2SafeMode : MonoBehaviour
{
    [Header("Safe Mode Settings")]
    public bool enableSafeMode = true;
    public bool disableUrdfRobot = true;
    public bool disableController = true;
    public bool disableFKRobot = true;
    
    private MonoBehaviour[] disabledComponents;
    
    void Start()
    {
        if (enableSafeMode)
        {
            DisableConflictingComponents();
        }
    }
    
    void DisableConflictingComponents()
    {
        MonoBehaviour[] allComponents = GetComponents<MonoBehaviour>();
        System.Collections.Generic.List<MonoBehaviour> toDisable = new System.Collections.Generic.List<MonoBehaviour>();
        
        foreach (MonoBehaviour component in allComponents)
        {
            if (component == this) continue; // Don't disable ourselves
            if (component.GetType().Name == "Go2PathController") continue; // Don't disable our controller
            if (component.GetType().Name == "Go2Debug") continue; // Don't disable debug
            
            string componentName = component.GetType().Name.ToLower();
            
            if (disableUrdfRobot && componentName.Contains("urdf"))
            {
                component.enabled = false;
                toDisable.Add(component);
                Debug.Log($"Disabled {component.GetType().Name} for safe mode");
            }
            else if (disableController && componentName.Contains("controller") && !componentName.Contains("go2path"))
            {
                component.enabled = false;
                toDisable.Add(component);
                Debug.Log($"Disabled {component.GetType().Name} for safe mode");
            }
            else if (disableFKRobot && componentName.Contains("fk"))
            {
                component.enabled = false;
                toDisable.Add(component);
                Debug.Log($"Disabled {component.GetType().Name} for safe mode");
            }
        }
        
        disabledComponents = toDisable.ToArray();
        
        if (disabledComponents.Length > 0)
        {
            Debug.Log($"Safe Mode: Disabled {disabledComponents.Length} potentially conflicting components");
        }
    }
    
    void OnDestroy()
    {
        // Re-enable components when this is destroyed
        if (disabledComponents != null)
        {
            foreach (MonoBehaviour component in disabledComponents)
            {
                if (component != null)
                {
                    component.enabled = true;
                }
            }
        }
    }
    
    [ContextMenu("Toggle Safe Mode")]
    public void ToggleSafeMode()
    {
        enableSafeMode = !enableSafeMode;
        
        if (enableSafeMode)
        {
            DisableConflictingComponents();
        }
        else
        {
            // Re-enable all disabled components
            if (disabledComponents != null)
            {
                foreach (MonoBehaviour component in disabledComponents)
                {
                    if (component != null)
                    {
                        component.enabled = true;
                        Debug.Log($"Re-enabled {component.GetType().Name}");
                    }
                }
            }
        }
    }
} 
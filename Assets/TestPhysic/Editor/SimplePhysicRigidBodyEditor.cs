using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(SimplePhysicRigidBody), true)]
public class SimplePhysicRigidBodyEditor : Editor
{
    public override void OnInspectorGUI()
    {
        var obj = (SimplePhysicRigidBody)target;
        base.OnInspectorGUI();

        if(GUILayout.Button("next loop collision"))
        {
            obj.debugImpulse = true;
        }
    }
}

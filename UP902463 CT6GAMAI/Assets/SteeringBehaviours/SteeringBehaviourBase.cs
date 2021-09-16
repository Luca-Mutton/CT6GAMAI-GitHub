using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class SteeringBehaviourBase : MonoBehaviour
{
    //Needs to be overidden in child classes
    public abstract Vector3 Calculate();
}

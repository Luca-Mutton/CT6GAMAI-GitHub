using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Seek : SteeringBehaviourBase
{
    public Vector3 SeekTargetPos;
    public bool SeekEnabled = false;

    public override Vector3 Calculate()
    {
        AI_States AI = GetComponent<AI_States>(); //get variables from AI_states script
        
        //(targetPosition -currentPosition).normalized * maxspeed of AI
        Vector3 DesiredVelocity = (SeekTargetPos - transform.position).normalized * AI.MaxSpeed; 

        //if seek is true
        if (SeekEnabled == true)
        {
            return (DesiredVelocity - AI.Velocity); //returns steering force
        }
        else
        {
            SeekEnabled = false; 
            return Vector3.zero;
        }

        
    }
}
